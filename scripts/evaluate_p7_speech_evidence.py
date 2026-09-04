#!/usr/bin/env python3
"""Independently validate Phase-7 local multilingual microphone evidence.

The evaluator deliberately validates artifacts rather than accepting a typed
transcript.  It recomputes microphone WAV hashes and the local model manifest,
requires Whisper's unhinted language result, and proves that the exact raw
transcript reached Phase 6's drawing-intent parser interface.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
import os
from pathlib import Path
import re
import tempfile
from typing import Any
import wave


class EvaluationError(RuntimeError):
    """Raised for a malformed or insufficient evidence artifact."""


EXPECTED_CASES = {"zh": "circle", "en": "square", "fr": "triangle"}
ALLOWED_CAPTURE_METHODS = {"pulse_parec", "windows_directshow"}
HEX_64 = re.compile(r"^[0-9a-f]{64}$")
HEX_40 = re.compile(r"^[0-9a-f]{40}$")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise EvaluationError(message)


def is_integer(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def integer(value: Any, label: str, *, minimum: int | None = None) -> int:
    require(is_integer(value), f"{label} is not an integer")
    if minimum is not None:
        require(value >= minimum, f"{label} is below {minimum}")
    return value


def finite(value: Any, label: str, *, minimum: float | None = None, maximum: float | None = None) -> float:
    require(not isinstance(value, bool), f"{label} is not numeric")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise EvaluationError(f"{label} is not numeric") from exc
    require(math.isfinite(result), f"{label} is not finite")
    if minimum is not None:
        require(result >= minimum, f"{label} is below {minimum}")
    if maximum is not None:
        require(result <= maximum, f"{label} is above {maximum}")
    return result


def nonempty_string(value: Any, label: str) -> str:
    require(isinstance(value, str) and bool(value.strip()), f"{label} is not a nonempty string")
    return value


def digest(value: Any, label: str) -> str:
    result = nonempty_string(value, label).lower()
    require(bool(HEX_64.fullmatch(result)), f"{label} is not a SHA-256 digest")
    return result


def load_json(path: Path, label: str) -> dict[str, Any]:
    require(path.is_file(), f"missing {label}: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise EvaluationError(f"invalid {label}: {path}: {exc}") from exc
    require(isinstance(value, dict), f"{label} root is not an object: {path}")
    return value


def canonical_json_sha256(value: Any) -> str:
    raw = json.dumps(value, ensure_ascii=True, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return sha256(raw).hexdigest()


def under(path: Path, parent: Path, label: str) -> None:
    try:
        path.relative_to(parent)
    except ValueError as exc:
        raise EvaluationError(f"{label} escapes its sealed case directory: {path}") from exc


def existing_absolute_file(value: Any, label: str) -> Path:
    raw = nonempty_string(value, label)
    path = Path(raw)
    require(path.is_absolute(), f"{label} must be absolute")
    try:
        resolved = path.resolve(strict=True)
    except OSError as exc:
        raise EvaluationError(f"{label} cannot be resolved: {path}") from exc
    require(resolved.is_file(), f"{label} is not a regular file: {resolved}")
    return resolved


def existing_absolute_directory(value: Any, label: str) -> Path:
    raw = nonempty_string(value, label)
    path = Path(raw)
    require(path.is_absolute(), f"{label} must be absolute")
    try:
        resolved = path.resolve(strict=True)
    except OSError as exc:
        raise EvaluationError(f"{label} cannot be resolved: {path}") from exc
    require(resolved.is_dir(), f"{label} is not a directory: {resolved}")
    return resolved


def file_sha256(path: Path) -> str:
    hasher = sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def exact_mapping(value: Any, expected_keys: set[str], label: str) -> dict[str, Any]:
    require(isinstance(value, dict), f"{label} is not an object")
    require(set(value) == expected_keys, f"{label} has unexpected keys: {sorted(set(value).symmetric_difference(expected_keys))}")
    return value


def validate_lock(value: Any, label: str) -> dict[str, Any]:
    lock = exact_mapping(value, {"schema_version", "backend", "model"}, label)
    require(lock["schema_version"] == 1, f"{label} has unsupported schema")
    backend = exact_mapping(lock["backend"], {"name", "package_version"}, f"{label}.backend")
    require(backend["name"] == "faster-whisper", f"{label} does not pin faster-whisper")
    nonempty_string(backend["package_version"], f"{label}.backend.package_version")
    model = exact_mapping(
        lock["model"],
        {"repository", "revision", "directory_name", "manifest_sha256"},
        f"{label}.model",
    )
    nonempty_string(model["repository"], f"{label}.model.repository")
    revision = nonempty_string(model["revision"], f"{label}.model.revision").lower()
    require(bool(HEX_40.fullmatch(revision)), f"{label}.model.revision is not a pinned 40-hex revision")
    nonempty_string(model["directory_name"], f"{label}.model.directory_name")
    digest(model["manifest_sha256"], f"{label}.model.manifest_sha256")
    return lock


def actual_model_files(model_path: Path) -> list[dict[str, Any]]:
    manifest_name = ".phase7_model_manifest.json"
    files: list[dict[str, Any]] = []
    for candidate in model_path.rglob("*"):
        relative = candidate.relative_to(model_path)
        if relative.name == manifest_name or ".cache" in relative.parts:
            continue
        require(not candidate.is_symlink(), f"local model contains an unsealed symlink: {relative}")
        if not candidate.is_file():
            continue
        files.append(
            {
                "path": relative.as_posix(),
                "sha256": file_sha256(candidate),
                "size_bytes": candidate.stat().st_size,
            }
        )
    files.sort(key=lambda item: str(item["path"]))
    require(bool(files), "local model contains no files")
    return files


def validate_local_model(model_path: Path, lock: dict[str, Any], cache: dict[Path, str]) -> str:
    if model_path in cache:
        return cache[model_path]
    model = lock["model"]
    require(model_path.name == model["directory_name"], "local model directory basename does not match lock")
    manifest_path = model_path / ".phase7_model_manifest.json"
    manifest = load_json(manifest_path, "local model manifest")
    manifest = exact_mapping(manifest, {"schema_version", "files"}, "local model manifest")
    require(manifest["schema_version"] == 1, "local model manifest has unsupported schema")
    require(isinstance(manifest["files"], list), "local model manifest files is not a list")
    actual = actual_model_files(model_path)
    require(manifest["files"] == actual, "local model contents differ from its sealed manifest")
    canonical = {"schema_version": 1, "files": actual}
    manifest_sha = canonical_json_sha256(canonical)
    require(manifest_sha == model["manifest_sha256"], "local model manifest hash differs from lock")
    cache[model_path] = manifest_sha
    return manifest_sha


def wav_properties(path: Path) -> dict[str, Any]:
    try:
        with wave.open(str(path), "rb") as audio:
            channels = audio.getnchannels()
            sample_rate = audio.getframerate()
            sample_width = audio.getsampwidth()
            frames = audio.getnframes()
    except (OSError, wave.Error) as exc:
        raise EvaluationError(f"invalid microphone WAV {path}: {exc}") from exc
    require(channels == 1, "microphone WAV is not mono")
    require(sample_rate == 16_000, "microphone WAV is not 16 kHz")
    require(sample_width == 2, "microphone WAV is not signed 16-bit")
    require(frames > 0, "microphone WAV contains no frames")
    return {
        "container": "WAV",
        "sample_rate_hz": sample_rate,
        "channels": channels,
        "sample_width_bytes": sample_width,
        "frames": frames,
        "duration_sec": frames / sample_rate,
    }


def validate_audio_format(value: Any, actual: dict[str, Any], label: str) -> None:
    format_value = exact_mapping(
        value,
        {"container", "sample_rate_hz", "channels", "sample_width_bytes", "frames", "duration_sec"},
        label,
    )
    require(format_value["container"] == actual["container"], f"{label}.container differs from WAV")
    for key in ("sample_rate_hz", "channels", "sample_width_bytes", "frames"):
        require(integer(format_value[key], f"{label}.{key}", minimum=1) == actual[key], f"{label}.{key} differs from WAV")
    require(
        math.isclose(finite(format_value["duration_sec"], f"{label}.duration_sec", minimum=0.0), actual["duration_sec"], abs_tol=1e-9),
        f"{label}.duration_sec differs from WAV",
    )


def validate_capture(value: Any, evidence_path: Path, created_at_ns: int) -> tuple[Path, dict[str, Any]]:
    capture = exact_mapping(
        value,
        {"metadata_path", "method", "input_kind", "acquired_at_unix_ns", "ended_at_unix_ns", "audio"},
        "capture",
    )
    case_root = evidence_path.parent.parent.resolve()
    metadata_path = existing_absolute_file(capture["metadata_path"], "capture.metadata_path")
    under(metadata_path, case_root, "capture metadata")
    metadata = load_json(metadata_path, "capture metadata")
    required_metadata_keys = {
        "schema_version",
        "capture_method",
        "input_kind",
        "source",
        "audio_filename",
        "started_at_unix_ns",
        "ended_at_unix_ns",
        "duration_requested_sec",
        "audio_sha256",
        "audio_size_bytes",
        "audio_format",
    }
    require(required_metadata_keys.issubset(metadata), "capture metadata omits required provenance")
    require(metadata["schema_version"] == 1, "capture metadata has unsupported schema")
    method = nonempty_string(capture["method"], "capture.method")
    require(method in ALLOWED_CAPTURE_METHODS, "capture method is not an approved microphone backend")
    require(metadata["capture_method"] == method, "capture method differs from capture metadata")
    require(capture["input_kind"] == "microphone" and metadata["input_kind"] == "microphone", "input is not microphone audio")
    source = nonempty_string(metadata["source"], "capture metadata source")
    require(not source.casefold().endswith(".monitor"), "capture used a monitor rather than a microphone")
    if method == "pulse_parec":
        nonempty_string(metadata.get("pulse_server"), "PulseAudio server")
    if method == "windows_directshow":
        require(metadata.get("capture_backend") == "ffmpeg_directshow", "DirectShow capture lacks ffmpeg provenance")

    acquired = integer(capture["acquired_at_unix_ns"], "capture.acquired_at_unix_ns", minimum=1)
    ended = integer(capture["ended_at_unix_ns"], "capture.ended_at_unix_ns", minimum=1)
    metadata_started = integer(metadata["started_at_unix_ns"], "capture metadata started_at_unix_ns", minimum=1)
    metadata_ended = integer(metadata["ended_at_unix_ns"], "capture metadata ended_at_unix_ns", minimum=1)
    require(acquired == metadata_started and ended == metadata_ended, "capture timestamps differ from microphone metadata")
    require(ended >= acquired, "capture ends before it starts")
    require(created_at_ns >= ended, "speech evidence predates microphone acquisition")
    requested_duration = finite(metadata["duration_requested_sec"], "capture metadata duration_requested_sec", minimum=1.0, maximum=60.0)

    audio = exact_mapping(
        capture["audio"],
        {"path", "sha256", "size_bytes", "format"},
        "capture.audio",
    )
    audio_path = existing_absolute_file(audio["path"], "capture.audio.path")
    under(audio_path, case_root, "microphone audio")
    require(metadata["audio_filename"] == audio_path.name, "capture metadata audio filename differs from evidence")
    observed_sha = file_sha256(audio_path)
    expected_sha = digest(audio["sha256"], "capture.audio.sha256")
    require(observed_sha == expected_sha, "microphone WAV hash differs from evidence")
    require(observed_sha == digest(metadata["audio_sha256"], "capture metadata audio_sha256"), "microphone WAV hash differs from metadata")
    observed_size = audio_path.stat().st_size
    require(integer(audio["size_bytes"], "capture.audio.size_bytes", minimum=1) == observed_size, "microphone WAV size differs from evidence")
    require(integer(metadata["audio_size_bytes"], "capture metadata audio_size_bytes", minimum=1) == observed_size, "microphone WAV size differs from metadata")
    actual_format = wav_properties(audio_path)
    validate_audio_format(audio["format"], actual_format, "capture.audio.format")
    validate_audio_format(metadata["audio_format"], actual_format, "capture metadata audio_format")
    require(actual_format["duration_sec"] >= 0.5, "microphone WAV is too short for a spoken command")
    require(actual_format["duration_sec"] <= requested_duration + 5.0, "microphone WAV exceeds its bounded requested duration")
    return audio_path, metadata


def validate_phrase(language: str, transcript: str) -> None:
    if language == "zh":
        require("画" in transcript and any(token in transcript for token in ("圆", "圓", "圈")), "Chinese transcript is not a spoken draw-circle command")
    elif language == "en":
        lowered = transcript.casefold()
        require(bool(re.search(r"\bdraw\b", lowered)) and bool(re.search(r"\bsquare\b", lowered)), "English transcript is not a spoken draw-square command")
    elif language == "fr":
        lowered = transcript.casefold()
        require(bool(re.search(r"\bdessine(?:z)?\b", lowered)) and bool(re.search(r"\btriangle\b", lowered)), "French transcript is not a spoken draw-triangle command")
    else:
        raise EvaluationError(f"unsupported evidence language: {language}")


def validate_one(path: Path, lock: dict[str, Any], model_cache: dict[Path, str]) -> dict[str, Any]:
    evidence = load_json(path, "speech evidence")
    require(path.name == "speech_execution_evidence.json" and path.parent.name == "runtime", f"{path}: evidence must be sealed at case/runtime/speech_execution_evidence.json")
    require(evidence.get("schema_version") == 1, f"{path}: unsupported evidence schema")
    require(evidence.get("scope") == "PHASE7_LOCAL_MULTILINGUAL_MICROPHONE_TO_DRAWING", f"{path}: unexpected evidence scope")
    status = evidence.get("status")
    require(isinstance(status, dict) and status.get("P7_SPEECH_RUNTIME") == "PASS", f"{path}: speech runtime did not pass")
    created_at_ns = integer(evidence.get("created_at_unix_ns"), f"{path}: created_at_unix_ns", minimum=1)
    require(evidence.get("model_lock") == lock, f"{path}: embedded model lock differs from supplied lock")

    _, capture_metadata = validate_capture(evidence.get("capture"), path, created_at_ns)
    asr = evidence.get("asr")
    require(isinstance(asr, dict), f"{path}: ASR record is missing")
    backend = asr.get("backend")
    require(isinstance(backend, dict), f"{path}: ASR backend record is missing")
    require(backend.get("name") == "faster-whisper", f"{path}: ASR backend is not faster-whisper")
    require(backend.get("package_version") == lock["backend"]["package_version"], f"{path}: ASR package differs from lock")
    model_path = existing_absolute_directory(backend.get("model_path"), f"{path}: ASR model path")
    require(backend.get("model_path_is_local") is True, f"{path}: ASR model is not asserted local")
    require(backend.get("runtime_network_enabled") is False, f"{path}: ASR runtime network is enabled")
    require(backend.get("automatic_download_permitted") is False, f"{path}: ASR permits automatic model download")
    nonempty_string(backend.get("device"), f"{path}: ASR device")
    nonempty_string(backend.get("compute_type"), f"{path}: ASR compute type")
    require(asr.get("model") == lock["model"], f"{path}: ASR model identity differs from lock")
    validate_local_model(model_path, lock, model_cache)
    require(asr.get("task") == "transcribe", f"{path}: ASR task is not transcription")
    require(asr.get("language_hint") is None, f"{path}: ASR received a manual language hint")
    language = asr.get("detected_language")
    require(language in EXPECTED_CASES, f"{path}: ASR language is unsupported")
    probability = finite(asr.get("native_language_probability"), f"{path}: ASR native language probability", minimum=0.60, maximum=1.0)
    segments = asr.get("segments")
    require(isinstance(segments, list) and bool(segments), f"{path}: ASR segments are missing")
    segment_text: list[str] = []
    for index, segment in enumerate(segments):
        require(isinstance(segment, dict), f"{path}: ASR segment {index} is malformed")
        text = nonempty_string(segment.get("text"), f"{path}: ASR segment {index} text")
        finite(segment.get("avg_logprob"), f"{path}: ASR segment {index} avg_logprob")
        finite(segment.get("no_speech_prob"), f"{path}: ASR segment {index} no_speech_prob", minimum=0.0, maximum=1.0)
        segment_text.append(text)
    transcript = nonempty_string(asr.get("raw_transcript"), f"{path}: ASR raw transcript")
    require(transcript == "".join(segment_text).strip(), f"{path}: raw transcript is not derived from ASR segments")
    validate_phrase(language, transcript)

    command = evidence.get("speech_command")
    require(isinstance(command, dict), f"{path}: SpeechCommand is missing")
    require(command.get("transcript") == transcript, f"{path}: SpeechCommand transcript differs from ASR")
    require(command.get("detected_language") == language, f"{path}: SpeechCommand language differs from ASR")
    command_confidence = finite(command.get("confidence"), f"{path}: SpeechCommand confidence", minimum=0.0, maximum=1.0)
    require(math.isclose(command_confidence, probability, rel_tol=0.0, abs_tol=1e-12), f"{path}: SpeechCommand confidence differs from native ASR probability")
    require(command.get("confidence_provenance") == "whisper_language_probability", f"{path}: SpeechCommand confidence has no native Whisper provenance")
    integer(command.get("timestamp_ns"), f"{path}: SpeechCommand timestamp_ns", minimum=1)
    require(command.get("intent") == "DRAW", f"{path}: SpeechCommand did not normalize to DRAW")

    dispatch = evidence.get("downstream_dispatch")
    require(isinstance(dispatch, dict), f"{path}: downstream drawing dispatch is missing")
    require(dispatch.get("interface") == "physical_ai_drawing.parse_drawing_instruction", f"{path}: transcript did not enter the Phase-6 parser interface")
    require(dispatch.get("input_transcript") == transcript, f"{path}: downstream received a different transcript")
    require(dispatch.get("accepted") is True, f"{path}: Phase-6 parser rejected the spoken command")
    drawing_intent = dispatch.get("drawing_intent")
    require(isinstance(drawing_intent, dict), f"{path}: downstream DrawingIntent is missing")
    shape = drawing_intent.get("shape")
    require(shape == EXPECTED_CASES[language], f"{path}: unexpected drawing shape for {language}: {shape!r}")
    require(drawing_intent.get("source_text") == transcript, f"{path}: DrawingIntent source text differs from ASR transcript")
    return {
        "language": language,
        "shape": shape,
        "transcript": transcript,
        "audio_sha256": capture_metadata["audio_sha256"],
        "native_language_probability": probability,
        "evidence": str(path.resolve()),
    }


def atomic_json_write(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence", type=Path, action="append", required=True, help="case/runtime/speech_execution_evidence.json (three times)")
    parser.add_argument("--model-lock", type=Path, required=True, help="sealed local-ASR lock JSON")
    parser.add_argument("--output", type=Path, required=True, help="evaluation JSON output")
    args = parser.parse_args()
    try:
        require(len(args.evidence) == 3, "exactly three microphone evidence files are required")
        lock = validate_lock(load_json(args.model_lock.resolve(), "model lock"), "model lock")
        model_cache: dict[Path, str] = {}
        cases = [validate_one(path.resolve(), lock, model_cache) for path in args.evidence]
        languages = {case["language"] for case in cases}
        require(languages == set(EXPECTED_CASES), f"evidence languages must be exactly {sorted(EXPECTED_CASES)}")
        require(len({case["audio_sha256"] for case in cases}) == 3, "each language requires a distinct microphone WAV")
        report = {
            "schema_version": 1,
            "status": "PASS",
            "scope": "PHASE7_LOCAL_MULTILINGUAL_MICROPHONE_TO_DRAWING",
            "model_lock": lock,
            "cases": sorted(cases, key=lambda item: item["language"]),
        }
        atomic_json_write(args.output.resolve(), report)
    except EvaluationError as exc:
        print(f"P7_SPEECH_EVALUATION=FAIL: {exc}")
        return 1
    print("P7_SPEECH_EVALUATION=PASS")
    print(f"OUTPUT={args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
