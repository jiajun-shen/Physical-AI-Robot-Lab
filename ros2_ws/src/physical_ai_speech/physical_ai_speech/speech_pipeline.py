"""One local microphone-to-Phase-6 speech pipeline for Phase 7.

There is intentionally no ``--transcript``, ``--language``, or ``--confidence``
flag.  Those values must come from the recorded WAV and local Faster-Whisper,
then be preserved as evidence through the Phase-6 drawing parser.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import tempfile
import time
from typing import Any, Sequence

from physical_ai_speech.local_asr import (
    LocalAsrError,
    inspect_wav,
    load_and_verify_model_lock,
    sha256_file,
    transcribe_local_wav,
)
from physical_ai_speech.normalizer import SpeechCommand, normalize
from physical_ai_speech.speech_to_drawing import route_speech_command_to_drawing


class SpeechPipelineError(RuntimeError):
    """Input provenance or a required local pipeline stage was invalid."""


_ALLOWED_CAPTURE_METHODS = frozenset({"pulse_parec", "windows_directshow"})


def _load_mapping(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file():
        raise SpeechPipelineError(f"{label}_MISSING: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise SpeechPipelineError(f"{label}_INVALID_JSON: {path}") from exc
    if not isinstance(value, dict):
        raise SpeechPipelineError(f"{label}_NOT_OBJECT: {path}")
    return value


def _positive_int(value: object, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise SpeechPipelineError(f"{label}_INVALID")
    return value


def _finite_positive(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SpeechPipelineError(f"{label}_INVALID")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise SpeechPipelineError(f"{label}_INVALID")
    return result


def validate_microphone_capture(audio_path: Path, metadata_path: Path) -> dict[str, Any]:
    """Verify microphone provenance and bind metadata to the exact WAV bytes."""

    audio_path = audio_path.resolve()
    metadata_path = metadata_path.resolve()
    wav = inspect_wav(audio_path)
    metadata = _load_mapping(metadata_path, "CAPTURE_METADATA")
    if metadata.get("schema_version") != 1:
        raise SpeechPipelineError("CAPTURE_METADATA_SCHEMA_UNSUPPORTED")
    method = metadata.get("capture_method")
    if method not in _ALLOWED_CAPTURE_METHODS:
        raise SpeechPipelineError("CAPTURE_METHOD_NOT_MICROPHONE")
    if metadata.get("input_kind") != "microphone":
        raise SpeechPipelineError("CAPTURE_INPUT_NOT_MICROPHONE")
    if not isinstance(metadata.get("source"), str) or not metadata["source"].strip():
        raise SpeechPipelineError("CAPTURE_SOURCE_MISSING")
    started = _positive_int(metadata.get("started_at_unix_ns"), "CAPTURE_START_TIME")
    ended = _positive_int(metadata.get("ended_at_unix_ns"), "CAPTURE_END_TIME")
    if ended < started:
        raise SpeechPipelineError("CAPTURE_TIME_ORDER_INVALID")
    expected_hash = metadata.get("audio_sha256")
    expected_size = metadata.get("audio_size_bytes")
    observed_hash = sha256_file(audio_path)
    observed_size = audio_path.stat().st_size
    if not isinstance(expected_hash, str) or len(expected_hash) != 64 or expected_hash != observed_hash:
        raise SpeechPipelineError("CAPTURE_AUDIO_HASH_MISMATCH")
    if expected_size != observed_size:
        raise SpeechPipelineError("CAPTURE_AUDIO_SIZE_MISMATCH")
    declared_format = metadata.get("audio_format")
    if not isinstance(declared_format, dict):
        raise SpeechPipelineError("CAPTURE_AUDIO_FORMAT_MISSING")
    observed_format = wav.as_dict()
    for field, expected in observed_format.items():
        declared = declared_format.get(field)
        if field == "duration_sec":
            if not isinstance(declared, (int, float)) or not math.isclose(
                float(declared), float(expected), rel_tol=0.0, abs_tol=1e-9
            ):
                raise SpeechPipelineError("CAPTURE_AUDIO_FORMAT_MISMATCH")
        elif declared != expected:
            raise SpeechPipelineError("CAPTURE_AUDIO_FORMAT_MISMATCH")
    return {
        "metadata_path": str(metadata_path),
        "method": method,
        "input_kind": "microphone",
        "acquired_at_unix_ns": started,
        "ended_at_unix_ns": ended,
        "audio": {
            "path": str(audio_path),
            "sha256": observed_hash,
            "size_bytes": observed_size,
            "format": observed_format,
        },
    }


def _reject_repository_output(output_root: Path) -> None:
    repository_root = Path(__file__).resolve().parents[4]
    try:
        output_root.resolve().relative_to(repository_root.resolve())
    except ValueError:
        return
    raise SpeechPipelineError("OUTPUT_ROOT_MUST_BE_OUTSIDE_REPOSITORY")


def _write_json_once(path: Path, value: dict[str, Any]) -> None:
    if path.exists():
        raise SpeechPipelineError(f"EVIDENCE_ALREADY_EXISTS: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
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


def execute_pipeline(
    *,
    audio_path: Path,
    capture_metadata_path: Path,
    model_path: Path,
    model_lock_path: Path,
    output_root: Path,
    expected_language: str | None = None,
    device: str = "cpu",
    compute_type: str = "int8",
) -> Path:
    """Execute the one shared multilingual pipeline and seal success evidence."""

    if expected_language is not None and expected_language not in {"zh", "en", "fr"}:
        raise SpeechPipelineError("EXPECTED_LANGUAGE_UNSUPPORTED")
    if not isinstance(device, str) or not device.strip() or not isinstance(compute_type, str) or not compute_type.strip():
        raise SpeechPipelineError("ASR_RUNTIME_CONFIGURATION_INVALID")
    output_root = output_root.resolve()
    _reject_repository_output(output_root)
    capture = validate_microphone_capture(audio_path, capture_metadata_path)
    model_path = model_path.resolve()
    model_lock_path = model_lock_path.resolve()
    model_lock = load_and_verify_model_lock(model_lock_path, model_path)
    result = transcribe_local_wav(
        audio_path.resolve(), model_path, device=device, compute_type=compute_type
    )
    if expected_language is not None and result.detected_language != expected_language:
        raise SpeechPipelineError(
            f"DETECTED_LANGUAGE_MISMATCH: expected {expected_language}, got {result.detected_language}"
        )
    speech_timestamp_ns = time.time_ns()
    command: SpeechCommand = normalize(
        result.transcript,
        result.detected_language,
        result.native_language_probability,
        speech_timestamp_ns,
    )
    if command.intent != "DRAW":
        raise SpeechPipelineError("PHASE7_CASE_MUST_ROUTE_TO_DRAW")
    dispatch = route_speech_command_to_drawing(command)
    if dispatch.get("status") != "ACCEPTED":
        raise SpeechPipelineError("DRAWING_DISPATCH_NOT_ACCEPTED")
    drawing_intent = dispatch.get("drawing_intent")
    if not isinstance(drawing_intent, dict) or drawing_intent.get("source_text") != result.transcript:
        raise SpeechPipelineError("DRAWING_DISPATCH_SOURCE_MISMATCH")
    evidence = {
        "schema_version": 1,
        "scope": "PHASE7_LOCAL_MULTILINGUAL_MICROPHONE_TO_DRAWING",
        "status": {"P7_SPEECH_RUNTIME": "PASS"},
        "created_at_unix_ns": time.time_ns(),
        "capture": capture,
        "asr": {
            "backend": {
                "name": "faster-whisper",
                "package_version": result.package_version,
                "model_path": str(model_path),
                "model_path_is_local": True,
                "device": device,
                "compute_type": compute_type,
                "runtime_network_enabled": False,
                "automatic_download_permitted": False,
            },
            "model": model_lock["model"],
            "task": "transcribe",
            "language_hint": None,
            "raw_transcript": result.transcript,
            "detected_language": result.detected_language,
            "native_language_probability": result.native_language_probability,
            "segments": list(result.segments),
        },
        "speech_command": {
            "transcript": command.transcript,
            "detected_language": command.detected_language,
            "confidence": command.confidence,
            "confidence_provenance": "whisper_language_probability",
            "timestamp_ns": command.timestamp_ns,
            "intent": command.intent,
        },
        "downstream_dispatch": {
            "interface": dispatch["interface"],
            "input_transcript": command.transcript,
            "accepted": True,
            "drawing_intent": drawing_intent,
        },
        "model_lock": model_lock,
    }
    evidence_path = output_root / "speech_execution_evidence.json"
    _write_json_once(evidence_path, evidence)
    return evidence_path


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run one local microphone-to-drawing speech command.")
    parser.add_argument("--audio", type=Path, required=True, help="16 kHz mono signed-16-bit microphone WAV")
    parser.add_argument(
        "--capture-metadata", type=Path, required=True, help="capture_metadata.json bound to --audio"
    )
    parser.add_argument("--model-path", type=Path, required=True, help="existing local Faster-Whisper directory")
    parser.add_argument("--model-lock", type=Path, required=True, help="resolved immutable local model lock JSON")
    parser.add_argument("--output-root", type=Path, required=True, help="new evidence directory outside the repository")
    parser.add_argument(
        "--expected-language",
        choices=("zh", "en", "fr"),
        help="post-ASR assertion only; it is never supplied to Whisper",
    )
    parser.add_argument("--device", default="cpu", help="Faster-Whisper device, default: cpu")
    parser.add_argument("--compute-type", default="int8", help="Faster-Whisper compute type, default: int8")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    try:
        evidence_path = execute_pipeline(
            audio_path=arguments.audio,
            capture_metadata_path=arguments.capture_metadata,
            model_path=arguments.model_path,
            model_lock_path=arguments.model_lock,
            output_root=arguments.output_root,
            expected_language=arguments.expected_language,
            device=arguments.device,
            compute_type=arguments.compute_type,
        )
    except (LocalAsrError, SpeechPipelineError, ValueError) as exc:
        print(f"P7_SPEECH_RUNTIME=FAIL: {exc}")
        return 2
    print("P7_SPEECH_RUNTIME=PASS")
    print(f"EVIDENCE={evidence_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
