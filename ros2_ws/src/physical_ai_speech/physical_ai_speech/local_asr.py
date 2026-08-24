"""Pinned, local-only Faster-Whisper support for Phase 7.

This module deliberately has no cloud fallback.  A normal runtime invocation
must receive an existing model directory and a resolved lock file; a missing or
changed model is an error rather than a reason to fetch a replacement.
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
from hashlib import sha256
from importlib import metadata
import json
import math
import os
from pathlib import Path
import socket
from typing import Any, Iterator


class LocalAsrError(RuntimeError):
    """The local ASR contract cannot be satisfied safely."""


@dataclass(frozen=True)
class WavFormat:
    """The PCM WAV properties accepted by the microphone evidence contract."""

    sample_rate_hz: int
    channels: int
    sample_width_bytes: int
    frames: int

    @property
    def duration_sec(self) -> float:
        return self.frames / self.sample_rate_hz

    def as_dict(self) -> dict[str, object]:
        return {
            "container": "WAV",
            "sample_rate_hz": self.sample_rate_hz,
            "channels": self.channels,
            "sample_width_bytes": self.sample_width_bytes,
            "frames": self.frames,
            "duration_sec": self.duration_sec,
        }


@dataclass(frozen=True)
class LocalTranscription:
    """Raw local-ASR output before speech-domain normalization."""

    transcript: str
    detected_language: str
    native_language_probability: float
    segments: tuple[dict[str, object], ...]
    package_version: str


def sha256_file(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def inspect_wav(path: Path) -> WavFormat:
    """Read a bounded PCM WAV header without accepting arbitrary containers."""

    import wave

    if not path.is_file():
        raise LocalAsrError(f"AUDIO_FILE_MISSING: {path}")
    try:
        with wave.open(str(path), "rb") as audio:
            result = WavFormat(
                sample_rate_hz=audio.getframerate(),
                channels=audio.getnchannels(),
                sample_width_bytes=audio.getsampwidth(),
                frames=audio.getnframes(),
            )
    except (EOFError, wave.Error, OSError) as exc:
        raise LocalAsrError(f"AUDIO_NOT_PCM_WAV: {path}") from exc
    if (
        result.sample_rate_hz != 16000
        or result.channels != 1
        or result.sample_width_bytes != 2
        or result.frames <= 0
        or result.duration_sec <= 0.0
    ):
        raise LocalAsrError("AUDIO_FORMAT_MUST_BE_WAV_16KHZ_MONO_S16LE")
    return result


def _canonical_manifest_payload(value: dict[str, Any]) -> bytes:
    files = value.get("files")
    if not isinstance(files, list):
        raise LocalAsrError("MODEL_MANIFEST_FILES_MISSING")
    return json.dumps(
        {"schema_version": value.get("schema_version"), "files": files},
        ensure_ascii=True,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _validate_manifest_files(model_path: Path, files: list[Any]) -> None:
    if not files:
        raise LocalAsrError("MODEL_MANIFEST_EMPTY")
    previous_path = ""
    for item in files:
        if not isinstance(item, dict):
            raise LocalAsrError("MODEL_MANIFEST_FILE_RECORD_INVALID")
        relative = item.get("path")
        expected_hash = item.get("sha256")
        expected_size = item.get("size_bytes")
        if (
            not isinstance(relative, str)
            or not relative
            or Path(relative).is_absolute()
            or ".." in Path(relative).parts
            or relative <= previous_path
            or not isinstance(expected_hash, str)
            or len(expected_hash) != 64
            or not isinstance(expected_size, int)
            or expected_size < 0
        ):
            raise LocalAsrError("MODEL_MANIFEST_FILE_RECORD_INVALID")
        candidate = (model_path / relative).resolve()
        try:
            candidate.relative_to(model_path)
        except ValueError as exc:
            raise LocalAsrError("MODEL_MANIFEST_PATH_ESCAPES_MODEL") from exc
        if not candidate.is_file() or candidate.stat().st_size != expected_size:
            raise LocalAsrError(f"MODEL_MANIFEST_FILE_MISMATCH: {relative}")
        if sha256_file(candidate) != expected_hash:
            raise LocalAsrError(f"MODEL_MANIFEST_HASH_MISMATCH: {relative}")
        previous_path = relative
    expected_paths = {str(item["path"]) for item in files}
    observed_paths: set[str] = set()
    for candidate in model_path.rglob("*"):
        relative_path = candidate.relative_to(model_path)
        if relative_path.name == ".phase7_model_manifest.json" or ".cache" in relative_path.parts:
            continue
        if candidate.is_symlink():
            raise LocalAsrError(f"MODEL_CONTAINS_SYMLINK: {relative_path.as_posix()}")
        if candidate.is_file():
            observed_paths.add(relative_path.as_posix())
    if observed_paths != expected_paths:
        raise LocalAsrError("MODEL_MANIFEST_FILE_SET_MISMATCH")


def load_and_verify_model_lock(lock_path: Path, model_path: Path) -> dict[str, Any]:
    """Verify the installed backend and every local model file before ASR."""

    if not lock_path.is_file():
        raise LocalAsrError(f"MODEL_LOCK_MISSING: {lock_path}")
    if not model_path.is_dir():
        raise LocalAsrError(f"LOCAL_MODEL_DIRECTORY_MISSING: {model_path}")
    try:
        lock = json.loads(lock_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise LocalAsrError(f"MODEL_LOCK_INVALID_JSON: {lock_path}") from exc
    if not isinstance(lock, dict) or lock.get("schema_version") != 1:
        raise LocalAsrError("MODEL_LOCK_SCHEMA_UNSUPPORTED")
    backend, model = lock.get("backend"), lock.get("model")
    if not isinstance(backend, dict) or not isinstance(model, dict):
        raise LocalAsrError("MODEL_LOCK_SECTION_MISSING")
    package_version = backend.get("package_version")
    if backend.get("name") != "faster-whisper" or not isinstance(package_version, str) or not package_version:
        raise LocalAsrError("MODEL_LOCK_BACKEND_INVALID")
    try:
        installed_version = metadata.version("faster-whisper")
    except metadata.PackageNotFoundError as exc:
        raise LocalAsrError("FASTER_WHISPER_NOT_INSTALLED") from exc
    if installed_version != package_version:
        raise LocalAsrError(
            f"FASTER_WHISPER_VERSION_MISMATCH: expected {package_version}, got {installed_version}"
        )
    revision = model.get("revision")
    manifest_sha = model.get("manifest_sha256")
    if (
        not isinstance(model.get("repository"), str)
        or not model["repository"]
        or not isinstance(revision, str)
        or len(revision) != 40
        or any(character not in "0123456789abcdef" for character in revision)
        or model.get("directory_name") != model_path.name
        or not isinstance(manifest_sha, str)
        or len(manifest_sha) != 64
        or any(character not in "0123456789abcdef" for character in manifest_sha)
    ):
        raise LocalAsrError("MODEL_LOCK_MODEL_INVALID")
    manifest_path = model_path / ".phase7_model_manifest.json"
    if not manifest_path.is_file():
        raise LocalAsrError("MODEL_MANIFEST_MISSING")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise LocalAsrError("MODEL_MANIFEST_INVALID_JSON") from exc
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 1:
        raise LocalAsrError("MODEL_MANIFEST_SCHEMA_UNSUPPORTED")
    files = manifest.get("files")
    if not isinstance(files, list):
        raise LocalAsrError("MODEL_MANIFEST_FILES_MISSING")
    _validate_manifest_files(model_path.resolve(), files)
    observed_manifest_sha = sha256(_canonical_manifest_payload(manifest)).hexdigest()
    if observed_manifest_sha != manifest_sha:
        raise LocalAsrError("MODEL_MANIFEST_LOCK_MISMATCH")
    return lock


@contextmanager
def _network_disabled() -> Iterator[None]:
    """Block Python-level network creation during the local-only inference call.

    Faster-Whisper receives an existing directory and ``local_files_only=True``
    as a second, library-level guard.  Blocking sockets here makes an accidental
    Python HTTP fallback fail closed rather than silently turn into cloud work.
    """

    original_socket = socket.socket
    original_create_connection = socket.create_connection

    def deny_network(*_args: Any, **_kwargs: Any) -> Any:
        raise LocalAsrError("RUNTIME_NETWORK_DISABLED")

    socket.socket = deny_network  # type: ignore[assignment]
    socket.create_connection = deny_network  # type: ignore[assignment]
    old_environment = {key: os.environ.get(key) for key in ("HF_HUB_OFFLINE", "TRANSFORMERS_OFFLINE")}
    os.environ["HF_HUB_OFFLINE"] = "1"
    os.environ["TRANSFORMERS_OFFLINE"] = "1"
    try:
        yield
    finally:
        socket.socket = original_socket  # type: ignore[assignment]
        socket.create_connection = original_create_connection  # type: ignore[assignment]
        for key, old_value in old_environment.items():
            if old_value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = old_value


def _finite_probability(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise LocalAsrError(f"ASR_{label}_INVALID") from exc
    if not math.isfinite(result) or not 0.0 <= result <= 1.0:
        raise LocalAsrError(f"ASR_{label}_INVALID")
    return result


def transcribe_local_wav(
    audio_path: Path,
    model_path: Path,
    *,
    device: str = "cpu",
    compute_type: str = "int8",
) -> LocalTranscription:
    """Transcribe one WAV with native language detection and no language hint."""

    inspect_wav(audio_path)
    try:
        from faster_whisper import WhisperModel
    except ImportError as exc:
        raise LocalAsrError("FASTER_WHISPER_NOT_INSTALLED") from exc
    try:
        package_version = metadata.version("faster-whisper")
    except metadata.PackageNotFoundError as exc:
        raise LocalAsrError("FASTER_WHISPER_NOT_INSTALLED") from exc
    # Do not add a language parameter.  The model must detect the source
    # language from the microphone WAV, not from the expected acceptance case.
    with _network_disabled():
        model = WhisperModel(
            str(model_path),
            device=device,
            compute_type=compute_type,
            local_files_only=True,
        )
        segment_stream, info = model.transcribe(
            str(audio_path),
            task="transcribe",
            language=None,
            vad_filter=False,
            condition_on_previous_text=False,
        )
        raw_segments: list[dict[str, object]] = []
        transcript_parts: list[str] = []
        for segment in segment_stream:
            # Preserve the backend's segment text exactly.  The single raw
            # transcript is reconstructed below as the concatenated segment
            # stream, so evidence can prove it was not manually rewritten.
            text = str(segment.text)
            if not text.strip():
                continue
            avg_logprob = float(segment.avg_logprob)
            no_speech_prob = _finite_probability(segment.no_speech_prob, "NO_SPEECH_PROBABILITY")
            if not math.isfinite(avg_logprob):
                raise LocalAsrError("ASR_AVG_LOGPROB_INVALID")
            raw_segments.append(
                {
                    "text": text,
                    "avg_logprob": avg_logprob,
                    "no_speech_prob": no_speech_prob,
                }
            )
            transcript_parts.append(text)
    transcript = "".join(transcript_parts).strip()
    language = str(info.language).strip().lower()
    probability = _finite_probability(info.language_probability, "LANGUAGE_PROBABILITY")
    if not transcript:
        raise LocalAsrError("EMPTY_ASR_TRANSCRIPT")
    if language not in {"zh", "en", "fr"}:
        raise LocalAsrError(f"UNSUPPORTED_DETECTED_LANGUAGE: {language or '<empty>'}")
    if not raw_segments:
        raise LocalAsrError("EMPTY_ASR_SEGMENTS")
    return LocalTranscription(
        transcript=transcript,
        detected_language=language,
        native_language_probability=probability,
        segments=tuple(raw_segments),
        package_version=package_version,
    )
