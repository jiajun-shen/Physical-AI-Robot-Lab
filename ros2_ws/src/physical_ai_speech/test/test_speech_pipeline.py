from __future__ import annotations

from hashlib import sha256
import json
from pathlib import Path
import wave

import pytest

from physical_ai_speech.local_asr import LocalTranscription
from physical_ai_speech import speech_pipeline


def _write_wav_and_metadata(tmp_path: Path) -> tuple[Path, Path]:
    audio = tmp_path / "microphone.wav"
    with wave.open(str(audio), "wb") as stream:
        stream.setnchannels(1)
        stream.setsampwidth(2)
        stream.setframerate(16000)
        stream.writeframes(b"\x00\x00" * 160)
    audio_bytes = audio.read_bytes()
    metadata = {
        "schema_version": 1,
        "capture_method": "windows_directshow",
        "input_kind": "microphone",
        "source": "test microphone",
        "started_at_unix_ns": 100,
        "ended_at_unix_ns": 200,
        "audio_sha256": sha256(audio_bytes).hexdigest(),
        "audio_size_bytes": len(audio_bytes),
        "audio_format": {
            "container": "WAV",
            "sample_rate_hz": 16000,
            "channels": 1,
            "sample_width_bytes": 2,
            "frames": 160,
            "duration_sec": 0.01,
        },
    }
    metadata_path = tmp_path / "capture_metadata.json"
    metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
    return audio, metadata_path


def test_capture_metadata_is_bound_to_exact_microphone_wav(tmp_path: Path):
    audio, metadata = _write_wav_and_metadata(tmp_path)

    capture = speech_pipeline.validate_microphone_capture(audio, metadata)

    assert capture["method"] == "windows_directshow"
    assert capture["audio"]["format"]["frames"] == 160


def test_capture_metadata_hash_tampering_is_rejected(tmp_path: Path):
    audio, metadata = _write_wav_and_metadata(tmp_path)
    record = json.loads(metadata.read_text(encoding="utf-8"))
    record["audio_sha256"] = "0" * 64
    metadata.write_text(json.dumps(record), encoding="utf-8")

    with pytest.raises(speech_pipeline.SpeechPipelineError, match="CAPTURE_AUDIO_HASH_MISMATCH"):
        speech_pipeline.validate_microphone_capture(audio, metadata)


def test_runtime_cli_cannot_accept_typed_asr_values():
    options = {
        option
        for action in speech_pipeline.build_argument_parser()._actions
        for option in action.option_strings
    }

    assert {"--transcript", "--language", "--detected-language", "--confidence"}.isdisjoint(options)


def test_pipeline_evidence_contains_only_asr_derived_command(monkeypatch, tmp_path: Path):
    audio, metadata = _write_wav_and_metadata(tmp_path)
    model = tmp_path / "faster-whisper-small"
    model.mkdir()
    lock = tmp_path / "model.lock.json"
    lock.write_text("{}", encoding="utf-8")
    expected_lock = {
        "schema_version": 1,
        "backend": {"name": "faster-whisper", "package_version": "1.1.1"},
        "model": {
            "repository": "Systran/faster-whisper-small",
            "revision": "a" * 40,
            "directory_name": model.name,
            "manifest_sha256": "b" * 64,
        },
    }
    monkeypatch.setattr(speech_pipeline, "load_and_verify_model_lock", lambda *_args: expected_lock)
    monkeypatch.setattr(
        speech_pipeline,
        "transcribe_local_wav",
        lambda *_args, **_kwargs: LocalTranscription(
            transcript="Draw a square.",
            detected_language="en",
            native_language_probability=0.93,
            segments=(
                {"text": "Draw a square.", "avg_logprob": -0.1, "no_speech_prob": 0.01},
            ),
            package_version="1.1.1",
        ),
    )

    evidence_path = speech_pipeline.execute_pipeline(
        audio_path=audio,
        capture_metadata_path=metadata,
        model_path=model,
        model_lock_path=lock,
        output_root=tmp_path / "evidence",
        expected_language="en",
    )
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))

    assert evidence["status"]["P7_SPEECH_RUNTIME"] == "PASS"
    assert evidence["asr"]["language_hint"] is None
    assert evidence["asr"]["raw_transcript"] == "Draw a square."
    assert evidence["speech_command"]["transcript"] == evidence["asr"]["raw_transcript"]
    assert evidence["speech_command"]["confidence"] == evidence["asr"]["native_language_probability"]
    assert evidence["downstream_dispatch"]["drawing_intent"]["shape"] == "square"
    assert evidence["downstream_dispatch"]["drawing_intent"]["source_text"] == "Draw a square."
