from __future__ import annotations

from hashlib import sha256
import json
from pathlib import Path

import pytest

from physical_ai_speech import local_asr


def _write_sealed_model(tmp_path: Path) -> tuple[Path, Path]:
    model = tmp_path / "faster-whisper-small"
    model.mkdir()
    payload = b"local-test-model"
    (model / "model.bin").write_bytes(payload)
    manifest = {
        "schema_version": 1,
        "files": [
            {
                "path": "model.bin",
                "sha256": sha256(payload).hexdigest(),
                "size_bytes": len(payload),
            }
        ],
    }
    (model / ".phase7_model_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    manifest_hash = sha256(
        json.dumps(manifest, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    lock = {
        "schema_version": 1,
        "backend": {"name": "faster-whisper", "package_version": "1.1.1"},
        "model": {
            "repository": "example/local-model",
            "revision": "a" * 40,
            "directory_name": model.name,
            "manifest_sha256": manifest_hash,
        },
    }
    lock_path = tmp_path / "model.lock.json"
    lock_path.write_text(json.dumps(lock), encoding="utf-8")
    return model, lock_path


def test_model_lock_binds_backend_and_every_model_file(monkeypatch, tmp_path: Path):
    model, lock_path = _write_sealed_model(tmp_path)
    monkeypatch.setattr(local_asr.metadata, "version", lambda _name: "1.1.1")

    lock = local_asr.load_and_verify_model_lock(lock_path, model)

    assert lock["model"]["directory_name"] == "faster-whisper-small"


def test_model_lock_fails_closed_when_a_model_file_changes(monkeypatch, tmp_path: Path):
    model, lock_path = _write_sealed_model(tmp_path)
    monkeypatch.setattr(local_asr.metadata, "version", lambda _name: "1.1.1")
    (model / "model.bin").write_bytes(b"changed")

    with pytest.raises(local_asr.LocalAsrError, match="MODEL_MANIFEST_FILE_MISMATCH|MODEL_MANIFEST_HASH_MISMATCH"):
        local_asr.load_and_verify_model_lock(lock_path, model)


def test_wav_contract_rejects_non_microphone_format(tmp_path: Path):
    path = tmp_path / "not-a-wav.wav"
    path.write_bytes(b"not wav")

    with pytest.raises(local_asr.LocalAsrError, match="AUDIO_NOT_PCM_WAV"):
        local_asr.inspect_wav(path)
