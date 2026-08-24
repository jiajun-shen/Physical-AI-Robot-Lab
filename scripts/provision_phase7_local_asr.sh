#!/usr/bin/env bash
# Explicit provisioning for the Phase-7 local ASR runtime.  Normal Phase-7
# execution never calls this script and must receive this already-downloaded
# model directory plus the emitted immutable lock file.
set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
readonly REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd -P)"
readonly DEFAULT_FASTER_WHISPER_VERSION="1.1.1"
readonly DEFAULT_HUGGINGFACE_HUB_VERSION="0.27.1"
readonly DEFAULT_MODEL_REPOSITORY="Systran/faster-whisper-small"
readonly DEFAULT_MODEL_REVISION="536b0662742c02347bc0e980a01041f333bce120"

die() { echo "ERROR: $*" >&2; exit 1; }
usage() {
  cat <<'EOF'
Usage:
  provision_phase7_local_asr.sh --model-dir DIRECTORY --lock-output FILE \
    --allow-network-download [options]

Options:
  --python PATH              Python interpreter (default: python3)
  --python-site-dir DIR      Isolated pip --target directory
                             (default: /var/tmp/physical-ai-p7-site)
  --package-version VERSION  faster-whisper pin (default: 1.1.1)
  --model-repository REPO    Hugging Face repository (default: Systran/faster-whisper-small)
  --model-revision SHA       Immutable 40-hex model revision (default:
                             536b0662742c02347bc0e980a01041f333bce120)
  --seal-existing-local-model
                             Write a manifest/lock for an already-downloaded
                             local model, without network access or pip.

This command downloads packages/model files only after the explicit
--allow-network-download acknowledgement.  It refuses a nonempty model
directory and emits both MODEL/.phase7_model_manifest.json and LOCK.  The
runtime must use the resulting existing local model directory; it must not
download a model or contact a cloud/API service.
EOF
}

for command in find mkdir python3 realpath; do
  command -v "${command}" >/dev/null 2>&1 || die "required command is unavailable: ${command}"
done

model_dir=""
lock_output=""
python_executable="python3"
python_site_dir="/var/tmp/physical-ai-p7-site"
package_version="${DEFAULT_FASTER_WHISPER_VERSION}"
model_repository="${DEFAULT_MODEL_REPOSITORY}"
model_revision="${DEFAULT_MODEL_REVISION}"
allow_network_download=false
seal_existing_local_model=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --model-dir)
      [[ $# -ge 2 ]] || die "--model-dir requires a value"
      model_dir="$2"
      shift 2
      ;;
    --lock-output)
      [[ $# -ge 2 ]] || die "--lock-output requires a value"
      lock_output="$2"
      shift 2
      ;;
    --python)
      [[ $# -ge 2 ]] || die "--python requires a value"
      python_executable="$2"
      shift 2
      ;;
    --python-site-dir)
      [[ $# -ge 2 ]] || die "--python-site-dir requires a value"
      python_site_dir="$2"
      shift 2
      ;;
    --package-version)
      [[ $# -ge 2 ]] || die "--package-version requires a value"
      package_version="$2"
      shift 2
      ;;
    --model-repository)
      [[ $# -ge 2 ]] || die "--model-repository requires a value"
      model_repository="$2"
      shift 2
      ;;
    --model-revision)
      [[ $# -ge 2 ]] || die "--model-revision requires a value"
      model_revision="$2"
      shift 2
      ;;
    --allow-network-download)
      allow_network_download=true
      shift
      ;;
    --seal-existing-local-model)
      seal_existing_local_model=true
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      die "unknown argument: $1"
      ;;
  esac
done

if [[ "${seal_existing_local_model}" == true ]]; then
  [[ "${allow_network_download}" == false ]] || die "--seal-existing-local-model cannot download from the network"
else
  [[ "${allow_network_download}" == true ]] || die "provisioning requires --allow-network-download"
fi
[[ -n "${model_dir}" && -n "${lock_output}" ]] || { usage >&2; exit 2; }
[[ "${model_repository}" =~ ^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$ ]] || die "model repository must be owner/name"
[[ "${package_version}" =~ ^[0-9][0-9A-Za-z._+-]*$ ]] || die "invalid package version"
if [[ -n "${model_revision}" ]]; then
  [[ "${model_revision}" =~ ^[0-9a-fA-F]{40}$ ]] || die "model revision must be a 40-hex commit SHA"
  model_revision="${model_revision,,}"
fi

model_dir="$(realpath -m "${model_dir}")"
lock_output="$(realpath -m "${lock_output}")"
python_site_dir="$(realpath -m "${python_site_dir}")"
python_executable="$(command -v "${python_executable}")" || die "Python interpreter is unavailable"
repository_real="$(realpath -e "${REPOSITORY_ROOT}")"
for path in "${model_dir}" "${python_site_dir}"; do
  case "${path}/" in
    "${repository_real}/"*) die "provisioning directories must not be inside repository: ${path}" ;;
  esac
done
[[ "${model_dir}" != / && "${python_site_dir}" != / ]] || die "provisioning directory must not be /"
[[ "${model_dir}" != "${python_site_dir}" ]] || die "model directory and python-site directory must differ"
[[ "${lock_output}" != "${model_dir}"/* ]] || die "lock output must not be inside model directory"
[[ ! -e "${model_dir}" || -d "${model_dir}" ]] || die "model path is not a directory"
if [[ "${seal_existing_local_model}" == true ]]; then
  [[ -d "${model_dir}" && -n "$(find "${model_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]] ||
    die "--seal-existing-local-model requires a nonempty existing model directory"
else
  if [[ -d "${model_dir}" ]] && [[ -n "$(find "${model_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
    die "model directory must be new or empty"
  fi
fi
[[ ! -e "${lock_output}" ]] || die "lock output already exists: ${lock_output}"
mkdir -p "${model_dir}" "$(dirname "${lock_output}")"

[[ ! -e "${python_site_dir}" || -d "${python_site_dir}" ]] || die "python-site path is not a directory"
if [[ "${seal_existing_local_model}" == false ]]; then
  if [[ -d "${python_site_dir}" ]] && [[ -n "$(find "${python_site_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
    die "python-site directory must be new or empty"
  fi
  mkdir -p "${python_site_dir}"
  "${python_executable}" -m pip --version >/dev/null 2>&1 || die "selected Python has no pip: ${python_executable}"
  echo "Installing pinned local-ASR dependencies into ${python_site_dir}" >&2
  PIP_DISABLE_PIP_VERSION_CHECK=1 "${python_executable}" -m pip install --target "${python_site_dir}" --no-input --disable-pip-version-check \
    "faster-whisper==${package_version}" "huggingface_hub==${DEFAULT_HUGGINGFACE_HUB_VERSION}"
  echo "Downloading immutable local model snapshot (explicit provisioning only)" >&2
else
  [[ -d "${python_site_dir}" ]] || die "existing Python site directory is missing: ${python_site_dir}"
  echo "Sealing existing local model without network access" >&2
fi
PYTHONPATH="${python_site_dir}${PYTHONPATH:+:${PYTHONPATH}}" HF_HUB_DISABLE_TELEMETRY=1 "${python_executable}" - "${model_dir}" "${lock_output}" \
  "${model_repository}" "${model_revision}" "${package_version}" "${seal_existing_local_model}" <<'PY'
from __future__ import annotations

from hashlib import sha256
from importlib.metadata import version
import json
import os
from pathlib import Path
import sys
import tempfile

model_dir = Path(sys.argv[1]).resolve()
lock_path = Path(sys.argv[2]).resolve()
repository = sys.argv[3]
requested_revision = sys.argv[4]
requested_package_version = sys.argv[5]
seal_existing = sys.argv[6] == "true"

installed_package_version = version("faster-whisper")
if installed_package_version != requested_package_version:
    raise SystemExit(
        f"installed faster-whisper {installed_package_version!r} does not match requested "
        f"{requested_package_version!r}"
    )

if seal_existing:
    resolved_revision = requested_revision
else:
    from huggingface_hub import HfApi, snapshot_download

    api = HfApi()
    model_info = api.model_info(repo_id=repository, revision=requested_revision)
    resolved_revision = str(model_info.sha).lower()
    if resolved_revision != requested_revision:
        raise SystemExit(
            f"requested revision {requested_revision} resolved to unexpected revision {resolved_revision}"
        )
    snapshot_download(
        repo_id=repository,
        revision=resolved_revision,
        local_dir=str(model_dir),
        local_dir_use_symlinks=False,
    )
if len(resolved_revision) != 40 or any(character not in "0123456789abcdef" for character in resolved_revision):
    raise SystemExit(f"model revision is not an immutable 40-hex revision: {resolved_revision!r}")

manifest_name = ".phase7_model_manifest.json"
files: list[dict[str, object]] = []
for candidate in model_dir.rglob("*"):
    relative = candidate.relative_to(model_dir)
    if relative.name == manifest_name or ".cache" in relative.parts:
        continue
    if candidate.is_symlink():
        raise SystemExit(f"local model contains a symlink, which is not accepted for sealed evidence: {relative}")
    if not candidate.is_file():
        continue
    digest = sha256()
    with candidate.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    files.append(
        {
            "path": relative.as_posix(),
            "sha256": digest.hexdigest(),
            "size_bytes": candidate.stat().st_size,
        }
    )
files.sort(key=lambda item: str(item["path"]))
if not files:
    raise SystemExit("model snapshot contained no usable files")
manifest = {"schema_version": 1, "files": files}
canonical_manifest = json.dumps(manifest, ensure_ascii=True, sort_keys=True, separators=(",", ":")).encode("utf-8")
manifest_sha256 = sha256(canonical_manifest).hexdigest()

def atomic_json_write(path: Path, value: object) -> None:
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

atomic_json_write(model_dir / manifest_name, manifest)
lock = {
    "schema_version": 1,
    "backend": {"name": "faster-whisper", "package_version": installed_package_version},
    "model": {
        "repository": repository,
        "revision": resolved_revision,
        "directory_name": model_dir.name,
        "manifest_sha256": manifest_sha256,
    },
}
atomic_json_write(lock_path, lock)
print(json.dumps({"model_directory": str(model_dir), "lock": str(lock_path), **lock}, ensure_ascii=False, sort_keys=True))
PY

echo "P7_LOCAL_ASR_PROVISION=PASS"
echo "MODEL_DIR=${model_dir}"
echo "LOCK=${lock_output}"
echo "PYTHON=${python_executable}"
echo "PYTHON_SITE_DIR=${python_site_dir}"
echo "RUNTIME_POLICY=EXISTING_LOCAL_MODEL_ONLY"
