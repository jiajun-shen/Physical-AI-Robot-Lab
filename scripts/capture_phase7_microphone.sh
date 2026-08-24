#!/usr/bin/env bash
# Capture one bounded, consented Phase-7 microphone WAV through the WSLg
# PulseAudio server.  This script deliberately has no text/transcript input:
# a typed command cannot be turned into Phase-7 microphone evidence.
set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
readonly REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd -P)"

die() { echo "ERROR: $*" >&2; exit 1; }
usage() {
  cat <<'EOF'
Usage:
  capture_phase7_microphone.sh --output-dir DIRECTORY --duration-sec SECONDS \
    --i-consent-to-record [--source PULSE_SOURCE]

Records exactly one mono 16 kHz signed-16-bit WAV and writes:
  DIRECTORY/microphone.wav
  DIRECTORY/capture_metadata.json

The output directory must be new or empty and must not be inside the source
repository.  `parec` and `pactl` are provided by pulseaudio-utils.  This is a
microphone capture operation; invoke it only after the speaker has agreed to
be recorded and is ready to speak.
EOF
}

output_dir=""
duration_sec=""
requested_source=""
consent=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir)
      [[ $# -ge 2 ]] || die "--output-dir requires a value"
      output_dir="$2"
      shift 2
      ;;
    --duration-sec)
      [[ $# -ge 2 ]] || die "--duration-sec requires a value"
      duration_sec="$2"
      shift 2
      ;;
    --source)
      [[ $# -ge 2 ]] || die "--source requires a value"
      requested_source="$2"
      shift 2
      ;;
    --i-consent-to-record)
      consent=true
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

for command in awk date find flock mv pactl parec python3 realpath rm sha256sum sleep wc; do
  command -v "${command}" >/dev/null 2>&1 || die "required command is unavailable: ${command}"
done

[[ "${consent}" == true ]] || die "microphone capture requires --i-consent-to-record"
[[ -n "${output_dir}" && -n "${duration_sec}" ]] || { usage >&2; exit 2; }
python3 - "${duration_sec}" <<'PY'
import math
import sys

try:
    value = float(sys.argv[1])
except ValueError as exc:
    raise SystemExit("duration must be numeric") from exc
if not math.isfinite(value) or value < 1.0 or value > 60.0:
    raise SystemExit("duration must be between 1 and 60 seconds")
PY

output_dir="$(realpath -m "${output_dir}")"
repository_real="$(realpath -e "${REPOSITORY_ROOT}")"
case "${output_dir}/" in
  "${repository_real}/"*) die "capture output directory must not be inside repository" ;;
esac
[[ "${output_dir}" != / ]] || die "capture output directory must not be /"
[[ ! -e "${output_dir}" || -d "${output_dir}" ]] || die "capture output path is not a directory"
if [[ -d "${output_dir}" ]] && [[ -n "$(find "${output_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  die "capture output directory must be new or empty"
fi
mkdir -p "${output_dir}"
readonly OUTPUT_DIR="$(realpath -e "${output_dir}")"
exec {lock_fd}>"${OUTPUT_DIR}/.phase7_capture.lock"
flock -n "${lock_fd}" || die "capture output directory is already in use"

readonly pulse_server="${PULSE_SERVER:-}"
[[ -n "${pulse_server}" ]] || die "PULSE_SERVER is not set; WSLg PulseAudio is unavailable"
if [[ "${pulse_server}" == unix:* ]]; then
  [[ -S "${pulse_server#unix:}" ]] || die "PULSE_SERVER socket is unavailable: ${pulse_server#unix:}"
fi

if [[ -n "${requested_source}" ]]; then
  source_name="${requested_source}"
else
  source_name="$(pactl get-default-source 2>/dev/null || true)"
  if [[ -z "${source_name}" ]]; then
    source_name="$(pactl list short sources 2>/dev/null | awk '$2 !~ /\.monitor$/ { print $2; exit }')"
  fi
fi
[[ -n "${source_name}" ]] || die "could not identify a PulseAudio input source; pass --source"
[[ "${source_name}" != *.monitor ]] || die "refusing monitor source; a physical microphone source is required"
pactl list short sources 2>/dev/null | awk -v source_name="${source_name}" '$2 == source_name { found=1 } END { exit(found ? 0 : 1) }' ||
  die "PulseAudio source does not exist: ${source_name}"

readonly AUDIO_PATH="${OUTPUT_DIR}/microphone.wav"
readonly AUDIO_TMP="${OUTPUT_DIR}/.microphone.wav.partial"
readonly METADATA_PATH="${OUTPUT_DIR}/capture_metadata.json"
readonly METADATA_TMP="${OUTPUT_DIR}/.capture_metadata.json.partial"
recorder_pid=""
cleanup() {
  local status="$?"
  trap - EXIT INT TERM
  if [[ -n "${recorder_pid}" ]] && kill -0 "${recorder_pid}" 2>/dev/null; then
    kill -TERM "${recorder_pid}" 2>/dev/null || true
    wait "${recorder_pid}" 2>/dev/null || true
  fi
  rm -f -- "${AUDIO_TMP}" "${METADATA_TMP}"
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

started_at_unix_ns="$(date +%s%N)"
parec --file-format=wav --format=s16le --rate=16000 --channels=1 --device="${source_name}" >"${AUDIO_TMP}" &
recorder_pid="$!"
sleep "${duration_sec}"
if kill -0 "${recorder_pid}" 2>/dev/null; then
  kill -INT "${recorder_pid}" 2>/dev/null || true
fi
recorder_status=0
if wait "${recorder_pid}"; then
  recorder_status=0
else
  recorder_status="$?"
fi
recorder_pid=""
ended_at_unix_ns="$(date +%s%N)"
case "${recorder_status}" in
  0|130) ;;
  *) die "parec exited unexpectedly with status ${recorder_status}" ;;
esac

python3 - "${AUDIO_TMP}" <<'PY'
import sys
import wave

path = sys.argv[1]
try:
    with wave.open(path, "rb") as audio:
        if audio.getnchannels() != 1:
            raise ValueError("expected one channel")
        if audio.getframerate() != 16_000:
            raise ValueError("expected 16 kHz")
        if audio.getsampwidth() != 2:
            raise ValueError("expected signed 16-bit samples")
        if audio.getnframes() <= 0:
            raise ValueError("WAV contains no frames")
except (OSError, wave.Error, ValueError) as exc:
    raise SystemExit(f"invalid microphone WAV: {exc}") from exc
PY
mv -- "${AUDIO_TMP}" "${AUDIO_PATH}"
audio_sha256="$(sha256sum "${AUDIO_PATH}" | awk '{print $1}')"
audio_size_bytes="$(wc -c <"${AUDIO_PATH}")"

python3 - "${METADATA_TMP}" "${METADATA_PATH}" "${source_name}" "${pulse_server}" \
  "${duration_sec}" "${started_at_unix_ns}" "${ended_at_unix_ns}" "${audio_sha256}" \
  "${audio_size_bytes}" <<'PY'
import json
import os
import sys
import wave

temporary, destination, source, pulse_server, requested_duration, started, ended, sha256, size = sys.argv[1:]
with wave.open(os.path.join(os.path.dirname(destination), "microphone.wav"), "rb") as audio:
    frames = audio.getnframes()
    rate = audio.getframerate()
    metadata = {
        "schema_version": 1,
        "capture_method": "pulse_parec",
        "input_kind": "microphone",
        "source": source,
        "pulse_server": pulse_server,
        "audio_filename": "microphone.wav",
        "started_at_unix_ns": int(started),
        "ended_at_unix_ns": int(ended),
        "duration_requested_sec": float(requested_duration),
        "audio_sha256": sha256,
        "audio_size_bytes": int(size),
        "audio_format": {
            "container": "WAV",
            "sample_rate_hz": rate,
            "channels": audio.getnchannels(),
            "sample_width_bytes": audio.getsampwidth(),
            "frames": frames,
            "duration_sec": frames / rate,
        },
    }
with open(temporary, "w", encoding="utf-8", newline="\n") as stream:
    json.dump(metadata, stream, ensure_ascii=False, indent=2, sort_keys=True)
    stream.write("\n")
os.replace(temporary, destination)
PY

echo "P7_MICROPHONE_CAPTURE=PASS"
echo "AUDIO=${AUDIO_PATH}"
echo "METADATA=${METADATA_PATH}"
echo "SHA256=${audio_sha256}"
trap - EXIT INT TERM
