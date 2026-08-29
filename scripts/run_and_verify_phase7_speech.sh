#!/usr/bin/env bash
# Build/test the Phase-7 speech packages and run exactly three sealed,
# microphone-derived local-ASR cases.  This intentionally invokes only the
# Phase-6 drawing parser interface; it does not relaunch Gazebo or redo the
# already-accepted Phase-6 robot execution episodes.
set -eo pipefail

if [[ "${P7_WATCHDOG_ACTIVE:-0}" != "1" ]]; then
  export P7_WATCHDOG_ACTIVE=1
  exec timeout --signal=TERM --kill-after=45s 1800s "$0" "$@"
fi

readonly ROS_SETUP=/opt/ros/jazzy/setup.bash
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
readonly REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd -P)"
readonly SOURCE_ROOT="${REPOSITORY_ROOT}/ros2_ws/src"
readonly EVALUATOR="${SCRIPT_DIR}/evaluate_p7_speech_evidence.py"

die() { echo "ERROR: $*" >&2; exit 1; }
usage() {
  cat <<'EOF'
Usage:
  run_and_verify_phase7_speech.sh --output-root DIRECTORY \
    --zh-audio WAV --zh-capture-metadata JSON \
    --en-audio WAV --en-capture-metadata JSON \
    --fr-audio WAV --fr-capture-metadata JSON \
    --model-path DIRECTORY --model-lock JSON [--python PYTHON] \
    [--python-site-dir DIRECTORY] \
    [--device cpu] [--compute-type int8]

All three WAVs must originate from the Phase-7 microphone capture tools.  The
explicit output directory must be new or empty, outside the repository, and
on a Linux filesystem.  The local model must already exist: this harness sets
offline environment controls and never provisions/downloads a model.
EOF
}

for command in awk cmp colcon cp env find findmnt flock git mkdir python3 realpath sha256sum sort tail tee timeout xargs; do
  command -v "${command}" >/dev/null 2>&1 || die "required command is unavailable: ${command}"
done
[[ -r "${ROS_SETUP}" ]] || die "missing ${ROS_SETUP}"
[[ -f "${EVALUATOR}" ]] || die "missing independent Phase-7 evaluator: ${EVALUATOR}"

output_root=""
zh_audio=""; zh_metadata=""
en_audio=""; en_metadata=""
fr_audio=""; fr_metadata=""
model_path=""; model_lock=""
asr_python="/usr/bin/python3"
asr_python_site_dir="/var/tmp/physical-ai-p7-site"
asr_device="cpu"
asr_compute_type="int8"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-root)
      [[ $# -ge 2 ]] || die "--output-root requires a value"
      output_root="$2"; shift 2 ;;
    --zh-audio)
      [[ $# -ge 2 ]] || die "--zh-audio requires a value"
      zh_audio="$2"; shift 2 ;;
    --zh-capture-metadata)
      [[ $# -ge 2 ]] || die "--zh-capture-metadata requires a value"
      zh_metadata="$2"; shift 2 ;;
    --en-audio)
      [[ $# -ge 2 ]] || die "--en-audio requires a value"
      en_audio="$2"; shift 2 ;;
    --en-capture-metadata)
      [[ $# -ge 2 ]] || die "--en-capture-metadata requires a value"
      en_metadata="$2"; shift 2 ;;
    --fr-audio)
      [[ $# -ge 2 ]] || die "--fr-audio requires a value"
      fr_audio="$2"; shift 2 ;;
    --fr-capture-metadata)
      [[ $# -ge 2 ]] || die "--fr-capture-metadata requires a value"
      fr_metadata="$2"; shift 2 ;;
    --model-path)
      [[ $# -ge 2 ]] || die "--model-path requires a value"
      model_path="$2"; shift 2 ;;
    --model-lock)
      [[ $# -ge 2 ]] || die "--model-lock requires a value"
      model_lock="$2"; shift 2 ;;
    --python)
      [[ $# -ge 2 ]] || die "--python requires a value"
      asr_python="$2"; shift 2 ;;
    --python-site-dir)
      [[ $# -ge 2 ]] || die "--python-site-dir requires a value"
      asr_python_site_dir="$2"; shift 2 ;;
    --device)
      [[ $# -ge 2 ]] || die "--device requires a value"
      asr_device="$2"; shift 2 ;;
    --compute-type)
      [[ $# -ge 2 ]] || die "--compute-type requires a value"
      asr_compute_type="$2"; shift 2 ;;
    --help|-h)
      usage
      exit 0 ;;
    *)
      die "unknown argument: $1" ;;
  esac
done
[[ -n "${output_root}" && -n "${zh_audio}" && -n "${zh_metadata}" && -n "${en_audio}" && -n "${en_metadata}" && -n "${fr_audio}" && -n "${fr_metadata}" && -n "${model_path}" && -n "${model_lock}" ]] || {
  usage >&2
  exit 2
}
[[ "${asr_device}" =~ ^[A-Za-z0-9_.:-]+$ ]] || die "invalid ASR device"
[[ "${asr_compute_type}" =~ ^[A-Za-z0-9_.:-]+$ ]] || die "invalid ASR compute type"

resolve_file() {
  local raw="$1" label="$2" resolved
  resolved="$(realpath -e "${raw}")" || die "${label} does not exist: ${raw}"
  [[ -f "${resolved}" ]] || die "${label} is not a regular file: ${resolved}"
  printf '%s\n' "${resolved}"
}
zh_audio="$(resolve_file "${zh_audio}" "Chinese microphone WAV")"
zh_metadata="$(resolve_file "${zh_metadata}" "Chinese capture metadata")"
en_audio="$(resolve_file "${en_audio}" "English microphone WAV")"
en_metadata="$(resolve_file "${en_metadata}" "English capture metadata")"
fr_audio="$(resolve_file "${fr_audio}" "French microphone WAV")"
fr_metadata="$(resolve_file "${fr_metadata}" "French capture metadata")"
model_path="$(realpath -e "${model_path}")" || die "local model does not exist"
[[ -d "${model_path}" ]] || die "local model path is not a directory"
model_lock="$(resolve_file "${model_lock}" "local ASR lock")"
asr_python="$(realpath -e "${asr_python}")" || die "ASR Python does not exist"
[[ -x "${asr_python}" ]] || die "ASR Python is not executable"
asr_python_site_dir="$(realpath -e "${asr_python_site_dir}")" || die "ASR Python site directory does not exist"
[[ -d "${asr_python_site_dir}" ]] || die "ASR Python site path is not a directory"

output_root="$(realpath -m "${output_root}")"
repository_real="$(realpath -e "${REPOSITORY_ROOT}")"
case "${output_root}/" in
  "${repository_real}/"*) die "output directory must not be inside repository" ;;
esac
[[ "${output_root}" != / ]] || die "output directory must not be /"
[[ ! -e "${output_root}" || -d "${output_root}" ]] || die "output path is not a directory"
if [[ -d "${output_root}" ]] && [[ -n "$(find "${output_root}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  die "explicit output directory must be empty"
fi
mkdir -p "${output_root}"
readonly OUTPUT_ROOT="$(realpath -e "${output_root}")"
exec {output_lock_fd}>"${OUTPUT_ROOT}/.phase7.lock"
flock -n "${output_lock_fd}" || die "output directory is already in use"
output_filesystem="$(findmnt -T "${OUTPUT_ROOT}" -n -o FSTYPE)"
case "${output_filesystem,,}" in
  9p|drvfs) die "output directory must be on a Linux filesystem" ;;
esac

readonly BUILD_BASE="${OUTPUT_ROOT}/build"
readonly INSTALL_BASE="${OUTPUT_ROOT}/install"
readonly COLCON_LOG_BASE="${OUTPUT_ROOT}/colcon-log"
readonly ROS_HOME_PATH="${OUTPUT_ROOT}/ros-home"
readonly SOURCE_MANIFEST_BEFORE="${OUTPUT_ROOT}/working_source_before.sha256"
readonly SOURCE_MANIFEST_AFTER="${OUTPUT_ROOT}/working_source_after.sha256"
readonly SEALED_LOCK="${OUTPUT_ROOT}/phase7_local_asr.lock.json"
readonly RESULT_LOG="${OUTPUT_ROOT}/result.txt"
mkdir -p "${COLCON_LOG_BASE}" "${ROS_HOME_PATH}" "${OUTPUT_ROOT}/ros-log"
export PYTHONDONTWRITEBYTECODE=1 ROS_HOME="${ROS_HOME_PATH}" ROS_LOG_DIR="${OUTPUT_ROOT}/ros-log"

[[ -z "$(git -C "${REPOSITORY_ROOT}" status --porcelain --untracked-files=all)" ]] || die "real Phase-7 evidence requires a clean source worktree"
readonly TESTED_COMMIT="$(git -C "${REPOSITORY_ROOT}" rev-parse HEAD)"
capture_source_manifest() {
  (
    cd "${REPOSITORY_ROOT}"
    git ls-files --cached --others --exclude-standard -z |
      LC_ALL=C sort -z |
      xargs -0 -r sha256sum
  )
}
capture_source_manifest >"${SOURCE_MANIFEST_BEFORE}"

{
  echo "TESTED_COMMIT=${TESTED_COMMIT}"
  echo "OUTPUT_FILESYSTEM=${output_filesystem}"
  echo "ASR_MODEL_PATH=${model_path}"
  echo "ASR_PYTHON=${asr_python}"
  echo "ASR_PYTHON_SITE_DIR=${asr_python_site_dir}"
  echo "ASR_DEVICE=${asr_device}"
  echo "ASR_COMPUTE_TYPE=${asr_compute_type}"
  echo "ASR_RUNTIME_NETWORK=DISABLED"
  echo "DOWNSTREAM_SCOPE=PHASE6_DRAWING_PARSER_INTERFACE_ONLY"
} >"${OUTPUT_ROOT}/run_metadata.txt"

run_step() {
  local label="$1" log_path="$2"
  shift 2
  if ! "$@" >"${log_path}" 2>&1; then
    echo "ERROR: ${label} failed; tail of ${log_path}:" >&2
    tail -n 220 "${log_path}" >&2 || true
    return 1
  fi
}

# shellcheck disable=SC1091
source "${ROS_SETUP}"
set -u
cd "${REPOSITORY_ROOT}/ros2_ws"
run_step "Phase-7 targeted build" "${OUTPUT_ROOT}/colcon_build.txt" \
  colcon --log-base "${COLCON_LOG_BASE}" build --base-paths "${SOURCE_ROOT}" \
  --packages-up-to physical_ai_speech physical_ai_drawing \
  --build-base "${BUILD_BASE}" --install-base "${INSTALL_BASE}"
run_step "Phase-7 targeted tests" "${OUTPUT_ROOT}/colcon_test.txt" \
  colcon --log-base "${COLCON_LOG_BASE}" test --base-paths "${SOURCE_ROOT}" \
  --packages-select physical_ai_interfaces physical_ai_speech physical_ai_drawing \
  --build-base "${BUILD_BASE}" --install-base "${INSTALL_BASE}"
run_step "Phase-7 test aggregation" "${OUTPUT_ROOT}/colcon_test_result.txt" \
  colcon test-result --all --verbose --test-result-base "${BUILD_BASE}"

# The ASR dependencies live in an explicit external pip --target directory,
# while the ROS packages are imported directly from the reviewed source. The
# local model is supplied by path and Hugging Face/Transformers offline
# controls are forced for each runtime invocation.
if [[ -n "${PYTHONPATH:-}" ]]; then
  export PYTHONPATH="${asr_python_site_dir}:${SOURCE_ROOT}/physical_ai_speech:${SOURCE_ROOT}/physical_ai_drawing:${PYTHONPATH}"
else
  export PYTHONPATH="${asr_python_site_dir}:${SOURCE_ROOT}/physical_ai_speech:${SOURCE_ROOT}/physical_ai_drawing"
fi
run_step "Phase-7 ASR/runtime import preflight" "${OUTPUT_ROOT}/asr_import_preflight.txt" \
  "${asr_python}" -c 'import physical_ai_speech.speech_pipeline, physical_ai_drawing.drawing_intent'
cp --preserve=mode,timestamps "${model_lock}" "${SEALED_LOCK}"
cmp -s "${model_lock}" "${SEALED_LOCK}" || die "could not seal local-ASR lock"

run_case() {
  local language="$1" source_audio="$2" source_metadata="$3"
  local case_root="${OUTPUT_ROOT}/cases/${language}"
  local input_root="${case_root}/input"
  local runtime_root="${case_root}/runtime"
  local copied_audio="${input_root}/microphone.wav"
  local copied_metadata="${input_root}/capture_metadata.json"
  mkdir -p "${input_root}" "${runtime_root}"
  cp --preserve=mode,timestamps "${source_audio}" "${copied_audio}"
  cp --preserve=mode,timestamps "${source_metadata}" "${copied_metadata}"
  cmp -s "${source_audio}" "${copied_audio}" || die "${language} microphone WAV changed while sealing"
  cmp -s "${source_metadata}" "${copied_metadata}" || die "${language} capture metadata changed while sealing"
  run_step "Phase-7 ${language} local microphone pipeline" "${runtime_root}/pipeline.log" \
    env -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY -u http_proxy -u https_proxy -u all_proxy \
      HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 HF_HUB_DISABLE_TELEMETRY=1 PIP_NO_INDEX=1 \
      "${asr_python}" -m physical_ai_speech.speech_pipeline \
      --audio "${copied_audio}" --capture-metadata "${copied_metadata}" \
      --model-path "${model_path}" --model-lock "${SEALED_LOCK}" \
      --output-root "${runtime_root}" --expected-language "${language}" \
      --device "${asr_device}" --compute-type "${asr_compute_type}"
  [[ -f "${runtime_root}/speech_execution_evidence.json" ]] || die "${language} pipeline did not write speech evidence"
}

run_case zh "${zh_audio}" "${zh_metadata}"
run_case en "${en_audio}" "${en_metadata}"
run_case fr "${fr_audio}" "${fr_metadata}"
run_step "Phase-7 independent evidence evaluation" "${OUTPUT_ROOT}/p7_speech_evaluator.log" \
  "${asr_python}" "${EVALUATOR}" \
  --evidence "${OUTPUT_ROOT}/cases/zh/runtime/speech_execution_evidence.json" \
  --evidence "${OUTPUT_ROOT}/cases/en/runtime/speech_execution_evidence.json" \
  --evidence "${OUTPUT_ROOT}/cases/fr/runtime/speech_execution_evidence.json" \
  --model-lock "${SEALED_LOCK}" --output "${OUTPUT_ROOT}/p7_speech_evaluation.json"
grep -Fxq P7_SPEECH_EVALUATION=PASS "${OUTPUT_ROOT}/p7_speech_evaluator.log" || die "independent Phase-7 evaluator did not pass"

capture_source_manifest >"${SOURCE_MANIFEST_AFTER}"
cmp -s "${SOURCE_MANIFEST_BEFORE}" "${SOURCE_MANIFEST_AFTER}" || die "source manifest changed during Phase-7 runtime"
[[ -z "$(git -C "${REPOSITORY_ROOT}" status --porcelain --untracked-files=all)" ]] || die "source worktree is dirty after Phase-7 runtime"
{
  echo P7_SPEECH=PASS
  echo A06=PASS
  echo A07=PASS
  echo A08=PASS
  echo PHASE7_STATUS=COMPLETE_PENDING_DOCUMENTATION
  echo "TESTED_COMMIT=${TESTED_COMMIT}"
  echo "EVIDENCE_ROOT=${OUTPUT_ROOT}"
  echo SOURCE_MANIFEST_UNCHANGED=PASS
  echo WORKTREE_CLEAN_AFTER_RUNTIME=PASS
} | tee "${RESULT_LOG}"
