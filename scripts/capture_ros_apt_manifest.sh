#!/usr/bin/env bash
set -euo pipefail

if (( $# != 1 )); then
  echo "Usage: $0 OUTPUT_FILE" >&2
  exit 2
fi

readonly output_file="$1"
readonly output_dir="$(dirname "${output_file}")"
if [[ ! -d "${output_dir}" ]]; then
  echo "ERROR: output directory does not exist: ${output_dir}" >&2
  exit 1
fi

temporary_file="$(mktemp "${output_file}.tmp.XXXXXX")"
cleanup() {
  rm -f -- "${temporary_file}"
}
trap cleanup EXIT

{
  echo "# Physical-AI-Robot-Lab installed ROS apt manifest"
  echo "# generated_at=$(date --iso-8601=seconds)"
  echo "# os=$(source /etc/os-release && printf '%s %s (%s)' "${ID}" "${VERSION_ID}" "${VERSION_CODENAME}")"
  echo "# architecture=$(dpkg --print-architecture)"
  printf 'package\tversion\n'
  dpkg-query -W -f='${Status}\t${binary:Package}\t${Version}\n' 'ros-jazzy-*' \
    | awk -F '\t' '$1 == "install ok installed" {print $2 "\t" $3}' \
    | LC_ALL=C sort
} >"${temporary_file}"

mv -- "${temporary_file}" "${output_file}"
trap - EXIT
echo "WROTE ${output_file}"
