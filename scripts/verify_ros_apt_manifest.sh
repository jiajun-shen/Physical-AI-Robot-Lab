#!/usr/bin/env bash
set -euo pipefail

if (( $# != 2 )); then
  echo "Usage: $0 INSTALLED_MANIFEST DIRECT_PACKAGE_LOCK" >&2
  exit 2
fi

readonly installed_manifest="$1"
readonly direct_package_lock="$2"

for required_file in "${installed_manifest}" "${direct_package_lock}"; do
  if [[ ! -r "${required_file}" ]]; then
    echo "ERROR: required file is not readable: ${required_file}" >&2
    exit 1
  fi
done

if [[ "$(sed -n '5p' "${installed_manifest}")" != $'package\tversion' ]]; then
  echo "ERROR: unexpected manifest header" >&2
  exit 1
fi

readonly data_start_line=6
tail -n "+${data_start_line}" "${installed_manifest}" | LC_ALL=C sort -c

if ! tail -n "+${data_start_line}" "${installed_manifest}" \
  | awk -F '\t' '
      NF != 2 || $1 !~ /^ros-jazzy-[a-z0-9-]+$/ || $2 == "" { bad = 1 }
      END { exit bad }
    '; then
  echo "ERROR: malformed installed-package row" >&2
  exit 1
fi

duplicate_packages="$(
  tail -n "+${data_start_line}" "${installed_manifest}" \
    | cut -f1 \
    | uniq -d
)"
if [[ -n "${duplicate_packages}" ]]; then
  echo "ERROR: duplicate installed-package entries" >&2
  echo "${duplicate_packages}" >&2
  exit 1
fi

direct_package_count=0
while IFS=$'\t' read -r package_name expected_version; do
  actual_version="$(dpkg-query -W -f='${Version}' "${package_name}")"
  if [[ "${actual_version}" != "${expected_version}" ]]; then
    echo "ERROR: ${package_name}: expected ${expected_version}, installed ${actual_version}" >&2
    exit 1
  fi

  if ! grep -Fxq "${package_name}"$'\t'"${expected_version}" "${installed_manifest}"; then
    echo "ERROR: direct package is absent from manifest: ${package_name}" >&2
    exit 1
  fi
  ((direct_package_count += 1))
done < <(
  awk '
    /^  ros-jazzy-[a-z0-9-]+:/ {
      package_name = $1
      sub(/:$/, "", package_name)
      print package_name "\t" $2
    }
  ' "${direct_package_lock}"
)

installed_package_count="$(tail -n "+${data_start_line}" "${installed_manifest}" | wc -l)"
echo "INSTALLED_ROS_PACKAGE_COUNT=${installed_package_count}"
echo "DIRECT_LOCK_MATCH_COUNT=${direct_package_count}"
echo "ROS_APT_MANIFEST=PASS"
