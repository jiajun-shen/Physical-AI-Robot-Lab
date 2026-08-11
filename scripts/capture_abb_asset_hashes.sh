#!/usr/bin/env bash
set -euo pipefail

if (( $# != 3 )); then
  echo "Usage: $0 SNAPSHOT_ROOT CLEAN_UPSTREAM_CHECKOUT OUTPUT_FILE" >&2
  exit 2
fi

readonly pinned_repository="https://github.com/PickNikRobotics/abb_ros2.git"
readonly pinned_commit="05cbabe774ef026b1cd36fe6b253b55f55e16340"
readonly snapshot_root="$(realpath -- "$1")"
readonly upstream_checkout="$(realpath -- "$2")"
readonly output_file="$(realpath -m -- "$3")"
readonly output_dir="$(dirname -- "${output_file}")"

if [[ ! -d "${snapshot_root}/source" || ! -f "${snapshot_root}/LICENSE" ||
      ! -f "${snapshot_root}/PROVENANCE.json" ]]; then
  echo "ERROR: incomplete snapshot root: ${snapshot_root}" >&2
  exit 1
fi
if [[ ! -d "${output_dir}" ]]; then
  echo "ERROR: output directory does not exist: ${output_dir}" >&2
  exit 1
fi
if [[ "${output_file}" == "${snapshot_root}" ||
      "${output_file}" == "${snapshot_root}/"* ]]; then
  echo "ERROR: output file must be outside the snapshot root" >&2
  exit 1
fi
if [[ "$(basename -- "${output_file}")" != "abb-irb1200-assets.sha256" ]]; then
  echo "ERROR: output filename must be abb-irb1200-assets.sha256" >&2
  exit 1
fi
if ! git -C "${upstream_checkout}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "ERROR: not a Git checkout: ${upstream_checkout}" >&2
  exit 1
fi

readonly observed_repository="$(git -C "${upstream_checkout}" remote get-url origin)"
readonly observed_commit="$(git -C "${upstream_checkout}" rev-parse HEAD)"
if [[ "${observed_repository%.git}" != "${pinned_repository%.git}" ]]; then
  echo "ERROR: unexpected upstream remote: ${observed_repository}" >&2
  exit 1
fi
if [[ "${observed_commit}" != "${pinned_commit}" ]]; then
  echo "ERROR: checkout HEAD is ${observed_commit}, expected ${pinned_commit}" >&2
  exit 1
fi
if [[ -n "$(git -C "${upstream_checkout}" status --porcelain=v1 --untracked-files=all)" ]]; then
  echo "ERROR: upstream checkout is not clean" >&2
  exit 1
fi
git -C "${upstream_checkout}" cat-file -e "${pinned_commit}^{commit}"

declare -a mappings=(
  "LICENSE|LICENSE"
  "abb_resources/urdf/common_colours.xacro|source/abb_resources/urdf/common_colours.xacro"
  "abb_resources/urdf/common_materials.xacro|source/abb_resources/urdf/common_materials.xacro"
  "robot_specific_config/abb_irb1200_support/package.xml|source/abb_irb1200_support/package.xml"
  "robot_specific_config/abb_irb1200_support/urdf/irb1200_5_90_macro.xacro|source/abb_irb1200_support/urdf/irb1200_5_90_macro.xacro"
)
for geometry_kind in collision visual; do
  extension="stl"
  if [[ "${geometry_kind}" == "visual" ]]; then
    extension="dae"
  fi
  for link_name in base_link link_1 link_2 link_3 link_4 link_5 link_6; do
    upstream_path="robot_specific_config/abb_irb1200_support/meshes/irb1200_5_90/${geometry_kind}/${link_name}.${extension}"
    snapshot_path="source/abb_irb1200_support/meshes/irb1200_5_90/${geometry_kind}/${link_name}.${extension}"
    mappings+=("${upstream_path}|${snapshot_path}")
  done
done

declare -a expected_source_paths=()
for mapping in "${mappings[@]}"; do
  upstream_path="${mapping%%|*}"
  snapshot_path="${mapping#*|}"
  if [[ ! -f "${snapshot_root}/${snapshot_path}" ]]; then
    echo "ERROR: missing selected snapshot file: ${snapshot_path}" >&2
    exit 1
  fi
  if ! git -C "${upstream_checkout}" cat-file blob \
      "${pinned_commit}:${upstream_path}" | cmp -s - "${snapshot_root}/${snapshot_path}"; then
    echo "ERROR: snapshot differs from pinned Git blob: ${snapshot_path}" >&2
    exit 1
  fi
  if [[ "${snapshot_path}" == source/* ]]; then
    expected_source_paths+=("${snapshot_path#source/}")
  fi
done

mapfile -t actual_source_paths < <(
  find "${snapshot_root}/source" -type f -printf '%P\n' | LC_ALL=C sort
)
mapfile -t expected_source_paths < <(
  printf '%s\n' "${expected_source_paths[@]}" | LC_ALL=C sort
)
if ! diff -u \
    <(printf '%s\n' "${expected_source_paths[@]}") \
    <(printf '%s\n' "${actual_source_paths[@]}") >&2; then
  echo "ERROR: snapshot source file set differs from the 18-file allowlist" >&2
  exit 1
fi

python3 - "${snapshot_root}/PROVENANCE.json" "${pinned_repository}" \
  "${pinned_commit}" "${#expected_source_paths[@]}" <<'PY'
import json
import pathlib
import sys

path, repository, commit, selected_count = sys.argv[1:]
provenance = json.loads(pathlib.Path(path).read_text(encoding="utf-8"))
expected = {
    "repository": repository,
    "commit": commit,
    "selected_file_count": int(selected_count),
    "source_files_are_unmodified": True,
}
for key, value in expected.items():
    if provenance.get(key) != value:
        raise SystemExit(
            f"ERROR: provenance {key}={provenance.get(key)!r}, expected {value!r}"
        )
PY

temporary_file="$(mktemp -- "${output_file}.tmp.XXXXXX")"
cleanup() {
  rm -f -- "${temporary_file}"
}
trap cleanup EXIT

{
  echo "# SHA-256 lock for ABB assets verified against a clean upstream Git checkout"
  echo "# upstream_repository=${pinned_repository}"
  echo "# upstream_commit=${pinned_commit}"
  for mapping in "${mappings[@]}"; do
    snapshot_path="${mapping#*|}"
    digest="$(sha256sum "${snapshot_root}/${snapshot_path}" | cut -d ' ' -f1)"
    printf '%s  %s\n' "${digest}" "${snapshot_path}"
  done | LC_ALL=C sort -k2
} >"${temporary_file}"

mv -- "${temporary_file}" "${output_file}"
trap - EXIT
echo "WROTE ${output_file}"
