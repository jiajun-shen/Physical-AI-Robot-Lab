# Phase 1 ABB IRB1200 Asset and Geometry Evidence

- Date: 2026-08-11
- Runtime: WSL2 Ubuntu 24.04.4, ROS 2 Jazzy
- Scope: pinned IRB1200-5/0.9 geometry/kinematic assets and a geometry-only
  ROS description package
- Truth boundary: no Gazebo spawn, ros2_control integration, controller, ABB
  motion, MoveIt planning, gripper, or real-hardware support is claimed

## Supply-chain boundary

The source lock is
`ros2_ws/src/abb_lab_description/config/abb-model-assets.repos`:

```text
repository: https://github.com/PickNikRobotics/abb_ros2.git
commit: 05cbabe774ef026b1cd36fe6b253b55f55e16340
upstream branch at audit time: rolling
license: Apache-2.0 for the selected support material
```

`vcs import` into an independent directory completed at the exact detached
commit. The selected snapshot contains 18 canonical source files plus the root
license, all locked by the package-local `abb-irb1200-assets.sha256`. A byte comparison
between the independent import and the committed snapshot matched 19/19 files.
The lock-update script now requires a clean checkout at the exact SHA with the
expected GitHub remote, checks the 18-file allowlist and provenance fields, and
compares every selected byte to `git cat-file` before it can rewrite the lock.
An attempted output path inside the snapshot was rejected before creating a
file.

The initial Windows audit checkout had expanded CRLF working-tree files,
including text-based DAE meshes. That copy was rejected for byte-level
provenance and replaced from the WSL `vcs import`, whose working tree matches the
Git blobs. The final lock therefore covers canonical LF source bytes, not the
Windows checkout representation.

Selected source scope:

- 7 STL collision meshes and 7 DAE visual meshes;
- the unmodified `irb1200_5_90_macro.xacro`;
- two unmodified ABB material resources;
- upstream `package.xml` retained as warning/author metadata;
- upstream Apache-2.0 license.

Explicit exclusions include the complete ABB EGM/RWS driver chain,
`abb_hardware_interface`, `abb_bringup`, upstream ros2_control and top-level
composition xacros, MoveIt config, launch, RViz config, and gripper content.

The installed `vcstool` 0.3.0 `vcs validate` command raised an internal
`UnboundLocalError` on the full-SHA manifest. This was not treated as evidence
of validity. A real fresh `vcs import` was executed instead and its `HEAD`, clean
status, and selected bytes were checked.

## Tests-first sequence

Before asset import, the structural suite failed with the explicit missing
macro path. After the locked snapshot was imported, the same suite passed.

Before the ROS package existed, the description source contract failed on the
missing package, CMake metadata, and geometry wrapper. After implementation,
the source suite passed.

The initial build script built and installed the package but then failed while
sourcing the colcon setup under Bash nounset because `COLCON_TRACE` was absent.
The harness now disables nounset only around the generated setup script. A
complete clean build and regression run in a new output directory then passed.
The final harness creates a unique empty ext4 directory by default, rejects a
nonempty/repository-local output, disables source-tree bytecode caches, and
gives build, test, and test-result operations explicit output-local log paths.

## Project-owned geometry package

`abb_lab_description` avoids shadowing any future upstream
`abb_irb1200_support` or `abb_resources` packages. It installs the locked meshes
under its own package URI and uses two visibly marked Apache-2.0 derivative
xacros. Automated byte-equivalence contracts permit only their documented
notice insertion and resource lookup path substitutions; changes to geometry,
frames, axes, limits, mass, or inertia fail the suite.

Snapshot, hashes, manifest, license, and tests are package-local. A copy of only
`abb_lab_description` was built and tested at an unrelated workspace depth on
WSL ext4, proving that no `../../..` repository-layout dependency remains. The
installed package also retains the raw snapshot and verifies all 19 hashes.

The geometry entry point intentionally contains no `<ros2_control>`, Gazebo
plugin, transmission, hardware interface, world joint, or gripper. Source
position/velocity limits remain reference inputs; the upstream effort value
`1000` is explicitly marked fictional and no operational control limit is yet
approved.

## Final executed regression

Command:

```bash
bash scripts/build_and_test_abb_description.sh
```

Observed final results:

```text
colcon build: 1 package finished
check_urdf: Successfully Parsed XML
root link: base_link
ABB_GEOMETRY_URDF=PASS
LINK_COUNT=10
JOINT_COUNT=9
REVOLUTE_JOINT_COUNT=6
MESH_REFERENCE_COUNT=14
INSTALLED_LOCKED_ASSET_COUNT=19
CTest targets: 2/2 passed
Python cases: 15 passed
colcon test-result: 17 records, 0 errors, 0 failures, 0 skipped
ABB_DESCRIPTION_BUILD=PASS
OUTPUT_ROOT=/tmp/physical-ai-robot-lab/abb-description.zDxfS7
```

An additional package-only regression in
`/tmp/abb_lab_description_standalone.QPuoVQ/unrelated/depth` produced
`ABB_DESCRIPTION_STANDALONE=PASS` with the same 17 zero-failure test records.

The automated contracts cover:

- exact source URL/full SHA, file allowlist, license, provenance, and SHA-256;
- exactly `joint_1` through `joint_6` with the expected parent/child chain;
- `base_link`, `base`, `flange`, and `tool0` frames;
- finite lower/upper/velocity values and documented fictional effort warning;
- seven positive, finite, positive-definite inertial matrices;
- all 14 source and installed mesh resources;
- package-local source lock, installed 19-file hash verification, and active
  mesh equality with the raw locked inputs;
- project-local package URIs and exact permitted derivative transformations;
- the six-axis reference YAML matches the source macro and remains explicitly
  non-operational (`NOT_UNIFIED`, `NOT_TESTED`, real robot not ready);
- absence of upstream control, bringup, MoveIt, Gazebo, and driver content;
- real Xacro expansion followed by URDF parser validation.

Build, install, all three colcon log trees, pytest caches, and generated URDF
stayed under `/tmp` and are not committed. A post-run ignored-file inspection
found only the pre-existing ignored `runs/` runtime-data directory; no repo-root
`log/`, test `__pycache__`, build, or install artifact remained.

The final log scan found no compiler, CMake, Xacro, URDF, or test error. Colcon
debug logging reported that the optional desktop-notification plugin could not
reach `org.freedesktop.Notifications` in the noninteractive WSL session; build
and test completion were unaffected. Matches saying `0 tests failed` are
successful summaries, not failures.

## Acceptance boundary

This evidence establishes `IRB1200_ASSET_CONTRACT=PASS`. It is not A01. A01 and
all other acceptance items remain `NOT STARTED` until an actual ABB model is
spawned and observed in Gazebo under the project runtime.
