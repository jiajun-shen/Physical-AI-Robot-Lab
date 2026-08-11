# Third-Party Register

Audit date: 2026-08-11. A narrow ABB asset snapshot is committed with its
license, provenance, full-SHA source lock, and per-file SHA-256 lock. No
dataset, checkpoint, or complete external driver repository is committed.

| Component | Repository/source | Branch/tag | Commit/version | License | Status/use |
|---|---|---|---|---|---|
| ABB ROS 2 port/model snapshot | [PickNikRobotics/abb_ros2](https://github.com/PickNikRobotics/abb_ros2) | `rolling` | `05cbabe774ef026b1cd36fe6b253b55f55e16340` | Selected support material Apache-2.0 | VENDORED: 18 source files + license; geometry-only |
| ABB IRB1200 authoritative lineage | [ros-industrial/abb](https://github.com/ros-industrial/abb) | tag `1.5.0` | `eefdf01c956a6f2129aacb88d598201feff3cf19` | Apache-2.0 | REFERENCE: source/limit provenance |
| MoveIt 2 | [moveit/moveit2](https://github.com/moveit/moveit2) | tag `2.12.4` | `1ade0e9dcf50dbbbc3a984b995c786cf12736235` | BSD-3-Clause | SELECTED through Jazzy apt |
| MoveIt Task Constructor | [moveit/moveit_task_constructor](https://github.com/moveit/moveit_task_constructor) | tag `ros2-0.1.5` | `56c6a2a40ebff8a4d1644d9e6fac819c3cd3542f` | BSD-3-Clause | SELECTED through Jazzy apt |
| ros2_control | [ros-controls/ros2_control](https://github.com/ros-controls/ros2_control) | tag `4.47.0` | `df23a4b7a21cb8ea257c80ecf98e85571b50ba01` | Apache-2.0 | REFERENCE; target apt may lag |
| ros2_controllers | [ros-controls/ros2_controllers](https://github.com/ros-controls/ros2_controllers) | tag `4.41.0` | `b69445a5c72bc1b0dfc0658ce992748e65e94cf6` | Apache-2.0 | REFERENCE; target apt may lag |
| gz_ros2_control | [ros-controls/gz_ros2_control](https://github.com/ros-controls/gz_ros2_control) | tag `1.2.19` | `3632af2998895cd18c03717bd40c7acd547db24d` | Apache-2.0 | SELECTED through Jazzy apt |
| ros_gz | [gazebosim/ros_gz](https://github.com/gazebosim/ros_gz) | tag `1.0.23` | `ec3a555b540ac492882d587a09752eb2eeeee3cd` | Apache-2.0 | REFERENCE; audited apt candidate 1.0.22 |
| openpi | [Physical-Intelligence/openpi](https://github.com/Physical-Intelligence/openpi) | `main` | `15a9616a00943ada6c20a0f158e3adb39df2ccac` | Code Apache-2.0; additional Gemma/model terms | RESEARCH PIN; not installed |

## ABB driver dependencies deliberately excluded from the baseline

The upstream `abb.repos` follows moving branches. These exact audit-time heads
are recorded for traceability, but Phase 1 simulation does not need the real
ABB EGM/RWS stack.

| Repository | Upstream branch | Audit-time head | License |
|---|---|---|---|
| `ros-industrial/abb_libegm` | `master` | `2c9350abfb67131cae796805fed8cd6eeb2ab7f9` | BSD-3-Clause style |
| `ros-industrial/abb_librws` | `master` | `9ba08371482f3975f364011db9b1368fa0fc6b32` | BSD-3-Clause style |
| `ros-industrial/abb_egm_rws_managers` | `master` | `7e554e53b415df1bbd743392b84a055c499b63d2` | BSD-3-Clause style |
| `gbartyzel/abb_ros2_msgs` | `rolling` | `071bb1fc2148a7cd850a1ff550c5987e67da6bdd` | BSD-3-Clause style |

These are not integration locks. A future, isolated real-hardware branch must
re-audit and pin them before use.

## ABB snapshot record

- Location: `ros2_ws/src/abb_lab_description/third_party/abb_irb1200_support/`
- Source manifest: `ros2_ws/src/abb_lab_description/config/abb-model-assets.repos`
- Byte lock: `ros2_ws/src/abb_lab_description/config/abb-irb1200-assets.sha256`
  (19/19 verified against the pinned Git objects)
- Provenance/exclusions:
  `ros2_ws/src/abb_lab_description/third_party/abb_irb1200_support/PROVENANCE.json`
- Project derivative notice: `ros2_ws/src/abb_lab_description/NOTICE.md`
- Installed package name: `abb_lab_description` (project-owned; it does not
  shadow upstream ABB package names)

The retained upstream `package.xml` is provenance and warning metadata, not the
project package manifest. The Rolling ros2_control composition and every
EGM/RWS/real-hardware dependency are excluded. Snapshot, locks, tests, and
licenses are package-local so `abb_lab_description` remains independently
buildable; the installed package retains the raw locked inputs for audit.

## Asset/checkpoint rules

- Preserve upstream copyright, LICENSE, and NOTICE files with vendored assets.
- Record modification notices for changed third-party files.
- Do not rely on the repository-level openpi code license for checkpoint terms;
  audit the model-specific terms before download/use.
- Never commit checkpoints, datasets, secrets, ROS logs, or generated build
  artifacts. Store them externally with checksums and provenance manifests.
