# ABB IRB1200 support asset snapshot

This directory is a deliberately narrow, vendored snapshot of unmodified
ABB IRB1200-5/0.9 model assets. It is not the complete `abb_ros2` repository,
not an ABB driver, and not a claim of ROS 2 Jazzy driver compatibility.

The full source repository is locked in
`share/abb_lab_description/config/abb-model-assets.repos`, and the selected
files are byte-locked in
`share/abb_lab_description/config/abb-irb1200-assets.sha256`. In the source
package those files are at `../../config/`. Exact source, lineage, exclusions,
license, and limitations are recorded in `PROVENANCE.json`.

The snapshot intentionally excludes upstream bringup, EGM/RWS hardware code,
the Rolling-oriented ros2_control xacro, MoveIt configuration, launch files,
and RViz configuration. The retained `package.xml` is warning/provenance
metadata; this directory is not itself a ROS package.

`irb1200_5_90_macro.xacro` still contains its original package URIs and material
include. The surrounding project-owned description package wraps those paths
without changing geometry. A later control-description increment must still
supply a fixed base and `gz_ros2_control`, and use one reviewed limit set before
any simulated ABB motion is accepted.
