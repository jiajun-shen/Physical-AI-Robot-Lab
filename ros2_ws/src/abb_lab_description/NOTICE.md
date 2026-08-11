# Third-party and modification notice

The IRB1200 geometry, kinematic values, material definitions, and meshes used by
this package come from `PickNikRobotics/abb_ros2` commit
`05cbabe774ef026b1cd36fe6b253b55f55e16340`, with model lineage in
`ros-industrial/abb` tag `1.5.0`. Those files are Apache-2.0 licensed.

`urdf/irb1200_5_90_macro.project.xacro` is a project modification of the
upstream macro. Only resource lookup paths were changed: the upstream
`abb_resources` material include and `abb_irb1200_support` mesh package URIs
now resolve inside `abb_lab_description`. Geometry, joints, frames, limits,
dynamics, masses, and inertias were not intentionally changed.

The upstream license, package metadata, provenance, and limitations are
installed under this package's `third_party` share directory.
