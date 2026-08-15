# Phase 6 — Parametric Drawing Evidence Contract

Phase 6 requires a Cartesian curve generated at runtime, not prerecorded joint motion.

The canonical heart is generated from
`x(t)=16 sin(t)^3`,
`y(t)=13 cos(t)-5 cos(2t)-2 cos(3t)-cos(4t)`,
then normalized, scaled in metres, and embedded in the drawing-board plane.

Exit evidence must preserve the requested scale, generated Cartesian samples, board frame, collision-aware approach/retreat, executed Cartesian/end-effector trace, and an offline geometric comparison. Two different requested scales must produce measurably different path extents. A05 stays `NOT STARTED` until a real Gazebo execution exists.
