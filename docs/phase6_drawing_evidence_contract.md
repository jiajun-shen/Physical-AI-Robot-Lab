# Phase 6 — Parametric Drawing Evidence Contract

Phase 6 requires a Cartesian curve generated at runtime, not prerecorded joint motion.

The canonical heart is generated from
`x(t)=16 sin(t)^3`,
`y(t)=13 cos(t)-5 cos(2t)-2 cos(3t)-cos(4t)`,
then normalized, scaled in metres, and embedded in the drawing-board plane.

The RGB-D board observation is specifically the visible robot-facing front
surface, not the board-volume centre. The runtime derives the solid board
centre along its calibrated normal before adding the collision box, while the
generated tip trace stays at its explicit contactless standoff from that
surface. A planar board class is forbidden from applying a camera-ray centre
offset.

The physical Phase-5 obstacle boxes are also required collision geometry. A
separate P5 RGB-D localizer must supply both top fiducials from one fresh visual
epoch; the runtime converts each marker to a conservative collision box and
rejects every approach, entry, curve, or retreat plan unless both boxes are
present in the live MoveIt scene.

The sealed headless harness runs each independent acceptance input in its own
fresh Gazebo, MoveIt, and vision episode.  It does not carry static-obstacle
observations across cases: after shutdown, the next case must newly observe the
board and both marker fiducials before its first motion.  This preserves the
freshness contract even when a previous drawing's final pose would occlude a
marker.

Before the first curve chord, the runtime must execute a separately preflighted
live-state Cartesian `entry_alignment` to exactly the first runtime-generated
waypoint. It uses its own bounded slow profile and must pass both the usual
joint endpoint observation and a three-sample TF TCP position/orientation gate.
The first curve segment can never absorb an unrecorded approach or controller
residual.

Exit evidence must preserve the requested scale, generated Cartesian samples,
front-surface/derived-centre board frame, visual static-obstacle provenance and
scene IDs, collision-aware approach/entry/curve/retreat, execution gates, and
the live end-effector trace. Two different requested scales or positions must
produce parameter variation. A05 stays `NOT STARTED` until three real Gazebo
cases (curve, polygon, and parameter variation) pass the independent evaluator.
