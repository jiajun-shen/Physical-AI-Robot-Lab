import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from physical_ai_drawing.board_frame import BoardFrame
from physical_ai_drawing.drawing_intent import (
    BoardPosition,
    DrawingIntent,
    DrawingIntentError,
    Shape,
    parse_drawing_instruction,
)
from physical_ai_drawing.geometry import extent, generate_local_geometry
from physical_ai_drawing.drawing_runtime import DrawingRuntime, VisionPose


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RUNTIME_CONFIG = PACKAGE_ROOT / "config" / "drawing_runtime.yaml"
CANONICAL_LIMITS = (
    PACKAGE_ROOT.parent
    / "abb_lab_description"
    / "config"
    / "irb1200_5_90_control_limits.yaml"
)


def test_drawing_domain_parser_handles_required_shapes_and_parameters():
    assert parse_drawing_instruction("画一个圆").shape is Shape.CIRCLE
    assert parse_drawing_instruction("画一个圈").shape is Shape.CIRCLE
    small_heart = parse_drawing_instruction("画一个小一点的爱心")
    assert small_heart.shape is Shape.HEART
    assert small_heart.size_m == pytest.approx(0.08)
    explicit_heart = parse_drawing_instruction("draw a 0.08 m heart")
    assert explicit_heart.size_m == pytest.approx(0.08)
    left_square = parse_drawing_instruction("在白板左边画一个正方形")
    assert left_square.shape is Shape.SQUARE
    assert left_square.position is BoardPosition.LEFT
    small_left_square = parse_drawing_instruction("在白板左边画一个小一点的正方形")
    assert small_left_square.size_m == pytest.approx(0.08)
    assert small_left_square.position is BoardPosition.LEFT
    assert parse_drawing_instruction("在白板下方画一个爱心").position is BoardPosition.BOTTOM
    assert parse_drawing_instruction("画一条波浪线").shape is Shape.WAVE


@pytest.mark.parametrize("shape", list(Shape))
def test_all_supported_shapes_are_runtime_generated_in_metric_local_coordinates(shape):
    intent = DrawingIntent(shape=shape, size_m=0.14)
    points = generate_local_geometry(intent, samples=49)
    assert len(points) == 49
    horizontal, vertical = extent(points)
    assert max(horizontal, vertical) == pytest.approx(0.14, rel=1e-9)


def test_geometry_changes_for_size_and_left_position_without_world_coordinates():
    small = generate_local_geometry(DrawingIntent(shape=Shape.HEART, size_m=0.10), samples=49)
    large_left = generate_local_geometry(
        DrawingIntent(shape=Shape.HEART, size_m=0.18, position=BoardPosition.LEFT), samples=49
    )
    assert max(extent(large_left)) > max(extent(small))
    assert sum(point.horizontal_m for point in large_left) / len(large_left) < 0.0


def test_board_frame_maps_local_geometry_through_live_center_and_calibrated_axes():
    frame = BoardFrame(
        center_world_m=(1.10, 0.0, 0.975),
        normal_world=(-1.0, 0.0, 0.0),
        horizontal_world=(0.0, 1.0, 0.0),
        vertical_world=(0.0, 0.0, 1.0),
        front_surface_offset_m=0.015,
    )
    local = generate_local_geometry(DrawingIntent(shape=Shape.LINE, size_m=0.12), samples=17)
    world = frame.map_points(local, standoff_m=0.03)
    assert all(point[0] == pytest.approx(1.055) for point in world)
    assert world[0][1] < world[-1][1]


def test_runtime_derives_the_board_collision_centre_from_its_observed_front_surface():
    runtime = SimpleNamespace(
        board_config={
            "observation_reference": "robot_facing_front_surface",
            "robot_facing_normal_world": [-1.0, 0.0, 0.0],
            "horizontal_axis_world": [0.0, 1.0, 0.0],
            "vertical_axis_world": [0.0, 0.0, 1.0],
            "front_surface_offset_m": 0.015,
            "size_xyz_m": [0.03, 0.70, 0.45],
        }
    )
    observed = VisionPose(
        class_id="drawing_board",
        x=1.085,
        y=0.0,
        z=0.975,
        stamp_ns=123,
        confidence=0.9,
        detection_id="board-123",
    )

    frame = DrawingRuntime.board_frame(runtime, observed)

    assert frame.center_world_m == pytest.approx((1.10, 0.0, 0.975))
    points = frame.map_points(
        generate_local_geometry(DrawingIntent(shape=Shape.LINE, size_m=0.08), samples=17),
        standoff_m=0.008,
    )
    assert all(point[0] == pytest.approx(1.077) for point in points)


def test_ambiguous_or_unsafe_drawing_requests_are_rejected():
    with pytest.raises(DrawingIntentError, match="AMBIGUOUS_DRAWING_SHAPE"):
        parse_drawing_instruction("draw a circle and square")
    with pytest.raises(DrawingIntentError):
        DrawingIntent(shape=Shape.CIRCLE, orientation_rad=3.0 * math.pi)


def test_drawing_preflight_margin_uses_the_canonical_abb_hard_limits():
    runtime = yaml.safe_load(RUNTIME_CONFIG.read_text(encoding="utf-8"))
    canonical = yaml.safe_load(CANONICAL_LIMITS.read_text(encoding="utf-8"))
    expected = {
        name: [limits["min_position"], limits["max_position"]]
        for name, limits in canonical["joint_limits"].items()
    }
    assert runtime["robot"]["joint_position_limits_rad"] == expected
    assert runtime["planner"]["preflight_minimum_joint_limit_margin_rad"] == pytest.approx(
        0.20
    )
