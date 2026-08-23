import math

import pytest

from physical_ai_drawing.board_frame import BoardFrame
from physical_ai_drawing.drawing_intent import (
    BoardPosition,
    DrawingIntent,
    DrawingIntentError,
    Shape,
    parse_drawing_instruction,
)
from physical_ai_drawing.geometry import extent, generate_local_geometry


def test_drawing_domain_parser_handles_required_shapes_and_parameters():
    assert parse_drawing_instruction("画一个圆").shape is Shape.CIRCLE
    small_heart = parse_drawing_instruction("画一个小一点的爱心")
    assert small_heart.shape is Shape.HEART
    assert small_heart.size_m == pytest.approx(0.08)
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


def test_ambiguous_or_unsafe_drawing_requests_are_rejected():
    with pytest.raises(DrawingIntentError, match="AMBIGUOUS_DRAWING_SHAPE"):
        parse_drawing_instruction("draw a circle and square")
    with pytest.raises(DrawingIntentError):
        DrawingIntent(shape=Shape.CIRCLE, orientation_rad=3.0 * math.pi)
