import pytest
from physical_ai_drawing.heart_curve import extent, generate_heart


def test_generated_curve_is_closed_and_metric():
    pts = generate_heart(scale_m=0.12, samples=241, board_x=0.7)
    assert len(pts) == 241
    assert pts[0].x == pytest.approx(0.7)
    assert pts[0].y == pytest.approx(pts[-1].y)
    assert pts[0].z == pytest.approx(pts[-1].z)
    assert max(extent(pts)) == pytest.approx(0.12, rel=1e-9)


def test_scale_changes_geometry_not_just_metadata():
    small = generate_heart(scale_m=0.08)
    large = generate_heart(scale_m=0.16)
    assert max(extent(large)) == pytest.approx(2 * max(extent(small)), rel=1e-9)


def test_unsafe_scale_is_rejected():
    with pytest.raises(ValueError):
        generate_heart(scale_m=0.5)
