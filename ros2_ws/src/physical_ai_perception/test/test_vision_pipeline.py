from pathlib import Path

import cv2
import numpy as np
import pytest

from physical_ai_perception.vision_pipeline import (
    ClassSpec,
    ColorRange,
    VisionInputError,
    apply_center_offset,
    detect_components,
    estimate_planar_rectangle_center,
    estimate_depth,
    load_class_specs,
    project_pixel,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / "config" / "vision_classes.yaml"


def bgr_from_hsv(hue: int, saturation: int = 220, value: int = 220) -> tuple[int, int, int]:
    pixel = np.array([[[hue, saturation, value]]], dtype=np.uint8)
    return tuple(int(value) for value in cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0, 0])


def spec(class_id: str, hue: int, *, minimum: int = 30, aspect: float = 5.0) -> ClassSpec:
    return ClassSpec(
        class_id=class_id,
        hsv_ranges=(ColorRange((max(0, hue - 3), 80, 40), (min(179, hue + 3), 255, 255)),),
        min_pixels=minimum,
        max_pixels=10000,
        min_aspect_ratio=0.5,
        max_aspect_ratio=aspect,
        center_offset_m=0.01,
        geometry_kind="surface_ray",
    )


def test_real_class_configuration_has_only_required_labels_and_no_scene_coordinates() -> None:
    specs = load_class_specs(CONFIG_PATH)
    assert {item.class_id for item in specs} == {
        "green_ball",
        "red_ball",
        "blue_cube",
        "yellow_cube",
        "drawing_board",
    }
    source = CONFIG_PATH.read_text(encoding="utf-8").lower()
    for forbidden in ("seed", "world", "position", "coordinate", "manifest"):
        assert forbidden not in source


def test_detect_components_selects_one_visual_component_per_color() -> None:
    image = np.zeros((180, 260, 3), dtype=np.uint8)
    cv2.circle(image, (35, 35), 12, bgr_from_hsv(60), -1)
    cv2.circle(image, (85, 35), 12, bgr_from_hsv(0), -1)
    cv2.rectangle(image, (20, 90), (50, 120), bgr_from_hsv(112), -1)
    cv2.rectangle(image, (75, 90), (105, 120), bgr_from_hsv(30), -1)
    cv2.rectangle(image, (140, 45), (230, 85), (230, 230, 230), -1)
    specs = (
        spec("green", 60),
        spec("red", 0),
        spec("blue", 112),
        spec("yellow", 30),
        ClassSpec(
            class_id="board",
            hsv_ranges=(ColorRange((0, 0, 180), (179, 40, 255)),),
            min_pixels=100,
            max_pixels=10000,
            min_aspect_ratio=1.1,
            max_aspect_ratio=5.0,
            center_offset_m=0.015,
            geometry_kind="planar_rectangle",
        ),
    )
    candidates = {item.class_id: item for item in detect_components(image, specs)}
    assert set(candidates) == {"green", "red", "blue", "yellow", "board"}
    assert candidates["green"].mask_pixel_count > 300
    assert candidates["board"].width_px > candidates["board"].height_px


def test_depth_validity_and_component_size_reject_background_and_large_color_regions() -> None:
    image = np.zeros((80, 120, 3), dtype=np.uint8)
    image[:, :] = bgr_from_hsv(30)
    cv2.rectangle(image, (20, 20), (35, 35), bgr_from_hsv(30), -1)
    valid = np.zeros((80, 120), dtype=bool)
    valid[20:36, 20:36] = True
    yellow = ClassSpec(
        class_id="yellow",
        hsv_ranges=(ColorRange((27, 80, 40), (33, 255, 255)),),
        min_pixels=50,
        max_pixels=500,
        min_aspect_ratio=0.5,
        max_aspect_ratio=2.0,
        center_offset_m=0.01,
        geometry_kind="surface_ray",
    )
    candidates = detect_components(image, (yellow,), valid_depth_mask=valid)
    assert len(candidates) == 1
    assert candidates[0].mask_pixel_count == 256


def test_depth_estimate_discards_zero_nan_and_inf_values() -> None:
    depth = np.full((10, 10), 1.25, dtype=np.float32)
    depth[0, 0] = np.nan
    depth[0, 1] = np.inf
    depth[0, 2] = 0.0
    estimate = estimate_depth(depth, np.ones((10, 10), dtype=bool), minimum_samples=50)
    assert estimate.valid_sample_count == 97
    assert estimate.depth_m == pytest.approx(1.25)
    assert estimate.depth_mad_m == pytest.approx(0.0)
    with pytest.raises(VisionInputError):
        estimate_depth(np.zeros((3, 3), dtype=np.float32), np.ones((3, 3), dtype=bool))


def test_projection_uses_the_supplied_camera_matrix_and_requires_valid_intrinsics() -> None:
    point = project_pixel(420.0, 290.0, 2.0, (200.0, 0.0, 320.0, 0.0, 250.0, 240.0, 0.0, 0.0, 1.0))
    assert point == pytest.approx((1.0, 0.4, 2.0))
    centered = apply_center_offset(point, 0.1)
    assert np.linalg.norm(np.asarray(centered) - np.asarray(point)) == pytest.approx(0.1)
    with pytest.raises(VisionInputError):
        project_pixel(1.0, 1.0, 1.0, (0.0,) * 9)
    with pytest.raises(VisionInputError):
        project_pixel(1.0, 1.0, 0.0, (200.0, 0.0, 0.0, 0.0, 200.0, 0.0, 0.0, 0.0, 1.0))


def test_planar_rectangle_center_uses_rgbd_geometry_not_a_fixed_coordinate() -> None:
    height, width = 80, 100
    depth = np.zeros((height, width), dtype=np.float32)
    mask = np.zeros((height, width), dtype=bool)
    mask[20:60, 30:80] = True
    rows, columns = np.where(mask)
    depth[rows, columns] = 1.5 + 0.002 * (columns - 55)
    center = estimate_planar_rectangle_center(
        depth,
        mask,
        (100.0, 0.0, 50.0, 0.0, 100.0, 40.0, 0.0, 0.0, 1.0),
    )
    assert center == pytest.approx((0.075, -0.0075, 1.51), abs=0.03)
