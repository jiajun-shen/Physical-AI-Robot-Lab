"""Pure image/depth and camera-intrinsic operations for the vision node."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np
import yaml


class VisionInputError(ValueError):
    """Raised when an image, depth sample, or intrinsic is unusable."""


@dataclass(frozen=True)
class ColorRange:
    """Inclusive HSV interval in OpenCV's 8-bit convention."""

    lower: tuple[int, int, int]
    upper: tuple[int, int, int]


@dataclass(frozen=True)
class ClassSpec:
    """Appearance and geometry priors that do not encode scene placement."""

    class_id: str
    hsv_ranges: tuple[ColorRange, ...]
    min_pixels: int
    max_pixels: int
    min_aspect_ratio: float
    max_aspect_ratio: float
    center_offset_m: float
    geometry_kind: str


@dataclass(frozen=True)
class DetectionCandidate:
    """A 2-D connected component selected from one visual class mask."""

    class_id: str
    center_u_px: float
    center_v_px: float
    x_offset_px: int
    y_offset_px: int
    width_px: int
    height_px: int
    mask_pixel_count: int
    confidence: float
    mask: np.ndarray
    center_offset_m: float
    geometry_kind: str


@dataclass(frozen=True)
class DepthEstimate:
    """Robust positive-depth statistics restricted to a component mask."""

    depth_m: float
    depth_mad_m: float
    valid_sample_count: int


def _finite_float(value: Any, label: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise VisionInputError(f"{label} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise VisionInputError(f"{label} must be finite")
    if minimum is not None and result < minimum:
        raise VisionInputError(f"{label} must be at least {minimum}")
    return result


def _hsv_triplet(value: Any, label: str) -> tuple[int, int, int]:
    if not isinstance(value, list) or len(value) != 3:
        raise VisionInputError(f"{label} must be a three-value list")
    parsed: list[int] = []
    for index, item in enumerate(value):
        if isinstance(item, bool) or not isinstance(item, int):
            raise VisionInputError(f"{label}[{index}] must be an integer")
        upper = 179 if index == 0 else 255
        if item < 0 or item > upper:
            raise VisionInputError(f"{label}[{index}] is outside the HSV range")
        parsed.append(item)
    return tuple(parsed)  # type: ignore[return-value]


def load_class_specs(path: str | Path) -> tuple[ClassSpec, ...]:
    """Load validated color/shape priors from a project-owned YAML file."""
    config_path = Path(path)
    if not config_path.is_file():
        raise VisionInputError(f"class-spec file is missing: {config_path}")
    try:
        payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise VisionInputError(f"cannot parse class-spec file: {config_path}") from exc
    if not isinstance(payload, dict) or not isinstance(payload.get("classes"), list):
        raise VisionInputError("class-spec document must contain a classes list")
    specs: list[ClassSpec] = []
    seen: set[str] = set()
    for entry in payload["classes"]:
        if not isinstance(entry, dict):
            raise VisionInputError("each class spec must be a mapping")
        class_id = entry.get("class_id")
        if not isinstance(class_id, str) or not class_id or class_id in seen:
            raise VisionInputError("each class_id must be nonempty and unique")
        ranges_data = entry.get("hsv_ranges")
        if not isinstance(ranges_data, list) or not ranges_data:
            raise VisionInputError(f"{class_id}.hsv_ranges must be a nonempty list")
        ranges: list[ColorRange] = []
        for range_entry in ranges_data:
            if not isinstance(range_entry, dict):
                raise VisionInputError(f"{class_id}.hsv_ranges entries must be mappings")
            lower = _hsv_triplet(range_entry.get("lower"), f"{class_id}.lower")
            upper = _hsv_triplet(range_entry.get("upper"), f"{class_id}.upper")
            if any(lower[index] > upper[index] for index in range(3)):
                raise VisionInputError(f"{class_id} HSV lower bound exceeds upper bound")
            ranges.append(ColorRange(lower=lower, upper=upper))
        min_pixels = entry.get("min_pixels")
        if isinstance(min_pixels, bool) or not isinstance(min_pixels, int) or min_pixels < 1:
            raise VisionInputError(f"{class_id}.min_pixels must be a positive integer")
        max_pixels = entry.get("max_pixels")
        if isinstance(max_pixels, bool) or not isinstance(max_pixels, int) or max_pixels < min_pixels:
            raise VisionInputError(f"{class_id}.max_pixels must be an integer at least min_pixels")
        min_aspect = _finite_float(
            entry.get("min_aspect_ratio"), f"{class_id}.min_aspect_ratio", minimum=0.0
        )
        max_aspect = _finite_float(
            entry.get("max_aspect_ratio"), f"{class_id}.max_aspect_ratio", minimum=min_aspect
        )
        if max_aspect <= 0.0:
            raise VisionInputError(f"{class_id}.max_aspect_ratio must be positive")
        center_offset = _finite_float(
            entry.get("center_offset_m"), f"{class_id}.center_offset_m", minimum=0.0
        )
        geometry_kind = entry.get("geometry_kind")
        if geometry_kind not in {"surface_ray", "planar_rectangle"}:
            raise VisionInputError(
                f"{class_id}.geometry_kind must be surface_ray or planar_rectangle"
            )
        if geometry_kind == "planar_rectangle" and center_offset != 0.0:
            raise VisionInputError(
                f"{class_id}.planar_rectangle must publish its visible surface without a ray offset"
            )
        specs.append(
            ClassSpec(
                class_id=class_id,
                hsv_ranges=tuple(ranges),
                min_pixels=min_pixels,
                max_pixels=max_pixels,
                min_aspect_ratio=min_aspect,
                max_aspect_ratio=max_aspect,
                center_offset_m=center_offset,
                geometry_kind=geometry_kind,
            )
        )
        seen.add(class_id)
    return tuple(specs)


def _combined_mask(hsv: np.ndarray, ranges: Iterable[ColorRange]) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for interval in ranges:
        partial = cv2.inRange(
            hsv,
            np.asarray(interval.lower, dtype=np.uint8),
            np.asarray(interval.upper, dtype=np.uint8),
        )
        mask = cv2.bitwise_or(mask, partial)
    kernel = np.ones((3, 3), dtype=np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)


def detect_components(
    bgr_image: np.ndarray,
    specs: Iterable[ClassSpec],
    *,
    valid_depth_mask: np.ndarray | None = None,
) -> tuple[DetectionCandidate, ...]:
    """Select at most one largest valid component for every requested class."""
    if not isinstance(bgr_image, np.ndarray) or bgr_image.ndim != 3 or bgr_image.shape[2] != 3:
        raise VisionInputError("BGR image must have shape HxWx3")
    if bgr_image.shape[0] < 1 or bgr_image.shape[1] < 1:
        raise VisionInputError("BGR image must be nonempty")
    if valid_depth_mask is not None:
        if not isinstance(valid_depth_mask, np.ndarray) or valid_depth_mask.shape != bgr_image.shape[:2]:
            raise VisionInputError("valid-depth mask must have the image height and width")
        valid_mask = valid_depth_mask.astype(bool)
    else:
        valid_mask = None
    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    candidates: list[DetectionCandidate] = []
    for spec in specs:
        mask = _combined_mask(hsv, spec.hsv_ranges)
        if valid_mask is not None:
            mask = np.where(valid_mask, mask, 0).astype(np.uint8)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        selected: tuple[np.ndarray, tuple[int, int, int, int], int] | None = None
        for contour in contours:
            component_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(component_mask, [contour], -1, 255, thickness=-1)
            pixel_count = int(cv2.countNonZero(component_mask))
            x_value, y_value, width, height = cv2.boundingRect(contour)
            if height <= 0:
                continue
            aspect = max(width / height, height / width)
            if (
                pixel_count < spec.min_pixels
                or pixel_count > spec.max_pixels
                or aspect < spec.min_aspect_ratio
                or aspect > spec.max_aspect_ratio
            ):
                continue
            if selected is None or pixel_count > selected[2]:
                selected = (component_mask, (x_value, y_value, width, height), pixel_count)
        if selected is None:
            continue
        component_mask, (x_value, y_value, width, height), pixel_count = selected
        moments = cv2.moments(component_mask, binaryImage=True)
        if moments["m00"] <= 0.0:
            continue
        center_u = float(moments["m10"] / moments["m00"])
        center_v = float(moments["m01"] / moments["m00"])
        confidence = min(1.0, 0.5 + 0.5 * pixel_count / (4.0 * spec.min_pixels))
        candidates.append(
            DetectionCandidate(
                class_id=spec.class_id,
                center_u_px=center_u,
                center_v_px=center_v,
                x_offset_px=x_value,
                y_offset_px=y_value,
                width_px=width,
                height_px=height,
                mask_pixel_count=pixel_count,
                confidence=confidence,
                mask=component_mask.astype(bool),
                center_offset_m=spec.center_offset_m,
                geometry_kind=spec.geometry_kind,
            )
        )
    return tuple(candidates)


def estimate_depth(depth_m: np.ndarray, mask: np.ndarray, *, minimum_samples: int = 20) -> DepthEstimate:
    """Return finite, positive masked-depth statistics in metres."""
    if not isinstance(depth_m, np.ndarray) or depth_m.ndim != 2:
        raise VisionInputError("depth image must have shape HxW")
    if not isinstance(mask, np.ndarray) or mask.shape != depth_m.shape:
        raise VisionInputError("depth mask must have the same shape as the depth image")
    values = np.asarray(depth_m[mask], dtype=np.float64)
    values = values[np.isfinite(values) & (values > 0.0)]
    if values.size < minimum_samples:
        raise VisionInputError(
            f"component has {values.size} valid depth samples; requires {minimum_samples}"
        )
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    if not math.isfinite(median) or median <= 0.0 or not math.isfinite(mad):
        raise VisionInputError("depth estimate is not finite and positive")
    return DepthEstimate(depth_m=median, depth_mad_m=mad, valid_sample_count=int(values.size))


def project_pixel(
    center_u_px: float, center_v_px: float, depth_m: float, camera_k: Iterable[float]
) -> tuple[float, float, float]:
    """Back-project an optical-frame XYZ using the observed CameraInfo K."""
    values = tuple(float(value) for value in camera_k)
    if len(values) != 9 or not all(math.isfinite(value) for value in values):
        raise VisionInputError("CameraInfo K must contain nine finite values")
    fx, cx, fy, cy = values[0], values[2], values[4], values[5]
    if fx <= 0.0 or fy <= 0.0:
        raise VisionInputError("CameraInfo focal lengths must be positive")
    u_value = _finite_float(center_u_px, "center_u_px")
    v_value = _finite_float(center_v_px, "center_v_px")
    z_value = _finite_float(depth_m, "depth_m", minimum=0.0)
    if z_value <= 0.0:
        raise VisionInputError("depth_m must be positive")
    return ((u_value - cx) * z_value / fx, (v_value - cy) * z_value / fy, z_value)


def estimate_planar_rectangle_center(
    depth_m: np.ndarray,
    mask: np.ndarray,
    camera_k: Iterable[float],
    *,
    minimum_samples: int = 100,
) -> tuple[float, float, float]:
    """Recover a rectangular planar surface center from its RGB-D point set.

    This calculation uses only the detected component, its observed depth, and
    the supplied CameraInfo. PCA establishes the two visual plane axes; robust
    edge extents then remove the perspective-density bias of an image centroid.
    """
    if not isinstance(depth_m, np.ndarray) or depth_m.ndim != 2:
        raise VisionInputError("depth image must have shape HxW")
    if not isinstance(mask, np.ndarray) or mask.shape != depth_m.shape:
        raise VisionInputError("planar mask must have the same shape as depth")
    values = tuple(float(value) for value in camera_k)
    if len(values) != 9 or not all(math.isfinite(value) for value in values):
        raise VisionInputError("CameraInfo K must contain nine finite values")
    fx, cx, fy, cy = values[0], values[2], values[4], values[5]
    if fx <= 0.0 or fy <= 0.0:
        raise VisionInputError("CameraInfo focal lengths must be positive")
    rows, columns = np.where(mask & np.isfinite(depth_m) & (depth_m > 0.0))
    if rows.size < minimum_samples:
        raise VisionInputError(
            f"planar component has {rows.size} valid samples; requires {minimum_samples}"
        )
    z_value = depth_m[rows, columns].astype(np.float64)
    points = np.column_stack(
        (
            (columns.astype(np.float64) - cx) * z_value / fx,
            (rows.astype(np.float64) - cy) * z_value / fy,
            z_value,
        )
    )
    centroid = np.mean(points, axis=0)
    _, singular_values, right_vectors = np.linalg.svd(points - centroid, full_matrices=False)
    if singular_values.size < 3 or singular_values[1] <= 1e-8:
        raise VisionInputError("planar component has insufficient two-dimensional geometry")
    plane_axes = right_vectors[:2]
    coordinates = (points - centroid) @ plane_axes.T
    lower = np.quantile(coordinates, 0.01, axis=0)
    upper = np.quantile(coordinates, 0.99, axis=0)
    center = centroid + plane_axes.T @ ((lower + upper) / 2.0)
    result = tuple(float(value) for value in center)
    if not all(math.isfinite(value) for value in result):
        raise VisionInputError("planar center is not finite")
    return result


def apply_center_offset(point_xyz: Iterable[float], offset_m: float) -> tuple[float, float, float]:
    """Move a visible-surface point along its observed optical viewing ray."""
    x_value, y_value, z_value = (float(value) for value in point_xyz)
    offset = _finite_float(offset_m, "center_offset_m", minimum=0.0)
    norm = math.sqrt(x_value * x_value + y_value * y_value + z_value * z_value)
    if not math.isfinite(norm) or norm <= 0.0:
        raise VisionInputError("camera point has no finite positive viewing ray")
    scale = offset / norm
    return (x_value + scale * x_value, y_value + scale * y_value, z_value + scale * z_value)


def annotate_image(bgr_image: np.ndarray, candidates: Iterable[DetectionCandidate]) -> np.ndarray:
    """Draw visual output provenance without changing the source observation."""
    annotated = bgr_image.copy()
    for candidate in candidates:
        top_left = (candidate.x_offset_px, candidate.y_offset_px)
        bottom_right = (
            candidate.x_offset_px + candidate.width_px,
            candidate.y_offset_px + candidate.height_px,
        )
        cv2.rectangle(annotated, top_left, bottom_right, (255, 255, 255), 1)
        cv2.circle(
            annotated,
            (round(candidate.center_u_px), round(candidate.center_v_px)),
            3,
            (255, 255, 255),
            -1,
        )
        cv2.putText(
            annotated,
            candidate.class_id,
            (candidate.x_offset_px, max(12, candidate.y_offset_px - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return annotated
