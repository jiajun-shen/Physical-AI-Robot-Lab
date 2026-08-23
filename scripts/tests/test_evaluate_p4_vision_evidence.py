"""Pure-Python checks for the standalone Phase-4 evidence evaluator."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "evaluate_p4_vision_evidence.py"
SPEC = importlib.util.spec_from_file_location("evaluate_p4_vision_evidence", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
EVALUATOR = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = EVALUATOR
SPEC.loader.exec_module(EVALUATOR)


CLASSES = tuple(EVALUATOR.THRESHOLDS_M)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True), encoding="utf-8")


def _world_pose(class_id: str, position_m: list[float]) -> dict[str, object]:
    return {
        "class_id": class_id,
        "source_mode": 1,
        "header": {"frame_id": "world"},
        "source_header": {"frame_id": "lab_camera_optical_frame"},
        "position_m": position_m,
    }


def _write_fixture(
    root: Path,
    *,
    vision_positions: dict[str, list[float]],
    live_models: dict[str, dict[str, object]],
) -> tuple[Path, Path]:
    evidence_root = root / "evidence"
    outputs_path = evidence_root / "vision_outputs.json"
    _write_json(
        outputs_path,
        {
            "world_poses": [
                _world_pose(class_id, vision_positions[class_id]) for class_id in CLASSES
            ]
        },
    )
    vision_evidence = evidence_root / "vision_evidence.json"
    _write_json(
        vision_evidence,
        {
            "perception_mode": "vision",
            "ground_truth_input_used": False,
            "sealed_at_utc": "2026-08-15T00:00:00+00:00",
            "expected_classes": list(CLASSES),
            "artifact_sha256": {"evidence/vision_outputs.json": _sha256(outputs_path)},
        },
    )
    live_ground_truth = root / "offline_gt" / "live_ground_truth.json"
    _write_json(
        live_ground_truth,
        {
            "access_boundary": "OFFLINE_AFTER_SEALED_VISION_EVIDENCE",
            "sealed_vision_evidence_sha256": _sha256(vision_evidence),
            "capture_started_at_utc": "2026-08-15T00:00:01+00:00",
            "models": live_models,
        },
    )
    return vision_evidence, live_ground_truth


class EvaluateP4VisionEvidenceTests(unittest.TestCase):
    def _fixture(
        self, *, board_rpy_rad: list[float] | None = None
    ) -> tuple[tempfile.TemporaryDirectory[str], Path, Path]:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        root = Path(directory.name)
        board_center = [1.10, 0.20, 0.975]
        board_rpy = board_rpy_rad or [0.0, 0.0, 0.0]
        board_surface, _ = EVALUATOR.drawing_board_front_surface(board_center, board_rpy)
        positions = {
            "green_ball": [0.50, 0.0, 0.785],
            "red_ball": [0.60, 0.0, 0.785],
            "blue_cube": [0.70, -0.2, 0.78],
            "yellow_cube": [0.80, 0.2, 0.78],
            "drawing_board": board_surface,
        }
        live_models: dict[str, dict[str, object]] = {
            class_id: {"position_m": positions[class_id], "rpy_rad": [0.0, 0.0, 0.0]}
            for class_id in CLASSES
        }
        live_models["drawing_board"] = {
            "position_m": board_center,
            "rpy_rad": board_rpy,
        }
        vision, capture = _write_fixture(
            root, vision_positions=positions, live_models=live_models
        )
        return directory, vision, capture

    def test_drawing_board_uses_robot_facing_front_surface(self) -> None:
        _, vision, capture = self._fixture()
        result = EVALUATOR.build_evaluation(
            argparse.Namespace(vision_evidence=vision, live_ground_truth=capture)
        )
        board = result["results"]["drawing_board"]
        self.assertEqual(result["schema_version"], 2)
        self.assertEqual(board["reference"]["name"], "robot_facing_front_surface")
        self.assertEqual(board["reference"]["live_model_volume_center_m"], [1.10, 0.20, 0.975])
        self.assertEqual(board["reference"]["front_surface_normal_world"], [-1.0, -0.0, 0.0])
        self.assertAlmostEqual(board["live_position_m"][0], 1.085)
        self.assertAlmostEqual(board["live_position_m"][1], 0.20)
        self.assertAlmostEqual(board["live_position_m"][2], 0.975)
        self.assertEqual(board["position_error_m"], 0.0)
        self.assertEqual(
            result["results"]["green_ball"]["reference"]["name"], "model_volume_center"
        )

    def test_board_surface_rotates_with_live_model_pose(self) -> None:
        _, vision, capture = self._fixture(board_rpy_rad=[0.0, 0.0, 1.5707963267948966])
        result = EVALUATOR.build_evaluation(
            argparse.Namespace(vision_evidence=vision, live_ground_truth=capture)
        )
        board = result["results"]["drawing_board"]
        self.assertAlmostEqual(board["live_position_m"][0], 1.10)
        self.assertAlmostEqual(board["live_position_m"][1], 0.185)
        self.assertAlmostEqual(board["reference"]["front_surface_normal_world"][0], 0.0)
        self.assertAlmostEqual(board["reference"]["front_surface_normal_world"][1], -1.0)

    def test_board_without_live_orientation_fails_closed(self) -> None:
        _, vision, capture = self._fixture()
        payload = json.loads(capture.read_text(encoding="utf-8"))
        del payload["models"]["drawing_board"]["rpy_rad"]
        _write_json(capture, payload)
        with self.assertRaisesRegex(EVALUATOR.EvaluationError, "drawing_board live model RPY"):
            EVALUATOR.build_evaluation(
                argparse.Namespace(vision_evidence=vision, live_ground_truth=capture)
            )


if __name__ == "__main__":
    unittest.main()
