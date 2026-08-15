"""Pure-Python checks for the standalone Phase-5 two-seed comparator."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "compare_p5_seed_evidence.py"
SPEC = importlib.util.spec_from_file_location("compare_p5_seed_evidence", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
COMPARATOR = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = COMPARATOR
SPEC.loader.exec_module(COMPARATOR)


P4_CLASSES = (
    "green_ball",
    "red_ball",
    "blue_cube",
    "yellow_cube",
    "drawing_board",
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True), encoding="utf-8")


def _header(stamp_ns: int, frame_id: str) -> dict[str, int | str]:
    return {
        "stamp_sec": stamp_ns // 1_000_000_000,
        "stamp_nanosec": stamp_ns % 1_000_000_000,
        "stamp_ns": stamp_ns,
        "frame_id": frame_id,
    }


def _pose(
    class_id: str,
    detection_id: str,
    position: list[float],
    stamp_ns: int,
    frame_id: str,
) -> dict[str, object]:
    return {
        "header": _header(stamp_ns, frame_id),
        "source_header": _header(stamp_ns, "lab_camera_optical_frame"),
        "detection_id": detection_id,
        "class_id": class_id,
        "position_m": position,
        "confidence": 0.95,
        "depth_m": 1.0,
        "depth_mad_m": 0.002,
        "valid_depth_samples": 32,
        "orientation_valid": False,
        "source_mode": 1,
    }


def _write_phase4_evidence(
    root: Path,
    *,
    stamp_ns: int,
    green_position: list[float],
    token: str,
) -> Path:
    evidence = root / "evidence"
    evidence.mkdir(parents=True, exist_ok=True)
    positions = {
        "green_ball": green_position,
        "red_ball": [0.70, 0.00, 0.785],
        "blue_cube": [0.52, 0.18, 0.79],
        "yellow_cube": [0.52, -0.18, 0.79],
        "drawing_board": [0.52, 0.30, 0.77],
    }
    detections: list[dict[str, object]] = []
    camera_poses: list[dict[str, object]] = []
    world_poses: list[dict[str, object]] = []
    for index, class_id in enumerate(P4_CLASSES):
        detection_id = f"{token}-{class_id}-{stamp_ns}-{index}"
        detections.append(
            {
                "header": _header(stamp_ns, "lab_camera_optical_frame"),
                "detection_id": detection_id,
                "class_id": class_id,
                "confidence": 0.95,
                "source_mode": 1,
            }
        )
        camera_poses.append(
            _pose(
                class_id,
                detection_id,
                positions[class_id],
                stamp_ns,
                "lab_camera_optical_frame",
            )
        )
        world_poses.append(_pose(class_id, detection_id, positions[class_id], stamp_ns, "world"))
    _write_json(
        evidence / "vision_outputs.json",
        {
            "detections": detections,
            "camera_poses": camera_poses,
            "world_poses": world_poses,
        },
    )
    for name, contents in (
        ("rgb.png", f"{token}:rgb".encode("utf-8")),
        ("annotated_rgb.png", f"{token}:annotated".encode("utf-8")),
        ("depth.npy", f"{token}:depth".encode("utf-8")),
        ("camera_info.json", f'{{"token":"{token}"}}'.encode("utf-8")),
        ("tf_chain.json", f'{{"token":"{token}"}}'.encode("utf-8")),
    ):
        (evidence / name).write_bytes(contents)
    artifact_hashes = {
        f"evidence/{name}": _sha256(evidence / name)
        for name in (
            "rgb.png",
            "annotated_rgb.png",
            "depth.npy",
            "camera_info.json",
            "tf_chain.json",
            "vision_outputs.json",
        )
    }
    evidence_path = evidence / "vision_evidence.json"
    _write_json(
        evidence_path,
        {
            "schema_version": 1,
            "scope": "SIMULATION_ONLY",
            "sealed_at_utc": "2026-08-15T00:00:00+00:00",
            "perception_mode": "vision",
            "ground_truth_input_used": False,
            "expected_classes": list(P4_CLASSES),
            "source_header": _header(stamp_ns, "lab_camera_optical_frame"),
            "minimum_source_stamp_ns": stamp_ns - 1,
            "artifact_sha256": artifact_hashes,
            "status": {
                "p4_rgb_transport": "PASS",
                "p4_depth_transport": "PASS",
                "p4_camera_info": "PASS",
                "p4_tf_chain": "PASS",
                "p4_all_five_typed_outputs": "PASS",
                "p4_vision_input_boundary": "PASS",
            },
        },
    )
    return evidence_path


def _write_execution(path: Path, *, seed: int, token: str) -> Path:
    _write_json(
        path,
        {
            "schema_version": 1,
            "scope": "SIMULATION_ONLY",
            "seed": seed,
            "run_token": token,
            "online_ground_truth_input_used": False,
            "status": "PENDING_POST_PLACE_VISION_EVALUATION",
        },
    )
    return path


def _write_evaluator(
    path: Path,
    *,
    seed: int,
    pre_pick: Path,
    execution: Path,
    post_place: Path,
) -> Path:
    _write_json(
        path,
        {
            "schema_version": 1,
            "scope": "SIMULATION_ONLY",
            "evaluation_mode": "OFFLINE_EVIDENCE_VALIDATION_ONLY",
            "seed": seed,
            "inputs": {
                "pre_pick_evidence": {
                    "path": str(pre_pick.resolve()),
                    "sha256": _sha256(pre_pick),
                },
                "execution_evidence": {
                    "path": str(execution.resolve()),
                    "sha256": _sha256(execution),
                },
                "post_place_evidence": {
                    "path": str(post_place.resolve()),
                    "sha256": _sha256(post_place),
                },
            },
            "markers": {marker: "PASS" for marker in COMPARATOR.P5_EVALUATOR_MARKERS},
            "status": "PASS",
        },
    )
    return path


def _read_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def _make_run(root: Path, *, name: str, seed: int, green_position: list[float]) -> Path:
    run = root / name
    run.mkdir(parents=True, exist_ok=True)
    (run / "run_metadata.txt").write_text(
        f"P5_SCENE_SEED={seed}\nGZ_PARTITION=physical_ai_p5_test_{name}\nHARNESS_RUN_ID={name}\n",
        encoding="utf-8",
    )
    pre_pick = _write_phase4_evidence(
        run / "pre_pick",
        stamp_ns=5_000_000_000 + seed,
        green_position=green_position,
        token=f"{name}-pre",
    )
    execution = _write_execution(
        run / "execution" / "execution_evidence.json",
        seed=seed,
        token=name,
    )
    post_place = _write_phase4_evidence(
        run / "post_place",
        stamp_ns=17_000_000_000 + seed,
        green_position=[0.65, 0.00, 0.785],
        token=f"{name}-post",
    )
    _write_evaluator(
        run / "evidence" / "p5_evaluation.json",
        seed=seed,
        pre_pick=pre_pick,
        execution=execution,
        post_place=post_place,
    )
    return run


def _update_evaluator_input_hash(run: Path, name: str) -> None:
    pre_pick, execution, post_place, evaluation = COMPARATOR.expected_paths(run)
    source = {
        "pre_pick_evidence": pre_pick,
        "execution_evidence": execution,
        "post_place_evidence": post_place,
    }[name]
    payload = _read_json(evaluation)
    inputs = payload["inputs"]
    assert isinstance(inputs, dict)
    record = inputs[name]
    assert isinstance(record, dict)
    record["path"] = str(source.resolve())
    record["sha256"] = _sha256(source)
    _write_json(evaluation, payload)


class CompareP5SeedEvidenceTests(unittest.TestCase):
    def test_distinct_complete_runs_pass_and_write_result(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = _make_run(
                root,
                name="seed42",
                seed=42,
                green_position=[0.40, 0.00, 0.785],
            )
            second = _make_run(
                root,
                name="seed43",
                seed=43,
                green_position=[0.46, 0.00, 0.785],
            )

            result = COMPARATOR.build_comparison(first, second)
            self.assertEqual(result["status"], "PASS")
            self.assertEqual(
                result["markers"]["P5_TWO_SEED_PICK_PLACE_EVIDENCE"], "PASS"
            )
            self.assertEqual(result["first_run"]["seed"], 42)
            self.assertEqual(result["second_run"]["seed"], 43)
            self.assertGreaterEqual(result["pre_pick_green_vision_position_delta_m"], 0.05)

            output = root / "p5_two_seed_comparison.json"
            self.assertEqual(
                COMPARATOR.main(
                    [
                        "--first-run",
                        str(first),
                        "--second-run",
                        str(second),
                        "--output",
                        str(output),
                    ]
                ),
                0,
            )
            self.assertEqual(_read_json(output)["status"], "PASS")

    def test_same_selected_seed_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = _make_run(
                root,
                name="seed42a",
                seed=42,
                green_position=[0.40, 0.00, 0.785],
            )
            second = _make_run(
                root,
                name="seed42b",
                seed=42,
                green_position=[0.46, 0.00, 0.785],
            )
            with self.assertRaisesRegex(COMPARATOR.ComparisonError, "same selected seed"):
                COMPARATOR.build_comparison(first, second)

    def test_execution_seed_must_match_run_metadata_and_evaluator(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = _make_run(
                root,
                name="seed42",
                seed=42,
                green_position=[0.40, 0.00, 0.785],
            )
            second = _make_run(
                root,
                name="seed43",
                seed=43,
                green_position=[0.46, 0.00, 0.785],
            )
            _, execution, _, _ = COMPARATOR.expected_paths(second)
            execution_payload = _read_json(execution)
            execution_payload["seed"] = 42
            _write_json(execution, execution_payload)
            _update_evaluator_input_hash(second, "execution_evidence")

            with self.assertRaisesRegex(
                COMPARATOR.ComparisonError, "execution seed disagrees with P5_SCENE_SEED"
            ):
                COMPARATOR.build_comparison(first, second)

    def test_reused_gazebo_partition_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = _make_run(
                root,
                name="seed42",
                seed=42,
                green_position=[0.40, 0.00, 0.785],
            )
            second = _make_run(
                root,
                name="seed43",
                seed=43,
                green_position=[0.46, 0.00, 0.785],
            )
            second_metadata = second / "run_metadata.txt"
            second_metadata.write_text(
                "P5_SCENE_SEED=43\nGZ_PARTITION=physical_ai_p5_test_seed42\nHARNESS_RUN_ID=seed43\n",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(COMPARATOR.ComparisonError, "same GZ_PARTITION"):
                COMPARATOR.build_comparison(first, second)

    def test_nonpassing_p5_evaluator_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = _make_run(
                root,
                name="seed42",
                seed=42,
                green_position=[0.40, 0.00, 0.785],
            )
            second = _make_run(
                root,
                name="seed43",
                seed=43,
                green_position=[0.46, 0.00, 0.785],
            )
            evaluation = second / "evidence" / "p5_evaluation.json"
            payload = _read_json(evaluation)
            markers = payload["markers"]
            assert isinstance(markers, dict)
            markers["P5_POST_PLACE"] = "FAIL"
            _write_json(evaluation, payload)

            with self.assertRaisesRegex(COMPARATOR.ComparisonError, "P5_POST_PLACE"):
                COMPARATOR.build_comparison(first, second)

    def test_reused_post_place_hash_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = _make_run(
                root,
                name="seed42",
                seed=42,
                green_position=[0.40, 0.00, 0.785],
            )
            second = _make_run(
                root,
                name="seed43",
                seed=43,
                green_position=[0.46, 0.00, 0.785],
            )
            _, _, first_post, _ = COMPARATOR.expected_paths(first)
            _, _, second_post, _ = COMPARATOR.expected_paths(second)
            for name in (
                "vision_evidence.json",
                "rgb.png",
                "annotated_rgb.png",
                "depth.npy",
                "camera_info.json",
                "tf_chain.json",
                "vision_outputs.json",
            ):
                (second_post.parent / name).write_bytes((first_post.parent / name).read_bytes())
            _update_evaluator_input_hash(second, "post_place_evidence")

            with self.assertRaisesRegex(
                COMPARATOR.ComparisonError, "same post-place-evidence hash"
            ):
                COMPARATOR.build_comparison(first, second)

    def test_insufficient_vision_green_delta_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = _make_run(
                root,
                name="seed42",
                seed=42,
                green_position=[0.40, 0.00, 0.785],
            )
            second = _make_run(
                root,
                name="seed43",
                seed=43,
                green_position=[0.449, 0.00, 0.785],
            )
            with self.assertRaisesRegex(COMPARATOR.ComparisonError, "requires at least 0.05"):
                COMPARATOR.build_comparison(first, second)

    def test_exact_minimum_vision_green_delta_is_accepted(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = _make_run(
                root,
                name="seed42",
                seed=42,
                green_position=[0.40, 0.00, 0.785],
            )
            second = _make_run(
                root,
                name="seed43",
                seed=43,
                green_position=[0.45, 0.00, 0.785],
            )
            result = COMPARATOR.build_comparison(first, second)
            self.assertEqual(result["status"], "PASS")
            self.assertGreaterEqual(
                result["pre_pick_green_vision_position_delta_m"] + 1e-12, 0.05
            )

    def test_tampered_pre_pick_vision_artifact_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = _make_run(
                root,
                name="seed42",
                seed=42,
                green_position=[0.40, 0.00, 0.785],
            )
            second = _make_run(
                root,
                name="seed43",
                seed=43,
                green_position=[0.46, 0.00, 0.785],
            )
            tampered_outputs = second / "pre_pick" / "evidence" / "vision_outputs.json"
            tampered_outputs.write_text('{"world_poses":[]}', encoding="utf-8")

            with self.assertRaisesRegex(
                COMPARATOR.ComparisonError, "artifact hash does not match: evidence/vision_outputs.json"
            ):
                COMPARATOR.build_comparison(first, second)


if __name__ == "__main__":
    unittest.main()
