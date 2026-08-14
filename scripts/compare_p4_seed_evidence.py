#!/usr/bin/env python3
"""Prove two completed vision-only Phase-4 runs used different live green poses."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from typing import Any


class ComparisonError(RuntimeError):
    """Raised when either completed seed evidence is incomplete or inconsistent."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def load_json(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file():
        raise ComparisonError(f"{label} is missing: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ComparisonError(f"cannot parse {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise ComparisonError(f"{label} must be a JSON object")
    return payload


def vector(value: Any, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ComparisonError(f"{label} must be a three-value list")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ComparisonError(f"{label} has a non-finite value")
    return result


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_path = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
    finally:
        try:
            os.unlink(temporary_path)
        except FileNotFoundError:
            pass


def run_summary(root: Path) -> dict[str, Any]:
    metadata: dict[str, str] = {}
    metadata_path = root / "run_metadata.txt"
    if not metadata_path.is_file():
        raise ComparisonError(f"run metadata is missing: {metadata_path}")
    for line in metadata_path.read_text(encoding="utf-8").splitlines():
        if "=" not in line:
            raise ComparisonError(f"invalid metadata line in {metadata_path}: {line!r}")
        key, value = line.split("=", 1)
        metadata[key] = value
    seed_text = metadata.get("P4_SCENE_SEED")
    if seed_text is None or not seed_text.isdecimal():
        raise ComparisonError(f"run metadata has no valid P4_SCENE_SEED: {metadata_path}")
    comparison = load_json(
        root / "offline_gt" / "offline_ground_truth_comparison.json",
        "offline comparison",
    )
    if comparison.get("status", {}).get("p4_offline_evaluation") != "PASS":
        raise ComparisonError(f"offline comparison did not pass: {root}")
    if comparison.get("status", {}).get("a03_green_ball_vision_localization") != "PASS":
        raise ComparisonError(f"A03 did not pass in run: {root}")
    live = load_json(root / "offline_gt" / "live_ground_truth.json", "live-pose evidence")
    green = vector(live.get("models", {}).get("green_ball", {}).get("position_m"), "live green position")
    return {"root": str(root.resolve()), "seed": int(seed_text), "green_live_position_m": green}


def build_comparison(first_root: Path, second_root: Path) -> dict[str, Any]:
    first = run_summary(first_root)
    second = run_summary(second_root)
    if first["seed"] == second["seed"]:
        raise ComparisonError("the two completed runs use the same selected seed")
    delta = math.sqrt(
        sum(
            (first["green_live_position_m"][index] - second["green_live_position_m"][index]) ** 2
            for index in range(3)
        )
    )
    if delta < 0.05:
        raise ComparisonError(
            f"live green positions vary only {delta:.6f} m; requires at least 0.05 m"
        )
    return {
        "schema_version": 1,
        "scope": "SIMULATION_ONLY",
        "compared_at_utc": utc_now(),
        "first_run": first,
        "second_run": second,
        "green_live_position_delta_m": delta,
        "status": {
            "two_distinct_selected_seeds": "PASS",
            "green_live_position_varies": "PASS",
            "two_seed_vision_only_localization": "PASS",
            "a03_green_ball_vision_localization": "PASS",
        },
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--first-run", required=True, type=Path)
    parser.add_argument("--second-run", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser


def main() -> int:
    arguments = build_parser().parse_args()
    try:
        result = build_comparison(arguments.first_run, arguments.second_run)
        atomic_write_json(arguments.output, result)
    except (ComparisonError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print("P4_TWO_SEED_VISION_ONLY_LOCALIZATION=PASS")
    print("A03_GREEN_BALL_VISION_LOCALIZATION=PASS")
    print(f"P4_SEED_COMPARISON={arguments.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
