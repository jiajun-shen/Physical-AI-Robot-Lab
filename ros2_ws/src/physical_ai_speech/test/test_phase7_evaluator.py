from __future__ import annotations

import importlib.util
from pathlib import Path


def _load_evaluator():
    evaluator_path = Path(__file__).resolve().parents[4] / "scripts" / "evaluate_p7_speech_evidence.py"
    specification = importlib.util.spec_from_file_location("phase7_evaluator", evaluator_path)
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def test_evaluator_accepts_spoken_chinese_circle_alias():
    evaluator = _load_evaluator()

    evaluator.validate_phrase("zh", "画一个圈")
