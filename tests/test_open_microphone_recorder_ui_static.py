"""Static contract checks for the visible, manual microphone recorder.

The test intentionally inspects only the public script path and broad
PowerShell structures. It permits implementation refactors and GUI toolkit
choices while preventing a regression to a timed/background recorder.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "open_microphone_recorder.ps1"


@dataclass(frozen=True)
class Block:
    start: int
    end: int
    body: str


def _source() -> str:
    assert SCRIPT.is_file(), f"manual recorder script is missing: {SCRIPT}"
    return SCRIPT.read_text(encoding="utf-8")


def _matching_brace(source: str, opening: int) -> int:
    """Return the matching PowerShell script-block brace for simple static parsing."""
    assert source[opening] == "{"
    depth = 0
    quote: str | None = None
    index = opening
    while index < len(source):
        character = source[index]
        if quote is not None:
            if quote == '"' and character == "`":
                index += 2
                continue
            if character == quote:
                # A doubled single quote remains inside a PowerShell single-quoted string.
                if quote == "'" and index + 1 < len(source) and source[index + 1] == "'":
                    index += 2
                    continue
                quote = None
        elif character in {"'", '"'}:
            quote = character
        elif character == "{":
            depth += 1
        elif character == "}":
            depth -= 1
            if depth == 0:
                return index
        index += 1
    raise AssertionError("unclosed PowerShell script block")


def _blocks(source: str, pattern: str) -> list[Block]:
    blocks: list[Block] = []
    for match in re.finditer(pattern, source, re.IGNORECASE | re.MULTILINE):
        opening = source.find("{", match.start(), match.end() + 1)
        if opening < 0:
            continue
        closing = _matching_brace(source, opening)
        blocks.append(Block(match.start(), closing + 1, source[opening + 1 : closing]))
    return blocks


def _click_handlers(source: str) -> list[Block]:
    return _blocks(source, r"\.Add_Click\s*\(\s*\{")


def _functions(source: str) -> dict[str, Block]:
    found: dict[str, Block] = {}
    pattern = re.compile(
        r"^\s*function\s+(?P<name>[A-Za-z][A-Za-z0-9_-]*)\s*(?:\([^)]*\))?\s*\{",
        re.IGNORECASE | re.MULTILINE,
    )
    for match in pattern.finditer(source):
        opening = source.find("{", match.start(), match.end())
        closing = _matching_brace(source, opening)
        found[match.group("name")] = Block(match.start(), closing + 1, source[opening + 1 : closing])
    return found


def _remove_blocks(source: str, blocks: list[Block]) -> str:
    characters = list(source)
    for block in blocks:
        for index in range(block.start, block.end):
            if characters[index] != "\n":
                characters[index] = " "
    return "".join(characters)


def _handler_references(handler: Block, function_name: str) -> bool:
    return bool(
        re.search(
            rf"(?<![A-Za-z0-9_-]){re.escape(function_name)}(?![A-Za-z0-9_-])",
            handler.body,
            re.IGNORECASE,
        )
    )


def _contains_ffmpeg_launch(body: str) -> bool:
    """Recognize a recorder launch, without mistaking Explorer for ffmpeg."""
    process_start = re.compile(
        r"\bStart-Process\b|\[\s*(?:System\.Diagnostics\.)?Process\s*\]::Start\s*\(|\.Start\s*\(\s*\)",
        re.IGNORECASE,
    )
    return "ffmpeg" in body.casefold() and bool(process_start.search(body))


def test_recorder_has_visible_start_stop_controls_and_handlers():
    source = _source()
    assert re.search(r"\bButton\b", source, re.IGNORECASE), "recorder UI has no button control"
    assert re.search(
        r"['\"](?:开始录音|start(?:\s+recording)?)['\"]", source, re.IGNORECASE
    ), "visible Start recording caption is missing"
    assert re.search(
        r"['\"](?:停止录音|stop(?:\s+recording)?)['\"]", source, re.IGNORECASE
    ), "visible Stop recording caption is missing"
    assert len(_click_handlers(source)) >= 2, "Start and Stop must be wired to separate visible click handlers"


def test_recorder_launch_is_user_triggered_not_an_idle_startup_side_effect():
    source = _source()
    handlers = _click_handlers(source)
    functions = _functions(source)
    assert handlers, "recorder has no click handler through which a user can opt in"

    launcher_functions = {
        name: block for name, block in functions.items() if _contains_ffmpeg_launch(block.body)
    }
    directly_started = [handler for handler in handlers if _contains_ffmpeg_launch(handler.body)]
    indirectly_started = [
        handler
        for handler in handlers
        if any(_handler_references(handler, name) for name in launcher_functions)
    ]
    assert directly_started or indirectly_started, "no Start click path launches the recorder"

    # At file scope, a launch primitive would record as soon as the UI opens.
    # Function bodies and click handlers are removed first so helper definitions
    # are allowed; only an actual top-level launch is rejected.
    top_level = _remove_blocks(source, [*functions.values(), *handlers])
    assert not _contains_ffmpeg_launch(top_level), "recorder launch is reachable at startup rather than only through Start"
    assert re.search(
        r"\b(?:idle|ready|not recording)\b", source, re.IGNORECASE
    ), "UI does not advertise an initial non-recording state"


def test_stop_path_sends_q_to_ffmpeg_standard_input_for_graceful_finalization():
    source = _source()
    handlers = _click_handlers(source)
    functions = _functions(source)
    q_write = re.compile(
        r"(?:StandardInput|stdin|input)\s*\.\s*Write(?:Line)?\s*\(\s*['\"]q['\"]\s*\)",
        re.IGNORECASE,
    )
    graceful_helpers = {
        name: block for name, block in functions.items() if q_write.search(block.body)
    }
    assert q_write.search(source), "Stop must send literal q to ffmpeg standard input"
    assert any(q_write.search(handler.body) for handler in handlers) or any(
        _handler_references(handler, name) for handler in handlers for name in graceful_helpers
    ), "the graceful q stop path is not reachable from a visible Stop click handler"


def test_window_close_cannot_skip_pending_wav_finalization():
    source = _source()
    close_handlers = _blocks(source, r"\.Add_FormClosing\s*\(\s*\{")
    assert close_handlers, "recorder has no window-close handler"
    close_handler = close_handlers[-1]

    assert re.search(
        r"\$eventArgs\s*\.\s*Cancel\s*=\s*\$true", close_handler.body, re.IGNORECASE
    ), "window close must wait for a pending recorder to finalize"
    assert re.search(
        r"\$script:recorder\s*\.\s*HasExited", close_handler.body, re.IGNORECASE
    ), "window-close path does not handle the Stop/exit race"
    assert _handler_references(close_handler, "Finish-Recording"), (
        "an already-exited recorder must be finalized before the window can close"
    )


def test_completed_capture_metadata_keeps_the_phase7_provenance_contract():
    source = _source()
    required_fields = {
        "schema_version",
        "capture_method",
        "input_kind",
        "source",
        "capture_backend",
        "audio_filename",
        "started_at_unix_ns",
        "ended_at_unix_ns",
        "duration_requested_sec",
        "audio_sha256",
        "audio_size_bytes",
        "audio_format",
        "sample_rate_hz",
        "channels",
        "sample_width_bytes",
        "frames",
        "duration_sec",
    }
    missing = sorted(
        field for field in required_fields if not re.search(rf"\b{re.escape(field)}\b", source)
    )
    assert not missing, f"capture metadata omits required field(s): {', '.join(missing)}"
    assert re.search(
        r"['\"]windows_directshow['\"]", source, re.IGNORECASE
    ), "capture method must identify Windows DirectShow"
    assert re.search(
        r"['\"]ffmpeg_directshow['\"]", source, re.IGNORECASE
    ), "metadata must identify the ffmpeg DirectShow backend"
    assert not re.search(r"\bGet-FileHash\b", source, re.IGNORECASE), (
        "recorder must not depend on a PowerShell cmdlet that can be unavailable in its UI session"
    )
    assert re.search(
        r"function\s+Get-Sha256Hex\b[\s\S]{0,900}\bSecurity\.Cryptography\.SHA256\b",
        source,
        re.IGNORECASE,
    ), "recorder must implement SHA-256 with the .NET cryptography runtime"
    assert re.search(
        r"\baudio_sha256\s*=\s*Get-Sha256Hex\s+-Path\s+\$script:audioPath\b",
        source,
        re.IGNORECASE,
    ), "metadata must compute a SHA-256 from the completed WAV"
    assert re.search(
        r"\binput_kind\b[\s\S]{0,80}['\"]microphone['\"]", source, re.IGNORECASE
    ), "metadata must explicitly identify microphone input"
