"""Regression checks for PowerShell-5-safe recorder stderr handling.

``Process.ErrorDataReceived`` executes its handler on a .NET worker thread.
Windows PowerShell 5 script blocks are bound to a PowerShell runspace and must
not be used as that callback: when ffmpeg first writes to stderr, PowerShell
can raise ``PSInvalidOperation`` / ``ScriptBlock.GetContextFromTLS`` and close
the recorder UI. stderr may either remain unredirected, or be drained through
a .NET task started from the UI thread and consumed only after ffmpeg exits.
"""

from __future__ import annotations

from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "open_microphone_recorder.ps1"


def test_recorder_never_runs_a_powershell_scriptblock_on_ffmpeg_stderr_thread():
    source = SCRIPT.read_text(encoding="utf-8")

    forbidden_async_callback_tokens = (
        "ErrorDataReceived",
        "DataReceivedEventHandler",
        "BeginErrorReadLine",
        "CancelErrorRead",
    )
    present = [token for token in forbidden_async_callback_tokens if token in source]
    assert not present, (
        "ffmpeg stderr must not invoke a PowerShell ScriptBlock on a worker "
        f"thread; remove: {', '.join(present)}"
    )

    if "RedirectStandardError = $true" in source:
        assert "StandardError.ReadToEndAsync()" in source
        assert "GetAwaiter().GetResult()" in source
    else:
        assert "RedirectStandardError = $false" in source, (
            "stderr must either remain unredirected or be safely drained by "
            "a .NET task"
        )
