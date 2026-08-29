# Phase 7 — Manual Microphone Recorder UI Contract

`scripts/open_microphone_recorder.ps1` is the canonical, project-wide entry
point for recording a command from the user's microphone. It is a visible,
user-operated recorder; it is not a background capture helper. Future voice
features must use this flow (or an equivalent UI that preserves this contract)
rather than starting a microphone process from an automation script.

For everyday use, double-click `scripts/open_microphone_recorder.cmd`. It
opens the same idle recorder window and does not start a capture by itself.

## Visible interaction

When the window opens it is in **Idle** state:

- it shows a physical microphone source selector/input and an external evidence
  output directory;
- it shows a visible **开始录音** / **Start recording** button, enabled;
- it shows a visible **停止录音** / **Stop recording** button, disabled; and
- it states that no recording is in progress.

The UI must not start ffmpeg, open a recording stream, create a WAV, or emit
successful capture metadata merely because the window was opened. A device
probe that does not read samples is permitted, but it must not be presented as
a recording.

Pressing **Start** is the user's explicit consent for that one capture and is
the only transition that may start the recorder. While active,
the window has a clear recording state/indicator and elapsed time; Start is
disabled and Stop is enabled. Pressing **Stop** is the normal completion
action. The Stop handler sends `q` to ffmpeg's standard input and waits for a
bounded graceful exit before the audio is finalized. A forced termination,
window-close interruption, or ffmpeg error is an incomplete capture: it must
not publish a `P7_MICROPHONE_CAPTURE=PASS` result or valid final metadata.

The UI displays a maximum session duration (at most 60 seconds) as a safety
bound. It does not start recording by itself and does not replace the
Start/Stop controls. The value is recorded as `duration_requested_sec`.

## Privacy and ownership

- There is no automatic, startup, scheduled, hidden, or background recording.
- Audio stays local. The recorder does not upload audio or use cloud ASR.
- The user chooses the physical microphone and output directory for each
  session. Monitor/loopback sources are refused for Phase 7 evidence.
- Audio is written outside the source repository. Temporary partial files are
  clearly named as incomplete and are never treated as evidence.
- The UI displays the completed audio path, metadata path, and SHA-256 so the
  user can inspect what was captured before running ASR.

## Completion artifacts and provenance

A successful Stop writes one `microphone.wav` and one
`capture_metadata.json` atomically in the chosen session directory. The WAV
is 16 kHz, mono, signed 16-bit PCM. The metadata is UTF-8 JSON and contains
the following required keys; their values describe the completed file, not a
typed command:

```json
{
  "schema_version": 1,
  "capture_method": "windows_directshow",
  "input_kind": "microphone",
  "source": "selected physical microphone name",
  "capture_backend": "ffmpeg_directshow",
  "audio_filename": "microphone.wav",
  "started_at_unix_ns": 0,
  "ended_at_unix_ns": 0,
  "duration_requested_sec": 60,
  "audio_sha256": "lowercase SHA-256",
  "audio_size_bytes": 0,
  "audio_format": {
    "container": "WAV",
    "sample_rate_hz": 16000,
    "channels": 1,
    "sample_width_bytes": 2,
    "frames": 0,
    "duration_sec": 0.0
  }
}
```

`audio_sha256`, size, frame count, and duration are calculated after the
gracefully completed WAV exists. Timestamps bracket the actual microphone
capture. The existing independent Phase-7 evaluator recomputes all of these
properties, so a copied, edited, silent, synthetic, typed, or stale artifact
cannot become microphone evidence just by editing JSON.

## Phase-7 use

From the repository root, open the recorder and select an external evidence
folder, for example one distinct folder each for `zh`, `en`, and `fr`:

```powershell
.\scripts\open_microphone_recorder.ps1 -OutputRoot C:\Physical-AI-evidence\phase7\zh
```

For each language, select the microphone, review the consent text, press
Start, speak the command, and press Stop yourself. Do not paste the command
into a terminal and do not reuse one WAV for multiple languages. Once all
three completed captures are visible, pass their audio and metadata paths to
`scripts/run_and_verify_phase7_speech.sh`. That pipeline performs local
Whisper transcription, language detection, `SpeechCommand` normalization, and
the existing Phase-6 drawing-intent parse; it does not take a manual transcript
as an acceptance input.

The static test `tests/test_open_microphone_recorder_ui_static.py` guards the
minimum UI and provenance contract. It is deliberately structural rather than
tied to a particular PowerShell variable name or GUI toolkit.
