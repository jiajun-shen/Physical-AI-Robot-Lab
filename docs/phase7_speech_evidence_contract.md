# Phase 7 — zh/en/fr Speech Evidence Contract

Phase 7 accepts microphone audio only; typed text is useful for unit tests but cannot pass A06-A08.

Every accepted run must save: audio artifact/checksum, acquisition timestamp, backend/model identity, raw transcript, detected language, confidence, normalized intent, and downstream command result. Supported language labels are `zh`, `en`, and `fr`. A confidence below 0.60 is rejected as `UNCERTAIN_SPEECH`; unknown intent is rejected rather than guessed.

Minimum microphone cases:

- Chinese: a spoken command that normalizes to one supported robot intent.
- English: an equivalent supported command.
- French: an equivalent supported command.

A06/A07/A08 remain `NOT STARTED` until those three microphone runs exist. A text-only transcript or fabricated confidence cannot change acceptance status.
