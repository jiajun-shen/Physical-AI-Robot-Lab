# Acceptance Status

Only an observed successful run can be marked `PASS`. Source code presence or
an expected result is insufficient.

| ID | Acceptance target | Status | Evidence |
|---|---|---|---|
| A01 | Launch Gazebo and observe ABB IRB1200 spawned normally | NOT STARTED | None |
| A02 | Execute HOME → specified Pose → HOME with no collision | NOT STARTED | None |
| A03 | Randomize green sphere pose and localize it in vision mode without ground truth | NOT STARTED | None |
| A04 | Green sphere is picked and placed next to red sphere over multiple seeds | NOT STARTED | None |
| A05 | Robot draws a heart from a generated Cartesian mathematical curve | NOT STARTED | None |
| A06 | Chinese microphone command executes | NOT STARTED | None |
| A07 | English microphone command executes | NOT STARTED | None |
| A08 | French microphone command executes | NOT STARTED | None |
| A09 | Long Chinese command becomes multiple verified sequential tasks | NOT STARTED | None |
| A10 | Moved target is re-observed and replanned rather than using stale pose | NOT STARTED | None |
| A11 | System Explorer shows end-to-end data/execution chain | NOT STARTED | None |
| A12 | VLA starts when supported, otherwise explicit unavailable + safe fallback | NOT STARTED | None |
| A13 | ABB demonstration episode is collected and converted to LeRobot format | NOT STARTED | None |
| A14 | openpi training config/dry-run validates; optional small overfit if feasible | NOT STARTED | None |

## Evidence required when a status changes

```text
Status:
Tested commit:
Date/time:
Environment:
Brain/perception mode:
Seed(s):
Command:
Observed result:
Evidence/log/artifact:
Known limitation:
Next exact step:
```

For A14, configuration/dry-run validation is mandatory. The subset overfit is a
separate conditional result and may remain unavailable when hardware does not
permit it.
