# Architecture decision records

Format follows cortex `docs/adr/template.md`: Status / Date / Related, Context, Decision, Consequences, and a closing *Current state* line. Files are numbered: `NNNN-short-title.md`.

scout-drone was created by two decisions recorded in racing-drone:
- [ADR 0007](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0007-outdoor-obstacle-avoidance-placement.md): outdoor obstacle avoidance lives in the platform repos and its own mission repo, this one;
- [ADR 0008](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0008-anatomy-mission-framework.md): the mission framework moves to anatomy, and scout-drone is its second client.

Decisions about scout-drone itself are recorded here.

| ADR | Decision | Status |
|---|---|---|
| [0001](0001-third-party-license-compatibility.md) | Third-party license compatibility (MIT; permissive dependencies only) | Accepted |
| [0002](0002-no-gpl-clean-room-policy.md) | No GPL code for any purpose; clean-room from papers | Accepted |
