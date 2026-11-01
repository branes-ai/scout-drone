# ADR 0002: no GPL code; clean-room implementation from papers

**Status:** Accepted
**Date:** 2026-10-31
**Source:** the stack's no-GPL policy, set in cortex's [ADR 0002](https://github.com/branes-ai/cortex/blob/main/docs/adr/0002-no-gpl-clean-room-policy.md)
**Related:** [ADR 0001](0001-third-party-license-compatibility.md)

## Context

cortex's ADR 0002 rules out GPL code anywhere in cortex, for any purpose. The reason applies to scout-drone unchanged: partners redistribute the stack inside their own certified products, and any GPL obligation, even one confined to a test, is something their legal reviews would have to clear.

scout-drone's task comes from Loquercio, Kaufmann et al., "Learning high-speed flight in the wild" (*Science Robotics*, 2021). That paper's code, UZH's `agile_autonomy`, is GPL-3.0. So are other UZH agile-flight projects (`rpg_mpc`, `rpg_quadrotor_mpc`, `rpg_time_optimal`), and several of the open flight stacks:
- Betaflight, INAV and Cleanflight (GPL-3.0)
- ArduPilot (GPL-3.0)
- Paparazzi (GPL-2.0)

Permissive references exist too: PX4 (BSD-3-Clause) and UZH's `agile_flight` (MIT). The task and its score come from the paper, which is not code.

## Decision

**No GPL-licensed code anywhere in scout-drone, for any purpose.** Not as a dependency, not in a test target, not as a build-time tool in what scout-drone builds, and not as a reference oracle. The scope is ADR 0001's: what scout-drone ships, the code that builds it and the tests, and not the docs site's toolchain.

- Everything is implemented **clean-room from papers, textbooks and specifications**, plus permissively licensed (MIT / BSD / Apache-2.0) references.
- **Check a reference's license before opening its code.** Reading GPL source and then writing the same thing "from scratch" is not clean-room. A GPL project's structure may be consulted, meaning its directory layout and how it splits the problem into modules, and nothing else: never its code, its algorithms' details, or its data, which includes tuning tables, gains, parameters and trained weights.
- **Test oracles are in-house or permissive:** scout-drone's own fields and simulator, flights it recorded itself, the results published in the paper, and permissive code such as PX4. Never compare against the output of a GPL program (`agile_autonomy`, Betaflight, ArduPilot, …), its simulator or its logs.
- **Conventions are not code.** Following a convention that a GPL project also uses (a frame, a rotor order, a unit) is fine, when it is taken from a permissive source or a specification.
- **LGPL, MPL, AGPL and SSPL code is rejected,** as a dependency and as a reference, as ADR 0001 rejects it. An exception would need an ADR of its own that supersedes this one.

## Consequences

- Each component costs more to build: the design comes from the literature, and validation uses scout-drone's own simulation and permissive references.
- A paper's results can be compared against; its GPL code cannot be read or run.
- Partners can redistribute scout-drone under MIT terms with no copyleft review.
- **Current state:** scout-drone's field, scorer and baselines are written in-house. No GPL code or oracle is used, and `agile_autonomy`'s code has not been opened.
