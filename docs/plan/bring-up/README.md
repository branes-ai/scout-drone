---
status: ready
issue: branes-ai/scout-drone#2
---
# Plan: bring up scout-drone on anatomy (S0)

Step 3 of racing-drone's [ADR 0008](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0008-anatomy-mission-framework.md): scout-drone becomes anatomy's second client. ADR 0008 says, "each assumption it exposes becomes a change in anatomy, checked against both clients." The mission is [ADR 0007](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0007-outdoor-obstacle-avoidance-placement.md)'s: reach a goal through static clutter the vehicle has not seen.

## Goal

1. **scout-drone is a mission repo like racing-drone,** built on anatomy v0.4.0 (core, mobility, glue, mobility_sim): its build, CI, releases, docs and docs site.
2. **It flies its first mission, S0:** from a start pose to a goal through a field of static obstacles. The stack sees the obstacles through an oracle sensor computed from truth.
3. **Every anatomy assumption S0 runs into is fixed in anatomy,** with racing-drone's outputs byte-identical.

The algorithms of ADR 0007 (stereo depth, the learned planner, the obstacle CBF) aren't part of this plan. They belong in cortex, synapse and reflex, and come later.

## What exists

- **scout-drone:** a README, the licence, and the records check (`records.yml`, scout-drone#1). There's no code yet.
- **anatomy v0.4.0** gives a roaming mission:
  - the mobility profile (`MobilityProfile`, `TickOutput`, `MobileStack`);
  - the four neutral slots with their nulls, `GroundTruthEstimator`'s core and `CascadedPid`;
  - slot construction over the mission's world (`BasicSlotContext<World>`, which needs `World::start`);
  - the composition generator (`cmake/composition.cmake`);
  - the scenario loader over a mission's `ScenarioSchema` (its world key and its own `sim:` keys);
  - the closed-loop simulator, `BasicSimulator<Oracle>`, with the mission's `OracleSensor`;
  - `run_closed_loop` over a mission's `RunScorer`, the batch statistics, the harness, replay and the log.
- **anatomy's own test mission** (`tests/sim/neutral_mission.cpp`) already flies a made-up scouting world, a goal with no obstacles, through these pieces. S0 makes it real.
- **racing-drone** is the model for the layout. Its mission part, about 3,000 lines, is the course world, three task slots, the `Stack`, the gate oracle, the scorer, its channels, the apps and the baseline generator. It pins anatomy v0.3.0 and reaches it through generated bridge headers that alias anatomy's names into `branes::racing`.

## The mission

**The world, `Field`** (`fields/<name>.yaml`):
- the start pose, and a geofence;
- the goal: a position and a radius;
- the obstacles: vertical cylinders, each a centre on the ground, a radius and a height. A field lists its obstacles, or generates them from the scenario's seed: a density, a radius range, and clear zones around the start and the goal.

**The oracle, `ObstacleOracle`:** at each camera frame, the obstacles within a sensing range of the true position, in the body frame, as `ObstacleObservations` on scout-drone's own channel `oracle_obstacles`. The scenario sets the range and the noise (its `sim:` keys).

**The slots.** The four neutral slots come from anatomy (controller, safety filter, DAA tracker, FC bridge). scout-drone defines the two that see the task:
- `Estimator`: IMU and frames, as in racing, with the obstacle observations where racing has gate observations.
- `Planner`: the field's goal, the latest obstacle observations, and the state estimate in; a `Reference` out.

There's no gate detector. The oracle stands in for perception until cortex's depth pipeline exists.

**The stack, `Stack<T, Est, Plan, Ctl, Safe, Daa, Fc>`,** like racing's, with `profile = MobilityProfile`.

**The baselines:**
- `null`: the null slots. The vehicle stays on the pad, and the run ends at its duration.
- `s0-gt-pid`: the true state (`GroundTruthEstimator`), a `GoalPlanner` that flies a straight line to the goal at a cruise speed and holds there, and `CascadedPid`. It doesn't avoid obstacles. It's the lower bound every avoiding baseline must beat, as B0 is racing's upper bound.

**The scorer:**
- **The outcome:** reached (within the goal radius), collided (the vehicle's radius touches an obstacle or the ground away from the pad), left the geofence, over the speed limit, or timed out.
- **The measures:** the time to the goal, the minimum clearance to any obstacle, the path length, and the peak speed.
- **`sitl_batch`:** the success rate over N seeds, with each outcome's count. The success rate against speed (ADR 0007) comes from running the batch at several cruise speeds.

**The apps,** as racing-drone's: `sitl_<baseline>`, `replay_<baseline>`, `sitl_batch_<baseline>`, `score` and `log_dump`. No `flight_stack` and no precision sweep yet.

## What S0 may expose in anatomy

These are the assumptions to check, from reading anatomy. S0 confirms or rules out each one. A confirmed one gets an anatomy PR, released, and scout-drone moves its pin.

1. **The world is fixed.** `BasicSlotContext<World>` and `BasicSimulator` take the world as given. A field generated from the seed needs the seed before the world exists, and a draw stream that doesn't shift the simulator's own streams.
   - **S0b: confirmed in part.** The world needs no change: scout builds the field from its file and the seed (`config::field_for_seed`) before it hands the field to anatomy. The stream did: its derivation was a static member of the simulator's class template, out of reach of a mission's core library. It moves to the core, `anatomy::stream_seed` ([anatomy#24](https://github.com/branes-ai/anatomy/pull/24)).
2. **The geofence comes from the world.** Racing's `Stack` passes `map_.geofence` to the safety filter. A field must carry one too, or the safety filter's contract must take it from somewhere else.
3. **The oracle sees only the truth.** `OracleSensor::observe(truth, t)` takes no random stream. Noisy obstacle observations need one that is deterministic in the seed, as racing's corner noise is.
4. **Mission channel ids.** Racing's `oracle_gates` is id 5. A second mission may pick the same id for a different channel. The registry rule (ADR 0001, amendment 5) leaves a mission's ids to the mission, and `log_dump` must name them.
5. **The run ends at the duration, or when the scorer stops it.** A run that reaches the goal must stop cleanly, and a vehicle that hovers at the goal must not count as a failure.
6. **Names.** racing-drone reaches anatomy through bridge headers. scout-drone uses anatomy's names directly, in `branes::scout`. If something is reachable only through a bridge, that's an anatomy gap.
7. **The YAML helpers are private.** Not in this list as first written; found in S0b. A mission's world file is loaded with the helpers anatomy's own loaders use, but they were private to anatomy's `sdk/src/config/`, and racing-drone keeps a copy. **Confirmed:** they become a public library, `branes::anatomy_yaml` ([anatomy#23](https://github.com/branes-ai/anatomy/pull/23)).

Each change to anatomy must keep racing-drone's outputs byte-identical (its `tools/compare_outputs.sh`, the reference-log fixture, and gates P0 and P1), as in steps 1 and 2.

## The PRs, in order

| PR | Repo | Content |
|---|---|---|
| **S0a** | scout-drone | The repo: CMake ≥ 4.0 with the shared presets, deps (anatomy v0.4.0, with the reflex pin anatomy uses), warnings and format, CI (GCC and Clang, debug and release, and MSVC), commitlint, release-please, `docs/` with its map and getting-started pages, the docs site with docs-kit v0.2.1, and this plan. A smoke test links anatomy |
| **S0b** | scout-drone | The world: `Field`, its loader and emitter (round trip), the generated obstacles, the scenario schema, and their tests |
| **S0c** | scout-drone | The contracts and slots: `ObstacleObservations` and its codec, the `oracle_obstacles` channel, the `Estimator` and `Planner` concepts, their nulls, `GroundTruthEstimator` and `GoalPlanner`, the `Stack`, the slot registry and the baseline generator, and the `null` and `s0-gt-pid` baselines |
| **S0d** | scout-drone | The simulator, the scorer and the apps: `ObstacleOracle`, `Simulator`, `Scorer`, `sitl`, `replay`, `sitl_batch`, `score`, `log_dump`, and the scenarios |
| **An** | anatomy | One PR per assumption confirmed, each released; racing-drone stays byte-identical, and scout-drone bumps its pin |
| **R** | racing-drone | ADR 0008's current state: step 5.3 done, with links to this plan |

S0b–S0d may each wait on an anatomy PR. Where anatomy's change is needed first, it lands and is released before the scout-drone PR that needs it.

## Progress

| PR | Repo | State |
|---|---|---|
| S0a | scout-drone | Merged as [#3](https://github.com/branes-ai/scout-drone/pull/3) |
| A1 | anatomy | [#23](https://github.com/branes-ai/anatomy/pull/23): the YAML helpers, public (assumption 7). racing-drone byte-identical |
| A2 | anatomy | [#24](https://github.com/branes-ai/anatomy/pull/24): `stream_seed` in the core (assumption 1). racing-drone byte-identical |
| S0b | scout-drone | The field, its file, the generated obstacles and the scenario schema, on anatomy with A1 and A2 |

## Acceptance (stated before any test is run)

1. **scout-drone builds and passes its tests** in CI: GCC and Clang, debug and release, and MSVC. Commitlint and the records check pass.
2. **It uses anatomy, not a copy of it.** No anatomy file is copied into scout-drone, and nothing in scout-drone re-implements an anatomy piece.
3. **Determinism:**
   - two `sitl_s0-gt-pid` runs with the same seed write byte-identical logs;
   - `replay_s0-gt-pid` reproduces every output of a log byte-identically.
4. **The null baseline** runs a scenario to its duration with no error, scored as timed out.
5. **S0 flies:**
   - **open field** (no obstacles), 100 seeds of the nominal randomization: `s0-gt-pid` reaches the goal in at least 99 of them;
   - **obstacle field**, 100 seeds at each of three cruise speeds: the batch reports the success rate and the count of each outcome. There's no threshold, because this baseline doesn't avoid anything. It's the lower bound.
6. **Each anatomy assumption** in the list above is recorded in this plan as confirmed (with the anatomy PR that fixes it) or ruled out (with why). Each anatomy PR keeps racing-drone's outputs byte-identical, checked with its `compare_outputs.sh`.
7. **The docs:** the docs site is published, and this plan is on the architecture site.
8. **ADR 0008's current state** says step 5.3 is done and links here.

## Risks

- **The straight-line baseline makes a weak lower bound** on a dense field. That's intended: the number to beat is the success rate of flying blind.
- **The oracle is not perception.** Results with the oracle bound a real depth pipeline from above. They're reported as oracle results.
- **An anatomy change can ripple into racing-drone.** Racing-drone pins v0.3.0, and anatomy is at v0.4.0. A change that S0 needs ships in a release racing-drone hasn't adopted. The byte-identity check runs against racing-drone at the newest anatomy, so the next bump stays safe.
- **The vehicle is racing's RP-1** until scout-drone has a vehicle of its own. Its configs are copied into `vehicles/`, not shared, because a vehicle is mission data.
