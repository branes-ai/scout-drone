---
status: ready
issue: branes-ai/scout-drone#2
---
# Plan: bring up scout-drone on anatomy (S0)

Step 3 of racing-drone's [ADR 0008](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0008-anatomy-mission-framework.md): scout-drone becomes anatomy's second client. ADR 0008 says, "each assumption it exposes becomes a change in anatomy, checked against both clients." The mission is [ADR 0007](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0007-outdoor-obstacle-avoidance-placement.md)'s: reach a goal through static clutter the vehicle has not seen.

## Goal

1. **scout-drone is a mission repo like racing-drone,** built on anatomy v0.6.1 (core, mobility, glue, mobility_sim): its build, CI, releases, docs and docs site.
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
- `Estimator`: IMU and frames, as in racing, with the obstacle observations where racing has gate observations. They arrive on their own channel, after the frame, so the estimator takes them as an input of their own.
- `Planner`: the field's goal, the latest obstacle observations, and the state estimate in; a `Reference` out.

There's no gate detector. The oracle stands in for perception until cortex's depth pipeline exists.

**The stack, `Stack<T, Est, Plan, Ctl, Safe, Daa, Fc>`,** like racing's, with `profile = ScoutProfile`: anatomy's `MobilityProfile` with one more input, `oracle_obstacles`, which the harness hands to the stack's `on_obstacles()`. As first written this said `MobilityProfile`, which can't deliver a mission's channel to the stack (assumption 8).

**The baselines:**
- `null`: the null slots. The vehicle stays on the pad, and the run ends at its duration.
- `s0-gt-pid`: the true state (`GroundTruthEstimator`), a `GoalPlanner` that flies a straight line to the goal at a cruise speed and holds there, and `CascadedPid`. The planner climbs from the pad to the goal's height first, as racing's waypoint planner climbs to its first gate. It doesn't avoid obstacles. It's the lower bound every avoiding baseline must beat, as B0 is racing's upper bound.

**The scorer:**
- **The outcome:** reached (within the goal radius), collided (the vehicle's radius touches an obstacle, or the vehicle is on the ground after takeoff, back on the pad included), left the geofence, over the speed limit, or timed out. The vehicle's radius is the scenario's `sim.collision_radius_m`, 0.25 m by default: the vehicle file has the arm length but no propeller size.
- **The measures:** the time to the goal, the minimum clearance to any obstacle, the path length, and the peak speed.
- **`sitl_batch`:** the success rate over N seeds, with each outcome's count. The success rate against speed (ADR 0007) comes from running the batch at several cruise speeds: `--set cruise_speed_m_s=<v>` overrides a numeric baseline parameter for one batch. `sitl` and `replay` don't take it, so a reference log always has the baseline file's parameters.

**The apps,** as racing-drone's: `sitl_<baseline>`, `replay_<baseline>`, `sitl_batch_<baseline>`, `score` and `log_dump`. No `flight_stack` and no precision sweep yet.

**The replay** (S0e, added after S0d): a SITL run plays in the docs site, in the trace player reflex publishes (`@branes-ai/trace-player`, reflex's animation standard), with the field drawn. The player drew only the vehicle, so reflex comes first:
- **`reflex-trace/1` gets a scene** (optional, so the format stays `/1`): entities, each with a role, a collision shape (box, cylinder or sphere), an optional visual drawn in its place (a procedural tree or gate sized from the shape, or a glTF model reference for later), and a pose that is fixed or read from trace channels, for moving entities. The collision shape is always there, because it is what the simulator and the scorer test, and the player can show it over a visual.
- **scout's `trace_export`** writes a log as a trace: the vehicle's channels as racing's does, the field as the scene (each obstacle a cylinder drawn as a tree, the goal, the geofence), and takeoff, the goal and a collision, with its obstacle, as events. The docs site plays a forest run that collides and one that gets through.

glTF models wait for a need, a CC0 source and a size budget: a model that looks wider than the trunk the scorer tests would mislead.

## What S0 may expose in anatomy

These are the assumptions to check, from reading anatomy. S0 confirms or rules out each one. A confirmed one gets an anatomy PR, released, and scout-drone moves its pin.

1. **The world is fixed.** `BasicSlotContext<World>` and `BasicSimulator` take the world as given. A field generated from the seed needs the seed before the world exists, and a draw stream that doesn't shift the simulator's own streams.
   - **S0b: confirmed in part.** The world needs no change: scout builds the field from its file and the seed (`config::field_for_seed`) before it hands the field to anatomy. The stream did: its derivation was a static member of the simulator's class template, out of reach of a mission's core library. It moves to the core, `anatomy::stream_seed` ([anatomy#24](https://github.com/branes-ai/anatomy/pull/24)).
2. **The geofence comes from the world.** Racing's `Stack` passes `map_.geofence` to the safety filter. A field must carry one too, or the safety filter's contract must take it from somewhere else.
   - **S0c: ruled out.** The field carries its geofence (`Field::geofence`, from S0b), and scout's `Stack` passes it to the safety filter as racing's passes the course's. The contract needs no change.
3. **The oracle sees only the truth.** `OracleSensor::observe(truth, t)` takes no random stream. Noisy obstacle observations need one that is deterministic in the seed, as racing's corner noise is.
   - **S0d-1: ruled out.** The oracle owns its stream. `ObstacleOracle` is built with the scenario's seed and draws its noise from a stream of its own (`kOracleStream`, through `anatomy::stream_seed`), so `observe` needs none passed in, and the simulator's streams don't shift. The noise is uniform with the scenario's σ, as the simulated IMU's is, so a seed draws the same noise on every platform. Racing's corner noise turned out not to be drawn at all: its σ only sets the observation's covariance.
4. **Mission channel ids.** Racing's `oracle_gates` is id 5. A second mission may pick the same id for a different channel. The registry rule (ADR 0001, amendment 5) leaves a mission's ids to the mission, and `log_dump` must name them.
   - **S0c: no clash.** `oracle_obstacles` is id 7, apart from racing's 5, so a racing log fails to open with scout's channels instead of being read as scout's. Naming them in `log_dump` is S0d-2's to check.
   - **S0d-2: no change needed.** `log_dump` names `oracle_obstacles` through anatomy's `ChannelFormat`, as racing's names `oracle_gates`, and the apps open a scout log with `kScoutChannels`.
5. **The run ends at the duration, or when the scorer stops it.** A run that reaches the goal must stop cleanly, and a vehicle that hovers at the goal must not count as a failure.
   - **S0d-1: ruled out.** The scorer marks the run complete when the vehicle comes within the goal radius, and scores nothing after that, so a vehicle holding at the goal can't fail. The closed loop (`run_closed_loop`) stops at completion. A SITL run logs to its duration, and scoring its log stops at the goal the same way.
6. **Names.** racing-drone reaches anatomy through bridge headers. scout-drone uses anatomy's names directly, in `branes::scout`. If something is reachable only through a bridge, that's an anatomy gap.
   - **S0c: none so far.** The slots, the stack, the slot registry and the baseline generator use anatomy's names directly. The registry names anatomy's neutral nulls and `CascadedPid` by their own types.
7. **The YAML helpers are private.** Not in this list as first written; found in S0b. A mission's world file is loaded with the helpers anatomy's own loaders use, but they were private to anatomy's `sdk/src/config/`, and racing-drone keeps a copy. **Confirmed:** they become a public library, `branes::anatomy_yaml` ([anatomy#23](https://github.com/branes-ai/anatomy/pull/23)).
8. **A mission's own input can't reach the stack in the loops.** Not in this list as first written; found in S0c. A stack's inputs are its profile's, and `MobilityProfile`'s are the IMU and the camera only. A mission channel such as racing's `oracle_gates` is recorded and passed through, never consumed. Scout's planner needs the obstacles seen, so `ScoutProfile` extends `MobilityProfile` with `oracle_obstacles`. anatomy's `harness::Runner` drives that stack, but `MobileStack` requires the profile to be `MobilityProfile` exactly, and `run_closed_loop` and `run_sitl` take a `MobileStack` only. **Confirmed:** the loops must take a stack whose profile has `MobilityProfile`'s truth, outputs and codecs, whatever inputs it adds. That's an anatomy PR (A3), released before S0d. Until then, a test in scout pins the gap (`!MobileStack<...>`).
   - **A3: fixed.** `MobileStack` takes a stack whose profile is a `MobileProfile`: `MobilityProfile`'s truth, outputs and codecs, with every one of its inputs and any inputs the mission adds ([anatomy#26](https://github.com/branes-ai/anatomy/pull/26), released in v0.6.0). scout-drone moves to v0.6.1, and its test asserts that its stack is a `MobileStack`.
9. **A log can't rebuild a generated world.** Not in this list as first written; found in S0d-2. The log header carries the world file verbatim, but not the scenario, and a generated field needs the scenario's seed. Neither can `score_log` read the scenario's collision radius or speed limit from it.
   - **S0d-2: worked around in scout, with no anatomy change.** `sitl` writes the field flown, its obstacles listed (`harness::flown_field_yaml`), as the header's world, so `replay` and `score_log` read back exactly that field, and `score_log` takes the collision radius and the speed limit as options, as racing's takes the speed limit. A header that carried the scenario would let a log keep the field file and its seed instead. That's an anatomy change for when a second mission needs it.

Each change to anatomy must keep racing-drone's outputs byte-identical (its `tools/compare_outputs.sh`, the reference-log fixture, and gates P0 and P1), as in steps 1 and 2.

## The PRs, in order

| PR | Repo | Content |
|---|---|---|
| **S0a** | scout-drone | The repo: CMake ≥ 4.0 with the shared presets, deps (anatomy v0.4.0, with the reflex pin anatomy uses), warnings and format, CI (GCC and Clang, debug and release, and MSVC), commitlint, release-please, `docs/` with its map and getting-started pages, the docs site with docs-kit v0.2.1, and this plan. A smoke test links anatomy |
| **S0b** | scout-drone | The world: `Field`, its loader and emitter (round trip), the generated obstacles, the scenario schema, and their tests |
| **S0c** | scout-drone | The contracts and slots: `ObstacleObservations` and its codec, the `oracle_obstacles` channel and `ScoutProfile`, the `Estimator` and `Planner` concepts, their nulls, `GroundTruthEstimator` and `GoalPlanner`, the `Stack`, the slot registry and the baseline generator, and the `null` and `s0-gt-pid` baselines |
| **S0d-1** | scout-drone | The simulator and the scorer: `ObstacleOracle`, `Simulator`, `Scorer`, and the scored run the batch shares (`run_one`), with their tests |
| **S0d-2** | scout-drone | The apps and the scenarios: `sitl`, `replay`, `sitl_batch` (with `--set`), `score`, `log_dump`, and the acceptance runs |
| **V1** | reflex | The scene in `reflex-trace/1`: its schema and validator, the player drawing collision shapes, procedural trees and gates, and moving entities, the standard, and a release |
| **S0e** | scout-drone | The replay: `trace_export`, its test, and a docs page that plays a forest run, on the released trace player |
| **An** | anatomy | One PR per assumption confirmed, each released; racing-drone stays byte-identical, and scout-drone bumps its pin |
| **R** | racing-drone | ADR 0008's current state: step 5.3 done, with links to this plan |

S0b–S0d-2 may each wait on an anatomy PR. Where anatomy's change is needed first, it lands and is released before the scout-drone PR that needs it.

## Progress

| PR | Repo | State |
|---|---|---|
| S0a | scout-drone | Merged as [#3](https://github.com/branes-ai/scout-drone/pull/3) |
| A1 | anatomy | Merged as [#23](https://github.com/branes-ai/anatomy/pull/23): the YAML helpers, public (assumption 7). racing-drone byte-identical. Released in v0.5.0 |
| A2 | anatomy | Merged as [#24](https://github.com/branes-ai/anatomy/pull/24): `stream_seed` in the core (assumption 1). racing-drone byte-identical. Released in v0.5.0 |
| S0b | scout-drone | Merged as [#7](https://github.com/branes-ai/scout-drone/pull/7): the field, its file, the generated obstacles and the scenario schema, on anatomy v0.5.0. Released in v0.2.0 |
| S0c | scout-drone | Merged as [#9](https://github.com/branes-ai/scout-drone/pull/9): the obstacles seen and their channel, `ScoutProfile`, the slots and the `Stack`, the slot registry and the baseline generator, and the `null` and `s0-gt-pid` baselines. Assumption 2 ruled out; 8 confirmed, for A3. Released in v0.3.0 |
| A3 | anatomy | Merged as [#26](https://github.com/branes-ai/anatomy/pull/26): the mobile loops take a stack whose profile adds inputs (assumption 8). racing-drone byte-identical. Released in v0.6.0; scout-drone moves to v0.6.1, and its stack is a `MobileStack` |
| S0d-1 | scout-drone | Merged as [#19](https://github.com/branes-ai/scout-drone/pull/19): the obstacle oracle, the simulator, the scorer (with `sim.collision_radius_m`) and the scored run. Assumptions 3 and 5 ruled out |
| S0d-2 | scout-drone | Merged as [#21](https://github.com/branes-ai/scout-drone/pull/21): the apps (`sitl`, `replay`, `sitl_batch` with `--set`, `score_log`, `log_dump`), the scenarios, and the acceptance runs. Assumption 4's `log_dump` check needs no change; 9 found, and worked around in scout. Released in v0.5.0 |
| V1 | reflex | Merged as [reflex#80](https://github.com/branes-ai/reflex/pull/80): the scene in `reflex-trace/1` (collision shapes, procedural trees, moving entities) and the player drawing it. Released in v0.10.0 |
| A4 | anatomy | Merged as [#34](https://github.com/branes-ai/anatomy/pull/34): reflex v0.10.0, whose library is unchanged since anatomy's pin, and which defines the trace writer for consumers. racing-drone byte-identical. Released in v0.6.3 |
| S0e | scout-drone | anatomy v0.6.3; `trace_export` (the field as the scene), its test, and the forest replays page |

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
9. **The replay:** a forest run that collides and one that reaches the goal play in the docs site, the trees and the goal drawn, and the collision shapes shown on demand. The trace validates against the released schema.

## Results (S0d-2)

Run with `s0-gt-pid` and the scenarios in `scenarios/`, at the anatomy pin of this plan:

- **Determinism (3):** two `sitl` runs of `s0-determinism` (the forest, with the randomization, the IMU, camera corruption and the oracle's noise) write byte-identical logs, and `replay --verify` reproduces all 6,000 outputs, its log byte-identical to the SITL log. CTest `gate.s0.determinism`.
- **The null baseline (4):** `null-smoke` runs its 5 s with no error, and scores as timed out on the pad. CTest `gate.null.smoke`.
- **The open field (5):** 100 of 100 nominal seeds reach the goal (p50 14.3 s, path 50.4 m). CTest `gate.s0.open`, at 99 of 100.
- **The forest (5), 100 nominal seeds at each cruise speed:**

  | Cruise speed | Reached | Collided | Other | Time to goal, p50 | Peak speed, p50 |
  |---|---|---|---|---|---|
  | 2 m/s | 32 | 68 | 0 | 18.9 s | 2.78 m/s |
  | 3 m/s | 32 | 68 | 0 | 14.3 s | 3.76 m/s |
  | 4 m/s | 32 | 68 | 0 | 12.3 s | 4.45 m/s |

  The same 32 seeds reach the goal at every speed: a straight line to the goal clears the forest or it doesn't, whatever the speed. That's the lower bound, flat in speed, which an avoiding baseline must beat. The peak speed runs over the cruise speed as the controller chases the planner's carrot: by about 0.8 m/s at 2 and 3 m/s, and by 0.45 m/s at 4 m/s, where the controller's speed cap (`max_speed_m_s`, 4.5 m/s) limits it.

## Results (S0e)

- **The replay (9):** [Watching a forest replay](../../guides/forest-replay.mdx) plays two SITL runs of `s0-gt-pid` on the forest, at reflex's trace player v0.10.0, the field drawn as the scene: seed 1706 (`scenarios/s0-forest-collision.yaml`), which collides with obstacle 0 at 11.8 s, 10.4 m short of the goal, and seed 1710 (`s0-forest-near-miss.yaml`), which reaches the goal in 14.3 s with 0.056 m to spare at its closest. *Collision shapes* shows the trunks the scorer tests. The docs build validates both traces against the released schema. Each replay's score matches the batch's for its seed, so a logged run and a closed-loop run agree.

## Risks

- **The straight-line baseline makes a weak lower bound** on a dense field. That's intended: the number to beat is the success rate of flying blind.
- **The oracle is not perception.** Results with the oracle bound a real depth pipeline from above. They're reported as oracle results.
- **An anatomy change can ripple into racing-drone.** Racing-drone pins v0.3.0, and anatomy is at v0.6.1. A change that S0 needs ships in a release racing-drone hasn't adopted. The byte-identity check runs against racing-drone at the newest anatomy, so the next bump stays safe.
- **The vehicle is racing's RP-1** until scout-drone has a vehicle of its own. Its configs are copied into `vehicles/`, not shared, because a vehicle is mission data.
