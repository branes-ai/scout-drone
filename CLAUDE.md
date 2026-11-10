# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

scout-drone is a Branes.AI *mission repo*: outdoor flight to a goal through unknown static clutter (mission "S0"). It is the second client of [`anatomy`](https://github.com/branes-ai/anatomy), the mission framework (after `racing-drone`). It owns the task only: its world, the slots that see it, its stack, scorer, and data. **Algorithms do not live here**: depth perception and the planner belong in cortex, policy execution in synapse, the safety filter in reflex. C++20, CMake ≥ 4.0, Ninja; GCC, Clang, MSVC.

## Commands

```bash
cmake --preset gcc-debug          # also gcc-release, clang-debug, clang-release (Linux); msvc, msvc-ninja, msvc-ninja-debug (Windows)
cmake --build --preset gcc-debug
ctest --preset gcc-debug

ctest --preset gcc-debug -R <regex>        # one test or group, e.g. -R gate.s0.open, -R '^s0-gt-pid\.' (baseline tests are prefixed with the baseline name)
ctest --preset gcc-debug -L gate           # only the slow gates (open-field batch, graph-vs-slot equality)
ctest --preset gcc-debug -N                # list every registered test
# Or run one test executable directly (named after its source stem, e.g. tests/sdk/field.cpp -> `field`;
# baseline tests are baseline_<id> and baseline_<id>_closed_loop). Find it under build/gcc-debug/, then:
<test_exe> "<Catch2 test name>"
```

Lint is CI-only: `clang-format --dry-run --Werror` over every tracked `*.cpp/*.hpp` (CI uses clang-format 18; the `.clang-format` uses `IncludeBlocks`/Regroup). Run `clang-format -i` on files you change.

The first configure fetches anatomy (pinned by commit in `cmake/deps.cmake`) and, through it, yaml-cpp, reflex and Universal; Catch2 is fetched only when testing. Tests build only when scout-drone is the top-level project (`SCOUT_BUILD_TESTS`).

Flying a baseline (from the repo root; `b=build/gcc-debug`):

```bash
$b/sitl_s0-gt-pid --scenario scenarios/s0-determinism.yaml --out run.brlog
$b/replay_s0-gt-pid --log run.brlog --out replay.brlog --verify
$b/apps/score_log --log run.brlog   # add --collision-radius / --speed-limit from the scenario: the log doesn't carry them
$b/apps/log_dump --log run.brlog --out-dir run-dump                             # channels as CSV
$b/sitl_s0-gt-pid_graph --scenario scenarios/s0-open.yaml --print-graph mermaid
$b/sitl_batch_s0-gt-pid --scenario scenarios/s0-forest.yaml --seeds 100 --min-success 0 --set cruise_speed_m_s=4
```

Docs site (generated from `docs/` by docs-kit; edit `docs/`, never the site): `cd docs-site && npm ci && npm run dev`.

## Architecture

**Libraries.** `sdk/` is `branes::scout` (compiled; namespace `branes::scout`, headers `<branes/scout/...>`): the world (`Field`), field loader and seed-generated forests, scenario schema, the two task slots (`Estimator`, `Planner`), the stack/profile, the graph (`flow/`), and log channels. `sim/` is `branes::scout_sim` (header-only): the obstacle oracle on anatomy's `BasicSimulator`, the scorer, the batch runner, and the log→`reflex-trace/1` exporter. Mission-neutral code is used directly from anatomy under `<branes/anatomy/...>`, with no bridge headers.

**Six slots, composed at configure time.** Slots are `estimator planner controller safety_filter daa_tracker fc_bridge` (this order is the generated `Stack<>`'s parameter order). Estimator and planner are scout's; the other four are anatomy's. A baseline (`baselines/<name>.yaml`) has a `composition:` block (slot → implementation) and a `parameters:` block (read at startup). `cmake/baselines.cmake` (`scout_add_baseline`) validates the composition against `cmake/slot_registry.cmake`, hashes it, generates `<build>/generated/<name>/scout_baseline.hpp` from `cmake/scout_baseline.hpp.in`, and creates `sitl_<name>`, `replay_<name>`, `sitl_batch_<name>` from the shared sources in `apps/{sitl,replay,sitl_batch}/main.cpp`. These are **not** per-baseline sources: the baseline is chosen by the generated header.

**Two builds per baseline.** With `GRAPH`, a baseline also gets a graph build (`*_graph` targets): the same stack expressed as anatomy's flow graph (six composites, bodies named in the baseline's `class:`/`graph:` keys and resolved through `cmake/flow_registry.cmake`; C++ in `sdk/include/branes/scout/flow/`). The graph build must write logs equal to the slot build's except `composition_hash`; `tests/gate/graph_gate.cmake` enforces this on every scenario in `scenarios/` and byte-equal batch output.

**Adding things:**
- *A slot implementation or operator*: register it in **both** `cmake/slot_registry.cmake` and `cmake/flow_registry.cmake` (same names), then reference it from a baseline file.
- *A baseline*: add `baselines/<name>.yaml`, call `scout_add_baseline(<name> [GRAPH])` in the root `CMakeLists.txt`, and add the name to the `foreach(baseline ...)` loops in `tests/CMakeLists.txt` (these lists are hand-maintained and also feed the gate tests).
- *A test*: drop `<name>.cpp` into `tests/sdk/` or `tests/sim/`; `cmake/compile_all.cmake` globs it into an executable named after the file stem (stems must be globally unique). Test names come from Catch2 cases via `catch_discover_tests`. Baseline tests live in `tests/baselines/` and are instantiated per baseline in `tests/CMakeLists.txt`.
- *Gates*: end-to-end checks (determinism, graph equality, CLI error paths) are CMake scripts in `tests/gate/` invoked via `add_test` in `tests/CMakeLists.txt`.

**Determinism is a core invariant.** Two SITL runs of the same scenario/seed must be byte-identical and replays must reproduce every output (`gate.s0.determinism*`). Random draws go through `anatomy::stream_seed` streams of their own (e.g. the oracle's `kOracleStream`) so they never shift the simulator's streams. A forest field is generated from the run's seed in `config::field_for_seed` before the field is handed to anatomy. Logs are recorded in `double`.

**Data directories.** `fields/` (worlds: start, goal, geofence, listed and/or generated obstacles), `scenarios/` (a run: field, vehicle, duration, seed, randomization, oracle `sim:` keys), `vehicles/` (currently racing-drone's RP-1). Mission channel `oracle_obstacles` is id 7 (racing's `oracle_gates` is 5; ids are a mission's own, so don't reuse 5).

## Conventions

- Dependencies must be permissive-licensed only; **no GPL code for any purpose**, algorithms clean-room from papers (`docs/adr/0001`, `0002`).
- Commits and PR titles follow Conventional Commits, enforced by commitlint (types: feat, fix, build, ci, chore, docs, test, refactor, perf, style, revert; lowercase scope; header ≤ 100 chars). release-please releases only feat, fix, perf, revert, refactor and build.
- Moving the anatomy pin (`cmake/deps.cmake`) is a `build(deps)` change; anatomy gaps found here are fixed upstream in anatomy, not worked around locally. Every pin is a tag or commit.
- `docs/plan/` and `docs/adr/` are validated in CI by the architecture-records check (a plan needs a `status:` front matter, an ADR a status block); the docs site build also fails on broken relative links/anchors. ADRs are numbered `NNNN-short-title.md` and listed in `docs/adr/README.md`.
- C++20 is required (concepts, `std::span`, `std::bit_cast`); don't lower it. Warnings are strict (`cmake/warnings.cmake`, `scout_set_warnings()` on every target). Debug presets keep contract asserts on; Release compiles them out, so CI runs both compilers in both configs.
