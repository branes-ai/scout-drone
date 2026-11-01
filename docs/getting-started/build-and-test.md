# Build & Test

scout-drone is C++20 and builds with CMake ≥ 4.0 and Ninja, on GCC, Clang, and MSVC, using the same presets as the sibling repos.

## Build and run the tests

```bash
cmake --preset gcc-debug              # also: gcc-release, clang-debug, clang-release, msvc, msvc-ninja, ...
cmake --build --preset gcc-debug
ctest --preset gcc-debug
```

The first configure fetches anatomy, pinned in `cmake/deps.cmake`, and what anatomy builds on: yaml-cpp, reflex and Universal. The tests are built only when scout-drone is the top-level project (`SCOUT_BUILD_TESTS`).

## Fly a baseline

Each baseline gets `sitl_<name>`, `replay_<name>` and `sitl_batch_<name>`, in the build directory; `score_log` and `log_dump` read any baseline's log, in its `apps/`. From the repo root:

```bash
b=build/gcc-debug
$b/sitl_s0-gt-pid --scenario scenarios/s0-determinism.yaml --out run.brlog   # fly, and write the log
$b/replay_s0-gt-pid --log run.brlog --out replay.brlog --verify              # reproduce every output
$b/apps/score_log --log run.brlog                                               # the score, as a JSON line
$b/apps/log_dump --log run.brlog --out-dir run-dump                             # every channel as CSV
$b/sitl_batch_s0-gt-pid --scenario scenarios/s0-forest.yaml --seeds 100 \
    --min-success 0 --set cruise_speed_m_s=4                                      # 100 seeds, at 4 m/s
```

A log records the field it flew, its obstacles listed, so it replays and scores on its own. `score_log` takes the scenario's collision radius and speed limit as `--collision-radius` and `--speed-limit`, since the log doesn't carry the scenario. `--set` changes numeric baseline parameters for one batch only.

## The docs site

This site is generated from [`docs/`](../README.md) by [docs-kit](https://github.com/branes-ai/docs-kit). To preview it:

```bash
cd docs-site
npm ci
npm run dev        # regenerates the pages from docs/, then serves them
```

Next: [repository layout](repository-layout.md).
