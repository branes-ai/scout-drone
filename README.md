# scout-drone

Mission capability development for drones that must reach goals fast, through terrain never seen, and without maps.

scout-drone is the Branes.AI mission repo for outdoor flight to a goal through unknown static clutter ([racing-drone ADR 0007](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0007-outdoor-obstacle-avoidance-placement.md)). It is the second client of [`anatomy`](https://github.com/branes-ai/anatomy), the mission framework, after `racing-drone` ([ADR 0008](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0008-anatomy-mission-framework.md), step 3).

Like every mission repo, it owns its task: its world, the slots that see it, its stack and scorer, and its data. Algorithms do not live here. The depth perception and the planner belong in cortex, the policy network's execution in synapse, and the safety filter in reflex.

## Status

The repo is being brought up ([plan](docs/plan/bring-up/README.md), [#2](https://github.com/branes-ai/scout-drone/issues/2)). Its first mission, S0, flies from a start pose to a goal through a field of static obstacles, seen through an oracle sensor.

## Building and testing

scout-drone is C++20 and builds with CMake ≥ 4.0 and Ninja, on GCC, Clang, and MSVC, using the same presets as the sibling repos.

```bash
cmake --preset gcc-debug              # also: gcc-release, clang-debug, clang-release, msvc, msvc-ninja, ...
cmake --build --preset gcc-debug
ctest --preset gcc-debug
```

The documentation in [`docs/`](docs/README.md) is published at <https://branes-ai.github.io/scout-drone/>.

## License

MIT; see [LICENSE](LICENSE).
