---
title: The Mission's Graph
description: Scout's stack as a graph of composites and regions, checked as it is built and run by anatomy's CPU stack, and how a baseline's graph build is held equal to its slot build.
---

Each baseline builds twice. The slot build composes the stack from the
file's `composition:`, as it always has. The graph build describes the same
stack as a graph, which anatomy checks as it is built and runs on its CPU
stack ([anatomy ADR 0004](https://github.com/branes-ai/anatomy/blob/main/docs/adr/0004-domain-flow-composition.md),
step 2). The graph build exists so that later steps can lower the graph
to other hardware: today it must fly exactly as the slot build does, and a
gate checks that it does.

The graph is C++, in `<branes/scout/flow/...>`, namespace `branes::scout::graph`:

| Header | What it holds |
|---|---|
| `domains.hpp` | The domain of each scout message: `Field`, `Goal`, `Obstacle`, `ObstacleObservation`, `ObstacleObservations` |
| `bindings.hpp` | The bindings of scout's two task composites, the estimator and the planner |
| `graph.hpp` | The graph's ports, the scoreboard graph, and `graph::Stack`, the CPU stack that runs it |

## The ports

| Port | Kind | Message |
|---|---|---|
| `imu` | input | `ImuSample` |
| `camera` | input, timed by the frame's mid-exposure | `CameraFrame` |
| `obstacles` | input | `ObstacleObservations` |
| `truth` | input, SITL only | `TruthState` |
| `tick` | the control tick | `Timestamp` |
| `field` | constant | `Field` |
| `detections` | constant, empty: nothing produces intruder detections yet | `IntruderDetections` |
| `estimate`, `reference`, `command` | outputs, at the tick | `StateEstimate`, `Reference`, `ThrustRates` |

## The composites

Six composites, one per slot of the slot stack. A composite's binding says
how the graph calls an implementation: one handler per method of the slot's
concept, each a struct with a static `call()`. So every slot implementation
is an operator of its composite, unchanged.

| Composite | Binding | Handlers |
|---|---|---|
| `estimator` | `graph::EstimatorBinding` | `imu`, `frame`, `obstacles`, `truth`, `estimate` (at a time) |
| `planner` | `graph::PlannerBinding` | `plan` (field, obstacles, estimate) |
| `controller` | anatomy's `ControllerBinding` | `step` |
| `safety_filter` | anatomy's `SafetyFilterBinding` | `filter` |
| `daa_tracker` | anatomy's `DaaTrackerBinding` | `update`, `conflicts` |
| `fc_bridge` | anatomy's `FcBridgeBinding` | `send` |

The estimator's `truth` handler hands the true state to an estimator that
reads it (`TruthConsumer`), and does nothing for one that doesn't, as the
slot stack's `on_truth()` does. The graph therefore reads the SITL-only
`truth` port whichever estimator fills it, and every graph build is SITL-only.

## The regions

Each input fires one region, and a region's firings run in the order the
graph writes them, which is the slot stack's order. That is what makes the
two builds write the same records.

1. **truth:** the estimator takes the true state.
2. **imu:** the estimator takes the sample.
3. **frame:** the estimate at the frame's time is read, the estimator takes
   the frame, and the DAA tracker is updated with that prior and the
   detections.
4. **obstacles:** the estimator takes the obstacles seen.
5. **tick:** at the tick's time, the estimate is read; the planner plans from
   the field, the latest obstacles seen and the estimate; the DAA tracker
   gives its conflicts; the controller steps; the safety filter filters the
   command, with the field's geofence; and the FC bridge sends it. The
   estimate, the reference and the filtered command are the tick's outputs.

`graph::Stack` gives the harnesses the slot stack's hooks (`on_truth()`,
`on_imu()`, `on_frame()`, `on_obstacles()`, `on_tick()`). Each hook hands the
stack its input, which runs that input's region; the hooks hold no logic.

## The baseline file

A graph build reads two more keys from the baseline's file:

```yaml
class: scoreboard
graph:
  estimator: GroundTruthEstimator
  planner: GoalPlanner
  controller: CascadedPid
  safety_filter: NullSafetyFilter
  daa_tracker: NullDaaTracker
  fc_bridge: NullFcBridge
```

A scoreboard baseline names a body for each composite, and nothing else: the
graph is fixed by scout's code, so a baseline can't rewire it. The stack
checks the file's `class:` and `graph:` against the graph it builds, and
fails to start if they differ.

A body is an operator registered in `cmake/flow_registry.cmake`, with its
header, its type and its binding:

```cmake
scout_register_operator(GoalPlanner HEADER branes/scout/slots/goal.hpp
                        TYPE branes::scout::GoalPlanner BINDING ${_scout}::PlannerBinding)
```

A new slot implementation joins the graph build by being registered here.

## The builds

`scout_add_baseline(<name> GRAPH)`, in `CMakeLists.txt`, adds the graph
build's harnesses beside the slot build's: `sitl_<name>_graph`,
`replay_<name>_graph` and `sitl_batch_<name>_graph`. Both builds compile the
same `apps/` sources, against a generated header that names the baseline's
composition or graph. `null` and `s0-gt-pid` have graph builds.

The graph build's log records the graph's hash as its `composition_hash`, and
replay checks that hash against the input log. Everything else in the two
builds' logs is the same. anatomy's `log_compare` checks this; the tool is
built under the anatomy dependency, in `_deps/anatomy-build/apps/`:

```bash
b=build/gcc-debug
$b/sitl_s0-gt-pid       --scenario scenarios/s0-forest.yaml --out slot.brlog
$b/sitl_s0-gt-pid_graph --scenario scenarios/s0-forest.yaml --out graph.brlog
$b/_deps/anatomy-build/apps/log_compare --a slot.brlog --b graph.brlog --except composition_hash
```

It exits 0 when the logs are equal, 1 when they differ, naming the first
difference, and 2 on an error.

`sitl_<name>_graph` also prints the graph its stack runs, and exits without
flying. `--print-graph text` gives the canonical form, which the log's
`composition_hash` is the hash of; `--print-graph mermaid` gives a flowchart,
which GitHub renders in a ` ```mermaid ` block. anatomy's
[reading a graph](https://github.com/branes-ai/anatomy/blob/main/docs/guides/reading-a-graph.md) explains the chart. The slot build has no graph and
refuses the option.

```bash
$b/sitl_s0-gt-pid_graph --scenario scenarios/s0-open.yaml --print-graph mermaid
```

## The gates

- `gate.<name>.graph`, for `null` and `s0-gt-pid`, flies both builds on every
  scenario in `scenarios/`. Their logs must be equal except
  `composition_hash`, and compared whole they must differ there, so the graph's
  hash is the one recorded. On `s0-open` and `s0-forest`, the two builds'
  batches of 10 seeds must write byte-identical JSON lines and reports.
- `gate.s0.determinism.graph` runs the determinism gate on the graph build of
  `s0-gt-pid`.

`ctest --preset gcc-debug -R '\.graph$'` runs the three.
