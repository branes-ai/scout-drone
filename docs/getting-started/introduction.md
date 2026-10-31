# Introduction

scout-drone is the Branes.AI mission repo for a quadrotor that flies to a goal through terrain it has never seen, with no map, as fast as it safely can.

## The mission

The task comes from [racing-drone ADR 0007](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0007-outdoor-obstacle-avoidance-placement.md), after Loquercio, Kaufmann et al., "Learning high-speed flight in the wild" (*Science Robotics*, 2021):
- **The task:** reach a goal through unknown, static clutter: a forest, a collapsed building, rough terrain.
- **The score:** the success rate against speed.
- **The sensing:** onboard only. A depth image and the IMU, in the paper.

It is a different mission from racing: no known course and no gates, and stereo is allowed.

## Where it fits

The Branes.AI stack has five tiers. scout-drone is in the top one, the mission tier, beside racing-drone.
- **Mission:** one repo per vehicle and mission. A mission repo owns its task: its world, the slots that see it, its stack and scorer, and its data. Algorithms do not live there.
- **Mission framework:** [anatomy](https://branes-ai.github.io/anatomy/), the body plan every mission composes its vehicle with: the data contracts, the slots and their composition, configs, the replay log, the harness, and the closed-loop simulator.
- **Platform:** [cortex](https://branes-ai.github.io/cortex/) (perception and planning), [synapse](https://github.com/branes-ai/synapse) (learned models) and [reflex](https://branes-ai.github.io/reflex/) (control and safety).
- **Math** and **hardware** below them.

scout-drone is anatomy's second client. Each assumption it finds in anatomy, one that held only for racing, is fixed in anatomy and checked against both missions ([ADR 0008](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0008-anatomy-mission-framework.md), step 3).

## The first mission, S0

S0 flies from a start pose to a goal through a field of static obstacles. The stack sees the obstacles through an oracle sensor, computed from the simulator's truth, until cortex has a depth pipeline. Its first baseline flies a straight line to the goal: the lower bound every avoiding baseline must beat. The [bring-up plan](../plan/bring-up/README.md) has the details.

Next: [build & test](build-and-test.md).
