# Simorgh Flight Software - Attitude Determination & Control (ADCS)

A real, working attitude determination and control subsystem: the
part of flight software responsible for figuring out which way a
spacecraft is pointing and commanding actuators to point it where you
want. Written in C++17, no external dependencies.

## What this actually is (please read)

This is **not** a complete flight software stack -- it's one
subsystem (ADCS), implemented for real and validated numerically, not
a stub or mockup. Everything below is a genuine implementation of a
technique actually used on real spacecraft, scoped down to something
a single person can build, test, and understand end-to-end:

| Component | What it actually does |
|---|---|
| `quaternion.hpp` | Full quaternion algebra (Hamilton product, rotation, conjugate, axis-angle) -- the standard way to represent spacecraft attitude without gimbal lock. |
| `attitude_estimator` | A Mahony-style complementary filter: fuses gyro integration with vector-sensor measurements (e.g. sun sensor) to estimate attitude *and* gyro bias, correcting for the drift that pure gyro integration would otherwise accumulate. This exact technique is used on real small satellites and drones. |
| `attitude_controller` | A textbook quaternion PD controller: computes body-frame torque to drive the spacecraft from its current attitude toward a commanded target. |
| `spacecraft_dynamics` | A simplified rigid-body simulator with 3 orthogonal reaction wheels -- the "plant" used to test the estimator/controller against realistic physics (Euler's rigid body equation, momentum exchange, wheel saturation). |
| `adcs_system` | Ties it together behind an explicit mode state machine (IDLE / SLEWING / POINTING / FAULT) and a small command interface, the way a real flight software C&DH layer wraps a control algorithm. |

What it is **not**: flight-qualified, radiation-tested, MISRA-C
certified, or a full C&DH/telemetry/mission-management stack. It
doesn't talk to real hardware (gyros, sun sensors, reaction wheel
motors) -- `spacecraft_dynamics` simulates all of that. Porting the
`adcs_system` module to real hardware would mean replacing the
simulated sensor/actuator calls with real driver calls; the
estimator/controller/state-machine logic itself doesn't need to
change, since it only depends on plain vectors and quaternions, not
on anything simulation-specific.

## Why this design

A full "flight software" claim (like this repo's earlier README) is a
big claim: real flight software involves fault management, redundancy
voting, ground command validation, telemetry formatting, uplink/
downlink protocols, and -- most of all -- a certification process that
costs more than most hobby projects will ever need. Rather than
faking that scope, this rebuild picks **one subsystem** (ADCS) and
implements it *properly*: the math is correct (verified against
analytical rotations), the control loop is verified in closed-loop
simulation (not just unit-tested in isolation), and the honest
limitations are documented rather than hidden.

## Building and running

Requires a C++17 compiler. No external dependencies.

```bash
mkdir build && cd build
cmake ..
make
./demo_slew        # runs a 90-degree slew maneuver simulation
ctest              # or: ./test_simorgh
```

(If you don't have CMake, you can build directly with g++, which is
how this was developed and verified:)

```bash
g++ -std=c++17 -Iinclude src/*.cpp examples/demo_slew.cpp -o demo_slew
g++ -std=c++17 -Iinclude src/*.cpp tests/test_simorgh.cpp -o test_simorgh
```

## What the demo shows

`demo_slew` simulates a spacecraft starting with a small initial
tumble, commanded to slew 90 degrees about its Z axis. Every 2
simulated seconds it prints the current mode, pointing error, and
wheel speeds. In the verified run, the maneuver smoothly reduces
pointing error from 90 degrees to under 1 degree over about 50
seconds, at which point the system automatically transitions from
SLEWING to POINTING mode.

## Real bugs found and fixed during this rebuild

Building this "properly" (write code, compile it, run it, check the
numbers make physical sense) surfaced three real bugs that a
compile-only pass would have missed entirely:

1. **Inverted feedback sign in the attitude estimator.** The
   cross-product order in the vector-measurement correction was
   backwards, turning the correction into positive feedback (drift
   reinforcement) instead of negative feedback (drift correction).
2. **Undersized reaction wheels for the demo's controller gains.**
   The default actuator torque/momentum limits were far too small for
   the commanded maneuver, causing chronic saturation and a stalled
   simulation.
3. **Inverted Newton's-third-law sign in the reaction wheel model.**
   The reaction torque applied to the spacecraft body from the
   wheels had the wrong sign, meaning the simulated physics fought
   the controller instead of obeying it -- this one caused the
   closed-loop simulation to spin up into a fault regardless of how
   the controller gains were tuned, until the sign itself was fixed.

All three are now fixed and covered by the closed-loop test
(`test_full_loop_converges_to_pointing`), which runs the estimator,
controller, and simulated dynamics together and checks that the
system actually reaches the commanded attitude -- not just that each
piece compiles.

## Project layout

```
Simorgh-Flight-Software/
├── README.md
├── CMakeLists.txt
├── include/simorgh/
│   ├── quaternion.hpp            # quaternion + vector math
│   ├── attitude_estimator.hpp      # Mahony complementary filter
│   ├── attitude_controller.hpp       # quaternion PD controller
│   ├── spacecraft_dynamics.hpp         # rigid body + reaction wheel simulator
│   └── adcs_system.hpp                   # state machine + command interface + telemetry
├── src/
│   ├── attitude_estimator.cpp
│   ├── attitude_controller.cpp
│   ├── spacecraft_dynamics.cpp
│   └── adcs_system.cpp
├── examples/
│   └── demo_slew.cpp              # 90-degree slew maneuver demo
└── tests/
    └── test_simorgh.cpp             # unit + closed-loop integration tests
```

## Known limitations

- `spacecraft_dynamics` uses a diagonal (principal-axis) inertia
  tensor and ignores wheel angular momentum's contribution to the
  body's gyroscopic coupling term -- an accurate simplification when
  wheel inertia is much smaller than spacecraft inertia (true here),
  but not a complete momentum-exchange model.
- The default controller gains were tuned empirically against the
  default actuator limits for a smooth, non-oscillatory demo
  response -- they are not derived from a specific real spacecraft's
  mass properties, and would need re-tuning for a different inertia
  or actuator configuration.
- No sensor fault detection beyond the single body-rate fault check;
  a real system would also validate sensor readings for
  plausibility/consistency before trusting them.
- Single-string (no redundancy/voting) -- real flight computers often
  run critical algorithms on multiple independent lanes and vote on
  the result; this project doesn't implement that pattern (see the
  sibling `hermes-flight-comm` repo for a worked TMR-voting example).

## Possible next steps

- Add a proper Extended Kalman Filter as an alternative to the
  Mahony filter, with a covariance-based comparison of the two.
- Model a non-diagonal (fully coupled) inertia tensor for a more
  complete rigid-body simulation.
- Add sensor fault detection/isolation (e.g. comparing redundant
  gyro readings).
- Wire this up to real hardware (e.g. an IMU + reaction wheel driver
  on a microcontroller) as a follow-on hardware-in-the-loop project.
