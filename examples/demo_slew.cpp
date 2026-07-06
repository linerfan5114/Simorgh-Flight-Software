/*
 * demo_slew.cpp - Simulates a 90-degree slew maneuver: the spacecraft
 * starts at a known attitude with some initial body rate, is
 * commanded to point at a target attitude 90 degrees away, and the
 * ADCS subsystem (estimator + controller) drives it there using a
 * simulated gyro and a simulated sun-sensor-like vector measurement.
 */
#include "simorgh/adcs_system.hpp"
#include "simorgh/spacecraft_dynamics.hpp"
#include <cstdio>

using namespace simorgh;

namespace {
constexpr double kPi = 3.14159265358979323846;
}

int main() {
    SpacecraftDynamics::Config sc_config;
    SpacecraftDynamics dynamics(sc_config);

    AdcsSystem::Config adcs_config;
    AdcsSystem adcs(adcs_config);

    // Start at identity attitude with a small initial tumble.
    Quaternion start_attitude = Quaternion::identity();
    Vector3 initial_rate(0.02, -0.01, 0.015); // rad/s
    dynamics.reset(start_attitude, initial_rate);
    adcs.resetEstimate(start_attitude);

    // Command a 90-degree rotation about the body Z axis.
    Quaternion target = Quaternion::fromAxisAngle(Vector3(0, 0, 1), kPi / 2.0);
    adcs.commandPointing(target);

    const Vector3 gyro_bias_truth(0.001, -0.0015, 0.0008); // rad/s, unknown to the estimator
    const Vector3 sun_direction_inertial(0, 0, 1);         // arbitrary fixed inertial reference

    const double dt = 0.1;      // 10 Hz control loop
    const double duration = 60; // seconds
    const int steps = static_cast<int>(duration / dt);

    printf("%8s %10s %8s %10s %10s %10s\n",
           "t(s)", "mode", "err(deg)", "wheelX", "wheelY", "wheelZ");

    for (int i = 0; i < steps; ++i) {
        Vector3 gyro_meas = dynamics.simulateGyroMeasurement(gyro_bias_truth, 0.0005);

        // Simulate a vector-sensor measurement every 10 cycles (1 Hz),
        // representative of a slower sensor than the gyro.
        bool has_vec = (i % 10 == 0);
        Vector3 measured_vec = has_vec
            ? dynamics.simulateVectorMeasurement(sun_direction_inertial, 0.01)
            : Vector3();

        Vector3 torque = adcs.update(gyro_meas, dt, has_vec, measured_vec, sun_direction_inertial);
        dynamics.step(torque, dt);
        adcs.setWheelSpeedsForTelemetry(dynamics.wheelSpeeds());

        if (i % 20 == 0) { // print every 2 simulated seconds
            AdcsTelemetry tm = adcs.telemetry();
            const char* mode_str =
                tm.mode == AdcsMode::IDLE ? "IDLE" :
                tm.mode == AdcsMode::SLEWING ? "SLEWING" :
                tm.mode == AdcsMode::POINTING ? "POINTING" : "FAULT";

            printf("%8.1f %10s %8.2f %10.2f %10.2f %10.2f\n",
                   tm.timestamp_s, mode_str, tm.pointing_error_deg,
                   tm.wheel_speeds.x, tm.wheel_speeds.y, tm.wheel_speeds.z);
        }
    }

    AdcsTelemetry final_tm = adcs.telemetry();
    printf("\nFinal pointing error: %.3f degrees\n", final_tm.pointing_error_deg);
    printf("Final mode: %s\n", final_tm.mode == AdcsMode::POINTING ? "POINTING (converged)" : "not converged");

    return 0;
}
