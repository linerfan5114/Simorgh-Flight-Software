#pragma once
#include "quaternion.hpp"
#include <random>

namespace simorgh {

/*
 * SpacecraftDynamics - a simplified rigid-body spacecraft simulator
 * with three orthogonal reaction wheels (aligned with the body X/Y/Z
 * axes).
 *
 * This exists to give the estimator/controller something realistic
 * to act on during simulation and unit testing. It is a simulated
 * "plant", NOT a substitute for real sensor and actuator hardware
 * drivers.
 *
 * Simplification note: this treats each wheel's reaction torque on
 * the body as the exact negative of the torque applied to that
 * wheel, and does not model the wheels' own angular momentum
 * contributing to the body's gyroscopic coupling term. That's an
 * accurate approximation when wheel inertia is much smaller than
 * spacecraft inertia (true for the default config below), but a
 * complete momentum-exchange model would need to track total system
 * angular momentum (body + all wheels) explicitly.
 */
/*
 * Config for SpacecraftDynamics. Declared outside the class (rather
 * than nested) so its default member initializers don't run into a
 * C++ ordering restriction around default arguments that reference a
 * nested type's defaults before the enclosing class is complete.
 */
struct SpacecraftConfig {
    Vector3 inertia_diag{10.0, 10.0, 10.0}; // kg*m^2, diagonal (principal-axis) inertia
    double wheel_inertia{0.02};              // kg*m^2, per wheel
    double max_wheel_torque{1.0};            // N*m, per wheel
    double max_wheel_speed{3000.0};          // rad/s, per wheel
};

class SpacecraftDynamics {
public:
    using Config = SpacecraftConfig;

    explicit SpacecraftDynamics(const Config& config = Config());

    void reset(const Quaternion& attitude, const Vector3& body_rate);

    /*
     * Commands a torque (N*m, one component per body axis) to be
     * applied via the reaction wheels, integrates wheel speed and
     * rigid-body attitude dynamics forward by `dt` seconds. Torque is
     * saturated by both the per-wheel max torque and by wheel speed
     * saturation (a wheel already at max speed can't accelerate
     * further in that direction).
     */
    void step(const Vector3& torque_command, double dt);

    Quaternion attitude() const { return q_; }
    Vector3 bodyRate() const { return omega_; }
    Vector3 wheelSpeeds() const { return wheel_speed_; }

    /*
     * Simulates a noisy, biased gyro reading of the true body rate --
     * a convenient way to feed AttitudeEstimator a realistic input
     * during simulation instead of the (unrealistic) perfect true rate.
     */
    Vector3 simulateGyroMeasurement(const Vector3& bias, double noise_std_dev) const;

    /*
     * Simulates a noisy measurement of a known inertial-frame
     * reference vector (e.g. sun direction) as seen in the body
     * frame -- for feeding AttitudeEstimator::correct().
     */
    Vector3 simulateVectorMeasurement(const Vector3& reference_inertial, double noise_std_dev) const;

private:
    Config config_;
    Quaternion q_;
    Vector3 omega_;       // true body angular rate (rad/s)
    Vector3 wheel_speed_; // true reaction wheel speeds (rad/s)
    mutable std::mt19937 rng_;
};

} // namespace simorgh
