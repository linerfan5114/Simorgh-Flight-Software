#pragma once
#include "quaternion.hpp"

namespace simorgh {

/*
 * AttitudeEstimator - a Mahony-style complementary filter.
 *
 * Fuses gyro-rate integration with one or more reference-vector
 * measurements (e.g. a sun sensor or magnetometer reading, compared
 * against a known inertial-frame reference direction) to prevent
 * gyro drift from accumulating without bound. It also estimates a
 * slowly-varying gyro bias.
 *
 * This is a well-established, computationally cheap alternative to a
 * full Extended Kalman Filter, and is genuinely used on small
 * satellites and drones where compute budget is tight. It is not as
 * statistically optimal as a properly tuned EKF, but it's simple
 * enough to reason about and verify, which matters for flight
 * software.
 */
class AttitudeEstimator {
public:
    explicit AttitudeEstimator(double kp = 1.0, double ki = 0.05);

    void reset(const Quaternion& initial_attitude);

    /*
     * `gyro_body`: measured angular velocity in the body frame (rad/s).
     * `dt`: time step in seconds.
     * Call this every control cycle. Call correct() beforehand
     * (same cycle) whenever a fresh vector measurement is available;
     * it need not be available every cycle.
     */
    void predict(const Vector3& gyro_body, double dt);

    /*
     * `measured_body`: a reference direction as measured in the body
     * frame (e.g. sun direction from a sun sensor), does not need to
     * be pre-normalized.
     * `reference_inertial`: the same reference direction, known in
     * the inertial frame (e.g. from an ephemeris). Does not need to
     * be pre-normalized.
     */
    void correct(const Vector3& measured_body, const Vector3& reference_inertial);

    Quaternion attitude() const { return q_; }
    Vector3 gyroBiasEstimate() const { return bias_; }

private:
    Quaternion q_;
    Vector3 bias_;
    double kp_, ki_;
    Vector3 pending_error_;
    bool has_pending_correction_;
};

} // namespace simorgh
