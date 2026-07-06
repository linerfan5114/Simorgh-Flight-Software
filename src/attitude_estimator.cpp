#include "simorgh/attitude_estimator.hpp"

namespace simorgh {

AttitudeEstimator::AttitudeEstimator(double kp, double ki)
    : q_(Quaternion::identity()),
      bias_(0, 0, 0),
      kp_(kp),
      ki_(ki),
      pending_error_(0, 0, 0),
      has_pending_correction_(false) {}

void AttitudeEstimator::reset(const Quaternion& initial_attitude) {
    q_ = initial_attitude.normalized();
    bias_ = Vector3(0, 0, 0);
    pending_error_ = Vector3(0, 0, 0);
    has_pending_correction_ = false;
}

void AttitudeEstimator::correct(const Vector3& measured_body, const Vector3& reference_inertial) {
    Vector3 measured = measured_body.normalized();
    Vector3 reference = reference_inertial.normalized();

    // What the reference direction *should* look like in the body
    // frame, given our current attitude estimate (q rotates body ->
    // inertial, so its conjugate rotates inertial -> body).
    Vector3 predicted_body = q_.conjugate().rotate(reference);

    // The cross product of "measured" and "predicted" is a small
    // rotation vector pointing from where we currently estimate the
    // reference to be, toward where it was actually measured -- feed
    // this back as a correction to the integrated rate. (Note the
    // argument order here matters: swapping it flips the sign of the
    // whole feedback loop from negative -- stabilizing -- to
    // positive -- diverging. This order matches the standard Mahony
    // filter convention.)
    Vector3 error = measured.cross(predicted_body);
    pending_error_ += error;
    has_pending_correction_ = true;
}

void AttitudeEstimator::predict(const Vector3& gyro_body, double dt) {
    Vector3 corrected_rate = gyro_body - bias_;

    if (has_pending_correction_) {
        corrected_rate += kp_ * pending_error_;
        bias_ -= ki_ * pending_error_ * dt;
        pending_error_ = Vector3(0, 0, 0);
        has_pending_correction_ = false;
    }

    Quaternion omega_quat(0.0, corrected_rate.x, corrected_rate.y, corrected_rate.z);
    Quaternion qdot = q_ * omega_quat; // kinematic equation: qdot = 0.5 * q * omega, 0.5 applied below
    q_ = (q_ + qdot * (0.5 * dt)).normalized();
}

} // namespace simorgh
