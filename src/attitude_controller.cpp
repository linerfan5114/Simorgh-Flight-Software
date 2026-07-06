#include "simorgh/attitude_controller.hpp"

namespace simorgh {

AttitudeController::AttitudeController(double kp, double kd) : kp_(kp), kd_(kd) {}

Vector3 AttitudeController::computeTorque(const Quaternion& q_current, const Vector3& omega_current,
                                           const Quaternion& q_target, const Vector3& omega_target) const {
    // Error quaternion: rotation needed to go from current to target attitude.
    Quaternion qe = (q_current.conjugate() * q_target).normalized();

    Vector3 qe_vec = qe.vec();
    // q and -q represent the same physical rotation. If the scalar
    // part is negative, qe represents "the long way around" -- flip
    // its sign so the controller always takes the shortest path.
    if (qe.w < 0.0) qe_vec = -qe_vec;

    Vector3 omega_error = omega_target - omega_current;

    return kp_ * qe_vec + kd_ * omega_error;
}

} // namespace simorgh
