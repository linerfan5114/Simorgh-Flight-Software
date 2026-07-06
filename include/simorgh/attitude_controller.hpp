#pragma once
#include "quaternion.hpp"

namespace simorgh {

/*
 * AttitudeController - a standard quaternion-based PD controller.
 *
 * Computes a commanded body-frame torque to drive the spacecraft
 * from its current attitude/rate toward a target attitude/rate.
 * This is the textbook quaternion PD law used widely in spacecraft
 * and drone attitude control: proportional term on the vector part
 * of the attitude error quaternion, derivative term on the angular
 * rate error.
 */
class AttitudeController {
public:
    AttitudeController(double kp, double kd);

    Vector3 computeTorque(const Quaternion& q_current, const Vector3& omega_current,
                           const Quaternion& q_target, const Vector3& omega_target) const;

private:
    double kp_, kd_;
};

} // namespace simorgh
