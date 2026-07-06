#include "simorgh/spacecraft_dynamics.hpp"
#include <algorithm>

namespace simorgh {

SpacecraftDynamics::SpacecraftDynamics(const Config& config)
    : config_(config),
      q_(Quaternion::identity()),
      omega_(0, 0, 0),
      wheel_speed_(0, 0, 0),
      rng_(std::random_device{}()) {}

void SpacecraftDynamics::reset(const Quaternion& attitude, const Vector3& body_rate) {
    q_ = attitude.normalized();
    omega_ = body_rate;
    wheel_speed_ = Vector3(0, 0, 0);
}

namespace {
double clampAbs(double v, double max_abs) {
    return std::max(-max_abs, std::min(max_abs, v));
}
}

void SpacecraftDynamics::step(const Vector3& torque_command, double dt) {
    // Guard against a non-positive time step: dt=0 would divide by
    // zero below (0/0 = NaN), silently corrupting all downstream
    // state with no diagnostic. A negative dt would silently
    // integrate backwards. Both are caller errors -- treat as a no-op
    // rather than corrupting simulation state.
    if (dt <= 0.0) return;

    // Saturate the commanded torque per wheel.
    Vector3 sat_torque(
        clampAbs(torque_command.x, config_.max_wheel_torque),
        clampAbs(torque_command.y, config_.max_wheel_torque),
        clampAbs(torque_command.z, config_.max_wheel_torque)
    );

    // `torque_command` is the torque the controller wants applied TO
    // THE SPACECRAFT BODY. To produce that reaction torque on the
    // body (Newton's third law), the motor must apply the *opposite*
    // torque to the wheel itself -- spinning the wheel up in one
    // direction pushes the body the other way.
    Vector3 motor_torque_on_wheel = -1.0 * sat_torque;

    Vector3 desired_speed = wheel_speed_ + (motor_torque_on_wheel * (1.0 / config_.wheel_inertia)) * dt;
    Vector3 new_speed(
        clampAbs(desired_speed.x, config_.max_wheel_speed),
        clampAbs(desired_speed.y, config_.max_wheel_speed),
        clampAbs(desired_speed.z, config_.max_wheel_speed)
    );
    Vector3 actual_motor_torque = (new_speed - wheel_speed_) * (config_.wheel_inertia / dt);
    wheel_speed_ = new_speed;

    // Rigid body dynamics (Euler's equation): I*domega/dt = torque - omega x (I*omega).
    // The reaction torque on the body is the negative of the torque
    // actually delivered to the wheel (Newton's third law) -- which,
    // when the wheel isn't saturated, works out to +sat_torque, i.e.
    // exactly what the controller asked for.
    Vector3 I = config_.inertia_diag;
    Vector3 I_omega(I.x * omega_.x, I.y * omega_.y, I.z * omega_.z);
    Vector3 gyroscopic = omega_.cross(I_omega);
    Vector3 reaction_torque_on_body = -1.0 * actual_motor_torque;
    Vector3 net_torque = reaction_torque_on_body - gyroscopic;
    Vector3 domega(net_torque.x / I.x, net_torque.y / I.y, net_torque.z / I.z);
    omega_ += domega * dt;

    // Attitude kinematics.
    Quaternion omega_quat(0.0, omega_.x, omega_.y, omega_.z);
    Quaternion qdot = q_ * omega_quat;
    q_ = (q_ + qdot * (0.5 * dt)).normalized();
}

Vector3 SpacecraftDynamics::simulateGyroMeasurement(const Vector3& bias, double noise_std_dev) const {
    std::normal_distribution<double> noise(0.0, noise_std_dev);
    return omega_ + bias + Vector3(noise(rng_), noise(rng_), noise(rng_));
}

Vector3 SpacecraftDynamics::simulateVectorMeasurement(const Vector3& reference_inertial,
                                                       double noise_std_dev) const {
    Vector3 true_body_vec = q_.conjugate().rotate(reference_inertial.normalized());
    std::normal_distribution<double> noise(0.0, noise_std_dev);
    Vector3 noisy = true_body_vec + Vector3(noise(rng_), noise(rng_), noise(rng_));
    return noisy.normalized();
}

} // namespace simorgh
