#include "simorgh/adcs_system.hpp"
#include <algorithm>
#include <cmath>

namespace simorgh {

namespace {
constexpr double kPi = 3.14159265358979323846;

double quaternionErrorAngleDeg(const Quaternion& a, const Quaternion& b) {
    Quaternion qe = (a.conjugate() * b).normalized();
    double w = std::min(1.0, std::max(-1.0, std::fabs(qe.w)));
    return 2.0 * std::acos(w) * 180.0 / kPi;
}
} // namespace

AdcsSystem::AdcsSystem(const Config& config)
    : config_(config),
      estimator_(config.estimator_kp, config.estimator_ki),
      controller_(config.controller_kp, config.controller_kd),
      mode_(AdcsMode::IDLE),
      target_attitude_(Quaternion::identity()),
      last_rate_estimate_(0, 0, 0),
      last_wheel_speeds_(0, 0, 0),
      sim_time_s_(0.0) {}

void AdcsSystem::commandPointing(const Quaternion& target_attitude) {
    if (mode_ == AdcsMode::FAULT) return;
    target_attitude_ = target_attitude.normalized();
    mode_ = AdcsMode::SLEWING;
}

void AdcsSystem::commandIdle() {
    if (mode_ == AdcsMode::FAULT) return;
    mode_ = AdcsMode::IDLE;
}

void AdcsSystem::resetEstimate(const Quaternion& attitude) {
    estimator_.reset(attitude);
}

Vector3 AdcsSystem::update(const Vector3& gyro_meas, double dt,
                            bool has_vector_measurement,
                            const Vector3& measured_body_vec,
                            const Vector3& reference_inertial_vec) {
    sim_time_s_ += dt;

    if (has_vector_measurement) {
        estimator_.correct(measured_body_vec, reference_inertial_vec);
    }
    estimator_.predict(gyro_meas, dt);

    Vector3 rate_est = gyro_meas - estimator_.gyroBiasEstimate();
    last_rate_estimate_ = rate_est;

    if (mode_ == AdcsMode::FAULT) {
        return Vector3(0, 0, 0);
    }

    if (rate_est.norm() > config_.fault_rate_limit_rad_s) {
        mode_ = AdcsMode::FAULT;
        return Vector3(0, 0, 0);
    }

    if (mode_ == AdcsMode::IDLE) {
        return Vector3(0, 0, 0);
    }

    double error_deg = quaternionErrorAngleDeg(estimator_.attitude(), target_attitude_);

    if (mode_ == AdcsMode::SLEWING && error_deg < config_.pointing_tolerance_deg) {
        mode_ = AdcsMode::POINTING;
    } else if (mode_ == AdcsMode::POINTING && error_deg > config_.pointing_tolerance_deg * 2.0) {
        // Hysteresis band so small excursions don't flap the mode
        // back and forth between SLEWING and POINTING every cycle.
        mode_ = AdcsMode::SLEWING;
    }

    return controller_.computeTorque(estimator_.attitude(), rate_est, target_attitude_, Vector3(0, 0, 0));
}

AdcsTelemetry AdcsSystem::telemetry() const {
    double error_deg = quaternionErrorAngleDeg(estimator_.attitude(), target_attitude_);
    return AdcsTelemetry{
        mode_,
        estimator_.attitude(),
        last_rate_estimate_,
        last_wheel_speeds_,
        error_deg,
        sim_time_s_
    };
}

} // namespace simorgh
