#pragma once
#include "quaternion.hpp"
#include "attitude_estimator.hpp"
#include "attitude_controller.hpp"

namespace simorgh {

enum class AdcsMode {
    IDLE,     // no active pointing command; no torque is commanded
    SLEWING,  // actively rotating toward a commanded target attitude
    POINTING, // within tolerance of the target; holding fine pointing
    FAULT     // an unrecoverable condition was detected; commands are ignored until cleared
};

struct AdcsTelemetry {
    AdcsMode mode;
    Quaternion attitude_estimate;
    Vector3 body_rate_estimate;
    Vector3 wheel_speeds; // set externally via setWheelSpeedsForTelemetry(); this subsystem doesn't own the wheels
    double pointing_error_deg;
    double timestamp_s;
};

/*
 * AdcsSystem - orchestrates the estimator + controller behind an
 * explicit mode state machine and a small command interface, similar
 * in spirit to how a real flight software C&DH (command and data
 * handling) layer wraps a control algorithm: the rest of the system
 * talks to modes and commands, not directly to the underlying math.
 *
 * This class deliberately does NOT own the reaction wheels or gyro
 * hardware -- it takes sensor readings in and returns a torque
 * command out, so it can be driven by either a simulator
 * (SpacecraftDynamics, see examples/) or real hardware drivers
 * without any change to this code.
 */
/*
 * Config for AdcsSystem. Declared outside the class for the same
 * reason as SpacecraftConfig above (see spacecraft_dynamics.hpp).
 */
struct AdcsConfig {
    double pointing_tolerance_deg = 1.0;  // SLEWING -> POINTING below this error
    double fault_rate_limit_rad_s = 5.0;  // body rate above this trips a FAULT (e.g. uncontrolled tumble)
    double controller_kp = 1.0;
    double controller_kd = 6.0;
    double estimator_kp = 1.0;
    double estimator_ki = 0.05;
};

class AdcsSystem {
public:
    using Config = AdcsConfig;

    explicit AdcsSystem(const Config& config = Config());

    // Commands a new target attitude; transitions to SLEWING (ignored while in FAULT).
    void commandPointing(const Quaternion& target_attitude);

    // Commands the system back to idle (zero torque commanded). Ignored while in FAULT.
    void commandIdle();

    /*
     * Runs one control cycle:
     *   - if `has_vector_measurement`, applies a vector-sensor correction to the estimator
     *   - integrates the gyro measurement into the attitude estimate
     *   - checks for a fault condition (excessive body rate)
     *   - computes a torque command appropriate to the current mode
     *
     * Returns the torque command (N*m) to apply to the reaction
     * wheels. The caller is responsible for actually applying it
     * (e.g. via SpacecraftDynamics::step() in simulation).
     */
    Vector3 update(const Vector3& gyro_meas, double dt,
                   bool has_vector_measurement = false,
                   const Vector3& measured_body_vec = Vector3(),
                   const Vector3& reference_inertial_vec = Vector3());

    AdcsTelemetry telemetry() const;

    // Seeds the attitude estimate directly, e.g. from a ground-uploaded initial attitude.
    void resetEstimate(const Quaternion& attitude);

    // The estimator doesn't know about real wheel hardware; the
    // caller (e.g. the simulation loop) reports wheel speeds here
    // purely so telemetry() can report them alongside everything else.
    void setWheelSpeedsForTelemetry(const Vector3& speeds) { last_wheel_speeds_ = speeds; }

    AdcsMode mode() const { return mode_; }

private:
    Config config_;
    AttitudeEstimator estimator_;
    AttitudeController controller_;
    AdcsMode mode_;
    Quaternion target_attitude_;
    Vector3 last_rate_estimate_;
    Vector3 last_wheel_speeds_;
    double sim_time_s_;
};

} // namespace simorgh
