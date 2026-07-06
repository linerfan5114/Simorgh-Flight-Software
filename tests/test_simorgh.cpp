/*
 * test_simorgh.cpp - Unit tests for the quaternion math, estimator,
 * controller, and full ADCS control loop. Simple assert-based tests
 * (no external test framework needed), mirroring the style used
 * elsewhere in this project's sibling repos.
 */
#include "simorgh/quaternion.hpp"
#include "simorgh/attitude_estimator.hpp"
#include "simorgh/attitude_controller.hpp"
#include "simorgh/adcs_system.hpp"
#include "simorgh/spacecraft_dynamics.hpp"

#include <cstdio>
#include <cmath>

using namespace simorgh;

static int g_failures = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { std::printf("FAIL: %s\n", msg); g_failures++; } \
    else { std::printf("PASS: %s\n", msg); } \
} while (0)

namespace {
constexpr double kPi = 3.14159265358979323846;
}

static void test_quaternion_identity_rotation() {
    Quaternion q = Quaternion::identity();
    Vector3 v(1, 2, 3);
    Vector3 out = q.rotate(v);
    CHECK(std::fabs(out.x - v.x) < 1e-9 && std::fabs(out.y - v.y) < 1e-9 &&
          std::fabs(out.z - v.z) < 1e-9,
          "identity quaternion leaves vectors unchanged");
}

static void test_quaternion_90deg_rotation() {
    // 90 degrees about Z should send X-axis to Y-axis.
    Quaternion q = Quaternion::fromAxisAngle(Vector3(0, 0, 1), kPi / 2.0);
    Vector3 out = q.rotate(Vector3(1, 0, 0));
    CHECK(std::fabs(out.x) < 1e-6 && std::fabs(out.y - 1.0) < 1e-6 && std::fabs(out.z) < 1e-6,
          "90-degree rotation about Z maps X-axis to Y-axis");
}

static void test_quaternion_from_zero_axis_returns_valid_identity() {
    // Regression test: fromAxisAngle used to return a non-unit
    // quaternion (norm != 1) when given a degenerate zero-length
    // axis, silently corrupting any attitude math built on top of it.
    Quaternion q = Quaternion::fromAxisAngle(Vector3(0, 0, 0), 1.5707963);
    CHECK(std::fabs(q.norm() - 1.0) < 1e-9,
          "fromAxisAngle with a zero-length axis returns a valid unit quaternion (regression)");
}

static void test_quaternion_conjugate_inverts_rotation() {
    Quaternion q = Quaternion::fromAxisAngle(Vector3(1, 1, 0), 0.7);
    Vector3 v(0.3, -0.5, 0.9);
    Vector3 rotated = q.rotate(v);
    Vector3 back = q.conjugate().rotate(rotated);
    CHECK(std::fabs(back.x - v.x) < 1e-6 && std::fabs(back.y - v.y) < 1e-6 &&
          std::fabs(back.z - v.z) < 1e-6,
          "rotating then rotating by the conjugate recovers the original vector");
}

static void test_estimator_tracks_known_rotation() {
    // Rotate at a constant, precisely-known rate with zero bias/noise;
    // after propagating for a known duration, the estimate should
    // match the analytically expected attitude.
    AttitudeEstimator est(1.0, 0.0);
    est.reset(Quaternion::identity());

    Vector3 rate(0, 0, 0.5); // rad/s about Z
    double dt = 0.001;
    int steps = 1000; // 1 second total -> 0.5 rad rotation about Z

    for (int i = 0; i < steps; ++i) {
        est.predict(rate, dt);
    }

    Quaternion expected = Quaternion::fromAxisAngle(Vector3(0, 0, 1), 0.5);
    Quaternion actual = est.attitude();
    double dot = std::fabs(expected.w * actual.w + expected.x * actual.x +
                            expected.y * actual.y + expected.z * actual.z);
    CHECK(dot > 0.999, "pure gyro integration matches analytical rotation for a known-rate maneuver");
}

static void test_estimator_corrects_bias_with_vector_measurements() {
    // A biased gyro with no correction should drift. The same biased
    // gyro *with* periodic vector-measurement correction should stay
    // close to the true (unchanging) attitude.
    Vector3 bias_truth(0.02, 0, 0); // deliberately large bias to make drift obvious
    Vector3 reference_inertial(0, 0, 1);
    double dt = 0.01;
    int steps = 2000; // 20 seconds

    AttitudeEstimator uncorrected(0.0, 0.0); // kp=0 => corrections have no effect, drifts freely
    uncorrected.reset(Quaternion::identity());
    for (int i = 0; i < steps; ++i) {
        uncorrected.predict(bias_truth, dt); // true rate is zero; gyro reads pure bias
    }

    AttitudeEstimator corrected(2.0, 0.1);
    corrected.reset(Quaternion::identity());
    for (int i = 0; i < steps; ++i) {
        if (i % 10 == 0) {
            // True attitude never changes, so the reference vector
            // always appears at `reference_inertial` in the body frame too.
            corrected.correct(reference_inertial, reference_inertial);
        }
        corrected.predict(bias_truth, dt);
    }

    Quaternion identity = Quaternion::identity();
    auto angleFrom = [&](const Quaternion& q) {
        double dot = std::fabs(q.w * identity.w + q.x * identity.x + q.y * identity.y + q.z * identity.z);
        dot = std::min(1.0, dot);
        return 2.0 * std::acos(dot) * 180.0 / kPi;
    };

    double drift_uncorrected = angleFrom(uncorrected.attitude());
    double drift_corrected = angleFrom(corrected.attitude());

    CHECK(drift_uncorrected > 1.0, "uncorrected biased gyro drifts noticeably over 20 seconds");
    CHECK(drift_corrected < drift_uncorrected,
          "vector-measurement correction keeps attitude estimate closer to truth than uncorrected drift");
}

static void test_controller_torque_direction() {
    // Current attitude is identity; target is a small positive
    // rotation about Z. The controller should command a torque with
    // a positive Z component to rotate toward it.
    AttitudeController controller(1.0, 0.0);
    Quaternion current = Quaternion::identity();
    Quaternion target = Quaternion::fromAxisAngle(Vector3(0, 0, 1), 0.2);

    Vector3 torque = controller.computeTorque(current, Vector3(0, 0, 0), target, Vector3(0, 0, 0));
    CHECK(torque.z > 0.0, "controller commands positive Z torque to close a positive Z attitude error");
}

static void test_full_loop_converges_to_pointing() {
    // Closed-loop test: spacecraft + estimator + controller together
    // should drive a real (simulated) rigid body to the commanded
    // attitude and settle into POINTING mode.
    SpacecraftDynamics dynamics;
    AdcsSystem adcs;

    Quaternion start = Quaternion::identity();
    dynamics.reset(start, Vector3(0.01, -0.01, 0.02));
    adcs.resetEstimate(start);

    Quaternion target = Quaternion::fromAxisAngle(Vector3(0, 1, 0), kPi / 3.0); // 60 degrees about Y
    adcs.commandPointing(target);

    Vector3 gyro_bias(0.001, 0.0, -0.0012);
    Vector3 reference_inertial(1, 0, 0);
    double dt = 0.05;
    int steps = 1200; // 60 seconds

    for (int i = 0; i < steps; ++i) {
        Vector3 gyro_meas = dynamics.simulateGyroMeasurement(gyro_bias, 0.0003);
        bool has_vec = (i % 20 == 0);
        Vector3 measured_vec = has_vec
            ? dynamics.simulateVectorMeasurement(reference_inertial, 0.005)
            : Vector3();

        Vector3 torque = adcs.update(gyro_meas, dt, has_vec, measured_vec, reference_inertial);
        dynamics.step(torque, dt);
    }

    AdcsTelemetry tm = adcs.telemetry();
    CHECK(tm.mode == AdcsMode::POINTING, "closed-loop simulation settles into POINTING mode within 60s");
    CHECK(tm.pointing_error_deg < 2.0, "closed-loop simulation converges to within 2 degrees of target");
}

static void test_fault_detection_on_excessive_rate() {
    AdcsSystem adcs;
    adcs.resetEstimate(Quaternion::identity());
    adcs.commandPointing(Quaternion::identity());

    // A wildly excessive body rate (e.g. simulating an uncontrolled
    // tumble) should trip the fault mode.
    Vector3 extreme_rate(10.0, 0, 0); // rad/s, far above default fault_rate_limit_rad_s = 5.0
    adcs.update(extreme_rate, 0.1);

    CHECK(adcs.mode() == AdcsMode::FAULT, "excessive body rate trips FAULT mode");

    Vector3 torque = adcs.update(Vector3(0, 0, 0), 0.1); // even with a sane rate now
    CHECK(torque.x == 0.0 && torque.y == 0.0 && torque.z == 0.0,
          "no torque is commanded while latched in FAULT mode");
}

static void test_dynamics_rejects_nonpositive_dt() {
    // Regression test: dt=0 used to divide by zero internally,
    // silently corrupting the body rate with NaN.
    SpacecraftDynamics dyn;
    dyn.reset(Quaternion::identity(), Vector3(0, 0, 0));
    dyn.step(Vector3(0.1, 0, 0), 0.0);
    Vector3 rate = dyn.bodyRate();
    CHECK(!std::isnan(rate.x) && !std::isnan(rate.y) && !std::isnan(rate.z),
          "SpacecraftDynamics::step rejects dt=0 instead of producing NaN (regression)");
}

int main() {
    test_quaternion_identity_rotation();
    test_quaternion_90deg_rotation();
    test_quaternion_from_zero_axis_returns_valid_identity();
    test_quaternion_conjugate_inverts_rotation();
    test_estimator_tracks_known_rotation();
    test_estimator_corrects_bias_with_vector_measurements();
    test_controller_torque_direction();
    test_full_loop_converges_to_pointing();
    test_fault_detection_on_excessive_rate();
    test_dynamics_rejects_nonpositive_dt();

    std::printf("\n%s\n", g_failures == 0 ? "ALL TESTS PASSED" : "SOME TESTS FAILED");
    return g_failures == 0 ? 0 : 1;
}
