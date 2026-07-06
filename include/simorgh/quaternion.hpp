#pragma once
#include <cmath>

namespace simorgh {

struct Vector3 {
    double x{0}, y{0}, z{0};

    Vector3() = default;
    Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vector3 operator+(const Vector3& o) const { return Vector3(x + o.x, y + o.y, z + o.z); }
    Vector3 operator-(const Vector3& o) const { return Vector3(x - o.x, y - o.y, z - o.z); }
    Vector3 operator-() const { return Vector3(-x, -y, -z); }
    Vector3 operator*(double s) const { return Vector3(x * s, y * s, z * s); }
    Vector3& operator+=(const Vector3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vector3& operator-=(const Vector3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }

    double dot(const Vector3& o) const { return x * o.x + y * o.y + z * o.z; }

    Vector3 cross(const Vector3& o) const {
        return Vector3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x);
    }

    double norm() const { return std::sqrt(dot(*this)); }

    Vector3 normalized() const {
        double n = norm();
        return n > 1e-12 ? Vector3(x / n, y / n, z / n) : Vector3(0, 0, 0);
    }
};

inline Vector3 operator*(double s, const Vector3& v) { return v * s; }

/*
 * Attitude quaternion convention used throughout this library: q
 * rotates vectors from the BODY frame into the INERTIAL frame, i.e.
 *
 *     v_inertial = q.rotate(v_body)   which implements   q * v * q^-1
 *
 * This convention is used consistently across quaternion.hpp,
 * attitude_estimator.hpp/.cpp, attitude_controller.hpp/.cpp, and
 * spacecraft_dynamics.hpp/.cpp. If you port any single piece of this
 * out on its own, double check the convention still matches --
 * flipping it silently inverts control loops without any compile
 * error.
 */
struct Quaternion {
    double w{1}, x{0}, y{0}, z{0};

    Quaternion() = default;
    Quaternion(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}

    static Quaternion identity() { return Quaternion(1, 0, 0, 0); }

    static Quaternion fromAxisAngle(const Vector3& axis, double angle_rad) {
        Vector3 a = axis.normalized();
        double half = angle_rad * 0.5;
        double s = std::sin(half);
        return Quaternion(std::cos(half), a.x * s, a.y * s, a.z * s);
    }

    Vector3 vec() const { return Vector3(x, y, z); }

    double normSq() const { return w * w + x * x + y * y + z * z; }
    double norm() const { return std::sqrt(normSq()); }

    Quaternion normalized() const {
        double n = norm();
        if (n < 1e-12) return Quaternion::identity();
        return Quaternion(w / n, x / n, y / n, z / n);
    }

    Quaternion conjugate() const { return Quaternion(w, -x, -y, -z); }

    // Hamilton product: this * other.
    Quaternion operator*(const Quaternion& o) const {
        return Quaternion(
            w * o.w - x * o.x - y * o.y - z * o.z,
            w * o.x + x * o.w + y * o.z - z * o.y,
            w * o.y - x * o.z + y * o.w + z * o.x,
            w * o.z + x * o.y - y * o.x + z * o.w
        );
    }

    Quaternion operator+(const Quaternion& o) const {
        return Quaternion(w + o.w, x + o.x, y + o.y, z + o.z);
    }

    Quaternion operator*(double s) const { return Quaternion(w * s, x * s, y * s, z * s); }

    // Rotates vector v by this quaternion: v_out = q * v * q_conjugate.
    Vector3 rotate(const Vector3& v) const {
        Vector3 qv = vec();
        Vector3 t = 2.0 * qv.cross(v);
        return v + w * t + qv.cross(t);
    }
};

inline Quaternion operator*(double s, const Quaternion& q) { return q * s; }

} // namespace simorgh
