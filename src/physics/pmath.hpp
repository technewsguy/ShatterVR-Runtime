#pragma once

// Vector/quaternion math for the physics layer. Single precision, like the original.

#include <cmath>

namespace sh::phys {

struct Vec3 {
    float x = 0, y = 0, z = 0;

    float& operator[](int i) { return (&x)[i]; }
    float operator[](int i) const { return (&x)[i]; }
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator-(Vec3 a) { return {-a.x, -a.y, -a.z}; }
inline Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 operator*(float s, Vec3 a) { return a * s; }
inline Vec3& operator+=(Vec3& a, Vec3 b) { return a = a + b; }
inline Vec3 mul(Vec3 a, Vec3 b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }  // per component
inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float length_sq(Vec3 a) { return dot(a, a); }
inline float length(Vec3 a) { return std::sqrt(dot(a, a)); }

struct Quat {
    float x = 0, y = 0, z = 0, w = 1;
};

// Hamilton product a * b (apply b first, then a).
inline Quat operator*(Quat a, Quat b) {
    const Vec3 av{a.x, a.y, a.z}, bv{b.x, b.y, b.z};
    const Vec3 v = a.w * bv + b.w * av + cross(av, bv);
    return {v.x, v.y, v.z, a.w * b.w - dot(av, bv)};
}

inline Quat normalized(Quat q) {
    const float n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!(n2 > 0)) return {};
    const float inv = 1.0f / std::sqrt(n2);
    return {q.x * inv, q.y * inv, q.z * inv, q.w * inv};
}

inline Quat conjugate(Quat q) { return {-q.x, -q.y, -q.z, q.w}; }

// Rotates v by unit quaternion q.
inline Vec3 rotate(Quat q, Vec3 v) {
    const Vec3 u{q.x, q.y, q.z};
    const Vec3 t = 2.0f * cross(u, v);
    return v + q.w * t + cross(u, t);
}

inline Quat axis_angle(Vec3 unit_axis, float angle) {
    const float s = std::sin(angle * 0.5f);
    return {unit_axis.x * s, unit_axis.y * s, unit_axis.z * s, std::cos(angle * 0.5f)};
}

// 3x3 rotation, row-major: r[row][col]. Columns are the body axes in world space.
struct Mat3 {
    float r[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    static Mat3 from_quat(Quat q) {
        const float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
        const float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
        const float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
        Mat3 m;
        m.r[0][0] = 1 - 2 * (yy + zz), m.r[0][1] = 2 * (xy - wz), m.r[0][2] = 2 * (xz + wy);
        m.r[1][0] = 2 * (xy + wz), m.r[1][1] = 1 - 2 * (xx + zz), m.r[1][2] = 2 * (yz - wx);
        m.r[2][0] = 2 * (xz - wy), m.r[2][1] = 2 * (yz + wx), m.r[2][2] = 1 - 2 * (xx + yy);
        return m;
    }
    Vec3 operator*(Vec3 v) const {  // local -> world
        return {r[0][0] * v.x + r[0][1] * v.y + r[0][2] * v.z, r[1][0] * v.x + r[1][1] * v.y + r[1][2] * v.z,
                r[2][0] * v.x + r[2][1] * v.y + r[2][2] * v.z};
    }
    Vec3 transposed_mul(Vec3 v) const {  // world -> local
        return {r[0][0] * v.x + r[1][0] * v.y + r[2][0] * v.z, r[0][1] * v.x + r[1][1] * v.y + r[2][1] * v.z,
                r[0][2] * v.x + r[1][2] * v.y + r[2][2] * v.z};
    }
};

// Rigid transform: world = r * local + p.
struct Pose {
    Mat3 r;
    Vec3 p;
    Vec3 apply(Vec3 v) const { return r * v + p; }
    Vec3 inverse_apply(Vec3 v) const { return r.transposed_mul(v - p); }
};

}  // namespace sh::phys
