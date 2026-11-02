#pragma once

// Quaternion rotations written in the exact float order the original's game layer uses (Body,
// generateContacts, Joint, insertJoints all inline the same formulas), so results round identically.

#include "physics/pmath.hpp"

namespace sh::phys {

// v rotated by the inverse of q.
inline Vec3 qi_rotate_inverse(Quat q, Vec3 d) {
    const float k2w = q.w + q.w, kk = q.w * k2w - 1.0f;
    const float dot = d.y * q.y + d.x * q.x + d.z * q.z;
    const float dot2 = dot + dot;
    return {(d.x * kk - k2w * (d.z * q.y - d.y * q.z)) + q.x * dot2,
            (d.y * kk - k2w * (d.x * q.z - d.z * q.x)) + q.y * dot2,
            (d.z * kk - k2w * (d.y * q.x - d.x * q.y)) + q.z * dot2};
}

// v rotated by q.
inline Vec3 qi_rotate(Quat q, Vec3 n) {
    const float k2w = q.w + q.w, kk = q.w * k2w - 1.0f;
    const float dot = n.y * q.y + n.x * q.x + n.z * q.z;
    const float dot2 = dot + dot;
    return {(k2w * (n.z * q.y - n.y * q.z) + n.x * kk) + q.x * dot2,
            (k2w * (n.x * q.z - n.z * q.x) + n.y * kk) + q.y * dot2,
            (k2w * (n.y * q.x - n.x * q.y) + n.z * kk) + q.z * dot2};
}

// conjugate(a) * b (b's rotation seen from a), grouped as the original computes it.
inline Quat qi_relative_rotation(Quat a, Quat b) {
    const float cx = -a.x, cy = -a.y, cz = -a.z;
    return {((b.z * cy - b.y * cz) + a.w * b.x) + b.w * cx, ((b.x * cz - b.z * cx) + a.w * b.y) + b.w * cy,
            ((b.y * cx - b.x * cy) + a.w * b.z) + b.w * cz, a.w * b.w - ((b.y * cy + b.x * cx) + b.z * cz)};
}

// a * b, grouped as the game code computes it (the script bindings, PropertyBag::asQuat).
inline Quat qi_mul(Quat a, Quat b) {
    return {((a.y * b.z - a.z * b.y) + a.w * b.x) + a.x * b.w, ((a.z * b.x - a.x * b.z) + a.w * b.y) + a.y * b.w,
            ((a.x * b.y - a.y * b.x) + a.w * b.z) + a.z * b.w, a.w * b.w - ((a.y * b.y + a.x * b.x) + a.z * b.z)};
}

// Euler angles (radians) as the game composes them: rotations about X, Y and Z as (qx * qy) * qz.
inline Quat qi_euler(float rx, float ry, float rz) {
    return qi_mul(qi_mul(axis_angle({1, 0, 0}, rx), axis_angle({0, 1, 0}, ry)), axis_angle({0, 0, 1}, rz));
}

// q * v + p (a point from a body's local frame to world).
inline Vec3 qi_transform(Vec3 p, Quat q, Vec3 v) {
    const Vec3 r = qi_rotate(q, v);
    return {r.x + p.x, r.y + p.y, r.z + p.z};
}

}  // namespace sh::phys
