#pragma once

// Small column-major 4x4 matrix helpers (OpenGL conventions: right-handed, -z forward).

#include <cmath>

namespace sh {

struct Vec3 {
    float x = 0, y = 0, z = 0;
};
inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }

struct Mat4 {
    float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};  // m[col * 4 + row]

    static Mat4 perspective(float fovy_radians, float aspect, float near_z, float far_z) {
        Mat4 r;
        const float f = 1.0f / std::tan(fovy_radians / 2);
        r.m[0] = f / aspect;
        r.m[5] = f;
        r.m[10] = (far_z + near_z) / (near_z - far_z);
        r.m[11] = -1;
        r.m[14] = 2 * far_z * near_z / (near_z - far_z);
        r.m[15] = 0;
        return r;
    }

    // View matrix for a camera at 'eye' rotated by yaw (about +y) then pitch (about +x).
    static Mat4 fly_camera(Vec3 eye, float yaw, float pitch) {
        const float cy = std::cos(yaw), sy = std::sin(yaw), cp = std::cos(pitch), sp = std::sin(pitch);
        // Camera basis in world space.
        const Vec3 right{cy, 0, -sy};
        const Vec3 up{sy * sp, cp, cy * sp};
        const Vec3 back{sy * cp, -sp, cy * cp};
        Mat4 r;
        r.m[0] = right.x, r.m[4] = right.y, r.m[8] = right.z;
        r.m[1] = up.x, r.m[5] = up.y, r.m[9] = up.z;
        r.m[2] = back.x, r.m[6] = back.y, r.m[10] = back.z;
        r.m[12] = -(right.x * eye.x + right.y * eye.y + right.z * eye.z);
        r.m[13] = -(up.x * eye.x + up.y * eye.y + up.z * eye.z);
        r.m[14] = -(back.x * eye.x + back.y * eye.y + back.z * eye.z);
        return r;
    }

    friend Mat4 operator*(const Mat4& a, const Mat4& b) {
        Mat4 r;
        for (int c = 0; c < 4; ++c)
            for (int row = 0; row < 4; ++row) {
                float s = 0;
                for (int k = 0; k < 4; ++k) s += a.m[k * 4 + row] * b.m[c * 4 + k];
                r.m[c * 4 + row] = s;
            }
        return r;
    }
};

}  // namespace sh
