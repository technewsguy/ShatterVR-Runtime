#include "game/viewport.hpp"

#include "physics/qi_rotate.hpp"

#include <cmath>

namespace sh::game {

Mat4 mat4_mul(const Mat4& a, const Mat4& b) {
    Mat4 r;
    for (int c = 0; c < 4; ++c)
        for (int row = 0; row < 4; ++row) {
            const float* bc = &b.m[4 * c];
            r.m[4 * c + row] = ((a.m[4 + row] * bc[1] + a.m[row] * bc[0]) + a.m[8 + row] * bc[2]) + a.m[12 + row] * bc[3];
        }
    return r;
}

// The adjugate by 2x2 sub-products, then the determinant along the first column. Every partial product and sum is
// formed in the original's order (the shared sub-products are named by the two entries they multiply).
bool mat4_invert(Mat4& mat) {
    float* o = mat.m;
    const float m0 = o[0], m1 = o[1], m2 = o[2], m3 = o[3], m4 = o[4], m5 = o[5], m6 = o[6], m7 = o[7];
    const float m8 = o[8], m9 = o[9], m10 = o[10], m11 = o[11], m12 = o[12], m13 = o[13], m14 = o[14], m15 = o[15];
    const float p15_2 = m15 * m2, p14_3 = m14 * m3, p14_11 = m14 * m11, p15_10 = m15 * m10;
    const float p14_7 = m14 * m7, p15_6 = m15 * m6, p11_2 = m11 * m2, p10_3 = m10 * m3;
    const float p11_6 = m11 * m6, p10_7 = m10 * m7, p6_3 = m6 * m3, p7_2 = m7 * m2;

    // Column 0 of the adjugate.
    const float a1 = ((p15_2 * m9 + p14_11 * m1) + p10_3 * m13) - ((p14_3 * m9 + p15_10 * m1) + p11_2 * m13);
    const float a0 = ((p14_7 * m9 + p15_10 * m5) + p11_6 * m13) - ((p15_6 * m9 + p14_11 * m5) + p10_7 * m13);
    const float a2 = ((p14_3 * m5 + p15_6 * m1) + p7_2 * m13) - ((p15_2 * m5 + p14_7 * m1) + p6_3 * m13);
    const float a3 = ((p11_2 * m5 + p10_7 * m1) + p6_3 * m9) - ((p10_3 * m5 + p11_6 * m1) + p7_2 * m9);

    const float p9_4 = m9 * m4, p5_12 = m5 * m12, p1_12 = m1 * m12, p9_0 = m9 * m0, p1_4 = m1 * m4;
    const float p5_8 = m5 * m8, p1_8 = m1 * m8, p5_0 = m5 * m0, p9_12 = m9 * m12;
    const float p13_8 = m13 * m8, p13_4 = m13 * m4, p13_0 = m13 * m0;

    // Columns 1 to 3.
    const float a4 = ((p15_6 * m8 + p14_11 * m4) + p10_7 * m12) - ((p14_7 * m8 + p15_10 * m4) + p11_6 * m12);
    const float a5 = ((p14_3 * m8 + p15_10 * m0) + p11_2 * m12) - ((p15_2 * m8 + p14_11 * m0) + p10_3 * m12);
    const float a6 = ((p15_2 * m4 + p14_7 * m0) + p6_3 * m12) - ((p14_3 * m4 + p15_6 * m0) + p7_2 * m12);
    const float a7 = ((p10_3 * m4 + p11_6 * m0) + p7_2 * m8) - ((p11_2 * m4 + p10_7 * m0) + p6_3 * m8);
    const float a8 = ((m11 * p5_12 + m7 * p13_8) + m15 * p9_4) - ((m11 * p13_4 + m7 * p9_12) + m15 * p5_8);
    const float a9 = ((m11 * p13_0 + m3 * p9_12) + m15 * p1_8) - ((m11 * p1_12 + m3 * p13_8) + m15 * p9_0);
    const float a10 = ((m7 * p1_12 + m3 * p13_4) + m15 * p5_0) - ((m7 * p13_0 + m3 * p5_12) + m15 * p1_4);
    const float a11 = ((m7 * p9_0 + m3 * p5_8) + m11 * p1_4) - ((m7 * p1_8 + m3 * p9_4) + m11 * p5_0);
    const float a12 = ((m14 * p5_8 + m10 * p13_4) + m6 * p9_12) - ((m6 * p13_8 + m14 * p9_4) + m10 * p5_12);
    const float a13 = ((m2 * p13_8 + m14 * p9_0) + m10 * p1_12) - ((m14 * p1_8 + m10 * p13_0) + m2 * p9_12);
    const float a14 = ((m14 * p1_4 + m6 * p13_0) + m2 * p5_12) - ((m2 * p13_4 + m14 * p5_0) + m6 * p1_12);
    const float a15 = ((m2 * p9_4 + m10 * p5_0) + m6 * p1_8) - ((m10 * p1_4 + m6 * p9_0) + m2 * p5_8);

    const float det = ((a1 * m4 + a0 * m0) + a2 * m8) + a3 * m12;
    const float adj[16] = {a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15};
    for (int i = 0; i < 16; ++i) o[i] = adj[i];
    if (det == 0.0f) return false;
    const float inv = 1.0f / det;
    for (int i = 0; i < 16; ++i) o[i] = adj[i] * inv;
    return true;
}

phys::Vec3 quat_base(phys::Quat q, int i) {
    if (i == 0) {
        const float s = q.z * q.z + q.y * q.y;
        const float a = q.z * q.w + q.y * q.x;
        const float b = q.z * q.x - q.y * q.w;
        return {1.0f - (s + s), a + a, b + b};
    }
    if (i == 1) {
        const float s = q.z * q.z + q.x * q.x;
        const float a = q.x * q.y - q.z * q.w;
        const float b = q.x * q.w + q.y * q.z;
        return {a + a, 1.0f - (s + s), b + b};
    }
    const float s = q.y * q.y + q.x * q.x;
    const float a = q.y * q.w + q.x * q.z;
    const float b = q.z * q.y - q.x * q.w;
    return {a + a, b + b, 1.0f - (s + s)};
}

void Viewport::set_bounds(int ax0, int ay0, int ax1, int ay1) {
    x0 = ax0, y0 = ay0, x1 = ax1, y1 = ay1;
    update_projection();
}

void Viewport::set_mode_3d(float f, float n, float fa) {
    fov = f, near_plane = n, far_plane = fa;
    mode = k3D;
    update_projection();
    update_modelview();
}

void Viewport::set_camera_pos(phys::Vec3 p) {
    camera_pos = p;
    update_modelview();
}

void Viewport::set_camera_rot(phys::Quat q) {
    camera_rot = q;
    update_modelview();
}

void Viewport::update_projection() {
    if (mode != k3D) return;  // the flat modes (pixel, ortho, 2D) are not ported
    const float t = std::tan((fov * 3.14159274f) / 360.0f);
    const float fx = near_plane * t;
    float a = 1.0f;
    if (y0 != y1 && x0 != x1) a = aspect != 0.0f ? aspect : static_cast<float>(x1 - x0) / static_cast<float>(y1 - y0);
    const float fy = fx / a;
    if (fx == 0.0f || fy == 0.0f) return;
    float* p = projection.m;
    const float n2 = near_plane + near_plane, f = far_plane;
    p[1] = p[2] = p[3] = p[4] = p[6] = p[7] = p[12] = p[13] = p[15] = 0.0f;
    p[11] = -1.0f;
    p[10] = (-f - near_plane) / (f - near_plane);
    p[14] = -(n2 * f) / (f - near_plane);
    p[8] = (fx - fx) / (fx + fx);
    p[0] = n2 / (fx + fx);
    p[5] = n2 / (fy + fy);
    p[9] = (fy - fy) / (fy + fy);
}

void Viewport::update_modelview() {
    float* m = modelview.m;
    if (mode == k3D) {
        const phys::Quat nq{-camera_rot.x, -camera_rot.y, -camera_rot.z, -camera_rot.w};
        const phys::Vec3 b0 = quat_base(nq, 0), b1 = quat_base(nq, 1), b2 = quat_base(nq, 2);
        m[0] = b0.x, m[4] = b0.y, m[8] = b0.z;
        m[1] = b1.x, m[5] = b1.y, m[9] = b1.z;
        m[2] = b2.x, m[6] = b2.y, m[10] = b2.z;
        m[3] = m[7] = m[11] = 0.0f;
        m[15] = 1.0f;
        const phys::Vec3 t = phys::qi_rotate_inverse(camera_rot, camera_pos);
        m[12] = -t.x, m[13] = -t.y, m[14] = -t.z;
        return;
    }
    if (mode == 5) return;
    for (float& v : modelview.m) v = 0.0f;
    m[0] = m[5] = m[10] = m[15] = 1.0f;
    if (mode == k2D) m[12] = -camera_pos.x, m[13] = -camera_pos.y;
}

phys::Vec3 Viewport::unproject(phys::Vec3 pixel) const {
    Mat4 inv = mat4_mul(projection, modelview);
    if (!mat4_invert(inv)) return {0, 0, 0};
    const float* m = inv.m;
    const float h = static_cast<float>(y1 - y0);
    const float ny01 = (h - (static_cast<float>(y0) + pixel.y)) / h;
    float nz = mode == k3D ? pixel.z : 0.0f;
    const float nx01 = (pixel.x - static_cast<float>(x0)) / static_cast<float>(x1 - x0);
    const float ny = (ny01 + ny01) - 1.0f;
    if (mode == k3D) nz = (nz + nz) - 1.0f;
    const float nx = (nx01 + nx01) - 1.0f;
    const float w = ((ny * m[7] + nx * m[3]) + nz * m[11]) + m[15];
    if (w == 0.0f) return {0, 0, 0};
    const float x = ((ny * m[4] + nx * m[0]) + nz * m[8]) + m[12];
    const float y = ((ny * m[5] + nx * m[1]) + nz * m[9]) + m[13];
    const float z = ((ny * m[6] + nx * m[2]) + nz * m[10]) + m[14];
    const float r = 1.0f / w;
    return {r * x, r * y, r * z};
}

phys::Vec3 Viewport::pixel_direction(float px, float py) const {
    if (mode != k3D || x0 == x1 || y0 == y1) return {-0.0f, -0.0f, -1.0f};  // minus the +z axis global
    const float w = static_cast<float>(x1 - x0), h = static_cast<float>(y1 - y0);
    float a = aspect;
    if (a == 0.0f) a = w / h;
    const float ny = -py / h;
    const float ia = 1.0f / a;
    float y = ia + (ny + ny) * ia;
    float x = px / w;
    x = (x + x) - 1.0f;
    const float len = std::sqrt((y * y + x * x) + 0x1.7aacdap+1f);  // 2.9583 (0x403d566d): about 1.72 squared
    float z;
    if (len > 0.0f) {
        x = x / len, z = -1.72f / len, y = y / len;
    } else {
        x = 1.0f, z = 0.0f, y = 0.0f;
    }
    return phys::qi_rotate(camera_rot, {x, y, z});
}

}  // namespace sh::game

namespace sh::game {

void Viewport::translate(phys::Vec3 t) {
    Mat4 m;
    m.m[12] = t.x, m.m[13] = t.y, m.m[14] = t.z;
    modelview = mat4_mul(modelview, m);
}

Mat4 pose_matrix(phys::Vec3 p, phys::Quat q) {
    const float x = q.x, y = q.y, z = q.z, w = q.w;
    const float yz2 = y * y + z * z, yx2 = y * y + x * x, zx2 = z * z + x * x;
    Mat4 m;
    m.m[0] = 1.0f - (yz2 + yz2);
    m.m[5] = 1.0f - (zx2 + zx2);
    m.m[10] = 1.0f - (yx2 + yx2);
    float v;
    v = y * x + z * w, m.m[1] = v + v;
    v = z * x - y * w, m.m[2] = v + v;
    v = y * x - z * w, m.m[4] = v + v;
    v = y * z + x * w, m.m[6] = v + v;
    v = z * x + y * w, m.m[8] = v + v;
    v = y * z - x * w, m.m[9] = v + v;
    m.m[12] = p.x, m.m[13] = p.y, m.m[14] = p.z;
    return m;
}

void Viewport::rotate(phys::Quat q) { modelview = mat4_mul(modelview, pose_matrix({0, 0, 0}, q)); }

void Viewport::scale(phys::Vec3 s) {
    Mat4 m;
    m.m[0] = s.x, m.m[5] = s.y, m.m[10] = s.z;
    modelview = mat4_mul(modelview, m);
}

phys::Vec3 Viewport::project(phys::Vec3 p) const {
    const float* mv = modelview.m;
    const float* pr = projection.m;
    const float ex = ((p.y * mv[4] + p.x * mv[0]) + p.z * mv[8]) + mv[12];
    const float ey = ((p.y * mv[5] + p.x * mv[1]) + p.z * mv[9]) + mv[13];
    const float ez = ((p.y * mv[6] + p.x * mv[2]) + p.z * mv[10]) + mv[14];
    const float ew = ((p.y * mv[7] + p.x * mv[3]) + p.z * mv[11]) + mv[15];
    float cx = ((ey * pr[4] + ex * pr[0]) + ez * pr[8]) + ew * pr[12];
    float cy = ((ey * pr[5] + ex * pr[1]) + ez * pr[9]) + ew * pr[13];
    float cz = ((ey * pr[6] + ex * pr[2]) + ez * pr[10]) + ew * pr[14];
    if (mode == k3D) {
        if (-ez == 0.0f) return {0, 0, 0};
        const float k = 1.0f / -ez;
        cx = cx * k, cy = cy * k, cz = cz * k;
    }
    const float w = static_cast<float>(x1 - x0), h = static_cast<float>(y1 - y0);
    return {static_cast<float>(x0) + (cx * 0.5f + 0.5f) * w, h - (static_cast<float>(y0) + h * (cy * 0.5f + 0.5f)),
            (cz + 1.0f) * 0.5f};
}

}  // namespace sh::game
