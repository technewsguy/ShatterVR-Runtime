#pragma once

// The engine's camera math (QiMatrix4 multiply / invert, QiViewport), in the exact float order of the original so
// that unprojected directions round the same. Matrices are column-major, 16 floats.

#include "physics/pmath.hpp"

namespace sh::game {

struct Mat4 {
    float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
};

// QiMatrix4::operator*: a * b.
Mat4 mat4_mul(const Mat4& a, const Mat4& b);
// QiMatrix4::invert: in place; false (with the adjugate left in m) when the determinant is 0.
bool mat4_invert(Mat4& m);

// QiMatrix4(QiTransform3): the column-major matrix of a pose (rotation from the quaternion, then the position).
Mat4 pose_matrix(phys::Vec3 p, phys::Quat q);

// QiQuat::getBase(i): the rotated x (0), y (1) or z (2) axis.
phys::Vec3 quat_base(phys::Quat q, int i);

class Viewport {
public:
    enum Mode { kPixel = 0, kPixelFlipped = 1, kOrtho = 2, k2D = 3, k3D = 4 };

    void init(int ax0, int ay0, int ax1, int ay1) { set_bounds(ax0, ay0, ax1, ay1); }  // QiViewport::init
    void set_bounds(int ax0, int ay0, int ax1, int ay1);                        // QiViewport::setBounds
    void set_mode_3d(float fov_degrees, float near_plane, float far_plane);     // QiViewport::setMode3D
    void set_camera_pos(phys::Vec3 p);                                          // QiViewport::setCameraPos
    void set_camera_rot(phys::Quat q);                                          // QiViewport::setCameraRot
    void update_projection();  // QiViewport::updateProjection (the perspective case; others are not ported)
    void update_modelview();   // QiViewport::updateModelview
    // QiViewport::translate / rotate / scale: the modelview times that transform (applied before it).
    void translate(phys::Vec3 t);
    void rotate(phys::Quat q);
    void scale(phys::Vec3 s);
    // QiViewport::project: a point to a pixel (y down) and a depth in [0, 1]; in 3D divided by minus the eye z
    // (0, 0, 0 when that is 0).
    phys::Vec3 project(phys::Vec3 p) const;
    // QiViewport::unproject: a pixel (y down) and a depth in [0, 1] to world space; (0, 0, 0) when singular.
    phys::Vec3 unproject(phys::Vec3 pixel) const;
    // QiViewport::getPixelDirection: the view ray through a pixel (a fixed 1.72 focal length), rotated by the camera.
    phys::Vec3 pixel_direction(float px, float py) const;

    int mode = kOrtho;                  // +0x0
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0; // +0x4..+0x10
    float f14 = 0;                      // +0x14 (a 2D rotation for the flat modes)
    float f18 = 0.7853982f;             // +0x18
    float aspect = 0;                   // +0x1c: 0 = width / height
    float fov = 0, near_plane = 0, far_plane = 0;  // +0x20..+0x28
    phys::Vec3 camera_pos;              // +0x2c
    phys::Quat camera_rot;              // +0x38
    Mat4 projection;                    // +0x48
    Mat4 modelview;                     // +0x88
};

}  // namespace sh::game
