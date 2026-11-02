#include "physics/body.hpp"

namespace sh::phys {

void Body::set_mass_as_box(float mass, Vec3 h) {
    if (!(mass > 0)) {
        inv_mass = 0;
        inv_inertia = {};
        return;
    }
    const float sx = (2 * h.x) * (2 * h.x), sy = (2 * h.y) * (2 * h.y), sz = (2 * h.z) * (2 * h.z);
    inv_mass = 1.0f / mass;
    inv_inertia = {12.0f / ((sy + sz) * mass), 12.0f / ((sz + sx) * mass), 12.0f / ((sy + sx) * mass)};
}

void Body::set_mass_as_sphere(float mass, float radius) { set_mass_as_box(mass, {radius, radius, radius}); }

void Body::set_mass_as_infinite_inertia(float mass) {
    inv_inertia = {};
    inv_mass = mass > 0 ? 1.0f / mass : 0.0f;
}

void Body::apply_impulse(Vec3 point, Vec3 impulse) {
    linear_velocity += inv_mass * impulse;
    const Mat3 rot = Mat3::from_quat(rotation);
    const Vec3 r_local = rot.transposed_mul(point - position);
    const Vec3 j_local = rot.transposed_mul(impulse);
    angular_velocity += rot * mul(inv_inertia, cross(r_local, j_local));
}

}  // namespace sh::phys
