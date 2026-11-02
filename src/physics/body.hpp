#pragma once

// Rigid body as seen by the solver. Behaviour:

#include "physics/pmath.hpp"

#include <cstdint>

namespace sh::phys {

struct Body {
    static constexpr std::uint32_t kStatic = 1;

    std::uint32_t flags = 0;
    float inv_mass = 0;
    Vec3 inv_inertia;  // diagonal, body-local
    Vec3 position;
    Quat rotation;
    Vec3 linear_velocity;   // world
    Vec3 angular_velocity;  // world
    int solver_index = -1;  // set by Solver::insert_body
    float contact_impulse = 0;  // sum of contact normal impulses this step; cleared by integrate_velocity

    bool is_static() const { return (flags & kStatic) != 0; }

    // Inertia helpers, matching the original's formulas (a sphere uses the cube formula).
    void set_mass_as_box(float mass, Vec3 half_extents);
    void set_mass_as_sphere(float mass, float radius);
    void set_mass_as_infinite_inertia(float mass);

    // World-space impulse applied at a world-space point.
    void apply_impulse(Vec3 point, Vec3 impulse);
};

}  // namespace sh::phys
