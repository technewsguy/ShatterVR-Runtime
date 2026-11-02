#pragma once

// Level::explosion: breaks breakable bodies near the point, pushes dynamic
// bodies and debris away, shakes the camera, and asks for sounds and particles.

#include "core/rand48.hpp"
#include "physics/debris.hpp"
#include "physics/physics_world.hpp"

namespace sh::phys {

// What Level::explosion reads and writes outside the physics.
struct ExplosionEnv {
    Vec3 camera_position;       // level +0xec
    float* camera_shake = nullptr;  // game +8 -> +0x8cc: raised to min(3 / distance, 2), never lowered
    Debris* debris = nullptr;   // level +0xcc
};

void explosion(PhysicsWorld& world, PhysicsEvents& events, Rand48& rng, const ExplosionEnv& env, Vec3 pos,
               float strength);

}  // namespace sh::phys
