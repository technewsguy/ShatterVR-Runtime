#pragma once

// Breakage debris (the original's Debris): small tetrahedra that fly, tumble and
// bounce on the floor under the debris job, outside the solver. The level owns one list (level +0xcc).

#include "core/rand48.hpp"
#include "physics/pmath.hpp"

#include <cstdint>
#include <vector>

namespace sh::phys {

class PhysicsWorld;

struct DebrisPiece {  // 0x74 bytes in the original
    const void* obstacle = nullptr;  // +0: the obstacle of the broken body (stored, not read by Debris)
    Vec3 position;                   // +4
    Quat rotation;                   // +0x10
    Vec3 points[4];                  // +0x20: the tetrahedron in the piece's frame
    std::uint32_t color = 0;         // +0x50: RGBA8 (r in the low byte)
    bool alive = true;               // +0x54: cleared once the piece comes to rest
    Vec3 velocity;                   // +0x58
    Vec3 angular_velocity;           // +0x64
    float floor = -10;               // +0x70: height it bounces at (refreshed by a raycast every 10th frame)
};

// What the debris job draws: the four corners in world space and the colour (0x34 bytes per piece).
struct DebrisVertices {
    Vec3 points[4];
    std::uint32_t color = 0;
};

// What Debris::update reads from the level and the game.
struct DebrisEnv {
    const PhysicsWorld* world = nullptr;  // level +0xc8: floor raycasts
    Vec3 gravity{0, -10, 0};              // Level::getGravity
    float dt = 1.0f / 60.0f;              // game +0x100
    int frame = 0;                        // game +0x104
    float camera_z = 0;                   // level +0xf4: pieces beyond it go
};

class Debris {
public:
    std::vector<DebrisPiece> pieces;
    std::vector<DebrisVertices> vertices;  // rebuilt by update, one per piece

    // Debris::spawn: a piece at the pose with a random size and spin; ignored beyond 400 pieces.
    void spawn(const void* obstacle, Vec3 position, Quat rotation, Vec3 velocity, const float rgba[4], Rand48& rng);
    // Debris::update: moves, bounces and retires pieces, then fills vertices.
    void update(const DebrisEnv& env, Rand48& rng);
    // Debris::translate: shifts every piece along z.
    void translate(float dz);
    void clear() { pieces.clear(); }
};

// Level::getGravity. Without the level object at +0xd0 (scale == nullptr): (0, -10, 0). With it: the
// vector (0, -10 * scale, 0) turned by the level rotation at +0x270: the current room's mgGravity and the
// camera roll (game::Level passes them; the physics step uses this as its gravity).
Vec3 level_gravity(const float* scale, Quat level_rotation);

// QiColor::asUInt32: each channel times 255, truncated, clamped to 0..255; r in the low byte.
std::uint32_t color_as_uint32(const float rgba[4]);

}  // namespace sh::phys
