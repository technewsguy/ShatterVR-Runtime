#pragma once

// Particle systems (the original's ParticleSystem): one per effect of effects/default.xml
// (sparks, smoke, dust, ...), each a capped list of sprites that fly with gravity, damping and a small random jitter,
// spin, and die after their life. Spawning and the jitter draw from the shared lrand48, so they are game state.

#include "core/rand48.hpp"
#include "physics/pmath.hpp"

#include <optional>
#include <string>
#include <vector>

namespace sh::game {

struct Particle {        // 0x3c bytes
    phys::Vec3 pos;      // +0
    phys::Vec3 vel;      // +0xc
    float color[4];      // +0x18
    float rotation = 0;  // +0x28
    float spin = 0;      // +0x2c: radians per second
    float life = 0;      // +0x30
    float age = 0;       // +0x34
    int sprite = 0;      // +0x38: cell of the rows x cols sprite grid
};

class ParticleSystem {
public:
    int rows = 1, cols = 1;               // +0, +4: the sprite grid in coords
    float uv0[2] = {0, 0}, uv1[2] = {0, 0};  // +8..+0x14: coords over the texture size
    int max = 0;                          // +0x24: capacity (0: spawning does nothing)
    float life[2] = {1, 2};               // +0x28, +0x2c: QiRndNormal range
    float rot[2] = {-10, 10};             // +0x30, +0x34: spin range
    float size[2] = {1, 1};               // +0x38, +0x3c: size at birth and death
    float alpha[2] = {1, 1};              // +0x40, +0x44: alpha at birth and death
    float gravity = -10;                  // +0x48
    float damping = 1;                    // +0x4c: velocity factor per update (1 - the file's damping, in [0, 1])
    float blending = 1;                   // +0x50: 1 additive-ish .. 0 (render; default "1")
    bool random_rot = true;               // +0x54: random start rotation
    bool angvel_size = false;             // +0x55 (render)
    std::vector<Particle> particles;      // +0x18 count, +0x20 data

    // ParticleSystem::load: the <effect name=...> of the effects file. texture_w/h: the sprite texture's size as the
    // engine reports it (gfx/sprites.png; twice the decoded size, see), for the coords. False
    // (and nothing set) when the effect is missing.
    bool load(const std::string& xml, const std::string& name, int texture_w, int texture_h);

    // spawn(pos, vel, color, sprite): sprite -1 picks a random cell. A full system reuses its oldest particle.
    void spawn(Rand48& rng, phys::Vec3 pos, phys::Vec3 vel, const float color[4], int sprite);
    // update(): game frame time dt and frame number; slow is the global byte 0x226eda (the time-scale flag),
    // which limits the jitter to every eighth frame.
    void update(Rand48& rng, float dt, int frame, bool slow);
    void translate(float dz);  // the floating origin
    void clear() { particles.clear(); }
};

// One corner of a particle quad, as sprite.glsl takes it (premultiplied in the shader; blend is aAlpha).
struct SpriteVertex {
    float pos[3];
    float uv[2];
    float color[4];
    float blend;
};

// ParticleSystem::render with RenderLevel::addSprite: one quad (four corners: -w-h, +w-h, +w+h, -w+h) per particle,
// in the system's sprite cell, size and alpha eased from birth to death, rotated about z by its rotation, faded out
// within 0.5 + 1.25 units of the camera. A particle without x/y velocity is a streak: turned a quarter about y and
// stretched along z by the camera speed. camera: the level camera (Level +0xec); camera_speed: Level +0x110.
void append_sprites(const ParticleSystem& system, phys::Vec3 camera, float camera_speed, std::vector<SpriteVertex>& out);

}  // namespace sh::game
