#include "physics/explosion.hpp"

#include "physics/breakage.hpp"
#include "physics/qi_random.hpp"
#include "physics/qi_rotate.hpp"

#include <cfloat>
#include <cmath>
#include <vector>

namespace sh::phys {

namespace {

float distance_to(Vec3 a, Vec3 b) {
    const Vec3 d{a.x - b.x, a.y - b.y, a.z - b.z};
    return std::sqrt((d.y * d.y + d.x * d.x) + d.z * d.z);
}

// Body::isBreakable: some shape has a finite strength.
bool is_breakable(const RigidBody& b) {
    for (const auto& s : b.shapes)
        if (s->material.strength < FLT_MAX) return true;
    return false;
}

}  // namespace

void explosion(PhysicsWorld& world, PhysicsEvents& events, Rand48& rng, const ExplosionEnv& env, Vec3 pos,
               float strength) {
    using Bank = PhysicsEvents::Bank;
    using Effect = PhysicsEvents::Effect;

    // Camera shake: 3 / distance, at most 2, never below the current value.
    if (env.camera_shake) {
        const float s = 3.0f / distance_to(env.camera_position, pos);
        float shake = 2.0f;
        if (s <= 2.0f) shake = s;
        if (*env.camera_shake > shake) shake = *env.camera_shake;
        *env.camera_shake = shake;
    }

    const Vec3 lo{pos.x - 2.0f, pos.y - 2.0f, pos.z - 2.0f}, hi{pos.x + 2.0f, pos.y + 2.0f, pos.z + 2.0f};
    std::vector<RigidBody*> near;
    world.collect(&near, nullptr, lo, hi, 0x7f, nullptr);

    // Breakable bodies break at their closest point to the explosion, with the strength over the squared
    // distance (at least 1).
    struct Hit {
        RigidBody* body;
        Vec3 point, dir;
        float force;
        const BodyShape* shape;
    };
    std::vector<Hit> hits;
    const BodyShape* shape = nullptr;  // carried over when a body reports no closest point, as the original
    for (RigidBody* b : near) {
        if (!is_breakable(*b)) continue;
        const Vec3 local = qi_rotate_inverse(b->rotation, pos - b->position);
        const Vec3 cp = closest_point(world.context, *b, local, &shape, nullptr, nullptr);
        const Vec3 r = qi_rotate(b->rotation, cp);
        const Vec3 point{b->position.x + r.x, b->position.y + r.y, b->position.z + r.z};
        const Vec3 d{point.x - pos.x, point.y - pos.y, point.z - pos.z};
        const float dist = std::sqrt((d.y * d.y + d.x * d.x) + d.z * d.z);
        const float div = dist + FLT_EPSILON;
        const float falloff = 1.0f < dist ? dist * dist : 1.0f;
        hits.push_back({b, point, {d.x / div, d.y / div, d.z / div}, strength / falloff, shape});
    }
    if (!hits.empty()) {
        float volume = 0;
        for (const Hit& h : hits) volume = volume + (h.shape ? h.shape->volume : 0.0f);
        const Bank bank = 0.5f < volume ? Bank::kExplosionBreakBig : 0.1f < volume ? Bank::kExplosionBreakSmall : Bank::kBreak;
        events.play_sound(events.pick_sound(bank), hits.front().body->position, 0.5f, 1.0f);
        for (const Hit& h : hits) break_body(world, *h.body, h.point, h.dir, h.force, 0.8f, rng);
    }

    // Push: dynamic bodies (pieces included) get a velocity change away from the point, falling off to 0 at
    // distance 2 and capped at 7.
    near.clear();
    world.collect(&near, nullptr, lo, hi, 0x7f, nullptr);
    for (RigidBody* b : near) {
        if (!b->dynamic) continue;
        const Vec3 d{b->solver.position.x - pos.x, b->solver.position.y - pos.y, b->solver.position.z - pos.z};
        const float dist = std::sqrt((d.y * d.y + d.x * d.x) + d.z * d.z);
        const float f = 1.0f - dist * 0.5f;
        const float div = dist + FLT_EPSILON;
        const Vec3 n{d.x / div, d.y / div, d.z / div};
        float scale = 0.0f;
        if (0.0f < f) scale = f < 1.0f ? f * f : 1.0f;
        const float k = ((strength * scale) * b->solver.inv_mass) * 0.03f;
        Vec3 imp{k * n.x, k * n.y, k * n.z};
        const float len2 = (imp.y * imp.y + imp.x * imp.x) + imp.z * imp.z;
        if (49.0f < len2) {
            const float len = std::sqrt(len2);
            if (0.0f < len) imp = {(imp.x / len) * 7.0f, (imp.y / len) * 7.0f, (imp.z / len) * 7.0f};
            else imp = {7.0f, 0.0f, 0.0f};
        }
        Vec3& v = b->solver.linear_velocity;
        v = {v.x + imp.x, imp.y + v.y, imp.z + v.z};
    }

    // Particles: 3 rounds of the two explosion systems, then 5 of the +0x194 system; white, flag -1.
    const PhysicsEvents::Rgba white{1, 1, 1, 1};
    auto puff = [&](Effect e, float spread, float speed) {
        const Vec3 o = random_unit(rng, rng.rnd_normal(0.0f, spread));
        const Vec3 p{pos.x + o.x, pos.y + o.y, pos.z + o.z};
        const Vec3 v = random_unit(rng, rng.rnd_normal(0.0f, speed));
        events.spawn_particle(e, p, v, white, -1);
    };
    for (int i = 0; i < 3; ++i) {
        puff(Effect::kExplosionB, 0.2f, 2.0f);
        puff(Effect::kExplosionA, 0.2f, 2.0f);
    }
    for (int i = 0; i < 5; ++i) puff(Effect::kAttached, 0.4f, 1.0f);
    events.play_sound(events.pick_sound(Bank::kExplosion), pos, 1.0f, 1.0f);

    // Debris within reach is woken and pushed away.
    if (env.debris) {
        for (DebrisPiece& piece : env.debris->pieces) {
            const Vec3 d{piece.position.x - pos.x, piece.position.y - pos.y, piece.position.z - pos.z};
            const float dist = std::sqrt((d.y * d.y + d.x * d.x) + d.z * d.z);
            const float falloff = 1.0f < dist ? dist * dist : 1.0f;
            const float f = (strength / falloff) * 0.3f;
            if (!(0.5f < f)) continue;
            const float div = dist + FLT_EPSILON;
            piece.alive = true;
            Vec3& v = piece.velocity;
            v = {v.x + f * (d.x / div), v.y + f * (d.y / div), v.z + f * (d.z / div)};
        }
    }
}

}  // namespace sh::phys
