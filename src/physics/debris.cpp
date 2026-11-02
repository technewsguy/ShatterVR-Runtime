#include "physics/debris.hpp"

#include "physics/collision.hpp"
#include "physics/physics_world.hpp"
#include "physics/qi_random.hpp"

#include <cmath>

namespace sh::phys {

namespace {

// QiQuat::setAxisAngle: sinf/cosf of the half angle.
Quat set_axis_angle(Vec3 axis, float angle) {
    const float half = angle * 0.5f;
    const float s = std::sin(half);
    return {axis.x * s, axis.y * s, axis.z * s, std::cos(half)};
}

// r * q, grouped as Debris::update computes it.
Quat spin(Quat r, Quat q) {
    return {((r.y * q.z - r.z * q.y) + q.x * r.w) + r.x * q.w, ((r.z * q.x - r.x * q.z) + q.y * r.w) + r.y * q.w,
            ((r.x * q.y - r.y * q.x) + q.z * r.w) + r.z * q.w, r.w * q.w - ((r.y * q.y + r.x * q.x) + r.z * q.z)};
}

float square_length(Vec3 v) { return (v.y * v.y + v.x * v.x) + v.z * v.z; }

}  // namespace

std::uint32_t color_as_uint32(const float rgba[4]) {
    std::uint32_t out = 0;
    for (int k = 0; k < 4; ++k) {
        // The original truncates with a saturating conversion (NaN -> 0) and clamps to 0..255.
        const float f = rgba[k] * 255.0f;
        std::uint32_t c = 0;
        if (!(f < 1.0f)) c = f >= 255.0f ? 255u : static_cast<std::uint32_t>(static_cast<int>(f));
        out |= c << (8 * k);
    }
    return out;
}

Vec3 level_gravity(const float* scale, Quat q) {
    if (!scale) return {0, -10, 0};
    // The vector (c, -10 scale, c) with c = 0 turned by q, every term kept (signed zeros included).
    const float c = 0.0f;
    const float k2w = q.w + q.w, kk = q.w * k2w - 1.0f;
    const float g = -(*scale * 10.0f);
    const float cx = q.x * c, cz = q.z * c, cy = q.y * c;
    const float a = (cz - cx) * k2w + g * kk;
    const float u = g * q.x - cy;
    const float w = cy - g * q.z;
    const float dot = (cx + g * q.y) + cz;
    const float dot2 = dot + dot;
    const float base = kk * c;
    return {(base + k2w * w) + q.x * dot2, a + q.y * dot2, (base + k2w * u) + q.z * dot2};
}

void Debris::spawn(const void* obstacle, Vec3 position, Quat rotation, Vec3 velocity, const float rgba[4],
                   Rand48& rng) {
    if (pieces.size() > 400) return;
    DebrisPiece& p = pieces.emplace_back();
    p.obstacle = obstacle;
    p.position = position;
    p.rotation = rotation;
    const float k = rng.rnd_normal(0.75f, 1.5f);
    p.velocity = {k * velocity.x, k * velocity.y, k * velocity.z};
    const Vec3 w = random_unit(rng, 1.0f);
    p.angular_velocity = {w.x * 4.0f, w.y * 4.0f, w.z * 4.0f};
    p.alive = true;
    p.floor = -10.0f;
    // Size: two squared bell draws for width and depth, height between 0.2 and 0.5 of the width; 60% of
    // pieces are half as wide and twice as deep.
    float n = rng.rnd_normal(0.0f, 1.0f);
    float a = (n * 0.1f) * n;
    n = rng.rnd_normal(0.0f, 1.0f);
    float b = (n * 0.1f) * n;
    if (rng.rnd(0.0f, 1.0f) < 0.6f) {
        a = a * 0.5f;
        b = b + b;
    }
    const float c = rng.rnd(a * 0.2f, a * 0.5f);
    const float x = (a + 0.02f) * 0.9f, y = (c + 0.02f) * 0.9f, z = (b + 0.02f) * 0.9f;
    p.points[0] = {-x, -y, z};
    p.points[1] = {x, -y, 0};
    p.points[2] = {-x, -y, -z};
    p.points[3] = {0, y, 0};
    const float half[4] = {rgba[0] * 0.5f, rgba[1] * 0.5f, rgba[2] * 0.5f, rgba[3]};
    p.color = color_as_uint32(half);
}

void Debris::update(const DebrisEnv& env, Rand48& rng) {
    constexpr Vec3 kUp{0, 1, 0};
    vertices.resize(pieces.size());
    const float dt = env.dt;
    for (size_t i = 0; i < pieces.size();) {
        DebrisPiece& p = pieces[i];
        // Gone past the camera or below the level: the last piece takes the slot (and is handled next).
        bool remove = p.position.z > env.camera_z || p.position.y < -10.0f;
        if (!remove && !p.alive) remove = pieces.size() > 200;  // resting pieces go once the list is long
        if (remove) {
            p = pieces.back();
            pieces.pop_back();
            continue;
        }
        if (p.alive) {
            const Vec3 g = env.gravity;
            const float w2 = square_length(p.angular_velocity);
            Vec3& v = p.velocity;
            v = {v.x + dt * g.x, v.y + dt * g.y, v.z + dt * g.z};
            p.position = {p.position.x + dt * v.x, p.position.y + dt * v.y, p.position.z + dt * v.z};
            const float len = std::sqrt(w2);
            if (len > 0.0001f) {
                const Vec3& w = p.angular_velocity;
                p.rotation = spin(p.rotation, set_axis_angle({w.x / len, w.y / len, w.z / len}, len * dt));
            }
            if ((static_cast<int>(i) + env.frame) % 10 == 0) {
                // The floor: a 1-unit vertical ray through the piece, against group 0x80 shapes.
                const Vec3 from{p.position.x + 0.0f, p.position.y + 0.5f, p.position.z + 0.0f};
                const Vec3 to{from.x, p.position.y - 0.5f, from.z};
                Vec3 hit{0, 0, 0};
                p.floor = env.world->raycast(from, to, 0x80, &hit, nullptr, nullptr, nullptr) ? hit.y + 0.05f : -10.0f;
            }
            if (p.position.y < p.floor && v.y < 0) {
                Vec3& w = p.angular_velocity;
                if (square_length(w) > 0) {  // the first bounce: a random yaw, no more spin
                    p.rotation = set_axis_angle(kUp, rng.rnd(0.0f, 6.28318548f));
                    w = {0, 0, 0};
                } else {
                    w = {w.x * 0.9f, w.y * 0.9f, w.z * 0.9f};
                }
                p.position.y = p.floor;
                v = {v.x * 0.9f, -(v.y * 0.3f), v.z * 0.9f};
                if (square_length(v) < 0.5f) p.alive = false;
            }
        }
        const Transform m = Transform::from(p.position, p.rotation);
        DebrisVertices& out = vertices[i];
        for (int k = 0; k < 4; ++k) out.points[k] = m.apply(p.points[k]);
        out.color = p.color;
        ++i;
    }
    vertices.resize(pieces.size());
}

void Debris::translate(float dz) {
    for (DebrisPiece& p : pieces) p.position.z = p.position.z + dz;
}

}  // namespace sh::phys
