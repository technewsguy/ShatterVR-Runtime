#include "game/particles.hpp"

#include "core/qi_string.hpp"
#include "physics/qi_random.hpp"

#include <pugixml.hpp>

#include <algorithm>
#include <cmath>

namespace sh::game {

bool ParticleSystem::load(const std::string& xml, const std::string& name, int texture_w, int texture_h) {
    pugi::xml_document doc;
    if (!doc.load_buffer(xml.data(), xml.size())) return false;
    const pugi::xml_node root = doc.child("effects");
    if (!root) return false;
    for (const pugi::xml_node e : root.children("effect")) {
        if (name != e.attribute("name").as_string("")) continue;
        auto attr = [&](const char* key, const char* fallback = "") { return std::string(e.attribute(key).as_string(fallback)); };
        auto word = [&](const char* key, int i) { return to_float(get_word(attr(key), i)); };
        const float coords[4] = {word("coords", 0), word("coords", 1), word("coords", 2), word("coords", 3)};
        const int r = to_int(attr("rows")), c = to_int(attr("cols")), m = to_int(attr("max"));
        gravity = to_float(attr("gravity"));
        const float d = to_float(attr("damping"));
        damping = d <= 0.0f ? 1.0f : (d < 1.0f ? 1.0f - d : 0.0f);
        life[0] = word("life", 0), life[1] = word("life", 1);
        alpha[0] = word("alpha", 0), alpha[1] = word("alpha", 1);
        rot[0] = word("rot", 0), rot[1] = word("rot", 1);
        size[0] = word("size", 0), size[1] = word("size", 1);
        blending = to_float(attr("blending", "1"));
        random_rot = attr("randomrot", "true") == "true";
        angvel_size = attr("angvelsize", "false") == "true";
        // ParticleSystem::init(coords lo, coords hi, rows, cols, max): empties the list.
        max = m;
        particles.clear();
        rows = r, cols = c;
        if (texture_w > 0 && texture_h > 0) {
            const float w = static_cast<float>(texture_w), h = static_cast<float>(texture_h);
            uv0[0] = coords[0] / w, uv0[1] = coords[1] / h, uv1[0] = coords[2] / w, uv1[1] = coords[3] / h;
        }
        return true;
    }
    return false;
}

void ParticleSystem::spawn(Rand48& rng, phys::Vec3 pos, phys::Vec3 vel, const float color[4], int sprite) {
    if (max == 0) return;
    if (sprite == -1) sprite = rng.rnd(0, cols * rows);
    const float rotation = random_rot ? rng.rnd(0.0f, 6.2831855f) : 0.0f;
    const float s = rng.rnd_normal(rot[0], rot[1]);
    const float l = rng.rnd_normal(life[0], life[1]);
    Particle p{pos, vel, {color[0], color[1], color[2], color[3]}, rotation, s, l, 0.0f, sprite};
    if (static_cast<int>(particles.size()) < max) {
        particles.push_back(p);
        return;
    }
    // Full: the oldest (largest age; the first of equals) makes room.
    size_t oldest = 0;
    for (size_t i = 1; i < particles.size(); ++i)
        if (particles[oldest].age < particles[i].age) oldest = i;
    if (!particles.empty()) particles[oldest] = p;
}

void ParticleSystem::update(Rand48& rng, float dt, int frame, bool slow) {
    const float g = dt * gravity;
    size_t kept = 0;
    for (size_t i = 0; i < particles.size(); ++i) {
        Particle p = particles[i];
        if (!(p.age < p.life)) continue;
        float vx = p.vel.x, vy = p.vel.y, vz = p.vel.z;
        // A small random push, unless the particle has no x/y velocity (or, in slow motion, seven frames of eight).
        if (!(vx == 0.0f && vy == 0.0f) && !(slow && (frame & 7) != 0)) {
            const phys::Vec3 r = phys::random_unit(rng, 0.06f);
            vy = vy + r.y, vx = vx + r.x, vz = vz + r.z;
        }
        vy = damping * (g + vy);
        vx = damping * vx;
        vz = damping * vz;
        p.vel = {vx, vy, vz};
        const float dy = dt * vy, dx = dt * vx, dz = dt * vz;
        p.pos.y = p.pos.y + dy;
        p.pos.x = p.pos.x + dx;
        p.pos.z = p.pos.z + dz;
        const float dr = dt * p.spin;
        p.rotation = p.rotation + dr;
        p.age = dt + p.age;
        particles[kept++] = p;
    }
    particles.resize(kept);
}

void ParticleSystem::translate(float dz) {
    for (Particle& p : particles) p.pos.z = p.pos.z + dz;
}

void append_sprites(const ParticleSystem& s, phys::Vec3 camera, float camera_speed, std::vector<SpriteVertex>& out) {
    if (s.rows <= 0 || s.cols <= 0) return;
    const phys::Quat streak = phys::axis_angle({0, 1, 0}, 1.5707964f);
    for (const Particle& p : s.particles) {
        // QiTexture::getTiledTexCoords: cell = sprite (row-major, cols per row) of the coords rectangle.
        const int row = p.sprite / s.cols, col = p.sprite % s.cols;
        const float du = s.uv1[0] - s.uv0[0], dv = s.uv1[1] - s.uv0[1];
        const float u0 = s.uv0[0] + static_cast<float>(col) * du / static_cast<float>(s.cols);
        const float u1 = s.uv0[0] + static_cast<float>(col + 1) * du / static_cast<float>(s.cols);
        const float v0 = s.uv0[1] + static_cast<float>(row) * dv / static_cast<float>(s.rows);
        const float v1 = s.uv0[1] + static_cast<float>(row + 1) * dv / static_cast<float>(s.rows);

        float t = p.age / p.life, birth;
        if (t <= 0.0f) birth = 1.0f, t = 0.0f;
        else if (t < 1.0f) birth = 1.0f - t;
        else birth = 0.0f, t = 1.0f;
        float size = s.size[1] * t + s.size[0] * birth;
        if (s.angvel_size) size = size + p.spin;
        const bool still = p.vel.x == 0.0f && p.vel.y == 0.0f;
        float w = size * 0.5f, h = w;
        phys::Quat q = phys::axis_angle({0, 0, 1}, p.rotation);
        if (still) q = streak, w = w * camera_speed, h = h * 0.3f;

        // RenderLevel::addSprite: gone at 0.5 units from the camera, full alpha from 1.75.
        const float dy = camera.y - p.pos.y, dx = camera.x - p.pos.x, dz = camera.z - p.pos.z;
        const float fade = (std::sqrt(dy * dy + dx * dx + dz * dz) - 0.5f) * 0.8f;
        if (fade < 0.0f) continue;
        const float alpha = p.color[3] * (s.alpha[1] * t + s.alpha[0] * birth) * std::min(fade, 1.0f);

        const float cx[4] = {-w, w, w, -w}, cy[4] = {-h, -h, h, h};
        const float cu[4] = {u0, u1, u1, u0}, cv[4] = {v1, v1, v0, v0};
        for (int k = 0; k < 4; ++k) {
            const phys::Vec3 c = phys::rotate(q, {cx[k], cy[k], 0.0f});
            out.push_back({{p.pos.x + c.x, p.pos.y + c.y, p.pos.z + c.z},
                           {cu[k], cv[k]},
                           {p.color[0], p.color[1], p.color[2], alpha},
                           s.blending});
        }
    }
}

}  // namespace sh::game
