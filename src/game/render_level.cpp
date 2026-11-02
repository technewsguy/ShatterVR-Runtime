#include "game/render_level.hpp"

#include "game/obstacle.hpp"
#include "game/powerup.hpp"
#include "physics/debris.hpp"
#include "physics/body_joint.hpp"
#include "physics/qi_rotate.hpp"

#include <algorithm>
#include <string>
#include <cmath>

namespace sh::game {

void append_quad(phys::Vec3 camera, phys::Vec3 pos, phys::Vec3 ax, phys::Vec3 ay, float w, float h,
                 const float uv[4][2], const float color[4], float blend, std::vector<SpriteVertex>& out) {
    const float dy = camera.y - pos.y, dx = camera.x - pos.x, dz = camera.z - pos.z;
    const float fade = (std::sqrt(dy * dy + dx * dx + dz * dz) - 0.5f) * 0.8f;
    if (fade < 0.0f) return;
    const float alpha = color[3] * std::min(fade, 1.0f);
    const float cx[4] = {-w, w, w, -w}, cy[4] = {-h, -h, h, h};
    for (int k = 0; k < 4; ++k) {
        out.push_back({{pos.x + cx[k] * ax.x + cy[k] * ay.x, pos.y + cx[k] * ax.y + cy[k] * ay.y,
                        pos.z + cx[k] * ax.z + cy[k] * ay.z},
                       {uv[k][0], uv[k][1]},
                       {color[0], color[1], color[2], alpha},
                       blend});
    }
}

void sprite_cell(int cell, float uv[4][2]) {
    // A negative index is cell ~index of a 2 x 2 grid (without the inset).
    const int grid = cell < 0 ? 2 : 8, index = cell < 0 ? ~cell : cell;
    const int row = index / grid, col = index % grid;
    const float inset = cell < 0 ? 0.0f : 1.0f / 512.0f, g = static_cast<float>(grid);
    const float u0 = col / g + inset, u1 = (col + 1) / g - inset;
    const float v0 = row / g, v1 = (row + 1) / g;
    const float c[4][2] = {{u0, v1}, {u1, v1}, {u1, v0}, {u0, v0}};
    std::copy(&c[0][0], &c[0][0] + 8, &uv[0][0]);
}

namespace {

phys::Vec3 normalized_or(phys::Vec3 v, phys::Vec3 fallback) {
    const float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return len > 0.0f ? phys::Vec3{v.x / len, v.y / len, v.z / len} : fallback;
}

// fillBuffers' sort of the level's bodies by z, larger first: an in-place quicksort with explicit begin / end stacks
// (300 levels), the first element of each range as the pivot. Its order for equal z is part of the output.
void sort_by_z(std::vector<const phys::RigidBody*>& a) {
    constexpr int kLevels = 300;
    int beg[kLevels], end[kLevels];
    beg[0] = 0, end[0] = static_cast<int>(a.size());
    auto z = [&](int k) { return a[static_cast<size_t>(k)]->position.z; };
    int i = 0;
    while (i >= 0) {
        int L = beg[i], R = end[i] - 1;
        if (!(R > L)) {
            --i;
            continue;
        }
        const phys::RigidBody* piv = a[static_cast<size_t>(L)];
        const float pz = piv->position.z;
        for (;;) {
            // From the right: the first element with a larger z than the pivot moves to L.
            bool found = pz < z(R);
            while (!found && L < R) {
                --R;
                found = pz < z(R);
            }
            if (!found || L >= R) break;
            a[static_cast<size_t>(L)] = a[static_cast<size_t>(R)];
            // From the left: the first element whose z is not larger (or NaN) moves to R.
            ++L;
            bool stop = !(pz < z(L));
            while (!stop && L < R) {
                ++L;
                stop = !(z(L) > pz);
            }
            if (!stop || L >= R) break;
            a[static_cast<size_t>(R)] = a[static_cast<size_t>(L)];
            --R;
            if (!(L < R)) break;
        }
        a[static_cast<size_t>(L)] = piv;
        beg[i + 1] = L + 1, end[i + 1] = end[i], end[i] = L;
        if (end[i + 1] - beg[i + 1] > end[i] - beg[i]) {
            std::swap(beg[i], beg[i + 1]);
            std::swap(end[i], end[i + 1]);
        }
        ++i;
    }
}

// QiMatrix4 times a point (with the translation) or a direction, in fillBuffers' order: (y m[4+r] + x m[r]) +
// z m[8+r] (+ m[12+r]).
phys::Vec3 mat_point(const Mat4& M, phys::Vec3 p) {
    const float* m = M.m;
    return {((p.y * m[4] + p.x * m[0]) + p.z * m[8]) + m[12], ((p.y * m[5] + p.x * m[1]) + p.z * m[9]) + m[13],
            ((p.y * m[6] + p.x * m[2]) + p.z * m[10]) + m[14]};
}
phys::Vec3 mat_dir(const Mat4& M, phys::Vec3 p) {
    const float* m = M.m;
    return {(p.y * m[4] + p.x * m[0]) + p.z * m[8], (p.y * m[5] + p.x * m[1]) + p.z * m[9],
            (p.y * m[6] + p.x * m[2]) + p.z * m[10]};
}
// fillBuffers' normalisation: the length from y, x, z in that order; (1, 0, 0) when it is not positive.
phys::Vec3 normalized_yxz(phys::Vec3 v) {
    const float len = std::sqrt((v.y * v.y + v.x * v.x) + v.z * v.z);
    return len > 0.0f ? phys::Vec3{v.x / len, v.y / len, v.z / len} : phys::Vec3{1, 0, 0};
}

std::uint32_t pack(const float c[4]) {
    // QiColor::asUInt32: each channel times 255, truncated, clamped to [0, 255] (NaN gives 0).
    auto b = [](float f) {
        const float x = f * 255.0f;
        return !(x >= 1.0f) ? 0u : x >= 255.0f ? 255u : static_cast<std::uint32_t>(x);
    };
    return b(c[0]) | b(c[1]) << 8 | b(c[2]) << 16 | b(c[3]) << 24;
}

void put(BodyVertex& v, phys::Vec3 p, phys::Vec3 n, float u, float t, std::uint32_t rgba) {
    v.pos[0] = p.x, v.pos[1] = p.y, v.pos[2] = p.z;
    v.normal[0] = n.x, v.normal[1] = n.y, v.normal[2] = n.z;
    v.uv[0] = u, v.uv[1] = t;
    for (int k = 0; k < 4; ++k) v.rgba[k] = static_cast<std::uint8_t>(rgba >> (8 * k));
}

// The planar texture coordinates of a body-frame point by its face's normal: (y, z) when x dominates, (x, z)
// when y dominates, else (x, y); "dominates" with a 0.1 margin.
void planar_uv(phys::Vec3 n, phys::Vec3 p, float& u, float& v) {
    const float ax = std::fabs(n.x), ay = std::fabs(n.y), az = std::fabs(n.z), eps = 0.1f;
    if (!(ax <= ay + eps) && !(ax <= az + eps)) {
        u = p.y, v = p.z;
    } else if (ax + eps < ay && az + eps < ay) {
        u = p.x, v = p.z;
    } else {
        u = p.x, v = p.y;
    }
}

}  // namespace

void build_body_buffers(const phys::PhysicsWorld& world, phys::Vec3 cam, float level_time, BodyBuffers& out) {
    out.clear();
    std::vector<const phys::RigidBody*> bodies(world.bodies.begin(), world.bodies.end());
    sort_by_z(bodies);  // nearest first
    for (const phys::RigidBody* body : bodies) {
        if (!body->alive || body->round) continue;
        const phys::Quat q = body->rotation;
        const phys::Vec3 bp = body->position;
        const Mat4 M = pose_matrix(bp, q);  // QiMatrix4(QiTransform3) of the body
        for (const auto& sp : body->shapes) {
            const phys::BodyShape& s = *sp;
            const phys::Material& m = s.material;
            if (m.color[3] == 0.0f) continue;  // invisible shape
            const int type = m.sound_type;     // 0 plain, 1 marble, 2 glass, 3 metal; 4 is never drawn
            if (type == 4) continue;
            float color[4] = {m.color[0], m.color[1], m.color[2], m.color[3]};
            if (type == 2) {  // glass: alpha 0, unless its obstacle was cleared for points (a white flash)
                color[3] = 0.0f;
                const auto* ob = static_cast<const Obstacle*>(body->near_despawn);
                if (ob && ob->state.cleared != 0.0f && ob->state.score >= 1) {
                    const float t = (level_time - ob->state.cleared) * 3.0f;
                    if (t <= 0.0f) {
                        color[3] = 1.0f;
                    } else if (t < 1.0f) {
                        const float f = 1.0f - t;
                        for (int k = 0; k < 3; ++k) color[k] = t + f * color[k];
                        color[3] = f;
                    } else {
                        color[0] = color[1] = color[2] = 1.0f;
                    }
                }
            }
            const std::uint32_t rgba = pack(color);
            const bool textured = m.b3c != 0;
            BodyMesh& mesh = type == 2 ? (textured ? out.glass_tex : out.glass) : type == 3 ? out.metal : out.basic;
            const float r = m.roundness;
            const phys::Polyhedron& poly = s.poly;
            for (const phys::Polyhedron::Face& f : poly.faces) {
                const phys::Vec3 n = mat_dir(M, f.normal);
                const auto first = static_cast<std::uint32_t>(mesh.vertices.size());
                const phys::Vec3 p0 = mat_point(
                    M, poly.vertices[static_cast<size_t>(poly.edges[static_cast<size_t>(f.edge)].vertex)].position);
                if (!(((p0.y - cam.y) * n.y + (p0.x - cam.x) * n.x) + (p0.z - cam.z) * n.z <= 0.0f)) continue;
                int e = f.edge, k = 0;
                do {
                    const phys::Polyhedron::Vertex& pv = poly.vertices[static_cast<size_t>(poly.edges[static_cast<size_t>(e)].vertex)];
                    phys::Vec3 vn = n;
                    if (r > 0.0f) {
                        const phys::Vec3 sn = mat_dir(M, pv.normal);
                        vn = normalized_yxz({(1 - r) * n.x + r * sn.x, (1 - r) * n.y + r * sn.y, (1 - r) * n.z + r * sn.z});
                    }
                    float u, v;
                    planar_uv(f.normal, pv.position, u, v);
                    if (textured) u = m.f2c + u * m.f34, v = m.f30 + v * m.f38;
                    mesh.vertices.emplace_back();
                    put(mesh.vertices.back(), mat_point(M, pv.position), vn, u, v, rgba);
                    const auto cur = static_cast<std::uint32_t>(mesh.vertices.size() - 1);
                    if (++k >= 3) mesh.triangle(first, cur - 1, cur);
                    e = poly.edges[static_cast<size_t>(e)].next;
                } while (e != f.edge);
            }
            // Glass creases (not textured, not hidden with mgHideEdges): each edge between two faces bent by more
            // than acos(0.98), as a line slightly towards the camera, unless both ends' normals face away.
            if (type != 2 || textured || m.b3d) continue;
            for (size_t i = 0; i < poly.edges.size(); ++i) {
                const phys::Polyhedron::Edge& e = poly.edges[i];
                const int va = e.vertex, vb = poly.edges[static_cast<size_t>(e.next)].vertex;
                if (va > vb) continue;
                const phys::Polyhedron::Face& fa = poly.faces[static_cast<size_t>(e.face)];
                const phys::Polyhedron::Face& fb = poly.faces[static_cast<size_t>(poly.edges[static_cast<size_t>(e.twin)].face)];
                if (fa.flag || fb.flag) continue;
                const phys::Vec3 na0 = fa.normal, nb0 = fb.normal;
                if (!((nb0.y * na0.y + nb0.x * na0.x) + nb0.z * na0.z <= 0.98f)) continue;
                const phys::Polyhedron::Vertex& A = poly.vertices[static_cast<size_t>(va)];
                const phys::Polyhedron::Vertex& B = poly.vertices[static_cast<size_t>(vb)];
                const phys::Vec3 avg = normalized_yxz(A.normal + B.normal);
                // With roundness each end leans towards its own vertex normal.
                phys::Vec3 na = avg, nb = avg;
                if (r > 0.0f) {
                    na = normalized_yxz({(1 - r) * avg.x + r * A.normal.x, (1 - r) * avg.y + r * A.normal.y,
                                         (1 - r) * avg.z + r * A.normal.z});
                    nb = normalized_yxz({(1 - r) * avg.x + r * B.normal.x, (1 - r) * avg.y + r * B.normal.y,
                                         (1 - r) * avg.z + r * B.normal.z});
                }
                const phys::Vec3 wa = mat_dir(M, na), wb = mat_dir(M, nb);
                if (wa.z < 0.0f && wb.z < 0.0f) continue;
                // Texture coordinates by the first end's normal, without a margin: (y, z) when x is the largest,
                // (x, z) when y is, else (x, y).
                const float ax = std::fabs(na.x), ay = std::fabs(na.y), az = std::fabs(na.z);
                float ua = A.position.x, va2 = A.position.y, ub = B.position.x, vb2 = B.position.y;
                if (ax > ay && az < ax) {
                    ua = A.position.y, va2 = A.position.z, ub = B.position.y, vb2 = B.position.z;
                } else if (ax < ay && az < ay) {
                    va2 = A.position.z, vb2 = B.position.z;
                }
                phys::Vec3 pa = mat_point(M, A.position), pb = mat_point(M, B.position);
                pa.z = pa.z + 0.01f, pb.z = pb.z + 0.01f;
                out.glass_edges.emplace_back();
                put(out.glass_edges.back(), pa, wa, ua, va2, rgba);
                out.glass_edges.emplace_back();
                put(out.glass_edges.back(), pb, wb, ub, vb2, rgba);
            }
        }
    }
    // Ropes: a strip 0.02 wide between the anchors, grey on one edge and white on the other, in the metal buffer.
    for (const phys::BodyJoint* j : world.joints) {
        if (j->type != phys::BodyJoint::kRope || !j->a || !j->b) continue;
        const phys::Vec3 p0 = phys::qi_transform(j->a->position, j->a->rotation, j->anchor_a);
        const phys::Vec3 p1 = phys::qi_transform(j->b->position, j->b->rotation, j->anchor_b);
        const phys::Vec3 d = normalized_or(p1 - p0, {1, 0, 0});
        phys::Vec3 side = std::fabs(d.z) <= 0.7f ? normalized_or({-d.y, d.x, 0.0f}, {0, 0, 0})
                                                 : normalized_or({0.0f, -d.z, d.y}, {0, 0, 0});
        side = {side.x * 0.01f, side.y * 0.01f, side.z * 0.01f};
        const bool flipped = side.y < 0.0f;
        if (flipped) side = {-side.x, -side.y, -side.z};
        const phys::Vec3 n = normalized_or({side.x * 10.0f, side.y * 10.0f, 1.0f}, {0, 0, 1});  // mostly towards +z
        const float grey[4] = {0.5f, 0.5f, 0.5f, 1}, white[4] = {1, 1, 1, 1};
        const auto first = static_cast<std::uint32_t>(out.metal.vertices.size());
        const phys::Vec3 corners[4] = {p0 - side, p1 - side, p1 + side, p0 + side};
        for (int k = 0; k < 4; ++k) {
            out.metal.vertices.emplace_back();
            put(out.metal.vertices.back(), corners[k], n, (k == 1 || k == 2) ? 1.0f : 0.0f, 0.5f,
                pack(k < 2 ? grey : white));
        }
        // QiIndexBuffer::quad(a, b, c, d): (a, b, c), (c, d, a); a flipped strip goes the other way round.
        const std::uint32_t second = first + (flipped ? 3 : 1), fourth = first + (flipped ? 1 : 3);
        if (first + 3 < out.metal.index_limit)
            out.metal.indices.insert(out.metal.indices.end(), {first, second, first + 2, first + 2, fourth, first});
    }
}

phys::Vec3 point_at_distance(const Level& level, float d) {
    const Room& room = *level.current;
    const float def_length = static_cast<float>(level.rooms[static_cast<size_t>(room.def_index)].length);
    if (def_length == 0.0f) return {0, 0, 0};
    return {0, 0, -room.start + -(((d - static_cast<float>(level.distance_base)) / def_length) * room.length)};
}

bool best_distance_marker(const Level& level, int player_mode, int best_distance, BestDistanceMarker& out) {
    if (!level.campaign || player_mode == 3 || level.current->type.find("boss") != std::string::npos) return false;
    const float pz = point_at_distance(level, static_cast<float>(best_distance)).z;
    const float cz = level.camera.z;
    float k, z;
    if (level.fade > 0.0f && level.distance > static_cast<float>(best_distance)) {
        // A new best at game over: the sign comes in front of the camera over the fade's last quarter.
        k = level.fade * 4.0f - 3.0f;
        z = cz - 3.0f;
        if (!(k > 0.0f)) return false;
        if (!(k < 1.0f)) k = 1.0f;
    } else {
        if (best_distance < 1 || !(pz < cz) || !(pz > cz - 100.0f)) return false;
        k = 1.0f, z = pz;
    }
    float f = 1.0f - std::fabs(cz - z) * 0.03f;
    f = !(f > 0.0f) ? 0.0f : (f < 1.0f ? f : 1.0f);
    out.z = z, out.alpha = k * f;
    return true;
}

void add_reflection(ReflectionMesh& out, phys::Vec3 camera, phys::Vec3 pos, phys::Quat rot, float radius,
                    const float rgb[3]) {
    const float dy = camera.y - pos.y, dx = camera.x - pos.x, dz = camera.z - pos.z;
    const float fade = (std::sqrt(dy * dy + dx * dx + dz * dz) - 0.5f) * 0.8f;
    if (fade < 0.0f) return;
    const float color[4] = {rgb[0], rgb[1], rgb[2], std::min(fade, 1.0f)};
    auto vertex = [&](phys::Vec3 p, std::uint8_t nx, std::uint8_t ny, std::uint8_t nz) {
        ReflectionVertex v{{p.x, p.y, p.z}, {nx, ny, nz, 0}, {color[0], color[1], color[2], color[3]}};
        out.vertices.push_back(v);
    };
    auto to_byte = [](float n) { return static_cast<std::uint8_t>(n * 127.0f + 128.0f); };
    const auto centre = static_cast<std::uint32_t>(out.vertices.size());
    vertex(pos, 0x80, 0x80, 0xff);
    for (int i = 0; i < 10; ++i) {
        const float a = static_cast<float>(i) * 0.62831855f;
        const float c = std::clamp(std::cos(a) * 1.05f, -1.0f, 1.0f), s = std::clamp(std::sin(a) * 1.05f, -1.0f, 1.0f);
        vertex(phys::qi_transform(pos, rot, {radius * c, radius * s, 0.0f}), to_byte(c), to_byte(s), 0);
    }
    for (std::uint32_t i = 1; i <= 10; ++i) out.indices.insert(out.indices.end(), {centre, centre + i, centre + 1 + i % 10});
}

bool add_shadow(const phys::PhysicsWorld& world, std::vector<ShadowVertex>& out, phys::Vec3 pos, float strength,
                float reach) {
    phys::Vec3 hit;
    if (!world.raycast(pos, {pos.x, pos.y - reach, pos.z}, 0x80, &hit, nullptr, nullptr, nullptr)) return false;
    const float h = (pos.y - hit.y) - 0.5f;
    const float alpha = 0.4f - (h + h);
    const int a = static_cast<int>(alpha * strength * 255.0f);
    const auto ab = static_cast<std::uint8_t>(std::clamp(a, 0, 255));
    const float y = hit.y + 0.01f;
    const float xs[4] = {hit.x - 0.6f, hit.x + 0.6f, hit.x + 0.6f, hit.x - 0.6f};
    const float zs[4] = {hit.z - 0.6f, hit.z - 0.6f, hit.z + 0.6f, hit.z + 0.6f};
    const std::uint8_t us[4] = {0, 255, 255, 0}, vs[4] = {0, 0, 255, 255};
    for (int k = 0; k < 4; ++k) out.push_back({{xs[k], y, zs[k]}, {us[k], vs[k]}, ab, 0});
    return true;
}

std::vector<std::uint8_t> shadow_blob(int size) {
    std::vector<std::uint8_t> px(static_cast<size_t>(size) * static_cast<size_t>(size));
    const int half = size / 2;
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x) {
            const float dy = static_cast<float>(y - half) / static_cast<float>(half);
            const float dx = static_cast<float>(x - half) / static_cast<float>(half);
            const float d = std::sqrt(dy * dy + dx * dx);
            float v = 0.0f;
            if (d <= 1.0f) {
                const float t = 1.0f - d * d;
                v = t * t * t * 1.5666814f * 0.5f;
            }
            px[static_cast<size_t>(y * size + x)] = v < 1.0f ? static_cast<std::uint8_t>(std::max(v, 0.0f) * 255.0f) : 255;
        }
    return px;
}

void debris_triangles(const std::vector<phys::DebrisVertices>& pieces, phys::Vec3 cam, std::vector<BodyVertex>& out) {
    static constexpr int kTri[4][3] = {{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3}};
    for (const phys::DebrisVertices& d : pieces) {
        for (const auto& t : kTri) {
            const phys::Vec3 a = d.points[t[0]], b = d.points[t[1]], c = d.points[t[2]];
            const phys::Vec3 n = normalized_or(phys::cross(b - a, c - a), {1, 0, 0});
            if (!((a.y - cam.y) * n.y + (a.x - cam.x) * n.x + (a.z - cam.z) * n.z <= 0.0f)) continue;
            for (int k = 0; k < 3; ++k) {
                out.emplace_back();
                put(out.back(), d.points[t[k]], n, 0.0f, 0.0f, d.color);
            }
        }
    }
}

void entity_sprites(const Level& level, const phys::PhysicsWorld& world, phys::Quat display_rotation,
                    std::vector<SpriteVertex>& sprites, ReflectionMesh& discs, std::vector<ShadowVertex>& shadows) {
    const phys::Vec3 cam = level.camera;
    float uv[4][2];
    // Balls: the level's body list order, round bodies.
    for (const phys::RigidBody* b : world.bodies) {
        if (!b->round) continue;
        float tint[3] = {1, 1, 1};
        if (b->type != 0) tint[0] = 2.0f;                      // armed
        else if (level.timers[5] > 0.0f) tint[1] = 1.4f;       // rapid fire (Level +0x16c)
        add_reflection(discs, cam, b->position, display_rotation, 0.11f, tint);
        const phys::Vec3 ax = phys::qi_rotate(display_rotation, {1, 0, 0}), ay = phys::qi_rotate(display_rotation, {0, 1, 0});
        const float white[4] = {1, 1, 1, 1};
        sprite_cell(0, uv);
        append_quad(cam, b->position, ax, ay, 0.11f, 0.11f, uv, white, 1.0f, sprites);
    }
    // Power-ups (Level +0x12c) not yet picked up.
    const phys::Quat q = level.camera_roll;  // Level +0xf8
    const phys::Vec3 ax = phys::qi_rotate(q, {1, 0, 0}), ay = phys::qi_rotate(q, {0, 1, 0});
    for (size_t i = 0; i < level.powerups.size(); ++i) {
        const Powerup& p = *level.powerups[i];
        if (p.picked) continue;
        int cell = 0;
        switch (p.type) {
        case Powerup::kBallFrenzy: cell = 1; break;
        case Powerup::kMultiball: cell = 2; break;
        case Powerup::kNitroBalls: cell = 3; break;
        case Powerup::kSlowMotion: cell = 4; break;
        case Powerup::kBalls: cell = 5; break;
        case Powerup::kBarrel: cell = 7; break;
        default: break;
        }
        phys::Vec3 pos = p.position;
        pos.y = pos.y + std::sin(static_cast<float>(i) + level.sim_time * 3.0f) * 0.1f;
        const float white[4] = {1, 1, 1, 1};
        sprite_cell(cell, uv);
        append_quad(cam, pos, ax, ay, 0.3f, 0.3f, uv, white, 1.0f, sprites);
        const float rgb[3] = {1, 1, 1};
        add_reflection(discs, cam, phys::qi_transform(pos, q, {0.0f, 0.0f, -0.05f}), q, 0.28f, rgb);
        add_shadow(world, shadows, pos, 1.0f);
    }
}

void button_sprites(const Level& level, std::vector<SpriteVertex>& sprites) {
    float uv[4][2];
    for (size_t i = 0; i < level.buttons.size(); ++i) {
        const phys::ButtonEntity& b = *level.buttons[i];
        const phys::Quat q = b.rotation;
        const phys::Vec3 ax = phys::qi_rotate(q, {1, 0, 0}), ay = phys::qi_rotate(q, {0, 1, 0});
        phys::Vec3 pos = b.position;
        pos.z = pos.z + 0.05f;
        const float white[4] = {1, 1, 1, 1};
        sprite_cell(b.down ? 9 : 8, uv);
        append_quad(level.camera, pos, ax, ay, 0.3f, 0.3f, uv, white, 1.0f, sprites);
        if (b.down) continue;
        pos.z = pos.z + 0.01f;
        const float phase = static_cast<float>(i % 100);
        const float glow[4] = {1, 1, 1, 0.8f + std::sin((phase + level.sim_time) * 9.0f) * 0.2f};
        sprite_cell(14, uv);
        append_quad(level.camera, pos, ax, ay, 0.3f, 0.3f, uv, glow, 0.0f, sprites);  // blend 0: additive
        // The tutorial hint: in the first rooms, two arrows (cell 13, half size 0.5) swing in from either side of a
        // button within 16 of the camera, pointing at it (turned -/+ 90 degrees about z, whatever the button's turn).
        const std::string& type = level.current->type;
        if ((type != "basic/basic" && type != "basic/dark") || !(std::fabs(pos.z - level.camera.z) < 16.0f)) continue;
        const float d = 0.8f + std::sin(level.sim_time * 7.0f) * 0.2f;
        sprite_cell(13, uv);
        const phys::Vec3 left{pos.x - d, pos.y, pos.z}, right{left.x + (d + d), pos.y, pos.z};
        const phys::Quat ql = phys::axis_angle({0, 0, 1}, 1.5707964f), qr = phys::axis_angle({0, 0, 1}, -1.5707964f);
        append_quad(level.camera, left, phys::qi_rotate(ql, {1, 0, 0}), phys::qi_rotate(ql, {0, 1, 0}), 0.5f, 0.5f, uv,
                    white, 1.0f, sprites);
        append_quad(level.camera, right, phys::qi_rotate(qr, {1, 0, 0}), phys::qi_rotate(qr, {0, 1, 0}), 0.5f, 0.5f, uv,
                    white, 1.0f, sprites);
    }
}

void decal_sprites(const Room& room, phys::Vec3 camera, int player_mode, std::vector<SpriteVertex>& sprites,
                   std::vector<SpriteVertex>& doors) {
    float uv[4][2];
    for (const Room::Decal& d : room.decals) {
        const phys::Vec3 pos = phys::qi_transform(room.body->position, room.body->rotation, d.pos);
        const phys::Quat q = phys::qi_mul(room.body->rotation, d.rot);
        sprite_cell(d.cell, uv);
        const bool tint = d.cell < 0 && player_mode == 2;
        const float color[4] = {d.color[0], d.color[1] * (tint ? 0.7f : 1.0f), d.color[2] * (tint ? 0.6f : 1.0f), d.color[3]};
        append_quad(camera, pos, phys::qi_rotate(q, {1, 0, 0}), phys::qi_rotate(q, {0, 1, 0}), d.size[0], d.size[1], uv,
                    color, d.blend, d.cell < 0 ? doors : sprites);
    }
}

void obstacle_score_sprites(const Room& room, const phys::PhysicsWorld& world, phys::Vec3 cam, float level_time,
                            std::vector<SpriteVertex>& sprites, std::vector<ShadowVertex>& shadows) {
    static constexpr int kDigitCell[10] = {44, 45, 46, 47, 52, 53, 54, 55, 60, 61};
    static constexpr float kDigitWidth[10] = {1, 0.5f, 1, 1, 1, 1, 1, 1, 1, 1};
    float uv[4][2];
    for (const auto& op : room.obstacles) {
        const Obstacle& o = *op;
        const script::ObstacleState& st = o.state;
        if (st.cleared == 0.0f || st.score < 1) {
            if (st.score == 0 || !o.script) continue;
            // The glow at the mgGlow point, moved half a unit towards the camera. Its spin angle is unverified
            // (taken as the level time here).
            phys::Vec3 p = phys::qi_transform(o.position, o.rotation, o.script->glow);
            const phys::Vec3 to = normalized_or(p - cam, {0.5f, 0, 0});
            p = {p.x - to.x * 0.5f, p.y - to.y * 0.5f, p.z - to.z * 0.5f};
            const phys::Quat q = phys::axis_angle({0, 0, 1}, level_time);
            const float c[4] = {1, 1, 1, 0.6f};
            sprite_cell(21, uv);
            append_quad(cam, p, phys::qi_rotate(q, {1, 0, 0}), phys::qi_rotate(q, {0, 1, 0}), 0.8f, 0.8f, uv, c, 1.0f, sprites);
            continue;
        }
        const float t = level_time - st.cleared;
        const float a = 2.5f - (t + t);
        if (!(a > 0.0f)) continue;
        const float alpha = std::min(a, 1.0f);
        phys::Vec3 p = phys::qi_transform(o.position, o.rotation, st.score_position);
        p.y = p.y + t * 0.1f;
        const float s = 1.0f;  // (a scale read from the obstacle in the original; unverified, 1 here)
        const std::string digits = std::to_string(st.score);
        float half = 0.0f;
        for (char ch : digits) half += kDigitWidth[std::clamp(ch - '0', 0, 9)] * 0.16f * s;
        half *= 0.5f;
        const phys::Vec3 ax{1, 0, 0}, ay{0, 1, 0};
        const float plate[4] = {0.3f, 0.7f, 1.0f, alpha};
        sprite_cell(62, uv);
        append_quad(cam, p, ax, ay, std::max(s * 0.25f, half + s * 0.05f), s * 0.25f, uv, plate, 1.0f, sprites);
        phys::Vec3 d{p.x - (half - s * 0.096f), p.y, p.z + 0.001f};
        const float white[4] = {1, 1, 1, alpha};
        for (char ch : digits) {
            const int k = std::clamp(ch - '0', 0, 9);
            sprite_cell(kDigitCell[k], uv);
            append_quad(cam, d, ax, ay, 0.16f * s, 0.16f * s, uv, white, 1.0f, sprites);
            d.x = d.x + 0.16f * s * kDigitWidth[k];
        }
        add_shadow(world, shadows, p, alpha, 10.0f);
    }
}

void room_reflections(const Room& room, std::vector<BodyVertex>& mask, std::vector<BodyVertex>& walls) {
    const phys::Vec3 bp = room.body->position;
    const phys::Quat bq = room.body->rotation;
    auto quad = [&](std::vector<BodyVertex>& out, const phys::Vec3* p, std::uint32_t rgba) {
        for (int k : {0, 1, 2, 0, 2, 3}) {
            out.emplace_back();
            put(out.back(), phys::qi_transform(bp, bq, p[k]), {0, 1, 0}, 0.0f, 0.0f, rgba);
        }
    };
    for (size_t i = 0; i + 3 < room.reflection_mask.size(); i += 4) quad(mask, &room.reflection_mask[i], 0xffffffffu);
    for (const Room::ReflectionQuad& q : room.reflection_walls) quad(walls, q.p, pack(q.color));
}

void level_fog(const Level& level, float lower[4], float upper[4]) {
    const Room& cur = *level.current;
    for (int i = 0; i < 4; ++i) lower[i] = cur.fog[0][i], upper[i] = cur.fog[1][i];
    if (!level.next) return;
    const Room& next = *level.next;
    const phys::Vec3 rel = phys::qi_rotate_inverse(next.body->rotation, level.camera - next.body->position);
    if (!(rel.z < 10.0f)) return;
    const float f = rel.z / 10.0f;
    float wn = 1.0f, wc = 0.0f;  // weights of the next and the current room
    if (f > 0.0f) {
        if (f < 1.0f) wn = 1.0f - f, wc = f;
        else wn = 0.0f, wc = 1.0f;
    }
    for (int i = 0; i < 3; ++i) {
        lower[i] = wn * next.fog[0][i] + wc * cur.fog[0][i];
        upper[i] = wn * next.fog[1][i] + wc * cur.fog[1][i];
    }
}

void laser_sprites(Rand48& rng, const std::vector<Level::Laser>& lasers, phys::Vec3 camera,
                   std::vector<SpriteVertex>& out) {
    for (const Level::Laser& l : lasers) {
        const phys::Vec3 d{l.to.x - l.from.x, l.to.y - l.from.y, l.to.z - l.from.z};
        const float len = std::sqrt(d.y * d.y + d.x * d.x + d.z * d.z);
        const float n = len + 1.1920929e-7f;
        const phys::Vec3 x{d.x / n, d.y / n, d.z / n};
        // A side axis across the beam: in the xy plane, unless the beam runs mostly along z (as the original picks it).
        phys::Vec3 y;
        if (std::fabs(x.z) <= 0.7f) {
            const float m = std::sqrt(x.x * x.x + x.y * x.y);
            y = m > 0.0f ? phys::Vec3{-x.y / m, x.x / m, 0.0f} : phys::Vec3{1, 0, 0};
        } else {
            const float m = std::sqrt(x.z * x.z + x.y * x.y);
            y = m > 0.0f ? phys::Vec3{0.0f, -x.z / m, x.y / m} : phys::Vec3{1, 0, 0};
        }
        const phys::Vec3 mid{(l.from.x + l.to.x) * 0.5f, (l.from.y + l.to.y) * 0.5f, (l.from.z + l.to.z) * 0.5f};
        const phys::Vec3 a{l.from.x + x.x * 0.1f, l.from.y + x.y * 0.1f, l.from.z + x.z * 0.1f};
        const phys::Vec3 b{l.to.x - x.x * 0.1f, l.to.y - x.y * 0.1f, l.to.z - x.z * 0.1f};
        float uv[4][2];
        float white[4] = {1, 1, 1, 1};
        white[3] = rng.rnd(0.8f, 1.0f);
        sprite_cell(16, uv);
        append_quad(camera, mid, x, y, len * 0.5f - 0.3f, 0.15f, uv, white, 0.25f, out);
        white[3] = rng.rnd(0.8f, 1.0f);
        sprite_cell(24, uv);
        append_quad(camera, a, x, y, 0.2f, 0.15f, uv, white, 0.25f, out);
        white[3] = rng.rnd(0.8f, 1.0f);
        sprite_cell(25, uv);
        append_quad(camera, b, x, y, 0.2f, 0.15f, uv, white, 0.25f, out);
    }
}

}  // namespace sh::game
