#include "physics/breakage.hpp"

#include "physics/qi_random.hpp"
#include "physics/qi_rotate.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace sh::phys {

namespace {

// n . v + d, summed y, x, z, then + d (every breakage plane test).
float plane_dist(const Plane& p, Vec3 v) { return ((p.n.y * v.y + p.n.x * v.x) + p.n.z * v.z) + p.d; }

// Whether every vertex of the shape lies at least -0.02 from the plane (on or past its outer side).
bool all_outside(const Plane& p, const Polyhedron& poly) {
    for (const auto& v : poly.vertices)
        if (plane_dist(p, v.position) < -0.02f) return false;
    return true;
}

bool outside_any(const std::vector<Plane>& planes, const Polyhedron& poly) {
    for (const Plane& p : planes)
        if (all_outside(p, poly)) return true;
    return false;
}

std::vector<Vec3> positions(const Polyhedron& poly) {
    std::vector<Vec3> out;
    out.reserve(poly.vertices.size());
    for (const auto& v : poly.vertices) out.push_back(v.position);
    return out;
}

float distance_result(const DistanceResult& r, float limit) {
    if (r.flags & kResultBeyondMax) return limit;
    if (r.flags & kResultPenetrating) return 0;
    if (!(r.flags & kResultValid)) return limit;
    return r.distance;
}

// Normalised v (squares summed y, x, z), or X.
Vec3 unit_or_x(Vec3 v) {
    const float len = std::sqrt((v.y * v.y + v.x * v.x) + v.z * v.z);
    if (0 < len) return {v.x / len, v.y / len, v.z / len};
    return {1, 0, 0};
}

// A new shape taking a piece cut from `source` (material and origin copied, marked modified).
std::unique_ptr<BodyShape> new_piece(World& world, const BodyShape& source, const Polyhedron& poly) {
    auto s = std::make_unique<BodyShape>();
    s->material = source.material;
    s->origin = source.origin;
    s->modified = true;
    s->poly = poly;
    s->serial = world.next_shape_serial++;
    return s;
}

// Body::removeShape: the shape's pairs go, then the shape leaves the body's list.
std::unique_ptr<BodyShape> remove_shape(PhysicsWorld& world, RigidBody& body, const BodyShape* shape) {
    world.remove_pairs(*shape);
    return body.take_shape(shape);
}

// Takes a shape out of a pool of detached shapes.
std::unique_ptr<BodyShape> take(std::vector<std::unique_ptr<BodyShape>>& pool, const BodyShape* shape) {
    for (auto& p : pool)
        if (p.get() == shape) return std::move(p);
    return nullptr;
}

// Shape::computeLocalBounds: the shape's box fields get the bounds of its collision hull (solver frame).
void compute_local_bounds(BodyShape& s) {
    Vec3 mn{FLT_MAX, FLT_MAX, FLT_MAX}, mx{-FLT_MAX, -FLT_MAX, -FLT_MAX};
    for (const Vec3& v : s.hull) {
        if (v.x <= mn.x) mn.x = v.x;
        if (v.y <= mn.y) mn.y = v.y;
        if (v.z <= mn.z) mn.z = v.z;
        if (mx.x <= v.x) mx.x = v.x;
        if (mx.y <= v.y) mx.y = v.y;
        if (mx.z <= v.z) mx.z = v.z;
    }
    s.bounds_min = mn;
    s.bounds_max = mx;
}

// The bodies' shared ending: if the broken body kept no shape, everything goes back to it; otherwise the
// clone keeps its shapes (or is destroyed when it has none) and takes over nearby joints. Returns the
// body that should be split further (null for none).
RigidBody* settle(PhysicsWorld& world, RigidBody& body, RigidBody& clone, bool split_body_when_empty) {
    if (body.shapes.empty()) {
        while (!clone.shapes.empty()) body.add_shape(remove_shape(world, clone, clone.shapes.front().get()));
        world.destroy_body(clone);
        body.compute_mass_properties();
        return split_body_when_empty ? &body : nullptr;
    }
    body.compute_mass_properties();
    if (clone.shapes.empty()) {
        world.destroy_body(clone);
        return nullptr;
    }
    clone.compute_mass_properties();
    hand_over_joints(world.context, body, {&body, &clone});
    return &clone;
}

}  // namespace

Vec3 closest_point(const CollisionContext& ctx, const RigidBody& body, Vec3 p) {
    return closest_point(ctx, body, p, nullptr, nullptr, nullptr);
}

Vec3 closest_point(const CollisionContext& ctx, const RigidBody& body, Vec3 p, const BodyShape** shape, Vec3* normal,
                   bool* inside) {
    if (inside) *inside = false;
    const Quat q = body.com_rotation;
    const Vec3 c = body.com_position;
    const Vec3 local = qi_rotate_inverse(q, {p.x - c.x, p.y - c.y, p.z - c.z});
    const Shape point = Shape::sphere(0);
    Vec3 best{0, 0, 0};
    float best_d2 = FLT_MAX;
    for (const auto& s : body.shapes) {
        if (s->hull.empty()) continue;
        const Shape hull = Shape::hull(s->hull.data(), static_cast<int>(s->hull.size()));
        DistanceQuery dq;
        dq.a = &hull;
        dq.b = &point;
        dq.b_position = local;
        dq.b_rotation = {0, 0, 0, 1};
        dq.flags = kQueryPoints | kQueryPenetration;
        const DistanceResult r = distance(ctx, dq);
        if (inside && (r.flags & kResultPenetrating)) *inside = true;
        if (!(r.flags & kResultPoints)) continue;
        const Vec3 d{local.x - r.point_a.x, local.y - r.point_a.y, local.z - r.point_a.z};
        const float d2 = (d.y * d.y + d.x * d.x) + d.z * d.z;
        if (d2 < best_d2) {
            best = r.point_a, best_d2 = d2;
            if (normal) *normal = r.normal;
            if (shape) *shape = s.get();
        }
    }
    if (normal) *normal = qi_rotate(q, *normal);  // whatever it holds, as the original
    const Vec3 w = qi_rotate(q, best);
    return {w.x + c.x, w.y + c.y, w.z + c.z};
}

float shape_distance(const CollisionContext& ctx, const BodyShape& a, const BodyShape& b, float limit) {
    const std::vector<Vec3> pa = positions(a.poly), pb = positions(b.poly);
    const Shape sa = Shape::hull(pa.data(), static_cast<int>(pa.size()));
    const Shape sb = Shape::hull(pb.data(), static_cast<int>(pb.size()));
    DistanceQuery dq;
    dq.a = &sa;
    dq.b = &sb;
    dq.b_rotation = {0, 0, 0, 1};  // the original passes a zeroed (translation-only, zero) transform
    dq.flags = kQueryMaxDistance;
    dq.max_distance = limit;
    return distance_result(distance(ctx, dq), limit);
}

float shape_distance(const CollisionContext& ctx, const BodyShape& a, Vec3 p, float limit) {
    const std::vector<Vec3> pa = positions(a.poly);
    const Shape sa = Shape::hull(pa.data(), static_cast<int>(pa.size()));
    const Shape point{};
    DistanceQuery dq;
    dq.a = &sa;
    dq.b = &point;
    dq.b_position = p;
    dq.b_rotation = {0, 0, 0, 1};
    dq.flags = kQueryMaxDistance;
    dq.max_distance = limit;
    return distance_result(distance(ctx, dq), limit);
}

std::vector<BodyShape*> split_shapes(World& world, const std::vector<BodyShape*>& shapes,
                                     const std::vector<Plane>& planes,
                                     std::vector<std::unique_ptr<BodyShape>>& created) {
    std::vector<BodyShape*> work = shapes, out;
    Polyhedron above, below;
    for (const Plane& p : planes) {
        out.clear();
        for (BodyShape* s : work) {
            if (outside_any(planes, s->poly)) {
                out.push_back(s);
                continue;
            }
            bool up = false, down = false;
            for (const auto& v : s->poly.vertices) up = up || 0.06f < plane_dist(p, v.position);
            for (const auto& v : s->poly.vertices) down = down || plane_dist(p, v.position) < -0.06f;
            if (up && down) {
                s->poly.split(p.n, p.d, above, below);
                if (!above.faces.empty() && !below.faces.empty()) {
                    s->modified = true;
                    s->poly = above;
                    created.push_back(new_piece(world, *s, below));
                    out.push_back(created.back().get());
                }
            }
            out.push_back(s);
        }
        work = out;
    }
    return out;
}

RigidBody& clone_without_shapes(PhysicsWorld& world, const RigidBody& body) {
    RigidBody& c = world.create_body();
    c.set_transform(body.position, body.rotation);
    c.dynamic = body.dynamic;
    c.f118 = body.f118;
    c.broken = body.broken;
    c.age = body.age;
    c.split_piece = body.split_piece;
    c.solver = body.solver;  // the whole solver body, velocities included (a raw copy in the original)
    c.near_despawn = body.near_despawn;
    if (body.obstacle_list) {  // Entity::setObstacle: the clone joins the obstacle's entity list
        c.obstacle_list = body.obstacle_list;
        c.obstacle_list->push_back(&c);
    }
    return c;
}

void hand_over_joints(const CollisionContext& ctx, RigidBody& body, const std::vector<RigidBody*>& parts) {
    const std::vector<BodyJoint*> joints = body.joints;
    for (BodyJoint* j : joints) {
        const Vec3 anchor = j->a == &body ? j->anchor_a : j->anchor_b;
        RigidBody* target = nullptr;
        float best = FLT_MAX;
        for (RigidBody* part : parts) {
            const Vec3 cp = closest_point(ctx, *part, anchor);
            const Vec3 d{anchor.x - cp.x, anchor.y - cp.y, anchor.z - cp.z};
            const float dist = std::sqrt((d.y * d.y + d.x * d.x) + d.z * d.z);
            if (dist < best) best = dist, target = part;
        }
        if (!target || target == &body) continue;
        auto& list = body.joints;
        for (size_t i = 0; i < list.size(); ++i) {
            while (i < list.size() && list[i] == j) {
                list[i] = list.back();
                list.pop_back();
            }
        }
        target->joints.push_back(j);
        if (j->a == &body) {
            j->a = target;
        } else {
            j->b = target;
        }
    }
}

void check_connectivity(PhysicsWorld& world, RigidBody& body) {
    const size_t n = body.shapes.size();
    if (n > 0) {
        for (auto& s : body.shapes) compute_local_bounds(*s);
        std::vector<int> label(n);
        for (size_t i = 0; i < n; ++i) label[i] = static_cast<int>(i);
        constexpr float kGap = 0.04f;
        for (size_t i = 0; i + 1 < n; ++i) {
            for (size_t j = i + 1; j < n; ++j) {
                const int li = label[i], lj = label[j];
                if (li == lj) continue;
                const BodyShape &a = *body.shapes[i], &b = *body.shapes[j];
                if (!(b.bounds_min.x - kGap <= a.bounds_max.x && a.bounds_min.x - kGap <= b.bounds_max.x &&
                      b.bounds_min.y - kGap <= a.bounds_max.y && a.bounds_min.y - kGap <= b.bounds_max.y &&
                      b.bounds_min.z - kGap <= a.bounds_max.z && a.bounds_min.z - kGap <= b.bounds_max.z))
                    continue;
                if (!(shape_distance(world.context, a, b, kGap) < kGap)) continue;
                const int lo = li <= lj ? li : lj;
                for (int& l : label)
                    if (l == lj || l == li) l = lo;
            }
        }
        std::vector<int> groups = label;
        std::sort(groups.begin(), groups.end());
        groups.erase(std::unique(groups.begin(), groups.end()), groups.end());
        if (groups.size() > 1) {
            std::vector<BodyShape*> all;
            for (auto& s : body.shapes) all.push_back(s.get());
            std::vector<std::unique_ptr<BodyShape>> pool;
            for (BodyShape* s : all) pool.push_back(remove_shape(world, body, s));
            std::vector<RigidBody*> parts;
            for (size_t k = 0; k < groups.size(); ++k) {
                RigidBody* target = &body;
                if (k != 0) {
                    target = &clone_without_shapes(world, body);
                    target->dynamic = true;
                    target->broken = true;
                }
                for (size_t i = 0; i < n; ++i)
                    if (label[i] == groups[k]) target->add_shape(take(pool, all[i]));
                target->compute_mass_properties();
                parts.push_back(target);
            }
            hand_over_joints(world.context, body, parts);
        }
    }
    body.compute_mass_properties();
}

void break_body_split(PhysicsWorld& world, RigidBody& body, Vec3 point, float force, Rand48& rng) {
    const Vec3 lp = qi_rotate_inverse(body.rotation, point - body.position);
    const Vec3 n = random_unit(rng, 1.0f);
    const Plane plane{n, (-(n.y * lp.y) - n.x * lp.x) - n.z * lp.z};
    body.broken = true;
    RigidBody& clone = clone_without_shapes(world, body);
    clone.dynamic = true;
    clone.broken = true;
    Polyhedron above, below;
    for (size_t i = 0; i < body.shapes.size(); ++i) {
        BodyShape& s = *body.shapes[i];
        if (s.material.mask == 0 || force < s.material.strength) continue;
        bool down = false, up = false;
        for (const auto& v : s.poly.vertices) down = down || plane_dist(plane, v.position) < -0.06f;
        for (const auto& v : s.poly.vertices) up = up || 0.06f < plane_dist(plane, v.position);
        if (!(down && up)) continue;
        s.poly.split(plane.n, plane.d, above, below);
        if (above.faces.empty() || below.faces.empty()) continue;
        s.modified = true;
        s.poly = above;
        clone.add_shape(new_piece(world, s, below));
    }
    settle(world, body, clone, false);
}

bool process_breakage(PhysicsWorld& world, PhysicsEvents& events, Rand48& rng) {
    struct Hit {
        RigidBody* body;
        Vec3 point, dir;  // world contact point and the direction into the breaking body
        float strength;
        const BodyShape* shape;
    };
    std::vector<Hit> hits;
    auto listed = [&](const RigidBody* b) {
        for (const Hit& h : hits)
            if (h.body == b) return true;
        return false;
    };
    for (PairList& list : world.pairs) {
        for (const ShapePair& p : list.pairs) {
            const BodyShape &a = *p.a, &b = *p.b;
            if (!(a.material.strength < FLT_MAX) && !(b.material.strength < FLT_MAX)) continue;
            // Normal impulses of the last solve, summed in point order (the original's sum would run
            // on into the friction impulses for counts above 4, which never occur).
            const ContactCache& c = p.cache;
            const float terms[7] = {c.normal_impulse[0], c.normal_impulse[1], c.normal_impulse[2], c.normal_impulse[3],
                                    c.friction_impulse.x, c.friction_impulse.y, c.friction_impulse.z};
            float impulse = 0;
            for (int k = 0; k < c.count && k < 7; ++k) impulse = impulse + terms[k];
            RigidBody &ba = *a.body, &bb = *b.body;
            if (a.material.strength <= impulse && !listed(&ba) && 0.5f < ba.age) {
                if (bb.round) bb.broke_something = true;
                const Vec3 n = qi_rotate(ba.solver.rotation, p.manifold.normal_a);
                hits.push_back({&ba, qi_transform(ba.solver.position, ba.solver.rotation, p.manifold.point_a[0]),
                                {-n.x, -n.y, -n.z}, a.material.strength, &a});
            }
            if (b.material.strength <= impulse && !listed(&bb) && 0.5f < bb.age) {
                if (ba.round) ba.broke_something = true;
                hits.push_back({&bb, qi_transform(bb.solver.position, bb.solver.rotation, p.manifold.point_b[0]),
                                qi_rotate(bb.solver.rotation, p.manifold.normal_b), b.material.strength, &b});
            }
        }
    }
    if (hits.empty()) return false;

    bool thin = false;
    for (const Hit& h : hits) thin = thin || h.shape->thin;
    const int sound = events.pick_sound(thin ? PhysicsEvents::Bank::kBreakThin : PhysicsEvents::Bank::kBreak);
    events.play_sound(sound, hits.front().body->position, 1.0f, 1.0f);

    using Rgba = PhysicsEvents::Rgba;
    using Effect = PhysicsEvents::Effect;
    auto color_of = [](const Material& m) { return Rgba{m.color[0], m.color[1], m.color[2], m.color[3]}; };
    int debris = 0;  // at most 8 debris pieces per body and 32 per call
    for (const Hit& h : hits) {
        RigidBody& body = *h.body;
        Rgba tint;
        if (!body.near_despawn || !body.obstacle_plain_debris)
            for (const auto& s : body.shapes)
                if (s->material.strength < FLT_MAX) tint = color_of(s->material);
        if (!body.broken) events.first_break(body);
        break_body(world, body, h.point, h.dir, h.strength, 0.25f, rng);

        if (debris <= 31) {
            int k = debris;
            do {
                const Vec3 r = random_unit(rng, 2.0f);
                const Vec3 v = body.solver.linear_velocity;
                events.spawn_debris(body.near_despawn, h.point, {r.x + v.x * 1.2f, r.y + v.y * 1.2f, r.z + v.z * 1.2f},
                                    tint);
                ++k;
            } while (k - debris <= 7 && k <= 31);
            debris = k;
        }

        const Vec3 v = body.solver.linear_velocity;
        const Vec3 half_v{v.x * 0.5f, v.y * 0.5f, v.z * 0.5f};
        Rgba pc;
        for (const auto& s : body.shapes)
            if (s->material.sound_type == 2) pc = color_of(s->material);
        pc.a = 0.4f;
        for (int i = 0; i < 10; ++i) {
            const Vec3 r1 = random_unit(rng, rng.rnd_normal(0.0f, 0.4f));
            const Vec3 pos{h.point.x + r1.x, h.point.y + r1.y, h.point.z + r1.z};
            const Vec3 r2 = random_unit(rng, rng.rnd_normal(0.0f, 5.0f));
            const Vec3 d = h.dir;
            const Vec3 vel{(half_v.x + (d.x + d.x)) + r2.x, (half_v.y + (d.y + d.y)) + r2.y,
                           (half_v.z + (d.z + d.z)) + r2.z};
            events.spawn_particle(Effect::kBreakShards, pos, vel, pc, -1);
        }
        pc = {0.5f + pc.r * 0.5f, 0.5f + pc.g * 0.5f, 0.5f + pc.b * 0.5f, 0.6f};
        for (int i = 0; i < 2; ++i) {
            const Vec3 r1 = random_unit(rng, rng.rnd_normal(0.0f, 0.4f));
            const Vec3 pos{h.point.x + r1.x, h.point.y + r1.y, h.point.z + r1.z};
            const Vec3 r2 = random_unit(rng, rng.rnd_normal(0.0f, 0.4f));
            events.spawn_particle(Effect::kBreakDust, pos, {half_v.x + r2.x, half_v.y + r2.y, half_v.z + r2.z}, pc,
                                  -1);
        }
    }
    return true;
}

void break_body(PhysicsWorld& world, RigidBody& body, Vec3 point, Vec3 dir, float force, float radius,
                Rand48& rng) {
    const Vec3 lp = qi_rotate_inverse(body.rotation, point - body.position);
    const Vec3 ld = qi_rotate_inverse(body.rotation, dir);

    // A slab of half-thickness `radius` across `dir`, then two more pairs of planes along jittered
    // perpendicular directions: each plane passes through lp + radius * n.
    std::vector<Plane> planes;
    auto add_plane = [&](Vec3 n) {
        const Vec3 c{lp.x + radius * n.x, lp.y + radius * n.y, lp.z + radius * n.z};
        planes.push_back({n, (c.x * -n.x - c.y * n.y) - c.z * n.z});
    };
    add_plane(ld);
    add_plane(-ld);
    Vec3 a{1, 0, 0};
    if (0.7f < std::fabs(ld.z)) {
        const float len = std::sqrt((0.0f + ld.z * ld.z) + ld.y * ld.y);
        if (0 < len) a = {0.0f / len, -ld.z / len, ld.y / len};
    } else {
        const float len = std::sqrt((ld.x * ld.x + ld.y * ld.y) + 0.0f);
        if (0 < len) a = {-ld.y / len, ld.x / len, 0.0f / len};
    }
    const Vec3 a1 = unit_or_x(a + random_unit(rng, 0.3f));
    add_plane(a1);
    const Vec3 a2 = unit_or_x(a1 + random_unit(rng, 0.3f));
    add_plane(-a2);
    const Vec3 b{ld.y * a2.z - ld.z * a2.y, ld.z * a2.x - ld.x * a2.z, ld.x * a2.y - ld.y * a2.x};
    const Vec3 b1 = unit_or_x(b + random_unit(rng, 0.3f));
    add_plane(b1);
    const Vec3 b2 = unit_or_x(b1 + random_unit(rng, 0.3f));
    add_plane(-b2);
    body.broken = true;

    std::vector<BodyShape*> candidates;
    for (const auto& s : body.shapes)
        if (s->material.mask != 0 && s->material.strength <= force && !outside_any(planes, s->poly))
            candidates.push_back(s.get());

    std::vector<std::unique_ptr<BodyShape>> pool;
    const std::vector<BodyShape*> pieces = split_shapes(world, candidates, planes, pool);
    if (pieces.size() == 1) {
        break_body_split(world, body, point, force, rng);
        return;
    }
    for (BodyShape* s : candidates) pool.push_back(remove_shape(world, body, s));
    RigidBody& clone = clone_without_shapes(world, body);
    clone.dynamic = true;
    clone.broken = true;
    // Pieces holding the impact point go to the clone, the rest stay.
    const float limit = radius * 0.99f;
    for (BodyShape* s : pieces) {
        const Vec3 p = qi_rotate_inverse(body.rotation, point - body.position);
        const float d = shape_distance(world.context, *s, p, limit);
        (d < limit ? clone : body).add_shape(take(pool, s));
    }
    RigidBody* next = settle(world, body, clone, true);
    check_connectivity(world, body);
    if (next) {
        next->split_piece = true;
        break_body_split(world, *next, point, force, rng);
    }
}

}  // namespace sh::phys
