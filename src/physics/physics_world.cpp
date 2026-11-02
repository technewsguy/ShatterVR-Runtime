#include "physics/physics_world.hpp"

#include "physics/breakage.hpp"
#include "physics/qi_random.hpp"
#include "physics/qi_rotate.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>

namespace sh::phys {

namespace {

constexpr Vec3 kAxes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

// QiQuat::getAxisAngle: angle 2 acos(w) (w clamped to [-1, 1]); axis (x, y, z) / sqrt(1 - w^2), or X.
void axis_angle_of(Quat q, Vec3& axis, float& angle) {
    float w = q.w;
    if (!(-1.0f < q.w)) w = -1.0f;  // also NaN
    else if (!(q.w < 1.0f)) w = 1.0f;
    const float a = std::acos(w);
    angle = a + a;
    const float s = 1.0f - q.w * q.w;
    axis = kAxes[0];
    if (s <= 0.0f) return;
    const float len = std::sqrt(s);
    if (0 < len) axis = {q.x / len, q.y / len, q.z / len};
}

// conjugate(q) * t, grouped as Physics::animate computes it.
Quat to_target(Quat q, Quat t) {
    const float cx = -q.x, cy = -q.y, cz = -q.z;
    return {((cz * t.y - cy * t.z) + cx * t.w) + q.w * t.x, ((cx * t.z - cz * t.x) + cy * t.w) + q.w * t.y,
            ((cy * t.x - cx * t.y) + cz * t.w) + q.w * t.z, q.w * t.w - ((cy * t.y + cx * t.x) + cz * t.z)};
}

// The drive bias clamp: [-limit, limit], written the way the original branches.
float clamp_drive(float b, float limit) {
    if (b <= -limit) b = -limit;
    return limit > b ? b : limit;
}

float speed(Vec3 v) { return std::sqrt((v.y * v.y + v.x * v.x) + v.z * v.z); }

}  // namespace

void animate(SolverCalls& solver, const std::vector<RigidBody*>& bodies, float dt) {
    for (RigidBody* b : bodies) {
        if (!b->alive || !b->anim) continue;
        const RigidBody::AnimTarget& t = *b->anim;
        const float mass = 1.0f / b->solver.inv_mass;
        solver.push_state();
        if (t.angular_strength != 0) {
            // About the body's hinge axis if it has a hinge (the last one listed), else about X, Y, Z.
            bool hinged = false;
            Vec3 hinge;
            for (const BodyJoint* j : b->joints) {
                if (j->type != BodyJoint::kHinge) continue;
                hinge = qi_rotate(b->rotation, j->a == b ? j->axis_a : j->axis_b);
                hinged = true;
            }
            solver.set_param(0, mass * t.angular_strength);
            solver.set_param(1, -(t.angular_strength * mass));
            Quat r = to_target(b->rotation, t.rotation);
            const Quat& q = b->rotation;
            if (((q.y * t.rotation.y + q.x * t.rotation.x) + q.z * t.rotation.z) + q.w * t.rotation.w < 0)
                r = {-r.x, -r.y, -r.z, -r.w};
            Vec3 axis;
            float angle;
            axis_angle_of(r, axis, angle);
            const Vec3 rv{angle * axis.x, angle * axis.y, angle * axis.z};
            for (int k = 0; k < (hinged ? 1 : 3); ++k) {
                const Vec3 n = hinged ? hinge : kAxes[k];
                AngularJoint row;
                row.a = &b->solver;
                row.axis_a = qi_rotate_inverse(b->solver.rotation, n);
                row.bias = clamp_drive(-((n.y * rv.y + n.x * rv.x) + n.z * rv.z) / dt, t.max_angular_speed);
                solver.insert_angular(row);
            }
        }
        if (0 < t.linear_strength) {
            solver.set_param(0, mass * t.linear_strength);
            solver.set_param(1, -(t.linear_strength * mass));
            const Vec3 e = t.position - b->position;
            for (const Vec3& n : kAxes) {
                LinearJoint row;
                row.a = &b->solver;
                row.dir_a = qi_rotate_inverse(b->solver.rotation, n);
                row.bias = clamp_drive(-((e.y * n.y + e.x * n.x) + e.z * n.z) / dt, t.max_linear_speed);
                solver.insert_linear(row);
            }
        }
        solver.pop_state();
    }
}

PhysicsWorld::PhysicsWorld() { solver.set_iteration_count(4); }

void PhysicsWorld::remove_pairs(const RigidBody& body) {
    pairs[0].remove_body(body);
    pairs[1].remove_body(body);
}

void PhysicsWorld::remove_pairs(const BodyShape& shape) {
    pairs[0].remove_shape(shape);
    pairs[1].remove_shape(shape);
}

bool PhysicsWorld::raycast(Vec3 from, Vec3 to, std::uint8_t mask, Vec3* point, Vec3* normal,
                           const BodyShape** shape, const void* obstacle) const {
    const Vec3 d{to.x - from.x, to.y - from.y, to.z - from.z};
    const float len = std::sqrt((d.y * d.y + d.x * d.x) + d.z * d.z);
    if (len == 0) return false;
    std::vector<void*> hits;
    tree_a.query_line(from, to, hits);
    tree_b.query_line(from, to, hits);
    const Vec3 dir{d.x / len, d.y / len, d.z / len};
    float best = FLT_MAX;
    for (void* h : hits) {
        const BodyShape& s = *static_cast<const BodyShape*>(h);
        if (!(mask & s.material.group)) continue;
        if (obstacle && s.body->near_despawn != obstacle) continue;
        const Shape cs = s.body->round ? Shape::sphere(0.2f) : Shape::hull(s.hull.data(), static_cast<int>(s.hull.size()));
        const Vec3 pos = s.body->solver.position;
        const Quat rot = s.body->solver.rotation;
        RaycastQuery q;
        q.shape = &cs;
        q.origin = qi_rotate_inverse(rot, from - pos);
        q.direction = qi_rotate_inverse(rot, dir);
        q.flags = 0x23;
        q.max_distance = len;
        const DistanceResult r = phys::raycast(context, q);
        if (r.flags & kResultPenetrating) {
            if (point) *point = from;
            if (normal) *normal = dir;
            if (shape) *shape = &s;
            return true;
        }
        if ((r.flags & (kResultValid | kResultPoints)) == (kResultValid | kResultPoints) && r.distance < best) {
            if (point) *point = qi_transform(pos, rot, r.point_a);
            if (normal) *normal = qi_rotate(rot, r.normal);
            best = r.distance;
            if (shape) *shape = &s;
        }
    }
    return best < FLT_MAX;
}

void PhysicsWorld::collect(std::vector<RigidBody*>* out_bodies, std::vector<BodyShape*>* out_shapes, Vec3 min, Vec3 max,
                           std::uint8_t mask, const void* obstacle) const {
    std::vector<void*> hits;
    tree_a.query_aabb(min, max, hits);
    tree_b.query_aabb(min, max, hits);
    std::vector<RigidBody*> seen;
    for (void* h : hits) {
        BodyShape* s = static_cast<BodyShape*>(h);
        if (!(mask & s->material.group)) continue;
        if (obstacle && s->body->near_despawn != obstacle) continue;
        if (out_shapes) out_shapes->push_back(s);
        if (!out_bodies || std::find(seen.begin(), seen.end(), s->body) != seen.end()) continue;
        seen.push_back(s->body);
        out_bodies->push_back(s->body);
    }
}
bool PhysicsWorld::closest_point(float radius, Vec3 p, std::uint8_t mask, Vec3* point, Vec3* normal,
                                 const BodyShape** shape, const void* obstacle) const {
    std::vector<RigidBody*> near;
    collect(&near, nullptr, {p.x - radius, p.y - radius, p.z - radius}, {radius + p.x, radius + p.y, radius + p.z},
            mask, nullptr);
    float best = FLT_MAX;
    const BodyShape* s = nullptr;  // carried over between bodies, as the original's local
    for (const RigidBody* b : near) {
        if (obstacle && b->near_despawn != obstacle) continue;
        Vec3 n{0, 0, 0};
        const Vec3 local = qi_rotate_inverse(b->rotation, p - b->position);
        const Vec3 cp = phys::closest_point(context, *b, local, &s, &n, nullptr);
        const Vec3 w = qi_transform(b->position, b->rotation, cp);
        const Vec3 d{w.x - p.x, w.y - p.y, w.z - p.z};
        const float dist = std::sqrt((d.y * d.y + d.x * d.x) + d.z * d.z);
        if (!(dist < best)) continue;
        if (point) *point = qi_transform(b->position, b->rotation, w);  // transformed twice, as the original
        if (normal) *normal = qi_rotate(b->rotation, n);
        best = dist;
        if (shape) *shape = s;
    }
    return best < FLT_MAX;
}

namespace {

bool overlaps_any(const PhysicsWorld& w, Vec3 p, const Shape& query, Vec3 lo, Vec3 hi) {
    std::vector<void*> hits;
    w.tree_a.query_aabb(lo, hi, hits);
    w.tree_b.query_aabb(lo, hi, hits);
    for (void* h : hits) {
        const BodyShape& s = *static_cast<const BodyShape*>(h);
        const Shape hull = Shape::hull(s.hull.data(), static_cast<int>(s.hull.size()));
        OverlapQuery q;
        q.a = &hull;
        q.b = &query;
        q.b_position = qi_rotate_inverse(s.body->solver.rotation, p - s.body->solver.position);
        q.b_rotation = {0, 0, 0, 1};
        if (overlap(w.context, q)) return true;
    }
    return false;
}

}  // namespace

bool PhysicsWorld::collide_sphere(Vec3 p, float radius) const {
    return overlaps_any(*this, p, Shape::sphere(radius), {p.x - radius, p.y - radius, p.z - radius},
                        {radius + p.x, radius + p.y, radius + p.z});
}

bool PhysicsWorld::collide_box(Vec3 p, Vec3 half) const {
    return overlaps_any(*this, p, Shape::box(half), {p.x - half.x, p.y - half.y, p.z - half.z},
                        {p.x + half.x, p.y + half.y, p.z + half.z});
}

RigidBody& PhysicsWorld::create_body() {
    owned_bodies.push_back(std::make_unique<RigidBody>(*this));
    bodies.push_back(owned_bodies.back().get());
    entities.push_back(owned_bodies.back().get());
    return *owned_bodies.back();
}

namespace {

// Removes every occurrence, the last entry taking each freed slot (the original's QiArray removal).
template <class T, class U>
void remove_all(std::vector<T*>& list, const U* x) {
    for (size_t i = 0; i < list.size();) {
        if (list[i] != x) {
            ++i;
            continue;
        }
        list[i] = list.back();
        list.pop_back();
    }
}

}  // namespace

void PhysicsWorld::add_entity(Entity& e) { entities.push_back(&e); }

void PhysicsWorld::remove_entity(Entity& e) {
    if (e.obstacle_list) remove_all(*e.obstacle_list, &e);
    remove_all(entities, &e);
}

void PhysicsWorld::destroy_joint(BodyJoint& joint) {
    remove_all(joint.a->joints, &joint);
    remove_all(joint.b->joints, &joint);
    remove_all(joints, &joint);
    for (size_t i = 0; i < owned_joints.size(); ++i) {
        if (owned_joints[i].get() != &joint) continue;
        owned_joints.erase(owned_joints.begin() + static_cast<std::ptrdiff_t>(i));
        break;
    }
}

BodyJoint& PhysicsWorld::create_joint(RigidBody* a, RigidBody* b) {
    owned_joints.push_back(std::make_unique<BodyJoint>(a, b));
    BodyJoint* j = owned_joints.back().get();
    a->joints.push_back(j);
    b->joints.push_back(j);
    joints.push_back(j);
    return *j;
}

void PhysicsWorld::destroy_shape(RigidBody& body, const BodyShape* shape) {
    remove_pairs(*shape);
    const std::unique_ptr<BodyShape> owned = body.take_shape(shape);
    if (owned->tree_leaf >= 0) (owned->in_tree_a ? tree_a : tree_b).destroy(owned->tree_leaf);
}

void PhysicsWorld::destroy_body(RigidBody& body, PhysicsEvents* events) {
    while (!body.joints.empty()) destroy_joint(*body.joints.front());
    if (body.round && events) events->ball_removed(body.broke_something);
    remove_pairs(body);
    remove_all(bodies, &body);
    remove_all(entities, &body);
    while (!body.shapes.empty()) destroy_shape(body, body.shapes.front().get());
    if (body.obstacle_list) remove_all(*body.obstacle_list, &body);  // ~Entity
    for (size_t i = 0; i < owned_bodies.size(); ++i) {
        if (owned_bodies[i].get() != &body) continue;
        owned_bodies.erase(owned_bodies.begin() + static_cast<std::ptrdiff_t>(i));
        break;
    }
}

void PhysicsWorld::update(PhysicsEvents& events, Rand48& rng) {
    const float dt = time_step;

    // Refresh the bounds of moving bodies and despawn the ones the camera has left behind.
    const bool camera_ahead = 0 < qi_rotate(camera_rotation, {0, 0, 1}).z;
    for (RigidBody* b : bodies) {
        if (!b->dynamic || !b->alive) continue;
        b->compute_bounds();
        if (b->anim || !b->joints.empty() || b->shapes.empty()) continue;
        for (const auto& s : b->shapes) {
            const float limit = camera_ahead && b->near_despawn ? camera_z + 3.0f : camera_z + 25.0f;
            if (s->bounds_min.z > limit || !(s->bounds_max.y >= -25.0f)) {
                b->alive = false;
                remove_pairs(*b);
            }
        }
    }

    SolverAdapter calls(solver);
    bool first = true;
    for (int passes = 3;;) {
        pairs[1].update(tree_b, tree_b);
        pairs[0].update(tree_a, tree_b);
        solver.reset();
        solver.set_linear_acc(gravity);
        for (RigidBody* b : bodies) {
            if (!b->alive) continue;
            if (b->round && b->attach_target) {
                const RigidBody& t = *b->attach_target;
                const Vec3 r = qi_rotate(t.rotation, b->attach_offset);
                const Vec3 d{(r.x + t.position.x) - b->position.x, (r.y + t.position.y) - b->position.y,
                             (r.z + t.position.z) - b->position.z};
                if ((d.y * d.y + d.x * d.x) + d.z * d.z < 1.0f) b->attach_target = nullptr;
                b->solver.linear_velocity.y = b->solver.linear_velocity.y + dt * 10.0f;
                if (frame % 10 == 0)
                    events.spawn_particle(PhysicsEvents::Effect::kAttached, b->position, random_unit(rng, 0.1f), {}, -1);
            }
            b->solver.flags = b->dynamic ? 0 : Body::kStatic;
            solver.insert_body(b->solver);
        }
        if (first) solver.integrate_velocity(dt);
        animate(calls, bodies, dt);
        insert_joints(calls, joints, dt);
        // The two near-phase jobs, run one after the other: even-numbered pairs of each list, then odd.
        for (int parity = 0; parity < 2; ++parity)
            for (PairList& list : pairs)
                for (size_t i = static_cast<size_t>(parity); i < list.pairs.size(); i += 2)
                    generate_contacts(list.pairs[i], context, dt);
        insert_contacts(solver, {&pairs[0], &pairs[1]});
        solver.solve_constraints(dt);
        if (--passes == 0 || !process_breakage(*this, events, rng)) break;
        first = false;
    }
    solver.integrate_position(dt);

    // Armed balls explode (and their bodies go away).
    for (size_t i = 0; i < bodies.size(); ++i) {
        RigidBody* b = bodies[i];
        if (!b->alive || !b->round || b->type != 2) continue;
        events.explosion(b, b->position, 10.0f);
        destroy_body(*b, &events);  // Entity::destroy -> Level::destroy: the last body takes slot i
        --i;
    }
    events.update_debris();

    // Entity poses from the solver, then sounds.
    for (RigidBody* b : bodies) {
        if (!b->alive || !b->dynamic) continue;
        const Quat q = b->solver.rotation, c = b->inv_com_rotation;
        b->rotation = {(q.y * c.z - q.z * c.y) + q.w * c.x + q.x * c.w, (q.z * c.x - q.x * c.z) + q.w * c.y + q.y * c.w,
                       (q.x * c.y - q.y * c.x) + q.w * c.z + q.z * c.w, q.w * c.w - (q.y * c.y + q.x * c.x + q.z * c.z)};
        b->position = qi_transform(b->solver.position, q, b->inv_com_position);
        const Vec3 v = b->solver.linear_velocity;
        const float sp = speed(v);
        if (!b->anim && std::fabs(v.y) < 0.1f && 0.1f < sp && 0 < b->solver.contact_impulse) {
            if (b->round) events.play_loop(PhysicsEvents::Loop::kBall, b->position, sp * 0.1f);
            else events.play_loop(PhysicsEvents::Loop::kObject, b->position, 0.4f * sp);
        }
        const float sq = sp * sp;
        const float drop = b->prev_speed_sq - sq;
        const float threshold = b->round ? 1.0f : 0.2f;
        if (threshold < drop && !b->anim && b->position.z < camera_z + 5.0f) {
            float volume = (drop - threshold) / 10.0f;
            if (!(volume > 0)) volume = 0;
            else if (!(volume < 1.0f)) volume = 1.0f;
            using Bank = PhysicsEvents::Bank;
            int sound = 0;
            bool chosen = false;
            if (!b->round) {
                const Bank bank = b->shapes[0]->material.sound_type == 3 ? Bank::kObjectType3
                                  : drop > 2.0f                          ? Bank::kObjectHard
                                                                         : Bank::kObjectSoft;
                sound = events.pick_sound(bank);
                chosen = true;
            } else if (b->impact_round) {
                sound = events.pick_sound(Bank::kBallBall);
                for (int k = 0; k < 8; ++k) {
                    const Vec3 r = random_unit(rng, 2.0f);
                    // + 0 on x and z as the original adds it (turns -0 into +0).
                    events.spawn_particle(PhysicsEvents::Effect::kBallBurst, b->position,
                                          {r.x + 0.0f, r.y + 2.0f, r.z + 0.0f}, {}, -1);
                }
                chosen = sound != 0;
            }
            if (!chosen) {
                if (!b->impact_static) {
                    sound = events.pick_sound(Bank::kBallSoft);
                } else {
                    const Vec3 vel = b->solver.linear_velocity;
                    const float len = speed(vel);
                    const float div = len + FLT_EPSILON;
                    const Vec3 n{vel.x / div, vel.y / div, vel.z / div};
                    const Vec3 in = b->impact_normal;
                    if (10.0f < len && std::fabs((in.y * n.y + n.x * in.x) + in.z * n.z) < 0.3f &&
                        0.5f < rng.rnd(0.0f, 1.0f))
                        events.play_sound(events.pick_sound(Bank::kBallGraze), b->position, 0.3f, 1.0f);
                    sound = events.pick_sound(in.y > 0.9f ? Bank::kBallHard : Bank::kBallSide);
                }
            }
            events.play_sound(sound, b->position, volume * 0.5f, 1.0f);
        }
        b->prev_speed_sq = sq;
        b->impact_static = false;
        b->impact_round = false;
    }
    tree_a.optimize(2, rng);
}

}  // namespace sh::phys
