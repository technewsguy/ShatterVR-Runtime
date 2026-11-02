#include "physics/pairs.hpp"

#include "physics/body_joint.hpp"
#include "physics/qi_rotate.hpp"

#include <cmath>
#include <cstdint>

namespace sh::phys {

namespace {



float speed(Vec3 v) { return std::sqrt((v.y * v.y + v.x * v.x) + v.z * v.z); }

bool far_point(Vec3 p) { return 1000000.0f < (p.y * p.y + p.x * p.x) + p.z * p.z; }

}  // namespace

bool shapes_may_collide(const BodyShape& s1, const BodyShape& s2) {
    const RigidBody &A = *s1.body, &B = *s2.body;
    if (&A == &B || !A.alive || !B.alive) return false;
    if (!A.dynamic && !B.dynamic) return false;
    if (!(s2.material.mask & s1.material.group) || !(s1.material.mask & s2.material.group)) return false;
    if (A.round && B.round && !(0.5f <= A.age) && !(0.5f <= B.age)) return false;  // two fresh balls
    for (const BodyJoint* j : A.joints) {
        if (!j->active) continue;
        const RigidBody* other = j->a == &A ? j->b : j->a;
        if (other == &B) return false;
    }
    return true;
}

void PairList::update(const AabbTree& tree1, const AabbTree& tree2) {
    // Pairs listed before this update; pairs added below are not looked up again.
    std::set<std::pair<std::uint32_t, std::uint32_t>> listed;
    for (const ShapePair& p : pairs) listed.insert({p.a->serial, p.b->serial});
    std::vector<std::pair<void*, void*>> found;
    tree1.query_tree(tree2, found);
    for (const auto& [u1, u2] : found) {
        BodyShape* s1 = static_cast<BodyShape*>(u1);
        BodyShape* s2 = static_cast<BodyShape*>(u2);
        if (!shapes_may_collide(*s1, *s2)) continue;
        BodyShape* lo = s2->serial < s1->serial ? s2 : s1;
        BodyShape* hi = lo == s1 ? s2 : s1;
        if (listed.count({lo->serial, hi->serial})) continue;
        ShapePair p;
        p.a = lo;
        p.b = hi;
        pairs.push_back(p);
    }
    // Drop pairs whose shape boxes are apart (the last pair takes the slot).
    for (size_t i = 0; i < pairs.size(); ++i) {
        const BodyShape &a = *pairs[i].a, &b = *pairs[i].b;
        if (b.bounds_max.x < a.bounds_min.x || a.bounds_max.x < b.bounds_min.x || b.bounds_max.y < a.bounds_min.y ||
            a.bounds_max.y < b.bounds_min.y || b.bounds_max.z < a.bounds_min.z || a.bounds_max.z < b.bounds_min.z) {
            pairs[i] = pairs.back();
            pairs.pop_back();
            --i;
        }
    }
}

void PairList::remove_body(const RigidBody& body) {
    for (size_t i = 0; i < pairs.size(); ++i) {
        while (i < pairs.size() && (pairs[i].a->body == &body || pairs[i].b->body == &body)) {
            pairs[i] = pairs.back();
            pairs.pop_back();
        }
    }
}

void PairList::remove_shape(const BodyShape& shape) {
    for (size_t i = 0; i < pairs.size(); ++i) {
        while (i < pairs.size() && (pairs[i].a == &shape || pairs[i].b == &shape)) {
            pairs[i] = pairs.back();
            pairs.pop_back();
        }
    }
}

void generate_contacts(ShapePair& pair, const CollisionContext& ctx, float dt) {
    RigidBody &A = *pair.a->body, &B = *pair.b->body;
    bool round = false;
    auto shape_of = [&round](BodyShape& s) {
        if (!s.body->round) return Shape::hull(s.hull.data(), static_cast<int>(s.hull.size()));
        round = true;
        if (s.body->type == 1) s.body->type = 2;
        return Shape::sphere(0.1f);
    };
    const Shape sa = shape_of(*pair.a);
    const Shape sb = shape_of(*pair.b);
    const Body &ba = A.solver, &bb = B.solver;
    const Vec3 pos = qi_rotate_inverse(ba.rotation, {bb.position.x - ba.position.x, bb.position.y - ba.position.y,
                                                  bb.position.z - ba.position.z});
    const Quat rot = qi_relative_rotation(ba.rotation, bb.rotation);
    float margin = speed(ba.linear_velocity) * dt;
    margin = margin + speed(bb.linear_velocity) * dt;
    margin = margin + 0.04f;
    Manifold& m = pair.manifold;
    if (!round) {
        ManifoldQuery q;
        q.a = &sa, q.b = &sb, q.b_position = pos, q.b_rotation = rot, q.max_distance = margin, q.tolerance = 0.02f;
        update_manifold(ctx, m, q);
    } else {
        DistanceQuery q;
        q.a = &sa, q.b = &sb, q.b_position = pos, q.b_rotation = rot, q.flags = 0x33, q.max_distance = margin;
        const DistanceResult r = distance(ctx, q);
        if (!(r.flags & kResultValid) || !(r.distance < margin) || !(r.flags & kResultPoints)) {
            m.count = 0;
            return;
        }
        m.count = 1;
        m.normal_a = r.normal;
        m.normal_b = qi_rotate_inverse(rot, r.normal);
        m.point_a[0] = r.point_a;
        m.point_b[0] = r.point_b;
        // Impact data for sounds, in world space via A's entity rotation (for either body).
        if (A.round) {
            A.impact_normal = qi_rotate(A.rotation, r.normal);
            A.impact_static = !B.dynamic;
        }
        if (B.round) {
            B.impact_normal = qi_rotate(A.rotation, r.normal);
            B.impact_static = !A.dynamic;
            if (A.round) B.impact_round = A.impact_round = true;
        }
    }
    // Points absurdly far from their bodies void the manifold.
    for (int i = 0; i < m.count; ++i) {
        if (far_point(m.point_a[i]) || far_point(m.point_b[i])) {
            m.count = 0;
            return;
        }
    }
}

void insert_contacts(Solver& solver, const std::vector<PairList*>& lists) {
    struct Adapter {
        Solver& s;
        void push_state() { s.push_state(); }
        void pop_state() { s.pop_state(); }
        void set_param(int i, float v) { s.set_param(i, v); }
        void insert(const ContactInput& c) { insert_contact(s, c); }
    } adapter{solver};
    insert_contacts_into(adapter, lists);
}

}  // namespace sh::phys
