#pragma once

// Shape pairs and their contacts, as the original's Physics keeps them:
// BroadPhaseJob::run (pair list upkeep), generateContacts (one manifold per pair) and
// Physics::insertContacts (manifolds to solver contacts with per-pair material parameters).

#include "physics/collision.hpp"
#include "physics/contact.hpp"
#include "physics/rigid_body.hpp"

#include <set>
#include <utility>
#include <vector>

namespace sh::phys {

class ShapePair {
public:
    BodyShape* a = nullptr;  // the earlier-created shape
    BodyShape* b = nullptr;
    Manifold manifold;       // contact points, A and B shape bodies' solver frames
    ContactCache cache;      // solver friction anchors, survives frames
};

// One of the original's two pair lists with its membership set.
class PairList {
public:
    std::vector<ShapePair> pairs;

    // BroadPhaseJob::run: add the overlapping, colliding shape pairs of the two trees (a tree may be
    // paired with itself) that are not listed yet, then drop pairs whose shape boxes no longer overlap.
    void update(const AabbTree& tree1, const AabbTree& tree2);
    // Physics::remove(Body*): drop every pair involving the body (the last pair takes the slot).
    void remove_body(const RigidBody& body);
    // Physics::remove(Shape*): the same for every pair involving the shape.
    void remove_shape(const BodyShape& shape);
};

// Whether the original lets two shapes form a pair (bodies, groups, ball age, joints).
bool shapes_may_collide(const BodyShape& s1, const BodyShape& s2);

// generateContacts: brings the pair's manifold up to date. Round bodies make it a single-point
// distance query between 0.1 spheres and record impact data on the round bodies.
void generate_contacts(ShapePair& pair, const CollisionContext& ctx, float time_step);

// Physics::insertContacts. Solver needs push_state, pop_state, set_param(int, float) and
// insert(const ContactInput&); the real solver is wrapped by insert_contacts(Solver&, ...).
template <class SolverLike>
void insert_contacts_into(SolverLike& solver, const std::vector<PairList*>& lists) {
    solver.push_state();
    solver.set_param(4, 0.02f);
    solver.set_param(2, 1.0f);
    for (PairList* list : lists) {
        for (ShapePair& p : list->pairs) {
            if (p.manifold.count <= 0) continue;
            ContactInput c;
            c.a = &p.a->body->solver;
            c.b = &p.b->body->solver;
            c.count = p.manifold.count;
            c.normal_a = p.manifold.normal_a;
            c.normal_b = p.manifold.normal_b;
            for (int i = 0; i < 4; ++i) c.point_a[i] = p.manifold.point_a[i], c.point_b[i] = p.manifold.point_b[i];
            c.cache = &p.cache;
            const Material &ma = p.a->material, &mb = p.b->material;
            solver.set_param(2, ma.friction * mb.friction);
            solver.set_param(3, ma.restitution <= mb.restitution ? mb.restitution : ma.restitution);
            solver.set_param(0, ma.strength < mb.strength ? ma.strength : mb.strength);
            solver.insert(c);
        }
    }
    solver.pop_state();
}

void insert_contacts(Solver& solver, const std::vector<PairList*>& lists);

}  // namespace sh::phys
