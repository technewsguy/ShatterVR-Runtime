#pragma once

// Contact constraints (solver slots 0 and 1). Behaviour:

#include "physics/body.hpp"
#include "physics/solver.hpp"

#include <memory>
#include <vector>

namespace sh::phys {

// Per body-pair state kept by the caller across frames (friction anchors, last impulses).
struct ContactCache {
    int state = 0;  // 0 none, 1 was not sliding, 2 anchored
    Vec3 anchor_a, anchor_b;
    Vec3 t1_a, t1_b;
    Vec3 n_a, n_b;
    float normal_impulse[4] = {0, 0, 0, 0};
    Vec3 friction_impulse;  // tangent 1, tangent 2, twist
    int count = 0;
};

struct ContactInput {
    Body* a = nullptr;  // null: the solver's static world body
    Body* b = nullptr;
    int count = 0;      // 0..4
    Vec3 normal_a;      // A-local, pointing from A to B
    Vec3 normal_b;      // the same normal in B's frame (used when A/B are swapped)
    Vec3 point_a[4];    // A-local
    Vec3 point_b[4];    // B-local
    ContactCache* cache = nullptr;
};

// Routes a contact to the dynamic or static slot (swapping A/B if A is static).
void insert_contact(Solver& solver, const ContactInput& contact);

// Shared storage for both contact slots.
class ContactTypeBase : public ConstraintType {
public:
    std::vector<ContactInput> inputs;
};

std::unique_ptr<ContactTypeBase> make_contact_type();         // slot 0: two dynamic bodies
std::unique_ptr<ContactTypeBase> make_static_contact_type();  // slot 1: dynamic vs static

// Pose of b expressed in a's frame.
Pose relative_pose(const Body& a, const Body& b);

}  // namespace sh::phys
