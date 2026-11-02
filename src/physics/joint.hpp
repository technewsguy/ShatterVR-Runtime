#pragma once

// Joint constraints (solver slots 2 and 3). Behaviour:

#include "physics/body.hpp"
#include "physics/solver.hpp"

#include <memory>

namespace sh::phys {

// Drives the relative velocity of two anchor points along a direction to -bias.
struct LinearJoint {
    Body* a = nullptr;  // null: the solver's static world body
    Body* b = nullptr;
    Vec3 anchor_a, anchor_b;  // body-local
    Vec3 dir_a, dir_b;        // the same direction in A's and B's frames
    float bias = 0;
};

// Drives (axis . wA - axis . wB) to -bias.
struct AngularJoint {
    Body* a = nullptr;
    Body* b = nullptr;
    Vec3 axis_a, axis_b;  // the same axis in A's and B's frames
    float bias = 0;
};

void insert_linear(Solver& solver, const LinearJoint& joint);
void insert_angular(Solver& solver, const AngularJoint& joint);

std::unique_ptr<ConstraintType> make_linear_type();
std::unique_ptr<ConstraintType> make_angular_type();

}  // namespace sh::phys
