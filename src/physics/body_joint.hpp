#pragma once

// The game's joints (the original's Joint class and Physics::insertJoints):
// ball, hinge, slider and rope joints between two bodies, turned into solver rows each pass.

#include "physics/joint.hpp"
#include "physics/rigid_body.hpp"

#include <cfloat>
#include <vector>

namespace sh::phys {

struct BodyJoint {
    enum Type : int { kNone = 0, kBall = 1, kHinge = 2, kSlider = 3, kRope = 4 };

    BodyJoint(RigidBody* body_a, RigidBody* body_b) : a(body_a), b(body_b) {}

    int type = kNone;
    RigidBody* a = nullptr;
    RigidBody* b = nullptr;
    Vec3 anchor_a, anchor_b;  // each body's entity frame
    Vec3 axis_a, axis_b;      // the same axis in each body's entity frame (hinge, slider)
    float rope_length = 0;    // rope: allowed anchor distance
    float strength = FLT_MAX; // unverified use (not read by insertJoints)
    bool active = true;       // the two bodies never collide while set; ropes clear it
    float motor_speed = 0;    // hinge motor: target relative spin about the axis (bias)
    float motor_torque = 0;   // hinge motor: impulse cap; 0 = no motor

    // Joint::init*: anchors (and axes) from a world point (and world axis) and the bodies' current poses.
    void init_ball(Vec3 point);
    void init_hinge(Vec3 point, Vec3 axis);
    void init_slider(Vec3 point, Vec3 axis);
    void init_rope(Vec3 point_a, Vec3 point_b, float slack);
};

// The solver calls insertJoints makes (an interface so tests can record them).
class SolverCalls {
public:
    virtual ~SolverCalls() = default;
    virtual void push_state() = 0;
    virtual void pop_state() = 0;
    virtual void set_param(int index, float value) = 0;
    virtual void insert_linear(const LinearJoint& row) = 0;
    virtual void insert_angular(const AngularJoint& row) = 0;
};

// Forwards to the real solver.
class SolverAdapter : public SolverCalls {
public:
    explicit SolverAdapter(Solver& s) : s_(s) {}
    void push_state() override;
    void pop_state() override;
    void set_param(int index, float value) override;
    void insert_linear(const LinearJoint& row) override;
    void insert_angular(const AngularJoint& row) override;

private:
    Solver& s_;
};

// Physics::insertJoints over the level's joint list, in order. dt: the step (Game +0x100).
void insert_joints(SolverCalls& solver, const std::vector<BodyJoint*>& joints, float dt);
void insert_joints(Solver& solver, const std::vector<BodyJoint*>& joints, float dt);

}  // namespace sh::phys
