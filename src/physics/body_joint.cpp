#include "physics/body_joint.hpp"

#include "physics/qi_rotate.hpp"

#include <cmath>

namespace sh::phys {

namespace {

// A unit vector perpendicular to v, the original's rule (as for contact tangents).
Vec3 perpendicular(Vec3 v) {
    if (0.7f < std::fabs(v.z)) {
        const float s = -v.z;
        const float len = std::sqrt(s * s + v.y * v.y);
        if (0 < len) return {0.0f / len, s / len, v.y / len};
    } else {
        const float s = -v.y;
        const float len = std::sqrt((v.x * v.x + s * s) + 0.0f);
        if (0 < len) return {s / len, v.x / len, 0.0f / len};
    }
    return {1, 0, 0};
}

// Normalised sum of two vectors, (1, 0, 0) if it vanishes.
Vec3 unit_sum(Vec3 a, Vec3 b) {
    const Vec3 s{a.x + b.x, a.y + b.y, a.z + b.z};
    const float len = std::sqrt((s.y * s.y + s.x * s.x) + s.z * s.z);
    if (0 < len) return {s.x / len, s.y / len, s.z / len};
    return {1, 0, 0};
}

Vec3 to_local(const Body& b, Vec3 world_point) { return qi_rotate_inverse(b.rotation, world_point - b.position); }


}  // namespace

void BodyJoint::init_ball(Vec3 point) {
    type = kBall;
    anchor_a = qi_rotate_inverse(a->rotation, point - a->position);
    anchor_b = qi_rotate_inverse(b->rotation, point - b->position);
}

void BodyJoint::init_hinge(Vec3 point, Vec3 axis) {
    init_ball(point);
    type = kHinge;
    axis_a = qi_rotate_inverse(a->rotation, axis);
    axis_b = qi_rotate_inverse(b->rotation, axis);
    motor_speed = 0;
    motor_torque = 0;
}

void BodyJoint::init_slider(Vec3 point, Vec3 axis) {
    init_hinge(point, axis);
    type = kSlider;
}

void BodyJoint::init_rope(Vec3 point_a, Vec3 point_b, float slack) {
    type = kRope;
    anchor_a = qi_rotate_inverse(a->rotation, point_a - a->position);
    anchor_b = qi_rotate_inverse(b->rotation, point_b - b->position);
    active = false;  // roped bodies still collide
    const Vec3 d = point_a - point_b;
    rope_length = slack + std::sqrt((d.y * d.y + d.x * d.x) + d.z * d.z);
}

void insert_joints(SolverCalls& solver, const std::vector<BodyJoint*>& joints, float dt) {
    for (const BodyJoint* j : joints) {
        const RigidBody &A = *j->a, &B = *j->b;
        Body* sa = const_cast<Body*>(&A.solver);
        Body* sb = const_cast<Body*>(&B.solver);
        if (j->type == BodyJoint::kBall || j->type == BodyJoint::kHinge || j->type == BodyJoint::kSlider) {
            // Three point rows along the world axes; a slider drops its own axis and uses two
            // directions perpendicular to it instead.
            Vec3 axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
            if (j->type == BodyJoint::kSlider) {
                axes[0] = qi_rotate(A.rotation, j->axis_a);
                axes[1] = perpendicular(axes[0]);
                axes[2] = cross(axes[0], axes[1]);
            }
            const Vec3 pa = qi_transform(A.position, A.rotation, j->anchor_a);
            const Vec3 pb = qi_transform(B.position, B.rotation, j->anchor_b);
            const Vec3 d = pa - pb;
            for (int k = j->type == BodyJoint::kSlider ? 1 : 0; k < 3; ++k) {
                const Vec3 n = axes[k];
                const float err = ((d.y * n.y + d.x * n.x) + d.z * n.z) * 0.5f;
                float bias = -0.1f;
                if (-0.1f < err) bias = 0.1f <= err ? 0.1f : err;
                LinearJoint row;
                row.a = sa, row.b = sb;
                row.anchor_a = to_local(A.solver, pa);
                row.anchor_b = to_local(B.solver, pb);
                row.dir_a = qi_rotate_inverse(A.solver.rotation, n);
                row.dir_b = qi_rotate_inverse(B.solver.rotation, n);
                row.bias = bias / dt;
                solver.insert_linear(row);
            }
            if (j->type == BodyJoint::kHinge) {
                // Two angular rows keep the axes aligned; their bias is the misalignment wa x wb.
                const Vec3 wa = qi_rotate(A.rotation, j->axis_a);
                const Vec3 wb = qi_rotate(B.rotation, j->axis_b);
                const Vec3 c = cross(wa, wb);
                const Vec3 ta = perpendicular(j->axis_a), ua = cross(j->axis_a, ta);
                const Vec3 tb = perpendicular(j->axis_b), ub = cross(j->axis_b, tb);
                const Vec3 dirs[2] = {unit_sum(qi_rotate(A.rotation, ta), qi_rotate(B.rotation, tb)),
                                      unit_sum(qi_rotate(A.rotation, ua), qi_rotate(B.rotation, ub))};
                for (const Vec3& n : dirs) {
                    AngularJoint row;
                    row.a = sa, row.b = sb;
                    row.axis_a = qi_rotate_inverse(A.solver.rotation, n);
                    row.axis_b = qi_rotate_inverse(B.solver.rotation, n);
                    row.bias = -(((c.y * n.y + c.x * n.x) + c.z * n.z) * 0.5f) / dt;
                    solver.insert_angular(row);
                }
                if (j->motor_torque != 0) {
                    // The motor row passes the entity-frame axes straight to the solver.
                    AngularJoint row;
                    row.a = sa, row.b = sb;
                    row.axis_a = j->axis_a;
                    row.axis_b = j->axis_b;
                    row.bias = j->motor_speed;
                    solver.push_state();
                    solver.set_param(0, j->motor_torque);
                    solver.insert_angular(row);
                    solver.pop_state();
                }
            }
        }
        if (j->type == BodyJoint::kRope) {
            // One row along the rope when it is stretched, with its impulse capped by how far.
            float m = 10000.0f;
            if (0 < A.solver.inv_mass) {
                m = 1.0f / A.solver.inv_mass;
                if (m > 10000.0f) m = 10000.0f;
            }
            if (0 < B.solver.inv_mass) {
                const float mb = 1.0f / B.solver.inv_mass;
                if (!(m < mb)) m = mb;
            }
            const Vec3 pa = qi_transform(A.position, A.rotation, j->anchor_a);
            const Vec3 pb = qi_transform(B.position, B.rotation, j->anchor_b);
            const Vec3 d = pb - pa;
            const float len = std::sqrt((d.y * d.y + d.x * d.x) + d.z * d.z);
            if (!(j->rope_length < len)) continue;
            const Vec3 n{d.x / len, d.y / len, d.z / len};
            LinearJoint row;
            row.a = sa, row.b = sb;
            row.anchor_a = to_local(A.solver, pa);
            row.anchor_b = to_local(B.solver, pb);
            row.dir_a = qi_rotate_inverse(A.solver.rotation, n);
            row.dir_b = qi_rotate_inverse(B.solver.rotation, n);
            const float stretch = len - j->rope_length;
            row.bias = -stretch;
            solver.push_state();
            solver.set_param(0, stretch * m);
            solver.insert_linear(row);
            solver.pop_state();
        }
    }
}

void SolverAdapter::push_state() { s_.push_state(); }
void SolverAdapter::pop_state() { s_.pop_state(); }
void SolverAdapter::set_param(int index, float value) { s_.set_param(index, value); }
void SolverAdapter::insert_linear(const LinearJoint& row) { phys::insert_linear(s_, row); }
void SolverAdapter::insert_angular(const AngularJoint& row) { phys::insert_angular(s_, row); }

void insert_joints(Solver& solver, const std::vector<BodyJoint*>& joints, float dt) {
    SolverAdapter s(solver);
    insert_joints(s, joints, dt);
}

}  // namespace sh::phys
