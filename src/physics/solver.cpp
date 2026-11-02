#include "physics/solver.hpp"

#include "physics/collision.hpp"

#include "physics/contact.hpp"
#include "physics/joint.hpp"

#include <algorithm>
#include <cmath>

namespace sh::phys {

namespace {
constexpr float kAngularLimit = 3.0f;         // rad/s around the inserted angular velocity
constexpr float kRotateThresholdSq = 0.001f;  // below this |w|^2 orientation is left alone
constexpr int kMaxBatch = 16;
}  // namespace

Solver::Solver() {
    world_.flags = Body::kStatic;
    initial_ = state_;
    types_[kContact] = make_contact_type();
    types_[kContactStatic] = make_static_contact_type();
    types_[kLinear] = make_linear_type();
    types_[kAngular] = make_angular_type();
}

void Solver::set_constraint_type(int slot, std::unique_ptr<ConstraintType> type) { types_[slot] = std::move(type); }

// State changes snapshot the initial state when they are the first command; constraint
// batches do not (matches the original).
void Solver::record_state_change(const Command& c) {
    if (commands_.empty()) initial_ = state_;
    commands_.push_back(c);
}

void Solver::set_param(int index, float value) {
    if (state_.params[index] == value) return;
    const Command c{Command::kParam, index, 0, 0, value};
    record_state_change(c);
    state_.params[index] = value;
}

void Solver::set_enabled(int index, bool value) {
    // Only index 0 is part of the state block in the original.
    if (index != 0 || state_.enabled == value) return;
    record_state_change({Command::kEnabled, index, 0, 0, value ? 1.0f : 0.0f});
    state_.enabled = value;
}

void Solver::push_state() { state_stack_.push_back(state_); }

void Solver::pop_state() {
    if (state_stack_.empty()) return;
    const SolverState saved = state_stack_.back();
    for (int i = 0; i < SolverState::kParamCount; ++i) set_param(i, saved.params[i]);
    set_enabled(0, saved.enabled);
    state_ = saved;
    state_stack_.pop_back();
}

void Solver::insert_body(Body& body) {
    const Mat3 rot = Mat3::from_quat(body.rotation);
    SolverBody sb;
    sb.linear = rot.transposed_mul(body.linear_velocity);
    sb.angular = rot.transposed_mul(body.angular_velocity);
    sb.gravity = rot.transposed_mul(state_.linear_acc);
    sb.inv_inertia = body.inv_inertia;
    sb.inv_mass = body.inv_mass;

    SolverBodyLimits lim;
    for (int k = 0; k < 3; ++k) {
        lim.max_angular[k] = std::max(sb.angular[k] + kAngularLimit, kAngularLimit);
        lim.min_angular[k] = std::min(sb.angular[k] - kAngularLimit, -kAngularLimit);
    }
    lim.angular_acc = state_.angular_acc;

    body.solver_index = static_cast<int>(bodies_.size());
    bodies_.push_back(&body);
    solver_bodies_.push_back(sb);
    limits_.push_back(lim);
}

void Solver::insert_particle(Particle& particle) {
    particle.solver_index = static_cast<int>(particles_.size());
    particles_.push_back(&particle);
    particle_velocity_.push_back(particle.velocity);
}

int Solver::add_constraint(int slot) {
    const int index = types_[slot]->add();
    Command* last = commands_.empty() ? nullptr : &commands_.back();
    if (last && last->kind == Command::kBatch && last->index == slot && last->count < kMaxBatch) {
        ++last->count;
    } else {
        commands_.push_back({Command::kBatch, slot, index, 1, 0});
    }
    ++constraint_count_;
    return index;
}

void Solver::integrate_velocity(float dt) {
    dt_ = dt;
    for (size_t i = 0; i < bodies_.size(); ++i) {
        Body& b = *bodies_[i];
        if (!b.is_static()) {
            SolverBody& sb = solver_bodies_[i];
            const Mat3 rot = Mat3::from_quat(b.rotation);
            sb.angular += rot.transposed_mul(dt * limits_[i].angular_acc);
            sb.linear += dt * sb.gravity;
        }
        b.contact_impulse = 0;
    }
}

void Solver::replay(Pass pass) {
    for (const Command& c : commands_) {
        switch (c.kind) {
            case Command::kParam: state_.params[c.index] = c.value; break;
            case Command::kEnabled: state_.enabled = c.value != 0; break;
            case Command::kBatch:
                if (ConstraintType* t = types_[c.index].get()) {
                    if (pass == Pass::kInit)
                        t->init(*this, state_, c.first, c.first + c.count);
                    else
                        t->solve(*this, state_, c.first, c.first + c.count);
                }
                break;
        }
    }
}

void Solver::solve(float dt) {
    dt_ = dt;
    if (bodies_.empty()) return;
    state_ = initial_;
    replay(Pass::kInit);
    for (current_iteration_ = 0; current_iteration_ < iterations_; ++current_iteration_) {
        state_ = initial_;
        replay(Pass::kSolve);
        for (size_t i = 0; i < bodies_.size(); ++i) {
            Vec3& w = solver_bodies_[i].angular;
            for (int k = 0; k < 3; ++k) w[k] = std::min(std::max(w[k], limits_[i].min_angular[k]), limits_[i].max_angular[k]);
        }
    }
    for (auto& t : types_)
        if (t) t->post_solve(*this);
    for (size_t i = 0; i < bodies_.size(); ++i) {
        Body& b = *bodies_[i];
        const Mat3 rot = Mat3::from_quat(b.rotation);
        b.linear_velocity = rot * solver_bodies_[i].linear;
        b.angular_velocity = rot * solver_bodies_[i].angular;
    }
}

void Solver::solve_constraints(float dt) {
    push_state();
    solve(dt);
    pop_state();
}

void Solver::integrate_position(float dt) {
    dt_ = dt;
    for (size_t i = 0; i < bodies_.size(); ++i) {
        Body& b = *bodies_[i];
        const SolverBody& sb = solver_bodies_[i];
        // Rotation matrix as tdTransformInitPQ builds it; sums in the original's y, x, z order.
        const Transform T = Transform::from(b.position, b.rotation);
        const auto R = [&T](int row, int col) { return T.m[col * 4 + row]; };
        const auto to_world = [&R](Vec3 v, int row) { return (v.y * R(row, 1) + v.x * R(row, 0)) + v.z * R(row, 2); };
        const Vec3 v = sb.linear;
        b.position = {b.position.x + dt * to_world(v, 0), b.position.y + dt * to_world(v, 1),
                      b.position.z + dt * to_world(v, 2)};
        const Vec3 wl = sb.angular;
        if ((wl.y * wl.y + wl.x * wl.x) + wl.z * wl.z > kRotateThresholdSq) {
            const Vec3 w{to_world(wl, 0), to_world(wl, 1), to_world(wl, 2)};
            const float len = std::sqrt((w.y * w.y + w.x * w.x) + w.z * w.z);
            const float half = (len * dt) * 0.5f;
            Vec3 axis{1, 0, 0};
            if (len > 0) axis = {w.x / len, w.y / len, w.z / len};
            const float s = std::sin(half), c = std::cos(half);
            const Quat d{s * axis.x, s * axis.y, s * axis.z, c};
            const Quat q = b.rotation;
            const Quat r{((d.y * q.z - d.z * q.y) + d.w * q.x) + d.x * q.w, ((d.z * q.x - d.x * q.z) + d.w * q.y) + d.y * q.w,
                         ((d.x * q.y - d.y * q.x) + d.w * q.z) + d.z * q.w, d.w * q.w - ((d.y * q.y + d.x * q.x) + d.z * q.z)};
            const float n2 = ((r.y * r.y + r.x * r.x) + r.z * r.z) + r.w * r.w;
            if (n2 > 0) {
                const float inv = 1.0f / std::sqrt(n2);
                b.rotation = {inv * r.x, inv * r.y, inv * r.z, inv * r.w};
            } else {
                b.rotation = {0, 0, 0, 1};
            }
        }
    }
    // Particles: nothing in the solver changes their velocity copy (unused by the game).
    // Like the original, they only move when at least one body (e.g. the world body) is present.
    for (size_t i = 0; i < particles_.size() && !bodies_.empty(); ++i) {
        Particle& p = *particles_[i];
        const Vec3 v = particle_velocity_[i];
        p.velocity = (v + p.velocity) * 0.5f;
        p.position += dt * v;
    }
}

void Solver::reset() {
    bodies_.clear();
    solver_bodies_.clear();
    limits_.clear();
    particles_.clear();
    particle_velocity_.clear();
    commands_.clear();
    constraint_count_ = 0;
    for (auto& t : types_)
        if (t) t->clear();
    push_state();
    state_.linear_acc = {};
    insert_body(world_);
    pop_state();
    initial_ = state_;
}

void Solver::step(float dt) {
    integrate_velocity(dt);
    solve_constraints(dt);
    integrate_position(dt);
    reset();
}

}  // namespace sh::phys
