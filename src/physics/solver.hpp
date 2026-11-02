#pragma once

// Sequential solver framework.
// Velocities are solved in each body's local frame; constraint types plug into 4 slots.

#include "physics/body.hpp"
#include "physics/pmath.hpp"

#include <array>
#include <memory>
#include <vector>

namespace sh::phys {

struct SolverState {
    static constexpr int kParamCount = 11;

    Vec3 linear_acc{0, -10, 0};  // gravity for bodies inserted while set
    Vec3 angular_acc;
    std::array<float, kParamCount> params{3.402823466e+38f, -3.402823466e+38f, 1, 0, 0.02f, 1, 1, 1, 1, 0, 1};
    bool enabled = false;
};

struct Particle {
    float tag = 0;  // first word of the original record; carried, not used by the solver
    Vec3 position;
    Vec3 velocity;
    int solver_index = -1;
};

// Per-body solver data, all in the body's local frame.
struct SolverBody {
    Vec3 linear;   // velocity
    Vec3 angular;  // velocity
    Vec3 gravity;
    Vec3 inv_inertia;
    float inv_mass = 0;
};

struct SolverBodyLimits {
    Vec3 max_angular, min_angular;  // local angular velocity clamp, applied after every iteration
    Vec3 angular_acc;               // world frame, captured at insertion
};

class Solver;

class ConstraintType {
public:
    virtual ~ConstraintType() = default;
    virtual void clear() = 0;
    virtual int add() = 0;  // allocates one constraint, returns its index within this type
    virtual void init(Solver& solver, const SolverState& state, int first, int end) = 0;
    virtual void solve(Solver& solver, const SolverState& state, int first, int end) = 0;
    virtual void post_solve(Solver&) {}
};

class Solver {
public:
    enum Slot { kContact = 0, kContactStatic = 1, kLinear = 2, kAngular = 3, kSlotCount = 4 };

    Solver();

    void set_constraint_type(int slot, std::unique_ptr<ConstraintType> type);
    ConstraintType* constraint_type(int slot) const { return types_[slot].get(); }

    // State. Parameter/enable changes are recorded so later constraints see them.
    void set_param(int index, float value);
    float param(int index) const { return state_.params[index]; }
    void set_enabled(int index, bool value);
    void set_linear_acc(Vec3 a) { state_.linear_acc = a; }
    Vec3 linear_acc() const { return state_.linear_acc; }
    void set_angular_acc(Vec3 a) { state_.angular_acc = a; }
    Vec3 angular_acc() const { return state_.angular_acc; }
    void set_iteration_count(int n) { iterations_ = n; }
    int iteration_count() const { return iterations_; }
    void push_state();
    void pop_state();

    void insert_body(Body& body);
    // Solver particles: present in the original's API but never used by the game.
    void insert_particle(Particle& particle);
    // Adds a constraint to a slot and records it in the command list; returns its index in the slot.
    int add_constraint(int slot);

    void integrate_velocity(float dt);
    void solve_constraints(float dt);
    void integrate_position(float dt);
    void reset();
    void step(float dt);

    // For constraint types.
    int body_count() const { return static_cast<int>(bodies_.size()); }
    Body& body(int i) { return *bodies_[i]; }
    SolverBody& solver_body(int i) { return solver_bodies_[i]; }
    Body& world_body() { return world_; }
    float dt() const { return dt_; }
    int current_iteration() const { return current_iteration_; }
    int constraint_count() const { return constraint_count_; }

private:
    struct Command {
        enum Kind { kBatch, kParam, kEnabled } kind;
        int index;    // slot (batch) or parameter index
        int first;    // batch: first constraint index in the slot
        int count;    // batch: number of constraints
        float value;  // param/enabled value
    };
    enum class Pass { kInit, kSolve };

    void record_state_change(const Command& c);
    void replay(Pass pass);
    void solve(float dt);

    SolverState state_, initial_;
    std::vector<SolverState> state_stack_;
    std::vector<Command> commands_;
    std::array<std::unique_ptr<ConstraintType>, kSlotCount> types_;
    std::vector<Body*> bodies_;
    std::vector<SolverBody> solver_bodies_;
    std::vector<SolverBodyLimits> limits_;
    std::vector<Particle*> particles_;
    std::vector<Vec3> particle_velocity_;  // copy taken at insertion
    Body world_;
    float dt_ = 0.02f;
    int iterations_ = 8;
    int current_iteration_ = 0;
    int constraint_count_ = 0;
};

}  // namespace sh::phys
