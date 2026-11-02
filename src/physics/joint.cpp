#include "physics/joint.hpp"

#include <vector>

namespace sh::phys {

namespace {

constexpr int kParamUpper = 0, kParamLower = 1;

// Joint rows clamp their accumulated impulse to [p1, p0], checking the lower bound first.
float clamp_accumulate(float& total, float delta, const SolverState& st) {
    const float next = total + delta;
    const float lo = st.params[kParamLower], hi = st.params[kParamUpper];
    if (next < lo) {
        delta = lo - total;
        total = lo;
    } else if (next > hi) {
        delta = hi - total;
        total = hi;
    } else {
        total = next;
    }
    return delta;
}

class LinearType final : public ConstraintType {
    struct Data {
        int a = 0, b = 0;
        Vec3 ra, rb, da, db;  // db = -dir_b
        Vec3 ang_a, ang_b;
        float bias = 0, impulse = 0, k = 0;
    };
    std::vector<LinearJoint> inputs_;
    std::vector<Data> data_;

public:
    std::vector<LinearJoint>& inputs() { return inputs_; }

    void clear() override {
        inputs_.clear();
        data_.clear();
    }
    int add() override {
        inputs_.emplace_back();
        data_.emplace_back();
        return static_cast<int>(inputs_.size()) - 1;
    }
    void init(Solver& s, const SolverState&, int first, int end) override {
        for (int i = first; i < end; ++i) {
            const LinearJoint& in = inputs_[i];
            Data& d = data_[i];
            d.a = (in.a ? *in.a : s.world_body()).solver_index;
            d.b = (in.b ? *in.b : s.world_body()).solver_index;
            const SolverBody& sa = s.solver_body(d.a);
            const SolverBody& sb = s.solver_body(d.b);
            d.ra = in.anchor_a;
            d.rb = in.anchor_b;
            d.da = in.dir_a;
            d.db = -in.dir_b;
            d.bias = in.bias;
            d.impulse = 0;
            d.ang_a = mul(sa.inv_inertia, cross(d.ra, d.da));
            d.ang_b = mul(sb.inv_inertia, cross(d.rb, d.db));
            d.k = (dot(d.da, cross(d.ang_a, d.ra)) + sa.inv_mass) + (dot(d.db, cross(d.ang_b, d.rb)) + sb.inv_mass);
        }
    }
    void solve(Solver& s, const SolverState& st, int first, int end) override {
        for (int i = first; i < end; ++i) {
            Data& d = data_[i];
            SolverBody& sa = s.solver_body(d.a);
            SolverBody& sb = s.solver_body(d.b);
            const float vrel = dot(sa.linear + cross(sa.angular, d.ra), d.da) + dot(sb.linear + cross(sb.angular, d.rb), d.db);
            const float dl = clamp_accumulate(d.impulse, (-vrel - d.bias) / d.k, st);
            sa.linear += (dl * sa.inv_mass) * d.da;
            sa.angular += dl * d.ang_a;
            sb.linear += (dl * sb.inv_mass) * d.db;
            sb.angular += dl * d.ang_b;
        }
    }
};

class AngularType final : public ConstraintType {
    struct Data {
        int a = 0, b = 0;
        Vec3 axa, axb;  // axb = -axis_b
        float bias = 0, impulse = 0, k = 0;
    };
    std::vector<AngularJoint> inputs_;
    std::vector<Data> data_;

    static float axis_mass(Vec3 ax, const SolverBody& sb) {
        return (ax.y * (ax.y * sb.inv_inertia.y) + ax.x * (ax.x * sb.inv_inertia.x)) + ax.z * (ax.z * sb.inv_inertia.z);
    }

public:
    std::vector<AngularJoint>& inputs() { return inputs_; }

    void clear() override {
        inputs_.clear();
        data_.clear();
    }
    int add() override {
        inputs_.emplace_back();
        data_.emplace_back();
        return static_cast<int>(inputs_.size()) - 1;
    }
    void init(Solver& s, const SolverState&, int first, int end) override {
        for (int i = first; i < end; ++i) {
            const AngularJoint& in = inputs_[i];
            Data& d = data_[i];
            d.a = (in.a ? *in.a : s.world_body()).solver_index;
            d.b = (in.b ? *in.b : s.world_body()).solver_index;
            d.axa = in.axis_a;
            d.axb = -in.axis_b;
            d.bias = in.bias;
            d.impulse = 0;
            d.k = axis_mass(d.axa, s.solver_body(d.a)) + axis_mass(d.axb, s.solver_body(d.b));
        }
    }
    void solve(Solver& s, const SolverState& st, int first, int end) override {
        for (int i = first; i < end; ++i) {
            Data& d = data_[i];
            SolverBody& sa = s.solver_body(d.a);
            SolverBody& sb = s.solver_body(d.b);
            const float vrel = dot(d.axa, sa.angular) + dot(d.axb, sb.angular);
            const float dl = clamp_accumulate(d.impulse, (-vrel - d.bias) / d.k, st);
            sa.angular += mul(dl * d.axa, sa.inv_inertia);
            sb.angular += mul(dl * d.axb, sb.inv_inertia);
        }
    }
};

}  // namespace

std::unique_ptr<ConstraintType> make_linear_type() { return std::make_unique<LinearType>(); }
std::unique_ptr<ConstraintType> make_angular_type() { return std::make_unique<AngularType>(); }

void insert_linear(Solver& solver, const LinearJoint& joint) {
    auto* type = static_cast<LinearType*>(solver.constraint_type(Solver::kLinear));
    const int index = solver.add_constraint(Solver::kLinear);
    type->inputs()[index] = joint;
}

void insert_angular(Solver& solver, const AngularJoint& joint) {
    auto* type = static_cast<AngularType*>(solver.constraint_type(Solver::kAngular));
    const int index = solver.add_constraint(Solver::kAngular);
    type->inputs()[index] = joint;
}

}  // namespace sh::phys
