// Tests for the solver framework and body helpers 
// and closed-form results.

#include "physics/body.hpp"
#include "physics/solver.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <numbers>
#include <utility>
#include <vector>

using namespace sh::phys;

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                \
    do {                                                                           \
        if (!(cond)) {                                                             \
            std::fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #cond); \
            ++g_failures;                                                          \
        }                                                                          \
    } while (0)

bool near(float a, float b, float eps = 1e-5f) { return std::fabs(a - b) <= eps; }
bool near(Vec3 a, Vec3 b, float eps = 1e-5f) { return near(a.x, b.x, eps) && near(a.y, b.y, eps) && near(a.z, b.z, eps); }

Body dynamic_box(Vec3 pos) {
    Body b;
    b.position = pos;
    b.set_mass_as_box(1.0f, {0.5f, 0.5f, 0.5f});
    return b;
}

// Runs one frame the way the game does: bodies are re-inserted every step.
void frame(Solver& s, std::vector<Body*> bodies, float dt) {
    for (Body* b : bodies) s.insert_body(*b);
    s.step(dt);
}

void test_mass_helpers() {
    Body b;
    b.set_mass_as_box(2.0f, {0.5f, 1.0f, 1.5f});  // full extents 1, 2, 3
    CHECK(near(b.inv_mass, 0.5f));
    CHECK(near(b.inv_inertia, {12.0f / 26.0f, 0.6f, 1.2f}));

    b.set_mass_as_sphere(3.0f, 1.0f);  // cube formula with half extent 1
    CHECK(near(b.inv_inertia, {0.5f, 0.5f, 0.5f}));

    b.set_mass_as_box(0.0f, {1, 1, 1});
    CHECK(b.inv_mass == 0 && near(b.inv_inertia, {}));

    b.set_mass_as_infinite_inertia(4.0f);
    CHECK(near(b.inv_mass, 0.25f) && near(b.inv_inertia, {}));
    b.set_mass_as_infinite_inertia(-1.0f);
    CHECK(b.inv_mass == 0);
}

void test_apply_impulse() {
    Body b;
    b.inv_mass = 0.5f;
    b.inv_inertia = {1, 2, 3};
    b.rotation = axis_angle({0, 1, 0}, std::numbers::pi_v<float> / 2);  // local x -> world -z
    b.apply_impulse({1, 0, 0}, {0, 1, 0});
    CHECK(near(b.linear_velocity, {0, 0.5f, 0}));
    // World torque (0,0,1) is about local x, whose inverse inertia is 1.
    CHECK(near(b.angular_velocity, {0, 0, 1}, 1e-5f));
}

void test_free_fall() {
    Solver s;
    s.reset();
    Body b = dynamic_box({0, 10, 0});
    const float dt = 0.02f;
    for (int n = 0; n < 50; ++n) frame(s, {&b}, dt);
    // Semi-implicit Euler with g = 10: v_n = -10 n dt, y_n = y0 - 10 dt^2 n(n+1)/2.
    CHECK(near(b.linear_velocity, {0, -10.0f, 0}, 1e-4f));
    CHECK(near(b.position.y, 10.0f - 10.0f * dt * dt * 50 * 51 / 2, 1e-3f));
}

void test_static_body_moves_but_is_not_accelerated() {
    Solver s;
    s.reset();
    Body b;
    b.flags = Body::kStatic;
    b.linear_velocity = {1, 0, 0};
    frame(s, {&b}, 0.1f);
    CHECK(near(b.position, {0.1f, 0, 0}));
    CHECK(near(b.linear_velocity, {1, 0, 0}));
}

void test_per_body_gravity_and_rotated_body() {
    Solver s;
    s.reset();
    Body a = dynamic_box({}), b = dynamic_box({});
    a.rotation = axis_angle({1, 0, 0}, 1.0f);  // gravity is world-down regardless of orientation
    s.insert_body(a);
    s.set_linear_acc({0, 0, 0});
    s.insert_body(b);
    s.set_linear_acc({0, -10, 0});
    s.step(0.1f);
    CHECK(near(a.linear_velocity, {0, -1.0f, 0}));
    CHECK(near(b.linear_velocity, {}));
}

void test_rotation_integration() {
    Solver s;
    s.reset();
    Body b = dynamic_box({});
    b.angular_velocity = {0, 0, std::numbers::pi_v<float>};
    s.set_linear_acc({});
    for (int n = 0; n < 50; ++n) frame(s, {&b}, 0.02f);  // half a turn about z
    const Vec3 x_axis = Mat3::from_quat(b.rotation) * Vec3{1, 0, 0};
    CHECK(near(x_axis, {-1, 0, 0}, 1e-4f));

    Body slow = dynamic_box({});
    slow.angular_velocity = {0.01f, 0, 0};  // |w|^2 = 1e-4 < 0.001: orientation is not updated
    frame(s, {&slow}, 0.02f);
    CHECK(slow.rotation.w == 1.0f && slow.rotation.x == 0.0f);
}

// Records what a constraint type sees.
struct RecordingType : ConstraintType {
    struct Call {
        bool init;
        int first, end;
        float param3;
        int iteration;
    };
    std::vector<Call> calls;
    int count = 0, post_solves = 0, clears = 0;
    Vec3 force_angular;  // if nonzero, written into body 1's angular velocity during solve
    bool force = false;

    void clear() override {
        count = 0;
        ++clears;
    }
    int add() override { return count++; }
    void init(Solver&, const SolverState& st, int first, int end) override {
        calls.push_back({true, first, end, st.params[3], -1});
    }
    void solve(Solver& s, const SolverState& st, int first, int end) override {
        calls.push_back({false, first, end, st.params[3], s.current_iteration()});
        if (force) s.solver_body(1).angular = force_angular;
    }
    void post_solve(Solver&) override { ++post_solves; }
};

void test_command_ordering() {
    Solver s;
    auto rec = std::make_unique<RecordingType>();
    RecordingType* r = rec.get();
    s.set_constraint_type(Solver::kLinear, std::move(rec));
    s.reset();
    Body b = dynamic_box({});
    s.insert_body(b);

    CHECK(s.add_constraint(Solver::kLinear) == 0);
    s.set_param(3, 0.5f);
    CHECK(s.add_constraint(Solver::kLinear) == 1);
    s.set_iteration_count(3);
    s.solve_constraints(0.02f);

    int inits = 0, solves = 0;
    for (const auto& c : r->calls) {
        if (c.init) ++inits; else ++solves;
        // Constraint 0 was added before the param change, constraint 1 after.
        CHECK(c.param3 == (c.first == 0 ? 0.0f : 0.5f));
        CHECK(c.end == c.first + 1);
    }
    CHECK(inits == 2 && solves == 6);
    CHECK(r->calls.back().iteration == 2);
    CHECK(r->post_solves == 1);
    CHECK(s.param(3) == 0.5f);  // live state survives the push/pop around solve

    s.reset();
    CHECK(r->clears == 2);
    r->calls.clear();
    s.insert_body(b);
    for (int i = 0; i < 20; ++i) s.add_constraint(Solver::kLinear);
    s.set_iteration_count(1);
    s.solve_constraints(0.02f);
    // Same-type constraints share batches of up to 16.
    CHECK(r->calls.size() == 4);
    CHECK(r->calls[0].first == 0 && r->calls[0].end == 16 && r->calls[1].first == 16 && r->calls[1].end == 20);
}

void test_angular_clamp() {
    Solver s;
    auto rec = std::make_unique<RecordingType>();
    RecordingType* r = rec.get();
    s.set_constraint_type(Solver::kAngular, std::move(rec));
    s.reset();  // world body is index 0
    Body b = dynamic_box({});
    b.angular_velocity = {10, 0, 0};
    s.set_linear_acc({});
    s.insert_body(b);
    s.add_constraint(Solver::kAngular);
    r->force = true;
    r->force_angular = {100, -100, 0.5f};
    s.solve_constraints(0.02f);
    // Limits: x in [min(10-3,-3), max(10+3,3)] = [-3, 13]; y, z in [-3, 3].
    CHECK(near(b.angular_velocity, {13, -3, 0.5f}));
}

void test_push_pop() {
    Solver s;
    s.set_param(1, 5.0f);
    s.push_state();
    s.set_param(1, 7.0f);
    s.set_linear_acc({1, 2, 3});
    s.pop_state();
    CHECK(s.param(1) == 5.0f);
    CHECK(near(s.linear_acc(), {0, -10, 0}));
}

void test_defaults() {
    Solver s;
    CHECK(s.iteration_count() == 8);
    CHECK(near(s.linear_acc(), {0, -10, 0}));
    CHECK(s.param(2) == 1.0f && s.param(3) == 0.0f && near(s.param(4), 0.02f) && s.param(9) == 0.0f);
    s.reset();
    CHECK(s.body_count() == 1 && s.body(0).is_static());
}

}  // namespace

int main() {
    test_mass_helpers();
    test_apply_impulse();
    test_free_fall();
    test_static_body_moves_but_is_not_accelerated();
    test_per_body_gravity_and_rotated_body();
    test_rotation_integration();
    test_command_ordering();
    test_angular_clamp();
    test_push_pop();
    test_defaults();
    if (g_failures) {
        std::fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("all physics solver tests passed\n");
    return 0;
}
