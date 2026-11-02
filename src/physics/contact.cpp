#include "physics/contact.hpp"

#include "physics/collision.hpp"
#include "physics/qi_rotate.hpp"

#include <cmath>
#include <utility>

namespace sh::phys {

namespace {

constexpr float kBaumgarte = 0.8f;         // penetration and anchor correction factor
constexpr float kMaxCorrection = -0.02f;   // separation is capped below at this
constexpr float kMaxCorrectionSpeed = -1;  // bias never pushes apart faster than 1 m/s
constexpr float kBounceSpeed = 0.02f;      // per dt: approach speed above which a separated contact may bounce
constexpr float kSlideDirectionSq = 0.1f;  // static contact: |tangential velocity|^2 above which t follows it
constexpr float kAnchorAlignment = -0.99f; // stored normals must stay this anti-aligned to keep the anchor
constexpr int kParamUpper = 0, kParamLower = 1, kParamFriction = 2, kParamRestitution = 3, kParamSlop = 4;

// Fixed tangent for a unit normal (used when there is no sliding direction); (1, 0, 0) if degenerate.
Vec3 perpendicular(Vec3 n) {
    if (std::fabs(n.z) > 0.7f) {
        const float s = -n.z;
        const float len2 = (s * s + 0.0f) + n.y * n.y;
        if (!(len2 > 0)) return {1, 0, 0};
        const float inv = 1.0f / std::sqrt(len2);
        return {inv * 0.0f, s * inv, n.y * inv};
    }
    const float s = -n.y;
    const float len2 = (n.x * n.x + s * s) + 0.0f;
    if (!(len2 > 0)) return {1, 0, 0};
    const float inv = 1.0f / std::sqrt(len2);
    return {s * inv, n.x * inv, inv * 0.0f};
}

// B's pose in A's frame exactly as the original builds it: TdSolver::getBodyTransformDiff (inverse rotation and
// conjugate product in its own grouping) feeding tdTransformInitPQ's matrix. (Generic quaternion maths rounds
// differently; this was the main source of contact drift.)
struct Frame {
    Transform t;
    float R(int row, int col) const { return t.m[col * 4 + row]; }
    Vec3 rotate(Vec3 v) const {
        return {(R(0, 1) * v.y + R(0, 0) * v.x) + R(0, 2) * v.z, (R(1, 1) * v.y + R(1, 0) * v.x) + R(1, 2) * v.z,
                (R(2, 1) * v.y + R(2, 0) * v.x) + R(2, 2) * v.z};
    }
    Vec3 apply(Vec3 v) const {
        const Vec3 r = rotate(v);
        return {r.x + t.m[12], r.y + t.m[13], r.z + t.m[14]};
    }
    Vec3 inverse_rotate(Vec3 v) const {
        return {(R(1, 0) * v.y + R(0, 0) * v.x) + R(2, 0) * v.z, (R(1, 1) * v.y + R(0, 1) * v.x) + R(2, 1) * v.z,
                (R(1, 2) * v.y + R(0, 2) * v.x) + R(2, 2) * v.z};
    }
    Vec3 inverse_apply(Vec3 v) const { return inverse_rotate({v.x - t.m[12], v.y - t.m[13], v.z - t.m[14]}); }
};

Frame frame_of(const Body& a, const Body& b) {
    const Vec3 d{b.position.x - a.position.x, b.position.y - a.position.y, b.position.z - a.position.z};
    return {Transform::from(qi_rotate_inverse(a.rotation, d), qi_relative_rotation(a.rotation, b.rotation))};
}

// Effective-mass contribution of one body for direction d applied at r (body-local).
struct Row {
    Vec3 angular;  // invI * (r x d): angular velocity change per unit impulse
    float k;       // invMass + d . (angular x r)
};
Row make_row(const SolverBody& sb, Vec3 r, Vec3 d) {
    const Vec3 a = mul(sb.inv_inertia, cross(r, d));
    return {a, sb.inv_mass + dot(d, cross(a, r))};
}

float twist_mass(const SolverBody& sb, Vec3 n) {
    return (n.y * (n.y * sb.inv_inertia.y) + n.x * (n.x * sb.inv_inertia.x)) + n.z * (n.z * sb.inv_inertia.z);
}

Vec3 point_velocity(const SolverBody& sb, Vec3 r) { return sb.linear + cross(sb.angular, r); }

// The centroid is accumulated as p / count (a division, not a reciprocal multiply), like the original.
Vec3 divided(Vec3 v, float d) { return {v.x / d, v.y / d, v.z / d}; }

void apply(SolverBody& sb, float impulse, Vec3 dir, Vec3 angular) {
    sb.linear += (impulse * sb.inv_mass) * dir;
    sb.angular += impulse * angular;
}

// Twist impulse about n: w += (n dl) invI per component (the original's grouping).
void apply_twist(SolverBody& sb, float dl, Vec3 n) {
    sb.angular = {sb.angular.x + (n.x * dl) * sb.inv_inertia.x, sb.angular.y + (n.y * dl) * sb.inv_inertia.y,
                  sb.angular.z + (n.z * dl) * sb.inv_inertia.z};
}

// Accumulates into total, clamps, and returns the impulse actually applied this time.
float clamp_accumulate(float& total, float delta, float lo, float hi, bool* clamped = nullptr) {
    const float next = total + delta;
    if (next < lo) {
        delta = lo - total;
        total = lo;
        if (clamped) *clamped = true;
    } else if (next > hi) {
        delta = hi - total;
        total = hi;
        if (clamped) *clamped = true;
    } else {
        total = next;
    }
    return delta;
}

// Normal impulse clamp: [0, p0], or [p1, p0] while the solver's enabled flag is set.
float normal_lower(const SolverState& st) { return st.enabled ? st.params[kParamLower] : 0.0f; }

// Separation bias for one point (shared by both contact types).
float point_bias(const SolverState& st, float dt, float separation, float normal_velocity) {
    const float restitution = st.params[kParamRestitution];
    if (separation <= kMaxCorrection) separation = kMaxCorrection;
    if (restitution > 0 && -normal_velocity > kBounceSpeed / dt && !(separation < 0)) separation = 0;
    float bias = separation * (kBaumgarte / dt);
    if (!(bias > kMaxCorrectionSpeed)) bias = kMaxCorrectionSpeed;
    return bias + restitution * normal_velocity;
}

struct Anchor {
    Vec3 bias;  // tangent 1, tangent 2, twist
};

// Anchor bookkeeping at init (identical for both contact types).
// t1_b / n_b: the A-frame directions as seen from B (physically opposite, B frame).
void update_anchor(ContactCache* cache, const Frame& b_in_a, float dt, Vec3 center, Vec3 n, Vec3 t1, Vec3 t2, Vec3 t1_b,
                   Vec3 n_b, Vec3& bias) {
    bias = {};
    if (!cache) return;
    if (cache->state == 1) {
        cache->anchor_a = center;
        cache->anchor_b = b_in_a.inverse_apply(center);
        cache->t1_a = t1;
        cache->t1_b = t1_b;
        cache->n_a = n;
        cache->n_b = n_b;
        cache->state = 2;
    } else if (cache->state == 2) {
        if (dot(b_in_a.rotate(cache->n_b), cache->n_a) < kAnchorAlignment) {
            const Vec3 ab = b_in_a.apply(cache->anchor_b);
            const Vec3 drift{ab.x - cache->anchor_a.x, ab.y - cache->anchor_a.y, ab.z - cache->anchor_a.z};
            const float k = kBaumgarte / dt;
            bias.x = -(dot(drift, t1) * k);
            bias.y = -(dot(drift, t2) * k);
            bias.z = k * dot(n, cross(cache->t1_a, b_in_a.rotate(cache->t1_b)));
        } else {
            cache->state = 0;
        }
    }
}

void post_solve_cache(ContactCache* cache, bool sliding, int count, const float* normal, Vec3 friction) {
    if (!cache) return;
    if (!sliding) {
        if (cache->state == 0) cache->state = 1;
    } else {
        cache->state = 0;
    }
    cache->count = count;
    for (int i = 0; i < count; ++i) cache->normal_impulse[i] = normal[i];
    cache->friction_impulse = friction;
}

// ---------------------------------------------------------------- dynamic vs static (slot 1)

class StaticContactType final : public ContactTypeBase {
    struct Point {
        Vec3 r;
        float bias = 0, impulse = 0, k = 0;
        Vec3 angular;
    };
    struct Data {
        int a = 0;
        Vec3 n;  // n' = -input normal
        int count = 0;
        Point p[4];
        Vec3 center, t1, t2;
        Vec3 friction_bias, friction_impulse;
        float k1 = 0, k2 = 0, k3 = 0;
        Vec3 ang1, ang2;
        bool sliding = false;
        ContactCache* cache = nullptr;
    };
    std::vector<Data> data_;

public:
    void clear() override {
        inputs.clear();
        data_.clear();
    }
    int add() override {
        inputs.emplace_back();
        data_.emplace_back();
        return static_cast<int>(inputs.size()) - 1;
    }

    void init(Solver& s, const SolverState& st, int first, int end) override {
        const float dt = s.dt();
        for (int i = first; i < end; ++i) {
            const ContactInput& in = inputs[i];
            Data& d = data_[i];
            Body& A = in.a ? *in.a : s.world_body();
            Body& B = in.b ? *in.b : s.world_body();
            d.a = A.solver_index;
            const SolverBody& sb = s.solver_body(d.a);

            const Vec3 n = in.normal_a;
            Vec3 t = sb.linear - n * dot(n, sb.linear);  // sliding direction if fast enough
            const float t2len = length_sq(t);
            t = t2len > kSlideDirectionSq ? t * (1.0f / std::sqrt(t2len)) : perpendicular(n);
            d.t2 = cross(t, n);
            d.t1 = -t;
            d.n = -n;

            const Frame T = frame_of(A, B);
            d.count = in.count;
            d.center = {};
            for (int j = 0; j < d.count; ++j) {
                const Vec3 pa = in.point_a[j];
                const Vec3 pb = T.apply(in.point_b[j]);
                const float separation =
                    -dot(d.n, {pb.x - pa.x, pb.y - pa.y, pb.z - pa.z}) - st.params[kParamSlop];
                const float vn = dot(point_velocity(sb, pa), d.n);
                Point& P = d.p[j];
                P.r = pa;
                P.bias = point_bias(st, dt, separation, vn);
                P.impulse = 0;
                const Row row = make_row(sb, pa, d.n);
                P.angular = row.angular;
                P.k = row.k;
                d.center += divided(pa, static_cast<float>(d.count));
            }
            const Row r1 = make_row(sb, d.center, d.t1), r2 = make_row(sb, d.center, d.t2);
            d.ang1 = r1.angular, d.k1 = r1.k;
            d.ang2 = r2.angular, d.k2 = r2.k;
            d.k3 = twist_mass(sb, d.n);
            d.friction_impulse = {};
            d.cache = in.cache;
            const Vec3 t1_b = T.inverse_rotate(-d.t1), n_b = T.inverse_rotate(-d.n);
            update_anchor(d.cache, T, dt, d.center, d.n, d.t1, d.t2, t1_b, n_b, d.friction_bias);
        }
    }

    void solve(Solver& s, const SolverState& st, int first, int end) override {
        for (int i = first; i < end; ++i) {
            Data& d = data_[i];
            if (d.count == 0) continue;
            SolverBody& sb = s.solver_body(d.a);
            float sum = 0;
            for (int j = 0; j < d.count; ++j) {
                Point& P = d.p[j];
                const float vrel = dot(point_velocity(sb, P.r), d.n);
                const float dl = clamp_accumulate(P.impulse, (-vrel - P.bias) / P.k, normal_lower(st), st.params[kParamUpper]);
                apply(sb, dl, d.n, P.angular);
                sum += P.impulse;
            }
            sum = std::fabs(sum);
            const float limit = sum * st.params[kParamFriction] - normal_lower(st);
            bool sliding = false;
            const Vec3 vc = point_velocity(sb, d.center);  // once, for both tangents
            apply(sb, clamp_accumulate(d.friction_impulse.x, (-dot(vc, d.t1) - d.friction_bias.x) / d.k1, -limit, limit, &sliding),
                  d.t1, d.ang1);
            apply(sb, clamp_accumulate(d.friction_impulse.y, (-dot(vc, d.t2) - d.friction_bias.y) / d.k2, -limit, limit, &sliding),
                  d.t2, d.ang2);
            d.sliding = sliding;
            if (d.count > 1 && d.k3 > 0) {
                const float dl = clamp_accumulate(d.friction_impulse.z, (-dot(sb.angular, d.n) - d.friction_bias.z) / d.k3, -limit, limit);
                apply_twist(sb, dl, d.n);
            }
        }
    }

    void post_solve(Solver& s) override {
        for (Data& d : data_) {
            float normal[4];
            for (int j = 0; j < d.count; ++j) normal[j] = d.p[j].impulse;
            post_solve_cache(d.cache, d.sliding, d.count, normal, d.friction_impulse);
            Body& a = s.body(d.a);
            for (int j = 0; j < d.count; ++j) a.contact_impulse += normal[j];
        }
    }
};

// ---------------------------------------------------------------- two dynamic bodies (slot 0)

class DynamicContactType final : public ContactTypeBase {
    struct Point {
        Vec3 ra, rb;
        float bias = 0, impulse = 0, ka = 0, kb = 0;
        Vec3 ang_a, ang_b;
    };
    struct Data {
        int a = 0, b = 0;
        Vec3 n, n_b;  // n' = -input normal (A frame); n_b = input normal in B frame
        int count = 0;
        Point p[4];
        Vec3 center_a, center_b;
        Vec3 t_a[2], t_b[2];  // tangents in A frame, and their physical opposites in B frame
        Vec3 friction_bias, friction_impulse;
        float k_a[3] = {0, 0, 0}, k_b[3] = {0, 0, 0};  // tangent 1, tangent 2, twist
        Vec3 ang_a[2], ang_b[2];
        bool sliding = false;
        ContactCache* cache = nullptr;
    };
    std::vector<Data> data_;

public:
    void clear() override {
        inputs.clear();
        data_.clear();
    }
    int add() override {
        inputs.emplace_back();
        data_.emplace_back();
        return static_cast<int>(inputs.size()) - 1;
    }

    void init(Solver& s, const SolverState& st, int first, int end) override {
        const float dt = s.dt();
        for (int i = first; i < end; ++i) {
            const ContactInput& in = inputs[i];
            Data& d = data_[i];
            Body& A = in.a ? *in.a : s.world_body();
            Body& B = in.b ? *in.b : s.world_body();
            d.a = A.solver_index;
            d.b = B.solver_index;
            const SolverBody& sa = s.solver_body(d.a);
            const SolverBody& sb = s.solver_body(d.b);

            const Vec3 n = in.normal_a;
            const Vec3 t = perpendicular(n);
            const Vec3 nt = cross(n, t);
            const Frame T = frame_of(A, B);
            d.n_b = T.inverse_rotate(n);
            d.t_b[0] = T.inverse_rotate(t);
            d.t_b[1] = T.inverse_rotate(nt);
            d.n = -n;
            d.t_a[0] = -t;
            d.t_a[1] = -nt;

            d.count = in.count;
            d.center_a = d.center_b = {};
            for (int j = 0; j < d.count; ++j) {
                const Vec3 pa = in.point_a[j], pb = in.point_b[j];
                const Vec3 pbt = T.apply(pb);
                const float separation =
                    -dot(d.n, {pbt.x - pa.x, pbt.y - pa.y, pbt.z - pa.z}) - st.params[kParamSlop];
                const float vn = dot(point_velocity(sa, pa), d.n) + dot(point_velocity(sb, pb), d.n_b);
                Point& P = d.p[j];
                P.ra = pa;
                P.rb = pb;
                P.bias = point_bias(st, dt, separation, vn);
                P.impulse = 0;
                const Row ra = make_row(sa, pa, d.n), rb = make_row(sb, pb, d.n_b);
                P.ang_a = ra.angular, P.ka = ra.k;
                P.ang_b = rb.angular, P.kb = rb.k;
                d.center_a += divided(pa, static_cast<float>(d.count));
                d.center_b += divided(pb, static_cast<float>(d.count));
            }
            for (int k = 0; k < 2; ++k) {
                const Row ra = make_row(sa, d.center_a, d.t_a[k]), rb = make_row(sb, d.center_b, d.t_b[k]);
                d.ang_a[k] = ra.angular, d.k_a[k] = ra.k;
                d.ang_b[k] = rb.angular, d.k_b[k] = rb.k;
            }
            d.k_a[2] = twist_mass(sa, d.n);
            d.k_b[2] = twist_mass(sb, d.n_b);
            d.friction_impulse = {};
            d.cache = in.cache;
            update_anchor(d.cache, T, dt, d.center_a, d.n, d.t_a[0], d.t_a[1], d.t_b[0], d.n_b, d.friction_bias);
        }
    }

    void solve(Solver& s, const SolverState& st, int first, int end) override {
        for (int i = first; i < end; ++i) {
            Data& d = data_[i];
            if (d.count == 0) continue;
            SolverBody& sa = s.solver_body(d.a);
            SolverBody& sb = s.solver_body(d.b);
            float sum = 0;
            for (int j = 0; j < d.count; ++j) {
                Point& P = d.p[j];
                const float vrel = dot(point_velocity(sa, P.ra), d.n) + dot(point_velocity(sb, P.rb), d.n_b);
                const float dl =
                    clamp_accumulate(P.impulse, (-vrel - P.bias) / (P.ka + P.kb), normal_lower(st), st.params[kParamUpper]);
                apply(sa, dl, d.n, P.ang_a);
                apply(sb, dl, d.n_b, P.ang_b);
                sum += P.impulse;
            }
            sum = std::fabs(sum);
            const float limit = sum * st.params[kParamFriction] - normal_lower(st);
            bool sliding = false;
            const Vec3 vca = point_velocity(sa, d.center_a), vcb = point_velocity(sb, d.center_b);  // once
            const float* bias = &d.friction_bias.x;
            float* total = &d.friction_impulse.x;
            for (int k = 0; k < 2; ++k) {
                const float vrel = dot(vca, d.t_a[k]) + dot(vcb, d.t_b[k]);
                const float dl = clamp_accumulate(total[k], (-vrel - bias[k]) / (d.k_a[k] + d.k_b[k]), -limit, limit, &sliding);
                apply(sa, dl, d.t_a[k], d.ang_a[k]);
                apply(sb, dl, d.t_b[k], d.ang_b[k]);
            }
            d.sliding = sliding;
            const float k3 = d.k_a[2] + d.k_b[2];
            if (d.count > 1 && k3 > 0) {
                const float vrel = dot(sa.angular, d.n) + dot(sb.angular, d.n_b);
                const float dl = clamp_accumulate(total[2], (-vrel - bias[2]) / k3, -limit, limit);
                apply_twist(sa, dl, d.n);
                apply_twist(sb, dl, d.n_b);
            }
        }
    }

    void post_solve(Solver& s) override {
        for (Data& d : data_) {
            float normal[4];
            for (int j = 0; j < d.count; ++j) normal[j] = d.p[j].impulse;
            post_solve_cache(d.cache, d.sliding, d.count, normal, d.friction_impulse);
            Body& a = s.body(d.a);
            Body& b = s.body(d.b);
            for (int j = 0; j < d.count; ++j) {
                a.contact_impulse += normal[j];
                b.contact_impulse += normal[j];
            }
        }
    }
};

}  // namespace

Pose relative_pose(const Body& a, const Body& b) {
    const Quat inv_a = conjugate(a.rotation);
    return {Mat3::from_quat(inv_a * b.rotation), rotate(inv_a, b.position - a.position)};
}

std::unique_ptr<ContactTypeBase> make_contact_type() { return std::make_unique<DynamicContactType>(); }
std::unique_ptr<ContactTypeBase> make_static_contact_type() { return std::make_unique<StaticContactType>(); }

void insert_contact(Solver& solver, const ContactInput& contact) {
    const bool a_static = contact.a ? contact.a->is_static() : true;
    const bool b_static = contact.b ? contact.b->is_static() : true;
    if (a_static && b_static) return;
    ContactInput c = contact;
    int slot = b_static ? Solver::kContactStatic : Solver::kContact;
    if (a_static) {
        std::swap(c.a, c.b);
        c.normal_a = -contact.normal_b;
        c.normal_b = -contact.normal_a;
        for (int i = 0; i < c.count && i < 4; ++i) std::swap(c.point_a[i], c.point_b[i]);
        slot = Solver::kContactStatic;
    }
    auto* type = static_cast<ContactTypeBase*>(solver.constraint_type(slot));
    const int index = solver.add_constraint(slot);
    type->inputs[index] = c;
}

}  // namespace sh::phys
