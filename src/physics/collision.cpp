#include "physics/collision.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

// The separated-distance search reproduces the original's arithmetic order (many dot
// products are summed y, x, z), because for parallel features its sign tests land on
// exact zeros and a one-ulp difference would change which witness points come out.

namespace sh::phys {

namespace {
// a.y*b.y + a.x*b.x + a.z*b.z, the summation order the original uses almost everywhere.
inline float dot_yxz(Vec3 a, Vec3 b) { return a.y * b.y + a.x * b.x + a.z * b.z; }
inline float len_sq_yxz(Vec3 a) { return a.y * a.y + a.x * a.x + a.z * a.z; }
inline float l1(Vec3 a) { return std::fabs(a.x) + std::fabs(a.y) + std::fabs(a.z); }
inline Vec3 neg(Vec3 a) { return {-a.x, -a.y, -a.z}; }
}  // namespace

Vec3 support(const Shape& s, Vec3 d) {
    switch (s.type) {
        case Shape::kSphere: {
            const float n2 = d.y * d.y + d.x * d.x + d.z * d.z;
            Vec3 u{1, 0, 0};
            if (0 < n2) {
                const float inv = 1.0f / std::sqrt(n2);
                u = {inv * d.x, inv * d.y, inv * d.z};
            }
            return {s.radius * u.x, u.y * s.radius, s.radius * u.z};
        }
        case Shape::kBox:
            return {d.x < 0 ? -s.half.x : s.half.x, d.y < 0 ? -s.half.y : s.half.y, d.z < 0 ? -s.half.z : s.half.z};
        case Shape::kHull: {
            int best = 0;
            float best_dot = dot_yxz(d, s.vertices[0]);
            for (int i = 1; i < s.count; ++i) {
                const float p = dot_yxz(d, s.vertices[i]);
                if (best_dot < p) best_dot = p, best = i;  // first maximum wins
            }
            return s.vertices[best];
        }
        case Shape::kPoint:
            break;
    }
    return {};
}

Transform Transform::from(Vec3 p, Quat q) {
    Transform t{};
    const float x = q.x, y = q.y, z = q.z, w = q.w;
    const float yy_zz = y * y + z * z, zz_xx = z * z + x * x, yy_xx = y * y + x * x;
    const float yx_zw = y * x + z * w, yx_mzw = y * x - z * w, zx_myw = z * x - y * w;
    const float zx_yw = z * x + y * w, yz_xw = y * z + x * w, yz_mxw = y * z - x * w;
    t.m[0] = 1 - (yy_zz + yy_zz), t.m[1] = yx_zw + yx_zw, t.m[2] = zx_myw + zx_myw, t.m[3] = 0;
    t.m[4] = yx_mzw + yx_mzw, t.m[5] = 1 - (zz_xx + zz_xx), t.m[6] = yz_xw + yz_xw, t.m[7] = 0;
    t.m[8] = zx_yw + zx_yw, t.m[9] = yz_mxw + yz_mxw, t.m[10] = 1 - (yy_xx + yy_xx), t.m[11] = 0;
    t.m[12] = p.x, t.m[13] = p.y, t.m[14] = p.z, t.m[15] = 1;
    return t;
}

Vec3 Transform::rotate(Vec3 v) const {
    return {(v.y * m[4] + v.x * m[0]) + v.z * m[8], (v.y * m[5] + v.x * m[1]) + v.z * m[9],
            (v.y * m[6] + v.x * m[2]) + v.z * m[10]};
}

Vec3 Transform::apply(Vec3 v) const {
    return {((v.y * m[4] + v.x * m[0]) + v.z * m[8]) + m[12], ((v.y * m[5] + v.x * m[1]) + v.z * m[9]) + m[13],
            ((v.y * m[6] + v.x * m[2]) + v.z * m[10]) + m[14]};
}

Vec3 Transform::inverse_rotate(Vec3 v) const {
    return {(v.y * m[1] + v.x * m[0]) + v.z * m[2], (v.y * m[5] + v.x * m[4]) + v.z * m[6],
            (v.y * m[9] + v.x * m[8]) + v.z * m[10]};
}

namespace {

struct Vtx {
    Vec3 w;  // a - T(b)
    Vec3 a;  // support on A, A frame
    Vec3 b;  // support on B, B frame
};

// Configuration-space object A - B in A's frame.
struct Cso {
    const Shape& A;
    const Shape& B;
    const Transform& T;
    Vec3 sweep{};  // shapecast only: the CSO is extended by the segment [0, sweep]
    bool swept = false;

    Vtx operator()(Vec3 d) const {
        Vtx v;
        v.a = support(A, d);
        v.b = support(B, T.inverse_rotate(neg(d)));
        const Vec3 bw = T.apply(v.b);
        v.w = {v.a.x - bw.x, v.a.y - bw.y, v.a.z - bw.z};
        if (swept && 0 < dot_yxz(sweep, d)) v.w = {v.w.x + sweep.x, v.w.y + sweep.y, v.w.z + sweep.z};
        return v;
    }

    static Vec3 centre(const Shape& s) {
        Vec3 c;
        if (s.type == Shape::kHull && s.count > 0) {
            const float inv = 1.0f / static_cast<float>(s.count);
            for (int i = 0; i < s.count; ++i) {
                c.x = c.x + inv * s.vertices[i].x;
                c.y = c.y + inv * s.vertices[i].y;
                c.z = c.z + inv * s.vertices[i].z;
            }
        }
        return c;
    }
    Vec3 midpoint() const {
        const Vec3 ca = centre(A), cb = T.apply(centre(B));
        return {(ca.x - cb.x) + sweep.x * 0.5f, (ca.y - cb.y) + sweep.y * 0.5f, (ca.z - cb.z) + sweep.z * 0.5f};
    }
};

// The original's GJK variant: the simplex grows by one vertex per step, with a
// direction chosen by the current vertex count, then is reduced.
struct Gjk {
    const Cso& cso;
    float tol;
    int iterations;
    float max_dist;
    bool max_active;

    Vtx v[4];
    int count = 0;
    Vec3 n;  // triangle normal (count 3)
    bool enclosed = false, beyond = false, exhausted = false;

    bool beyond_test(Vec3 d, Vec3 w, bool l1_scaled) const {
        if (!max_active) return false;
        return dot_yxz(d, w) < -(l1_scaled ? max_dist * l1(d) : max_dist);
    }

    void reduce();
    void run();
};

void Gjk::run() {
    const float tol2 = tol * tol;
    for (int iter = 0; iter < iterations; ++iter) {
        int next = count;
        switch (count) {
            case 0: {
                const Vec3 d = neg(cso.midpoint());
                v[0] = cso(d);
                next = 1;
                if (beyond_test(d, v[0].w, true)) {
                    beyond = true;
                    return;
                }
                break;
            }
            case 1: {
                const Vec3 w0 = v[0].w;
                const Vec3 d = neg(w0);
                v[1] = cso(d);
                Vec3& c = v[1].w;
                const Vec3 diff{w0.x - c.x, w0.y - c.y, w0.z - c.z};
                if (len_sq_yxz(diff) < tol2) return;  // converged
                if (beyond_test(d, c, true)) {
                    beyond = true;
                    return;
                }
                const Vec3 cr{w0.y * c.z - w0.z * c.y, w0.z * c.x - w0.x * c.z, w0.x * c.y - w0.y * c.x};
                if (dot_yxz(w0, c) < 0 && len_sq_yxz(cr) < tol2) {
                    // Origin almost on the segment: push the new point sideways by tol.
                    Vec3 p;
                    const float n2 = d.y * d.y + d.x * d.x + d.z * d.z;
                    if (0 < n2) {
                        const float inv = 1.0f / std::sqrt(n2);
                        const float uy = inv * d.y;
                        p = {1, 0, 0};
                        if (std::fabs(inv * d.z) <= 0.7f) {
                            const float ux = inv * d.x, nuy = -uy;
                            const float l2 = ux * ux + nuy * nuy + 0.0f;
                            if (0 < l2) {
                                const float k = 1.0f / std::sqrt(l2);
                                p = {k * nuy, k * ux, k * 0.0f};
                            }
                        } else {
                            const float nuz = -(inv * d.z);
                            const float l2 = 0.0f + nuz * nuz + uy * uy;
                            if (0 < l2) {
                                const float k = 1.0f / std::sqrt(l2);
                                p = {k * 0.0f, nuz * k, uy * k};
                            }
                        }
                    } else {
                        p = {-0.0f, 1, 0};
                    }
                    c = {c.x + tol * p.x, c.y + tol * p.y, c.z + tol * p.z};
                }
                next = 2;
                break;
            }
            case 2: {
                const Vec3 w0 = v[0].w, w1 = v[1].w;
                Vec3 e{w1.x - w0.x, w1.y - w0.y, w1.z - w0.z};
                const float l2 = len_sq_yxz(e);
                if (0 < l2) {
                    const float inv = 1.0f / std::sqrt(l2);
                    e = {e.x * inv, e.y * inv, e.z * inv};
                } else {
                    e = {1, 0, 0};
                }
                const float proj = dot_yxz(e, w0);
                const Vec3 d{-(w0.x - proj * e.x), -(w0.y - proj * e.y), -(w0.z - proj * e.z)};
                v[2] = cso(d);
                const Vec3 c = v[2].w;
                const Vec3 r{c.x - w0.x, c.y - w0.y, c.z - w0.z};
                const float along = dot_yxz(r, e);
                const Vec3 q{r.x - along * e.x, r.y - along * e.y, r.z - along * e.z};
                if (len_sq_yxz(q) < tol2) return;  // converged
                if (beyond_test(d, c, true)) {
                    beyond = true;
                    return;
                }
                n = {(w1.y - w0.y) * (c.z - w1.z) - (w1.z - w0.z) * (c.y - w1.y),
                     (w1.z - w0.z) * (c.x - w1.x) - (w1.x - w0.x) * (c.z - w1.z),
                     (w1.x - w0.x) * (c.y - w1.y) - (w1.y - w0.y) * (c.x - w1.x)};
                if (0 < dot_yxz(w0, n)) {
                    n = neg(n);
                    std::swap(v[0], v[1]);
                }
                next = 3;
                break;
            }
            case 3: {
                const float s = (std::fabs(n.x) + std::fabs(n.y)) + std::fabs(n.z);
                const Vec3 d{n.x / s, n.y / s, n.z / s};
                v[3] = cso(d);
                const Vec3 w0 = v[0].w, c = v[3].w;
                if (dot_yxz({c.x - w0.x, c.y - w0.y, c.z - w0.z}, d) < tol) return;  // converged
                if (beyond_test(d, c, false)) {
                    beyond = true;
                    return;
                }
                next = 4;
                break;
            }
            default:
                break;
        }
        count = next;
        reduce();
        if (enclosed) return;
    }
    exhausted = true;
}

void Gjk::reduce() {
    if (count == 2) {
        const Vec3 A = v[0].w, B = v[1].w;
        if (B.y * (A.y - B.y) + B.x * (A.x - B.x) + B.z * (A.z - B.z) <= 0) return;
        v[0] = v[1];
        count = 1;
        return;
    }
    if (count == 3) {
        const Vec3 A = v[0].w, B = v[1].w, C = v[2].w;
        const Vec3 cb{C.x - B.x, C.y - B.y, C.z - B.z}, ac{A.x - C.x, A.y - C.y, A.z - C.z};
        const float s1 = B.y * (cb.x * n.z - cb.z * n.x) + B.x * (cb.z * n.y - cb.y * n.z) + B.z * (cb.y * n.x - cb.x * n.y);
        const float s2 = C.y * (ac.x * n.z - ac.z * n.x) + C.x * (ac.z * n.y - ac.y * n.z) + C.z * (ac.y * n.x - ac.x * n.y);
        const float t = C.y * ac.y + C.x * ac.x + C.z * ac.z;
        const float c = C.y * cb.y + C.x * cb.x + C.z * cb.z;
        enum { kTriangle, kAC, kBC, kC } keep;
        if (s1 <= 0 && s2 <= 0) keep = kTriangle;
        else if (s1 <= 0) keep = t <= 0 ? kAC : kC;
        else if (s2 <= 0) keep = c >= 0 ? kBC : kC;
        else keep = c >= 0 ? kBC : (t <= 0 ? kAC : kC);
        switch (keep) {
            case kTriangle: break;
            case kAC: v[1] = v[2], count = 2; break;
            case kBC: v[0] = v[1], v[1] = v[2], count = 2; break;
            case kC: v[0] = v[2], count = 1; break;
        }
        return;
    }
    if (count != 4) return;

    const Vec3 A = v[0].w, B = v[1].w, C = v[2].w, D = v[3].w;
    const Vec3 a{A.x - D.x, A.y - D.y, A.z - D.z}, b{B.x - D.x, B.y - D.y, B.z - D.z}, c{C.x - D.x, C.y - D.y, C.z - D.z};
    const Vec3 nAB{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    const Vec3 nBC{b.y * c.z - b.z * c.y, b.z * c.x - b.x * c.z, b.x * c.y - b.y * c.x};
    const Vec3 nCA{a.z * c.y - a.y * c.z, a.x * c.z - a.z * c.x, a.y * c.x - a.x * c.y};
    const bool fAB = 0 <= dot_yxz(D, nAB), fBC = 0 <= dot_yxz(D, nBC), fCA = 0 <= dot_yxz(D, nCA);
    if (fAB && fBC && fCA) {
        enclosed = true;
        return;
    }
    // Per face: g tests the edge towards D's side, h the other edge .
    bool ab_g = fAB, ab_h = fAB, bc_g = fBC, bc_h = fBC, ca_g = fCA, ca_h = fCA;
    auto face_ab = [&] {
        const float g = D.y * (a.x * nAB.z - a.z * nAB.x) + D.x * (a.z * nAB.y - a.y * nAB.z) + D.z * (a.y * nAB.x - a.x * nAB.y);
        const float h = B.y * (nAB.z * (D.x - B.x) - nAB.x * (D.z - B.z)) + B.x * (nAB.y * (D.z - B.z) - nAB.z * (D.y - B.y)) +
                        B.z * (nAB.x * (D.y - B.y) - nAB.y * (D.x - B.x));
        ab_g = 0 < g, ab_h = 0 < h;
        if (h <= 0 && g <= 0) {
            v[2] = v[3];
            n = nAB;
            count = 3;
            return true;
        }
        return false;
    };
    auto face_bc = [&] {
        const float g = D.y * (b.x * nBC.z - b.z * nBC.x) + D.x * (b.z * nBC.y - b.y * nBC.z) + D.z * (b.y * nBC.x - b.x * nBC.y);
        const float h = C.y * (nBC.z * (D.x - C.x) - nBC.x * (D.z - C.z)) + C.x * (nBC.y * (D.z - C.z) - nBC.z * (D.y - C.y)) +
                        C.z * (nBC.x * (D.y - C.y) - nBC.y * (D.x - C.x));
        bc_h = 0 < h, bc_g = 0 < g;
        if (h <= 0 && g <= 0) {
            v[0] = v[1], v[1] = v[2], v[2] = v[3];
            n = nBC;
            count = 3;
            return true;
        }
        return false;
    };
    auto face_ca = [&] {
        const float g = D.y * (c.x * nCA.z - c.z * nCA.x) + D.x * (c.z * nCA.y - c.y * nCA.z) + D.z * (c.y * nCA.x - c.x * nCA.y);
        const float h = A.y * (nCA.z * (D.x - A.x) - nCA.x * (D.z - A.z)) + A.x * (nCA.y * (D.z - A.z) - nCA.z * (D.y - A.y)) +
                        A.z * (nCA.x * (D.y - A.y) - nCA.y * (D.x - A.x));
        ca_g = 0 < g, ca_h = 0 < h;
        if (h <= 0 && g <= 0) {
            const Vtx va = v[0];
            v[0] = v[2], v[1] = va, v[2] = v[3];
            n = nCA;
            count = 3;
            return true;
        }
        return false;
    };
    if (!(fAB && fBC)) {
        if (!fAB && face_ab()) return;
        if (!fBC && face_bc()) return;
    }
    if (!fCA && face_ca()) return;

    if (bc_g && ab_h && dot_yxz(D, b) < 0) {
        v[0] = v[1], v[1] = v[3], count = 2;
    } else if (ca_g && bc_h && dot_yxz(D, c) < 0) {
        v[0] = v[2], v[1] = v[3], count = 2;
    } else if (ca_h && ab_g && dot_yxz(D, a) < 0) {
        v[1] = v[3], count = 2;
    } else {
        v[0] = v[3], count = 1;
    }
}

// Witness points on a triangle (v0, v1, v2): the short-edge rotation and 2x2 solve shared by
// the separated triangle case and EPA. Rotation is applied to the caller's vertices.
template <class V>
void triangle_witness(V& p0, V& p1, V& p2, Vec3 (*w)(const V&), Vec3 (*a)(const V&), Vec3 (*b)(const V&), Vec3& pa,
                      Vec3& pb) {
    auto d2 = [](Vec3 x, Vec3 y) { return len_sq_yxz({x.x - y.x, x.y - y.y, x.z - y.z}); };
    const float s4 = d2(w(p1), w(p2)) * 4.0f;
    if (s4 < d2(w(p0), w(p1)) && s4 < d2(w(p0), w(p2))) {
        const V t = p0;
        p0 = p1, p1 = p2, p2 = t;
    }
    const Vec3 w0 = w(p0), w1 = w(p1), w2 = w(p2);
    const Vec3 e1{w1.x - w0.x, w1.y - w0.y, w1.z - w0.z}, e2{w2.x - w0.x, w2.y - w0.y, w2.z - w0.z};
    const float e12 = dot_yxz(e1, e2), e11 = len_sq_yxz(e1), e22 = len_sq_yxz(e2);
    const float e1w = dot_yxz(e1, w0), e2w = dot_yxz(e2, w0);
    const float k = 1.0f / std::fabs(e11 * e22 - e12 * e12);
    const float s = (e12 * e2w - e22 * e1w) * k, t = (e12 * e1w - e11 * e2w) * k;
    auto mix = [&](Vec3 q0, Vec3 q1, Vec3 q2) {
        return Vec3{q0.x + (q1.x - q0.x) * s + (q2.x - q0.x) * t, q0.y + (q1.y - q0.y) * s + (q2.y - q0.y) * t,
                    q0.z + (q1.z - q0.z) * s + (q2.z - q0.z) * t};
    };
    pa = mix(a(p0), a(p1), a(p2));
    pb = mix(b(p0), b(p1), b(p2));
}

// Result from the final simplex of a separated query.
DistanceResult separated_result(Gjk& g, bool want_points) {
    DistanceResult r;
    r.flags = kResultSeparated | kResultValid | kResultDistance;
    Vtx* v = g.v;
    if (g.count == 2) {
        const Vec3 w0 = v[0].w, w1 = v[1].w;
        Vec3 e{w1.x - w0.x, w1.y - w0.y, w1.z - w0.z};
        const float l2 = len_sq_yxz(e);
        if (0 < l2) {
            const float inv = 1.0f / std::sqrt(l2);
            e = {e.x * inv, e.y * inv, e.z * inv};
        } else {
            e = {1, 0, 0};
        }
        const float proj = dot_yxz(e, w0);
        const Vec3 p{w0.x - proj * e.x, w0.y - proj * e.y, w0.z - proj * e.z};
        const float d = std::sqrt(len_sq_yxz(p));
        r.distance = d;
        r.normal = 0 < d ? Vec3{-(p.x / d), -(p.y / d), -(p.z / d)} : Vec3{-1, -0.0f, -0.0f};
        if (want_points) {
            const Vec3 dw{w1.x - w0.x, w1.y - w0.y, w1.z - w0.z};
            const float len = std::sqrt(len_sq_yxz(dw));
            const float t = -(w0.y * (dw.y / len) + w0.x * (dw.x / len) + w0.z * (dw.z / len)) / len;
            const float u = 1.0f - t;
            r.point_a = {t * v[1].a.x + u * v[0].a.x, t * v[1].a.y + u * v[0].a.y, t * v[1].a.z + u * v[0].a.z};
            r.point_b = {t * v[1].b.x + u * v[0].b.x, t * v[1].b.y + u * v[0].b.y, t * v[1].b.z + u * v[0].b.z};
        }
    } else if (g.count == 3) {
        Vec3 nn = g.n;
        const float l2 = len_sq_yxz(nn);
        if (0 < l2) {
            const float inv = 1.0f / std::sqrt(l2);
            nn = {inv * nn.x, inv * nn.y, inv * nn.z};
        } else {
            nn = {1, 0, 0};
        }
        r.normal = nn;
        const Vec3 w0 = v[0].w;
        r.distance = (-(w0.y * nn.y) - w0.x * nn.x) - w0.z * nn.z;
        if (want_points) {
            Vtx p0 = v[0], p1 = v[1], p2 = v[2];
            triangle_witness<Vtx>(p0, p1, p2, [](const Vtx& x) { return x.w; }, [](const Vtx& x) { return x.a; },
                                  [](const Vtx& x) { return x.b; }, r.point_a, r.point_b);
        }
    } else if (g.count == 1) {
        const Vec3 p = neg(v[0].w);
        const float d = std::sqrt(len_sq_yxz(p));
        r.distance = d;
        r.normal = 0 < d ? Vec3{p.x / d, p.y / d, p.z / d} : Vec3{1, 0, 0};
        if (want_points) {
            r.point_a = v[0].a;
            r.point_b = v[0].b;
        }
    } else {
        r.distance = 0;
        r.normal = {1, 0, 0};
    }
    if (want_points) r.flags |= kResultPoints;
    if (g.exhausted) r.flags |= kResultIterationLimit;
    return r;
}

// ---- Penetration: the original's EPA, then its MPR refinement ----

constexpr float kEpaAccuracy = 0.001f;    // stop when the new support gains no more than this
constexpr float kEpaPlaneEps = -0.001f;   // face sees a point when n.w - d >= this
constexpr float kEpaInsideEps = -0.01f;   // inside measure p >= this counts as 0
constexpr float kEpaMinNormal = 0.001f;   // faces with a shorter (unnormalised) normal are degenerate
constexpr int kEpaMaxVertices = 32, kEpaMaxFaces = 64, kEpaMaxIterations = 32;
constexpr int kNext[3] = {1, 2, 0}, kPrev[3] = {2, 0, 1};

struct EpaFace {
    Vec3 n;
    float d = 0, p = 0;
    const Vtx* c[3] = {nullptr, nullptr, nullptr};
    EpaFace* f[3] = {nullptr, nullptr, nullptr};
    EpaFace* prev = nullptr;
    EpaFace* next = nullptr;
    int e[3] = {0, 0, 0};
    int pass = 0;
};

struct EpaList {
    EpaFace* root = nullptr;
    int count = 0;
    void push(EpaFace* f) {
        f->prev = nullptr;
        f->next = root;
        if (root) root->prev = f;
        root = f;
        ++count;
    }
    void remove(EpaFace* f) {
        if (f->next) f->next->prev = f->prev;
        if (f->prev) f->prev->next = f->next;
        if (f == root) root = f->next;
        --count;
    }
};

struct EpaHorizon {
    EpaFace* cf = nullptr;
    EpaFace* ff = nullptr;
    int nf = 0;
};

class Epa {
public:
    enum Status { kValid = 0, kTouching = 1, kDegenerated = 2, kNonConvex = 3, kInvalidHull = 4, kOutOfFaces = 5,
                  kOutOfVertices = 6, kAccuracyReached = 7, kIterationLimit = 8, kFailed = 10 };

    explicit Epa(const Cso& cso) : cso_(cso) {
        for (EpaFace& f : faces_) stock_.push(&f);
    }

    // Returns the status; on success fills outer (best face) with possibly rotated vertices.
    int evaluate(const Vtx (&simplex)[4], bool rotate_for_witness) {
        status_ = kValid;
        EpaFace* t0 = newface(&simplex[2], &simplex[1], &simplex[0], true);
        EpaFace* t1 = newface(&simplex[1], &simplex[2], &simplex[3], true);
        EpaFace* t2 = newface(&simplex[0], &simplex[1], &simplex[3], true);
        EpaFace* t3 = newface(&simplex[2], &simplex[0], &simplex[3], true);
        if (hull_.count != 4) return kFailed;
        EpaFace* best = findbest();
        outer_ = *best;
        bind(t0, 0, t1, 0);
        bind(t0, 1, t2, 0);
        bind(t0, 2, t3, 0);
        bind(t1, 1, t3, 2);
        bind(t1, 2, t2, 1);
        bind(t2, 2, t3, 1);
        status_ = kValid;
        int pass = 0;
        for (;;) {
            if (nextsv_ >= kEpaMaxVertices) {
                status_ = kOutOfVertices;
                break;
            }
            Vtx* w = &sv_[nextsv_++];
            best->pass = ++pass;
            *w = cso_(best->n);
            const float wdist = dot_yxz(best->n, w->w) - best->d;
            if (wdist <= kEpaAccuracy) {
                status_ = kAccuracyReached;
                break;
            }
            EpaHorizon horizon;
            bool valid = true;
            for (int j = 0; j < 3 && valid; ++j) valid = expand(pass, w, best->f[j], best->e[j], horizon);
            if (!valid || horizon.nf < 3) {
                status_ = kInvalidHull;
                break;
            }
            bind(horizon.cf, 1, horizon.ff, 2);
            hull_.remove(best);
            stock_.push(best);
            best = findbest();
            if (outer_.p <= best->p) outer_ = *best;
            if (pass >= kEpaMaxIterations) break;
        }
        if (pass == kEpaMaxIterations) status_ = kIterationLimit;
        if (rotate_for_witness) {
            // The original computes EPA witness points here (later overwritten) and in doing so
            // may rotate the outer face's vertex order; that order feeds the MPR refinement.
            Vec3 pa, pb;
            triangle_witness<const Vtx*>(
                outer_.c[0], outer_.c[1], outer_.c[2], [](const Vtx* const& v) { return v->w; },
                [](const Vtx* const& v) { return v->a; }, [](const Vtx* const& v) { return v->b; }, pa, pb);
        }
        return status_;
    }

    const EpaFace& outer() const { return outer_; }

private:
    static void bind(EpaFace* fa, int ea, EpaFace* fb, int eb) {
        fa->e[ea] = eb, fa->f[ea] = fb;
        fb->e[eb] = ea, fb->f[eb] = fa;
    }

    EpaFace* newface(const Vtx* a, const Vtx* b, const Vtx* c, bool forced) {
        EpaFace* face = stock_.root;
        if (!face) {
            status_ = kOutOfFaces;
            return nullptr;
        }
        stock_.remove(face);
        hull_.push(face);
        face->c[0] = a, face->c[1] = b, face->c[2] = c;
        face->pass = 0;
        const Vec3 A = a->w, B = b->w, C = c->w;
        const Vec3 n{(B.y - A.y) * (C.z - A.z) - (B.z - A.z) * (C.y - A.y), (B.z - A.z) * (C.x - A.x) - (B.x - A.x) * (C.z - A.z),
                     (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x)};
        const float l = std::sqrt(n.y * n.y + n.x * n.x + n.z * n.z);
        // Inside measure: how far the origin is outside the triangle's edges (min over edges).
        const float va = A.y * ((A.x - B.x) * n.z - (A.z - B.z) * n.x) + A.x * ((A.z - B.z) * n.y - (A.y - B.y) * n.z) +
                         A.z * ((A.y - B.y) * n.x - (A.x - B.x) * n.y);
        const float vb = B.y * (n.z * (B.x - C.x) - n.x * (B.z - C.z)) + B.x * (n.y * (B.z - C.z) - n.z * (B.y - C.y)) +
                         B.z * (n.x * (B.y - C.y) - n.y * (B.x - C.x));
        const float vc = C.y * (n.z * (C.x - A.x) - n.x * (C.z - A.z)) + C.x * (n.y * (C.z - A.z) - n.z * (C.y - A.y)) +
                         C.z * (n.x * (C.y - A.y) - n.y * (C.x - A.x));
        float m = vb;
        if (va < m) m = va;
        const float vmin = m < vc ? m : vc;
        const float den = kEpaMinNormal < l ? l : 1.0f;
        face->p = vmin / den >= kEpaInsideEps ? 0.0f : vmin / den;
        if (l <= kEpaMinNormal) {
            status_ = kDegenerated;
        } else {
            face->d = (A.y * n.y + A.x * n.x + A.z * n.z) / l;
            face->n = {n.x / l, n.y / l, n.z / l};
            if (forced || kEpaPlaneEps <= face->d) return face;
            status_ = kNonConvex;
        }
        hull_.remove(face);
        stock_.push(face);
        return nullptr;
    }

    EpaFace* findbest() {
        EpaFace* minf = hull_.root;
        float mind = minf->d * minf->d, maxp = minf->p;
        for (EpaFace* f = minf->next; f; f = f->next) {
            const float sqd = f->d * f->d;
            if (maxp <= f->p && sqd < mind) minf = f, mind = sqd, maxp = f->p;
        }
        return minf;
    }

    bool expand(int pass, const Vtx* w, EpaFace* f, int e, EpaHorizon& horizon) {
        if (f->pass == pass) return false;
        const int e1 = kNext[e];
        if (dot_yxz(w->w, f->n) - f->d < kEpaPlaneEps) {
            EpaFace* nf = newface(f->c[e1], f->c[e], w, false);
            if (!nf) return false;
            bind(nf, 0, f, e);
            if (horizon.cf) bind(horizon.cf, 1, nf, 2);
            else horizon.ff = nf;
            horizon.cf = nf;
            ++horizon.nf;
            return true;
        }
        const int e2 = kPrev[e];
        f->pass = pass;
        if (expand(pass, w, f->f[e1], f->e[e1], horizon) && expand(pass, w, f->f[e2], f->e[e2], horizon)) {
            hull_.remove(f);
            stock_.push(f);
            return true;
        }
        return false;
    }

    const Cso& cso_;
    EpaFace faces_[kEpaMaxFaces];
    Vtx sv_[kEpaMaxVertices];
    int nextsv_ = 0;
    EpaList hull_, stock_;
    EpaFace outer_;
    int status_ = kValid;
};

// The original's MPR depth refinement: a portal (v0, v1, v2) and direction d.
struct MprRefine {
    const Cso& cso;
    Vec3 d;
    Vtx v[3];
    float tol;
    int iterations;

    // Triple product of the three edge planes against d; all >= 0 when the ray along d crosses the portal.
    static float edge_test(Vec3 p, Vec3 e, Vec3 d) {
        return (e.z * d.x - e.x * d.z) * p.y + (e.y * d.z - e.z * d.y) * p.x + (e.x * d.y - e.y * d.x) * p.z;
    }
    bool oriented_wrong() const {
        const Vec3 a = v[0].w, b = v[1].w, c = v[2].w;
        return ((b.z - a.z) * (c.x - b.x) - (b.x - a.x) * (c.z - b.z)) * d.y +
                   ((b.y - a.y) * (c.z - b.z) - (b.z - a.z) * (c.y - b.y)) * d.x +
                   ((b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x)) * d.z <
               0;
    }
    // New third vertex: support perpendicular to v0->v1 within the plane of d, facing away from v0.
    void rebuild_third() {
        const Vec3 a = v[0].w, b = v[1].w;
        const Vec3 r{b.x - a.x, b.y - a.y, b.z - a.z};
        Vec3 k{r.z * d.y - r.y * d.z, r.x * d.z - r.z * d.x, r.y * d.x - r.x * d.y};
        if (0 < a.y * k.y + a.x * k.x + a.z * k.z) k = neg(k);
        v[2] = cso(k);
        if (oriented_wrong()) std::swap(v[0], v[1]);
    }

    Vec3 track_surface() {
        Vec3 out;
        for (int it = 0; it < iterations; ++it) {
            const Vec3 a = v[0].w, b = v[1].w, c = v[2].w;
            const Vec3 e1{b.x - a.x, b.y - a.y, b.z - a.z}, e2{c.x - b.x, c.y - b.y, c.z - b.z};
            Vec3 n{e1.y * e2.z - e1.z * e2.y, e1.z * e2.x - e1.x * e2.z, e1.x * e2.y - e1.y * e2.x};
            const float l2 = n.y * n.y + n.x * n.x + n.z * n.z;
            if (0 < l2) {
                const float inv = 1.0f / std::sqrt(l2);
                n = {n.x * inv, n.y * inv, n.z * inv};
            } else {
                n = {1, 0, 0};
            }
            out = n;
            const Vtx fresh = cso(n);  // full support (the emulator trace shows real supports here)
            const Vec3 s = fresh.w;
            if ((s.y - a.y) * n.y + (s.x - a.x) * n.x + (s.z - a.z) * n.z < tol) break;
            auto turn = [&](Vec3 q) {  // d . (s x (q - s)), in the original's evaluation order
                const Vec3 m{q.x - s.x, q.y - s.y, q.z - s.z};
                return (m.x * s.z - m.z * s.x) * d.y + (m.z * s.y - m.y * s.z) * d.x + d.z * (m.y * s.x - m.x * s.y);
            };
            const float t0 = turn(a), t1 = turn(b);
            if (t0 < 0 || 0 < t1) {
                const float t2 = turn(c);
                if (t1 < 0 || 0 < t2) {
                    if (t2 < 0 || 0 < t0) return out;
                    v[1] = v[0], v[0] = v[2], v[2] = fresh;
                } else {
                    v[0] = v[1], v[1] = v[2], v[2] = fresh;
                }
            } else {
                v[2] = fresh;
            }
        }
        return out;
    }

    void run() {
        for (int outer = 0; outer < iterations; ++outer) {
            for (int it = 0;; ++it) {
                const Vec3 a = v[0].w, b = v[1].w, c = v[2].w;
                const float s0 = edge_test(a, {b.x - a.x, b.y - a.y, b.z - a.z}, d);
                const float s1 = edge_test(b, {c.x - b.x, c.y - b.y, c.z - b.z}, d);
                const float s2 = edge_test(c, {a.x - c.x, a.y - c.y, a.z - c.z}, d);
                if ((0 <= s0 && 0 <= s1 && 0 <= s2) || iterations <= it) break;
                if (s0 < 0) {
                    if (s1 < 0) {
                        v[0] = v[1];
                        v[1] = cso(d);
                    } else if (s2 < 0) {
                        v[1] = cso(d);
                    }  // else: keep v0, v1 and only rebuild the third vertex
                } else if (s1 < 0) {
                    if (s2 < 0) {
                        v[0] = v[2];
                        v[1] = cso(d);
                    } else {
                        v[0] = v[1], v[1] = v[2];
                    }
                } else {  // s2 < 0
                    const Vtx old0 = v[0];
                    v[0] = v[2], v[1] = old0;
                }
                rebuild_third();
            }
            const Vec3 nd = track_surface();
            if (0.99f < d.y * nd.y + d.x * nd.x + d.z * nd.z) return;
            d = nd;
        }
    }
};

// Final penetration result from the refined portal: ray along d from the origin onto the portal plane.
DistanceResult portal_result(const MprRefine& m, bool want_points, bool bad_status) {
    DistanceResult r;
    r.flags = kResultPenetrating | kResultValid | kResultDistance;
    if (bad_status) r.flags |= kResultIterationLimit;
    const Vec3 d = m.d;
    const Vec3 w0 = m.v[0].w;
    const Vec3 e1{m.v[1].w.x - w0.x, m.v[1].w.y - w0.y, m.v[1].w.z - w0.z};
    const Vec3 e2{m.v[2].w.x - w0.x, m.v[2].w.y - w0.y, m.v[2].w.z - w0.z};
    const Vec3 c{e1.y * e2.z - e1.z * e2.y, e1.z * e2.x - e1.x * e2.z, e1.x * e2.y - e1.y * e2.x};
    const float den = -(c.y * d.y + c.x * d.x + c.z * d.z);
    float u = 0, v = 0, dist = -0.0f;
    if (den != 0) {
        const Vec3 mo = neg(w0);
        const Vec3 q{mo.y * d.z - mo.z * d.y, mo.z * d.x - mo.x * d.z, mo.x * d.y - mo.y * d.x};
        const float inv = 1.0f / den;
        u = inv * (e2.y * q.y + e2.x * q.x + e2.z * q.z);
        v = -((e1.y * q.y + e1.x * q.x + e1.z * q.z) * inv);
        dist = -((c.y * mo.y + c.x * mo.x + c.z * mo.z) * inv);
    }
    r.normal = d;
    r.distance = dist;
    if (want_points) {
        r.flags |= kResultPoints;
        auto mix = [&](Vec3 q0, Vec3 q1, Vec3 q2) {
            return Vec3{q0.x + u * (q1.x - q0.x) + v * (q2.x - q0.x), q0.y + u * (q1.y - q0.y) + v * (q2.y - q0.y),
                        q0.z + u * (q1.z - q0.z) + v * (q2.z - q0.z)};
        };
        r.point_a = mix(m.v[0].a, m.v[1].a, m.v[2].a);
        r.point_b = mix(m.v[0].b, m.v[1].b, m.v[2].b);
    }
    return r;
}

DistanceResult penetration(const CollisionContext& ctx, const Cso& cso, const Gjk& g, bool want_points) {
    Epa epa(cso);
    const int status = epa.evaluate(g.v, want_points);
    MprRefine mpr{cso, {}, {}, ctx.tolerance, ctx.max_iterations};
    if (status != Epa::kFailed) {
        const EpaFace& o = epa.outer();
        mpr.d = o.n;
        mpr.v[0] = *o.c[0], mpr.v[1] = *o.c[1], mpr.v[2] = *o.c[2];
    } else {
        // KNOWN GAP: the original rebuilds a portal with TdMpr::adjustPortal here (only when GJK's
        // final tetrahedron is degenerate). Approximation: refine from the tetrahedron's first face.
        mpr.v[0] = g.v[0], mpr.v[1] = g.v[1], mpr.v[2] = g.v[2];
        const Vec3 a = g.v[0].w, b = g.v[1].w, c = g.v[2].w;
        Vec3 n = cross(b - a, c - a);
        if (dot(n, a) < 0) n = neg(n);
        const float l = length(n);
        mpr.d = l > 0 ? n * (1.0f / l) : Vec3{1, 0, 0};
    }
    mpr.run();
    const bool good = status == Epa::kValid || status == Epa::kTouching || status == Epa::kAccuracyReached;
    return portal_result(mpr, want_points, !good);
}

}  // namespace

DistanceResult distance(const CollisionContext& ctx, const DistanceQuery& q) {
    // Spheres become points; their radii widen the max distance and come off the result.
    Shape a = *q.a, b = *q.b;
    float ra = 0, rb = 0;
    float max_dist = q.max_distance;
    if (a.type == Shape::kSphere) ra = a.radius, max_dist = max_dist + ra, a = Shape{};
    if (b.type == Shape::kSphere) rb = b.radius, max_dist = max_dist + rb, b = Shape{};

    const Transform T = Transform::from(q.b_position, q.b_rotation);
    const Cso cso{a, b, T};
    const bool use_max = (q.flags & kQueryMaxDistance) != 0;
    Gjk g{cso, ctx.tolerance, ctx.max_iterations, use_max ? max_dist : 3.402823466e+38f,
          use_max && max_dist < 3.402823466e+38f};
    g.run();

    DistanceResult r;
    if (g.beyond) {
        r.flags = kResultBeyondMax;
        r.distance = max_dist;
    } else if (!g.enclosed) {
        r = separated_result(g, (q.flags & kQueryPoints) != 0);
    } else {
        r = penetration(ctx, cso, g, (q.flags & kQueryPoints) != 0);
    }

    if (ra != 0) {
        r.distance = r.distance - ra;
        if (r.flags & kResultPoints)
            r.point_a = {r.point_a.x + ra * r.normal.x, r.point_a.y + ra * r.normal.y, r.point_a.z + ra * r.normal.z};
    }
    if (rb != 0) {
        r.distance = r.distance - rb;
        if (r.flags & kResultPoints) {
            const Vec3 nb = T.inverse_rotate({rb * r.normal.x, rb * r.normal.y, rb * r.normal.z});
            r.point_b = {r.point_b.x - nb.x, r.point_b.y - nb.y, r.point_b.z - nb.z};
        }
    }
    if (r.distance < 0) r.flags = (r.flags & ~kResultSeparated) | kResultPenetrating;
    return r;
}

bool overlap(const CollisionContext& ctx, const OverlapQuery& q, OverlapSimplex* simplex) {
    // The same GJK steps as distance(), with the max-distance cut fixed at 0: the search stops as
    // soon as a support point fails to reach past the origin (a separating direction).
    const Transform T = Transform::from(q.b_position, q.b_rotation);
    const Cso cso{*q.a, *q.b, T};
    Gjk g{cso, ctx.tolerance, ctx.max_iterations, 0.0f, true};
    g.run();
    if (simplex) {
        *simplex = {};
        if (!g.beyond) {
            simplex->count = g.count;
            for (int i = 0; i < g.count; ++i) simplex->a[i] = g.v[i].a, simplex->b[i] = g.v[i].b;
        }
    }
    return g.enclosed;
}

DistanceResult shapecast(const CollisionContext& ctx, const ShapecastQuery& q) {
    const Transform T = Transform::from(q.b_position, q.b_rotation);
    Cso cso{*q.a, *q.b, T};
    // Sweep length: twice the distance between the shape centres.
    const Vec3 mid = cso.midpoint();
    float len = std::sqrt(len_sq_yxz(mid));
    len = len + len;
    const Vec3 dir = q.sweep_direction;
    cso.sweep = {len * dir.x, len * dir.y, len * dir.z};
    cso.swept = true;

    // Does the swept object contain the origin at all?
    Gjk g{cso, ctx.tolerance, ctx.max_iterations, 3.402823466e+38f, false};
    g.run();
    DistanceResult r;
    if (!g.enclosed) {
        r.flags = kResultSeparated | kResultNoHit;
        return r;
    }

    // Portal: the first face of the tetrahedron that the ray from the origin along dir crosses.
    auto crosses = [&](const Vtx& p0, const Vtx& p1, const Vtx& p2) {
        const Vec3 a = p0.w, b = p1.w, c = p2.w;
        return !(MprRefine::edge_test(a, {b.x - a.x, b.y - a.y, b.z - a.z}, dir) < 0 ||
                 MprRefine::edge_test(b, {c.x - b.x, c.y - b.y, c.z - b.z}, dir) < 0 ||
                 MprRefine::edge_test(c, {a.x - c.x, a.y - c.y, a.z - c.z}, dir) < 0);
    };
    MprRefine m{cso, dir, {g.v[0], g.v[1], g.v[3]}, ctx.tolerance, ctx.max_iterations};
    if (!crosses(g.v[0], g.v[1], g.v[3])) {
        if (crosses(g.v[1], g.v[2], g.v[3])) {
            m.v[0] = g.v[1], m.v[1] = g.v[2], m.v[2] = g.v[3];
        } else if (crosses(g.v[2], g.v[0], g.v[3])) {
            m.v[0] = g.v[2], m.v[1] = g.v[0], m.v[2] = g.v[3];
        } else {
            m.v[0] = g.v[1], m.v[1] = g.v[0], m.v[2] = g.v[2];
        }
    }
    r.normal = m.track_surface();

    // Where the ray along dir meets the portal plane: dir * t, with barycentric (u, v).
    const Vec3 w0 = m.v[0].w;
    const Vec3 e1{m.v[1].w.x - w0.x, m.v[1].w.y - w0.y, m.v[1].w.z - w0.z};
    const Vec3 e2{m.v[2].w.x - w0.x, m.v[2].w.y - w0.y, m.v[2].w.z - w0.z};
    const Vec3 c{e1.y * e2.z - e1.z * e2.y, e1.z * e2.x - e1.x * e2.z, e1.x * e2.y - e1.y * e2.x};
    const float den = -(c.y * dir.y + c.x * dir.x + c.z * dir.z);
    float u = 0, v = 0, t = 0;  // the original leaves u, v unset when den is 0
    if (den != 0) {
        const Vec3 mo = neg(w0);
        const Vec3 qv{mo.y * dir.z - mo.z * dir.y, mo.z * dir.x - mo.x * dir.z, mo.x * dir.y - mo.y * dir.x};
        const float inv = 1.0f / den;
        u = inv * (e2.y * qv.y + e2.x * qv.x + e2.z * qv.z);
        v = -((e1.y * qv.y + e1.x * qv.x + e1.z * qv.z) * inv);
        t = inv * (c.y * mo.y + c.x * mo.x + c.z * mo.z);
    }
    r.distance = len - t;
    r.flags = kResultValid | kResultDistance | (r.distance < 0 ? kResultPenetrating : kResultSeparated);
    if (r.distance < 0 && !(q.flags & kQueryPenetration)) return r;
    if ((q.flags & kQueryMaxDistance) && q.max_distance < r.distance) {
        r.flags |= kResultBeyondMax;
        return r;
    }
    if (!(q.flags & kQueryPoints)) return r;
    r.flags |= kResultPoints;
    auto mix = [&](Vec3 q0, Vec3 q1, Vec3 q2) {
        return Vec3{q0.x + u * (q1.x - q0.x) + v * (q2.x - q0.x), q0.y + u * (q1.y - q0.y) + v * (q2.y - q0.y),
                    q0.z + u * (q1.z - q0.z) + v * (q2.z - q0.z)};
    };
    r.point_a = mix(m.v[0].a, m.v[1].a, m.v[2].a);
    r.point_b = mix(m.v[0].b, m.v[1].b, m.v[2].b);
    return r;
}

DistanceResult raycast(const CollisionContext& ctx, const RaycastQuery& q) {
    const Shape point{};
    ShapecastQuery s;
    s.a = q.shape;
    s.b = &point;
    s.b_position = q.origin;
    s.sweep_direction = {-q.direction.x, -q.direction.y, -q.direction.z};
    s.flags = q.flags;
    s.max_distance = q.max_distance;
    DistanceResult r = shapecast(ctx, s);
    r.point_b = {};
    return r;
}

void update_manifold(const CollisionContext& ctx, Manifold& m, const ManifoldQuery& q) {
    const float tol = q.tolerance, tol_sq = tol * tol;
    const Transform T = Transform::from(q.b_position, q.b_rotation);
    auto sub = [](Vec3 a, Vec3 b) { return Vec3{a.x - b.x, a.y - b.y, a.z - b.z}; };

    if (m.count == 0) {
        // Separated: skip the query while B's reference points stay where they were.
        if (m.normal_a.x == kManifoldSeparated) {
            int i = 0;
            while (i < 3 && !(tol_sq * 0.1f < len_sq_yxz(sub(m.point_a[i], T.apply(m.point_b[i]))))) ++i;
            if (i == 3) return;
        }
    } else if (m.count > 0) {
        const Vec3 n = m.normal_a;
        int count = m.count;
        bool drifted = false;
        for (int i = 0; i < count; ++i) {
            const Vec3 d = sub(T.apply(m.point_b[i]), m.point_a[i]);
            const float s = dot_yxz(d, n);
            const Vec3 lateral{d.x - s * n.x, d.y - s * n.y, d.z - s * n.z};
            if (tol_sq * 4.0f < len_sq_yxz(lateral)) {  // slid sideways: start over
                m.count = 0;
                drifted = true;
                break;
            }
            if (tol + tol < s) {  // pulled apart along the normal: drop this point
                --count;
                m.point_a[i] = m.point_a[count];
                m.point_b[i] = m.point_b[count];
                m.count = count;
                --i;
            }
        }
        if (!drifted && count > 0 && 0.99999f < dot_yxz(T.rotate(m.normal_b), n)) return;
    }

    DistanceQuery dq;
    dq.a = q.a;
    dq.b = q.b;
    dq.b_position = q.b_position;
    dq.b_rotation = q.b_rotation;
    dq.flags = 0xb3;
    dq.max_distance = q.max_distance;
    const DistanceResult r = distance(ctx, dq);
    if (r.flags & kResultIterationLimit) {
        m.count = 0;
        return;
    }

    if (!(r.flags & kResultBeyondMax) && r.distance <= q.max_distance) {
        m.normal_a = r.normal;
        m.normal_b = T.inverse_rotate(r.normal);
        if (m.count < 1) {
            m.point_a[0] = r.point_a;
            m.point_b[0] = r.point_b;
            m.count = 1;
            return;
        }
        // Replace the stored point nearest to the new one, or append if it is new and there is room.
        float best = len_sq_yxz(sub(m.point_a[0], r.point_a));
        int slot = 0;
        for (int i = 1; i < m.count; ++i) {
            const float d = len_sq_yxz(sub(r.point_a, m.point_a[i]));
            if (d < best) best = d, slot = i;
        }
        if (m.count < 4 && tol_sq < best) slot = m.count++;
        m.point_a[slot] = r.point_a;
        m.point_b[slot] = r.point_b;
        return;
    }

    // Beyond reach: store the separated state (other words keep their old values, as in the original).
    m.count = 0;
    m.normal_a.x = kManifoldSeparated;
    const Vec3 p{T.m[12], T.m[13], T.m[14]};
    m.point_a[0] = {1, 0, 0};
    m.point_a[1] = {0, 1, 0};
    m.point_a[2] = {0, 0, 1};
    m.point_b[0] = T.inverse_rotate({1.0f - p.x, 0.0f - p.y, 0.0f - p.z});
    m.point_b[1] = T.inverse_rotate({0.0f - p.x, 1.0f - p.y, 0.0f - p.z});
    m.point_b[2] = T.inverse_rotate({0.0f - p.x, 0.0f - p.y, 1.0f - p.z});
}

}  // namespace sh::phys
