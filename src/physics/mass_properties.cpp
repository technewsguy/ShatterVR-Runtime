#include "physics/mass_properties.hpp"

#include "physics/polyhedron.hpp"

#include <cmath>
#include <vector>

// Volume integrals follow Mirtich's method ("Fast and Accurate Computation of Polyhedral Mass
// Properties", 1996), with the sums evaluated in the order the original's compiled code uses.

namespace sh::phys {

namespace {

struct Tri {
    float n[3];
    float w;
    int v[3];
};

struct Integrals {
    const std::vector<Vec3>& pos;
    int A = 0, B = 1, C = 2;
    // projection integrals
    float P1 = 0, Pa = 0, Pb = 0, Paa = 0, Pab = 0, Pbb = 0, Paaa = 0, Paab = 0, Pabb = 0, Pbbb = 0;
    // face integrals
    float Fa = 0, Fb = 0, Fc = 0, Faa = 0, Fbb = 0, Fcc = 0, Faaa = 0, Fbbb = 0, Fccc = 0, Faab = 0, Fbbc = 0, Fcca = 0;
    // volume integrals
    float T0 = 0, T1[3]{}, T2[3]{}, TP[3]{};

    void projection(const Tri& f) {
        float p1 = 0, pa = 0, pb = 0, paa = 0, pab = 0, pbb = 0, paaa = 0, paab = 0, pabb = 0, pbbb = 0;
        for (int i = 0; i < 3; ++i) {
            const Vec3& v0 = pos[f.v[i]];
            const Vec3& v1 = pos[f.v[(i + 1) % 3]];
            const float a0 = v0[A], b0 = v0[B], a1 = v1[A], b1 = v1[B];
            const float b0sq = b0 * b0, a0sq = a0 * a0, a0cu = a0sq * a0, b1sq = b1 * b1, a1sq = a1 * a1;
            const float t = (a1 + a1) * a0;
            const float kab = a1sq + t + a0sq * 3.0f;
            const float cb = b0sq + (b1 + b0) * b1;
            const float ca = a0sq + a1 * (a1 + a0);
            const float b0cu = b0sq * b0;
            const float cab = t + a1sq * 3.0f + a0sq;
            const float caa = a0cu + a1 * ca;
            const float cbb = b0cu + b1 * cb;
            const float db = b1 - b0, da = a1 - a0;
            paab = paab + (b0 * (a0cu * 4.0f + a1 * kab) + b1 * (a1sq * a1 * 4.0f + a0 * cab)) * db;
            paaa = paaa + db * (a0cu * a0 + a1 * caa);
            pab = pab + (b0 * kab + b1 * cab) * db;
            pa = pa + db * ca;
            paa = paa + db * caa;
            p1 = p1 + db * (a1 + a0);
            pabb = pabb + (a0 * (b1sq * b1 + (b1sq + b1sq) * b0 + b1 * 3.0f * b0sq + b0cu * 4.0f) +
                           a1 * (b1sq * 3.0f * b0 + b1sq * b1 * 4.0f + (b1 + b1) * b0sq + b0cu)) *
                              da;
            pb = pb + da * cb;
            pbb = pbb + da * cbb;
            pbbb = pbbb + da * (b0cu * b0 + b1 * cbb);
        }
        Pabb = pabb / -60.0f;
        P1 = p1 * 0.5f;
        Pa = pa / 6.0f;
        Paa = paa / 12.0f;
        Paaa = paaa / 20.0f;
        Pb = pb / -6.0f;
        Pab = pab / 24.0f;
        Pbb = pbb / -12.0f;
        Paab = paab / 60.0f;
        Pbbb = pbbb / -20.0f;
    }

    void face(const Tri& f) {
        projection(f);
        const float w = f.w, na = f.n[A], nb = f.n[B];
        const float na2 = na * na, nb2 = nb * nb, t = nb * (na + na);
        const float k1 = 1.0f / f.n[C];
        const float s9 = nb * Pab + na * Paa;
        const float s12 = Pb * nb + Pa * na;
        const float s11 = t * Pab + Paa * na2 + Pbb * nb2;
        const float s8 = w * P1;
        const float k3 = k1 * k1 * k1;
        const float k2n = -(k1 * k1);
        Fa = k1 * Pa;
        Fb = k1 * Pb;
        Fbbb = k1 * Pbbb;
        Faaa = k1 * Paaa;
        Faab = k1 * Paab;
        Fcca = (t * Paab + na2 * Paaa + nb2 * Pabb + (s9 + s9 + w * Pa) * w) * k3;
        Faa = k1 * Paa;
        Fc = k2n * (s12 + s8);
        Fbbc = k2n * (nb * Pbbb + na * Pabb + w * Pbb);
        Fbb = k1 * Pbb;
        // Grouping taken from the machine code (the order matters for these products).
        Fcc = (s11 + (s8 + (s12 + s12)) * w) * k3;
        Fccc = -(k1 * k3) * ((nb * (na2 * 3.0f)) * Paab + Paaa * (na * na2) + (nb2 * (na * 3.0f)) * Pabb +
                             Pbbb * (nb * nb2) + s11 * (w * 3.0f) + (w * w) * (s8 + s12 * 3.0f));
    }

    void volume(const std::vector<Tri>& tris) {
        for (const Tri& f : tris) {
            const float nx = std::fabs(f.n[0]), ny = std::fabs(f.n[1]), nz = std::fabs(f.n[2]);
            if (ny < nx && nz < nx) {
                C = 0, A = 1, B = 2;
            } else if (nz < ny) {
                C = 1, A = 2, B = 0;
            } else {
                C = 2, A = 0, B = 1;
            }
            face(f);
            T1[A] = T1[A] + f.n[A] * Faa;
            TP[A] = TP[A] + f.n[A] * Faab;
            T2[A] = T2[A] + f.n[A] * Faaa;
            T1[B] = T1[B] + f.n[B] * Fbb;
            TP[B] = TP[B] + f.n[B] * Fbbc;
            T2[B] = T2[B] + f.n[B] * Fbbb;
            T0 = T0 + f.n[0] * (A == 0 ? Fa : (B == 0 ? Fb : Fc));
            T1[C] = T1[C] + f.n[C] * Fcc;
            T2[C] = T2[C] + f.n[C] * Fccc;
            TP[C] = TP[C] + f.n[C] * Fcca;
        }
        for (int k = 0; k < 3; ++k) {
            T1[k] = T1[k] * 0.5f;
            T2[k] = T2[k] / 3.0f;
            TP[k] = TP[k] * 0.5f;
        }
    }
};

}  // namespace

namespace {
// Positions and triangles (normal, plane offset) as the original's readPolyhedron builds them.
void read_polyhedron(const Polyhedron& poly, std::vector<Vec3>& pos, std::vector<Tri>& tris);
}  // namespace

VolumeIntegrals volume_integrals(const Polyhedron& poly) {
    std::vector<Vec3> pos;
    std::vector<Tri> tris;
    read_polyhedron(poly, pos, tris);
    Integrals in{pos};
    in.volume(tris);
    VolumeIntegrals r;
    r.T0 = in.T0;
    for (int k = 0; k < 3; ++k) r.T1[k] = in.T1[k], r.T2[k] = in.T2[k], r.TP[k] = in.TP[k];
    const float P[10] = {in.Pbbb, in.Pabb, in.Paab, in.Paaa, in.Pbb, in.Pab, in.Paa, in.Pb, in.Pa, in.P1};
    const float F[12] = {in.Fa, in.Fb, in.Fc, in.Faa, in.Fbb, in.Fcc, in.Faaa, in.Fbbb, in.Fccc, in.Faab, in.Fbbc, in.Fcca};
    for (int k = 0; k < 10; ++k) r.P[k] = P[k];
    for (int k = 0; k < 12; ++k) r.F[k] = F[k];
    return r;
}

namespace {
void read_polyhedron(const Polyhedron& poly, std::vector<Vec3>& pos, std::vector<Tri>& tris) {
    pos.reserve(poly.vertices.size());
    for (const auto& v : poly.vertices) pos.push_back(v.position);
    for (size_t i = 0; i + 2 < poly.triangles.size(); i += 3) {
        Tri f;
        f.v[0] = poly.triangles[i], f.v[1] = poly.triangles[i + 1], f.v[2] = poly.triangles[i + 2];
        const Vec3 p0 = pos[f.v[0]], p1 = pos[f.v[1]], p2 = pos[f.v[2]];
        const Vec3 e1{p1.x - p0.x, p1.y - p0.y, p1.z - p0.z};
        const Vec3 e2{p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
        const float ny = e1.z * e2.x - e2.z * e1.x;
        const float nx = e1.y * e2.z - e2.y * e1.z;
        const float nz = e1.x * e2.y - e2.x * e1.y;
        const float len = std::sqrt(ny * ny + nx * nx + nz * nz);
        float mx, my, mz;  // -n.x, n.y, n.z as the plane offset uses them
        if (0 < len) {
            f.n[0] = nx / len, f.n[1] = ny / len, f.n[2] = nz / len;
            mx = -(nx / len), my = ny / len, mz = nz / len;
        } else {
            f.n[0] = 0, f.n[1] = 0, f.n[2] = 1;
            mx = 0, my = 0, mz = 1;
        }
        f.w = (mx * p0.x - my * p0.y) - mz * p0.z;
        tris.push_back(f);
    }
}
}  // namespace

PolyMass polyhedral_mass(const Polyhedron& poly, float density) {
    std::vector<Vec3> pos;
    std::vector<Tri> tris;
    read_polyhedron(poly, pos, tris);
    Integrals in{pos};
    in.volume(tris);

    PolyMass r;
    const float t0 = in.T0;
    const float rx = in.T1[0] / t0, ry = in.T1[1] / t0, rz = in.T1[2] / t0;
    r.mass = density * t0;
    r.com = {rx, ry, rz};
    const float m = r.mass, nd = -density;
    float* J = r.inertia;
    J[0] = (in.T2[1] + in.T2[2]) * density - m * (ry * ry + rz * rz);
    // Products grouped as in the machine code: r_a * (r_b * m).
    const float jxy = ry * (rx * m) + nd * in.TP[0];
    J[1] = J[3] = jxy;
    const float jyz = rz * (ry * m) + nd * in.TP[1];
    J[5] = J[7] = jyz;
    const float jzx = rx * (rz * m) + nd * in.TP[2];
    J[2] = J[6] = jzx;
    J[4] = (in.T2[2] + in.T2[0]) * density - m * (rz * rz + rx * rx);
    J[8] = (in.T2[1] + in.T2[0]) * density - m * (ry * ry + rx * rx);
    return r;
}

void diagonalize(float m[9], float rot[9], float threshold, int max_steps) {
    for (int i = 0; i < 9; ++i) rot[i] = (i % 4 == 0) ? 1.0f : 0.0f;
    int steps = max_steps;
    while (0 < steps) {
        // Largest off-diagonal element (p, q); r is the remaining axis.
        int p = 0, q = 1, r = 2;
        float mx = std::fabs(m[1]);
        if (mx < std::fabs(m[2])) p = 0, q = 2, r = 1, mx = std::fabs(m[2]);
        if (mx < std::fabs(m[5])) p = 1, q = 2, r = 0, mx = std::fabs(m[5]);
        const float t = (std::fabs(m[0]) + std::fabs(m[4]) + std::fabs(m[8])) * threshold;
        int next;
        if (mx <= t) {
            if (mx <= t * 1e-5f) return;
            next = 0;
        } else {
            next = steps - 1;
        }
        const int pq = p * 3 + q, qp = q * 3 + p, pp = p * 4, qq = q * 4;
        const int rp = r * 3 + p, rq = r * 3 + q, pr = p * 3 + r, qr = q * 3 + r;
        const float mpq = m[pq];
        float theta = (m[qq] - m[pp]) / (mpq + mpq);
        const float theta2 = theta * theta;
        float tt, c, s;
        if (1e6f <= theta2 * theta2) {
            tt = 1.0f / ((0.5f / theta2 + 2.0f) * theta);
            c = 1.0f - tt * 0.5f * tt;
            s = c * tt;
        } else {
            theta = theta < 0 ? theta - std::sqrt(theta2 + 1.0f) : theta + std::sqrt(theta2 + 1.0f);
            tt = 1.0f / theta;
            c = 1.0f / std::sqrt(tt * tt + 1.0f);
            s = tt * c;
        }
        m[qp] = 0;
        m[pq] = 0;
        m[pp] = m[pp] - tt * mpq;
        m[qq] = m[qq] + tt * mpq;
        const float mrq = m[rq], mrp = m[rp];
        const float a = c * mrp - s * mrq, b = s * mrp + c * mrq;
        m[pr] = a, m[rp] = a, m[qr] = b, m[rq] = b;
        for (int i = 0; i < 3; ++i) {
            const float vp = rot[i * 3 + p], vq = rot[i * 3 + q];
            rot[i * 3 + q] = c * vq - s * vp;
            rot[i * 3 + p] = s * vq + c * vp;
        }
        steps = next;
    }
}

Quat quat_from_matrix(const float m[9]) {
    float v[3];
    float w;
    const float trace = m[0] + m[4] + m[8];
    if (0 < trace) {
        const float s = std::sqrt(trace + 1.0f);
        const float f = 0.5f / s;
        w = s * 0.5f;
        v[0] = (m[7] - m[5]) * f;
        v[1] = (m[2] - m[6]) * f;
        v[2] = (m[3] - m[1]) * f;
    } else {
        // Largest diagonal element i, with the original's index tables.
        int i, j, k, wa, wb, ja, jb, ka, kb;
        float d0, d1, d2;  // sqrt(((d0 - d1) - d2) + 1)
        if (m[4] <= m[0] && !(m[0] < m[8])) {
            i = 0, j = 1, k = 2, wa = 7, wb = 5, ja = 3, jb = 1, ka = 6, kb = 2;
            d0 = m[0], d1 = m[4], d2 = m[8];
        } else if (m[0] < m[4] && m[8] <= m[4]) {
            i = 1, j = 2, k = 0, wa = 2, wb = 6, ja = 7, jb = 5, ka = 1, kb = 3;
            d0 = m[4], d1 = m[8], d2 = m[0];
        } else {
            i = 2, j = 0, k = 1, wa = 3, wb = 1, ja = 2, jb = 6, ka = 5, kb = 7;
            d0 = m[8], d1 = m[0], d2 = m[4];
        }
        const float s = std::sqrt(((d0 - d1) - d2) + 1.0f);
        const float f = 0.5f / s;
        w = (m[wa] - m[wb]) * f;
        v[i] = s * 0.5f;
        v[j] = (m[ja] + m[jb]) * f;
        v[k] = (m[ka] + m[kb]) * f;
    }
    return {-v[0], -v[1], -v[2], w};
}

Vec3 equivalent_box_size(float i0, float i1, float i2, float mass) {
    const float a = (i0 * 12.0f) / mass;
    const float b = (i1 * 12.0f) / mass;
    const float z = (((i2 * 12.0f) / mass - a) + b) * 0.5f;
    const float x2 = 0 < z ? z : 0.0f;
    const float y = (a - b) + z;
    const float y2 = 0 < y ? y : 0.0f;
    const float z2 = 0 < b - z ? b - z : 0.0f;
    return {std::sqrt(x2), std::sqrt(y2), std::sqrt(z2)};
}

}  // namespace sh::phys
