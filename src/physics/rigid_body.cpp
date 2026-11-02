#include "physics/rigid_body.hpp"

#include "physics/collision.hpp"
#include "physics/convex_hull.hpp"
#include "physics/mass_properties.hpp"
#include "physics/qi_rotate.hpp"

#include <cmath>
#include <stdexcept>

namespace sh::phys {

namespace {


// Column-major 4x4 matrix times point, the original's order.
Vec3 apply(const float m[16], Vec3 v) {
    return {((v.y * m[4] + v.x * m[0]) + v.z * m[8]) + m[12], ((v.y * m[5] + v.x * m[1]) + v.z * m[9]) + m[13],
            ((v.y * m[6] + v.x * m[2]) + v.z * m[10]) + m[14]};
}

// The same without the translation (directions).
Vec3 apply_rotation(const float m[16], Vec3 v) {
    return {(v.y * m[4] + v.x * m[0]) + v.z * m[8], (v.y * m[5] + v.x * m[1]) + v.z * m[9],
            (v.y * m[6] + v.x * m[2]) + v.z * m[10]};
}

// A bevel band normal as Body::addCylinder computes it: (-1, y, z) over its length; (1, 0, 0) if the
// length is not positive.
Vec3 band_normal(float y, float z) {
    const float len = std::sqrt((1.0f + y * y) + z * z);
    if (!(0 < len)) return {1, 0, 0};
    return {-1.0f / len, y / len, z / len};
}

// Body::addBox corner order and face list.
constexpr float kCorners[8][3] = {{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
                                  {-1, -1, 1},  {1, -1, 1},  {1, 1, 1},  {-1, 1, 1}};
constexpr int kBoxFaces[30] = {0, 3, 2, 1, -1, 0, 1, 5, 4, -1, 0, 4, 7, 3, -1,
                               1, 2, 6, 5, -1, 3, 7, 6, 2, -1, 4, 5, 6, 7, -1};

// The "thin" test on equivalent-box sizes, as the original's branches evaluate it.
bool is_thin(Vec3 b) {
    enum { kStart, k26d8, k26ec, k2958 } at = kStart;
    if (0.15f <= b.x || b.y <= 0.5f) {
        if (b.x <= 0.5f) {
            at = k26ec;
        } else if (b.y < 0.15f) {
            at = k26d8;
        } else {
            at = k2958;
        }
    } else {
        at = k26d8;
    }
    if (at == k26d8) {
        if (!(b.z <= 0.5f)) return true;
        at = k26ec;
    }
    if (at == k26ec) {
        if (!(0.5f < b.x)) return false;
        at = k2958;
    }
    return b.y <= 0.5f ? false : b.z < 0.15f;
}

}  // namespace

void BodyShape::compute_bounds() {
    const Transform T = Transform::from(body->solver.position, body->solver.rotation);
    Vec3 mn{FLT_MAX, FLT_MAX, FLT_MAX}, mx{-FLT_MAX, -FLT_MAX, -FLT_MAX};
    for (const Vec3& v : hull) {
        const Vec3 p = T.apply(v);
        mn = {mn.x < p.x ? mn.x : p.x, mn.y < p.y ? mn.y : p.y, mn.z < p.z ? mn.z : p.z};
        mx = {p.x < mx.x ? mx.x : p.x, p.y < mx.y ? mx.y : p.y, p.z < mx.z ? mx.z : p.z};
    }
    // Dynamic bodies sweep their box by one step of linear velocity, on the side they move to.
    Vec3 lo, hi;
    if (body->dynamic) {
        const float dt = body->world().time_step;
        const Vec3 s{dt * body->solver.linear_velocity.x, dt * body->solver.linear_velocity.y,
                     dt * body->solver.linear_velocity.z};
        lo = {s.x < 0 ? s.x : 0.0f, s.y < 0 ? s.y : 0.0f, s.z < 0 ? s.z : 0.0f};
        hi = {s.x <= 0 ? 0.0f : s.x, s.y <= 0 ? 0.0f : s.y, 0 < s.z ? s.z : 0.0f};
    }
    constexpr float kMargin = 0.02f;
    bounds_min = {(lo.x + mn.x) - kMargin, (lo.y + mn.y) - kMargin, (lo.z + mn.z) - kMargin};
    bounds_max = {hi.x + mx.x + kMargin, hi.y + mx.y + kMargin, hi.z + mx.z + kMargin};
    AabbTree& tree = in_tree_a ? body->world().tree_a : body->world().tree_b;
    if (tree_leaf >= 0) {
        tree.update(tree_leaf, bounds_min, bounds_max);
    } else {
        tree_leaf = tree.create(bounds_min, bounds_max, this);
    }
}

void RigidBody::compute_bounds() {
    for (auto& s : shapes) s->compute_bounds();
    // The original reads only each shape's min corner here: the "max" it stores is the larger of
    // the last shape's min and the running min, i.e. the last shape's min corner. Kept as observed.
    Vec3 mn{FLT_MAX, FLT_MAX, FLT_MAX}, mx{-FLT_MAX, -FLT_MAX, -FLT_MAX};
    for (const auto& s : shapes) {
        const Vec3 a = s->bounds_min;
        mn = {mn.x < a.x ? mn.x : a.x, mn.y < a.y ? mn.y : a.y, mn.z < a.z ? mn.z : a.z};
        mx = {a.x < mn.x ? mn.x : a.x, a.y < mn.y ? mn.y : a.y, a.z < mn.z ? mn.z : a.z};
    }
    bounds_min = mn;
    bounds_max = mx;
}

void RigidBody::solver_pose_from_entity() {
    const Quat q = rotation, c = com_rotation;
    solver.rotation = {(q.y * c.z - q.z * c.y) + q.w * c.x + q.x * c.w, (q.z * c.x - q.x * c.z) + q.w * c.y + q.y * c.w,
                       (q.x * c.y - q.y * c.x) + q.w * c.z + q.z * c.w, q.w * c.w - (q.y * c.y + q.x * c.x + q.z * c.z)};
    const Vec3 p = com_position, e = position;
    const float k2w = q.w + q.w, kk = q.w * k2w - 1.0f;
    const float dot = q.y * p.y + q.x * p.x + q.z * p.z;
    const float dot2 = dot + dot;
    // The entity position is added last on every axis (the machine code's grouping).
    solver.position = {(k2w * (q.y * p.z - q.z * p.y) + p.x * kk + q.x * dot2) + e.x,
                       (k2w * (q.z * p.x - q.x * p.z) + p.y * kk + q.y * dot2) + e.y,
                       (k2w * (q.x * p.y - q.y * p.x) + p.z * kk + q.z * dot2) + e.z};
}

void RigidBody::set_transform(Vec3 pos, Quat rot) {
    position = pos;
    rotation = rot;
    solver_pose_from_entity();
    compute_bounds();
}

BodyShape& RigidBody::add_box(const float m[16], Vec3 h, const Material* material, float bevel) {
    auto shape = std::make_unique<BodyShape>();
    if (material) shape->material = *material;
    if (bevel == 0) {
        Vec3 corners[8];
        for (int i = 0; i < 8; ++i)
            corners[i] = apply(m, {h.x * kCorners[i][0], h.y * kCorners[i][1], h.z * kCorners[i][2]});
        shape->poly.set_polygons(corners, 8, kBoxFaces, 30);
    } else {
        // Each corner (z outermost, then y, then x) becomes three points, each pulled in by the
        // bevel along two axes: (x, y), (y, z), (x, z). The hull of the 24 points is the shape.
        Vec3 pts[24];
        int n = 0;
        for (int sz = -1; sz != 3; sz += 2) {
            for (int sy = -1; sy != 3; sy += 2) {
                for (int sx = -1; sx != 3; sx += 2) {
                    const float fx = static_cast<float>(sx), fy = static_cast<float>(sy), fz = static_cast<float>(sz);
                    pts[n++] = apply(m, {h.x * fx - bevel * fx, h.y * fy - bevel * fy, h.z * fz});
                    pts[n++] = apply(m, {fx * h.x, fy * h.y - bevel * fy, fz * h.z - bevel * fz});
                    pts[n++] = apply(m, {fx * h.x - bevel * fx, fy * h.y, fz * h.z - bevel * fz});
                }
            }
        }
        const ConvexHull hull(pts, 24);
        shape->poly.set_triangles(hull.points.data(), static_cast<int>(hull.points.size()), hull.faces.data(),
                                  hull.face_count());
    }
    shape->body = this;
    shape->serial = world_->next_shape_serial++;
    shapes.push_back(std::move(shape));
    return *shapes.back();
}

BodyShape& RigidBody::add_convex(const float m[16], const ConvexHull& hull, const Material* material) {
    std::vector<Vec3> pts;
    pts.reserve(hull.points.size());
    for (const Vec3& p : hull.points) pts.push_back(apply(m, p));
    auto shape = std::make_unique<BodyShape>();
    shape->poly.set_triangles(pts.data(), static_cast<int>(pts.size()), hull.faces.data(), hull.face_count());
    if (material) shape->material = *material;
    shape->body = this;
    shape->serial = world_->next_shape_serial++;
    shapes.push_back(std::move(shape));
    return *shapes.back();
}

BodyShape& RigidBody::add_cylinder(const float m[16], float radius, float half_length, const Material* material,
                                   float bevel) {
    constexpr int kSegments = 24;
    float cs[kSegments], sn[kSegments];
    for (int i = 0; i < kSegments; ++i) {
        if (i == 0) {
            cs[i] = 1, sn[i] = 0;
            continue;
        }
        const float a = (static_cast<float>(i) * 6.28318548f) / 24.0f;
        cs[i] = std::cos(a), sn[i] = std::sin(a);
    }
    const float h = half_length, r = radius - bevel;
    std::vector<Vec3> pts, normals;
    std::vector<int> idx;
    auto add = [&](Vec3 p, Vec3 n) {
        pts.push_back(apply(m, p));
        normals.push_back(apply_rotation(m, n));
    };
    // Bands of quads between consecutive point pairs, starting at index `base`.
    auto band_indices = [&](int base) {
        for (int i = 0; i < kSegments; ++i) {
            const int b = base + 2 * i, n = base + 2 * ((i + 1) % kSegments);
            idx.insert(idx.end(), {b + 1, b, n, n + 1, -1});
        }
    };
    // The caps (radius - bevel) at x = -h and x = +h; the axis is the matrix's x.
    for (int i = 0; i < kSegments; ++i) add({-h, -(cs[i] * r), r * sn[i]}, {-1, 0, 0}), idx.push_back(i);
    idx.push_back(-1);
    for (int i = 0; i < kSegments; ++i) add({h, r * cs[i], r * sn[i]}, {1, 0, 0}), idx.push_back(kSegments + i);
    idx.push_back(-1);
    // The side between x = +-(h - bevel), full radius. Its normals are not unit (length = radius).
    int base = static_cast<int>(pts.size());
    for (int i = 0; i < kSegments; ++i) {
        const float y = radius * cs[i], z = radius * sn[i];
        add({bevel - h, y, z}, {0, y, z});
        add({h - bevel, y, z}, {0, y, z});
    }
    band_indices(base);
    if (0 < bevel) {
        // Bevel bands from each cap rim to the side. Both bands' normals point to -x (as in the original).
        base = static_cast<int>(pts.size());
        for (int i = 0; i < kSegments; ++i) {
            const float y0 = r * cs[i], z0 = r * sn[i], y1 = radius * cs[i], z1 = radius * sn[i];
            add({-h, y0, z0}, band_normal(y0, z0));
            add({bevel - h, y1, z1}, band_normal(y1, z1));
        }
        band_indices(base);
        base = static_cast<int>(pts.size());
        for (int i = 0; i < kSegments; ++i) {
            const float y0 = radius * cs[i], z0 = radius * sn[i], y1 = r * cs[i], z1 = r * sn[i];
            add({h - bevel, y0, z0}, band_normal(y0, z0));
            add({h, y1, z1}, band_normal(y1, z1));
        }
        band_indices(base);
    }
    auto shape = std::make_unique<BodyShape>();
    if (material) shape->material = *material;
    if (shape->material.strength < FLT_MAX) {
        // Breakable: the convex hull of the points instead of the polygons.
        const ConvexHull hull(pts.data(), static_cast<int>(pts.size()));
        shape->poly.set_triangles(hull.points.data(), static_cast<int>(hull.points.size()), hull.faces.data(),
                                  hull.face_count());
    } else {
        shape->poly.set_polygons(pts.data(), static_cast<int>(pts.size()), idx.data(), static_cast<int>(idx.size()),
                                 normals.data());
        shape->material.roundness = 1.0f;
    }
    shape->body = this;
    shape->serial = world_->next_shape_serial++;
    shapes.push_back(std::move(shape));
    return *shapes.back();
}

void RigidBody::add_shape(std::unique_ptr<BodyShape> shape) {
    shape->body = this;
    shapes.push_back(std::move(shape));
}

std::unique_ptr<BodyShape> RigidBody::take_shape(const BodyShape* shape) {
    for (size_t i = 0; i < shapes.size(); ++i) {
        if (shapes[i].get() != shape) continue;
        std::unique_ptr<BodyShape> out = std::move(shapes[i]);
        if (i + 1 != shapes.size()) shapes[i] = std::move(shapes.back());
        shapes.pop_back();
        out->body = nullptr;
        return out;
    }
    return nullptr;
}

void RigidBody::compute_mass_properties() {
    if (shapes.empty()) throw std::runtime_error("compute_mass_properties on a body without shapes (not ported)");
    dynamic = true;
    bool infinite_inertia = false;
    for (const auto& s : shapes) {
        if (s->material.type == Material::kStatic) dynamic = false;
        if (s->material.type == Material::kInfiniteInertia) infinite_inertia = true;
    }

    if (!dynamic) {
        solver.inv_mass = 0;
        solver.inv_inertia = {};
        com_position = {};
        com_rotation = {0, 0, 0, 1};
        inv_com_position = {};
        inv_com_rotation = {0, 0, 0, 1};
    } else {
        // Recentre all polyhedra on their common vertex average (restored afterwards).
        float sx = 0, sy = 0, sz = 0;
        int n = 0;
        for (const auto& s : shapes) {
            for (const auto& v : s->poly.vertices) sx = sx + v.position.x, sy = sy + v.position.y, sz = sz + v.position.z;
            n += static_cast<int>(s->poly.vertices.size());
        }
        const float inv_n = 1.0f / static_cast<float>(n);
        const Vec3 c{sx * inv_n, sy * inv_n, sz * inv_n};
        for (auto& s : shapes)
            for (auto& v : s->poly.vertices) v.position = {v.position.x - c.x, v.position.y - c.y, v.position.z - c.z};

        struct Part {
            float mass;
            Vec3 com;
            float inertia[9];
        };
        std::vector<Part> parts;
        float M = 0, mx = 0, my = 0, mz = 0;
        for (auto& s : shapes) {
            const PolyMass pm = polyhedral_mass(s->poly, s->material.density);
            s->volume = pm.mass / s->material.density;
            float J[9], rot[9];
            for (int i = 0; i < 9; ++i) J[i] = pm.inertia[i];
            diagonalize(J, rot, 1e-4f, 16);
            const Vec3 box = equivalent_box_size(J[0], J[4], J[8], pm.mass);
            s->thin = is_thin(box);
            const Vec3 o = pm.com;
            s->box_axes[0] = {o.x + box.x * rot[0], o.y + box.x * rot[1], o.z + box.x * rot[2]};
            s->box_axes[1] = {o.x + box.y * rot[3], o.y + box.y * rot[4], o.z + box.y * rot[5]};
            s->box_axes[2] = {o.x + box.z * rot[6], o.y + box.z * rot[7], o.z + box.z * rot[8]};
            Part p{pm.mass, pm.com, {}};
            for (int i = 0; i < 9; ++i) p.inertia[i] = pm.inertia[i];
            parts.push_back(p);
            M = M + pm.mass;
            mx = mx + pm.mass * pm.com.x;
            my = my + pm.mass * pm.com.y;
            mz = mz + pm.mass * pm.com.z;
        }

        // Body inertia about the combined centre: each part's own inertia plus a parallel-axis term
        // (with positive products, as the original writes it).
        float I[9] = {};
        for (const Part& p : parts) {
            for (int i = 0; i < 9; ++i) I[i] = I[i] + p.inertia[i];
            const float dy = p.com.y - my / M, dz = p.com.z - mz / M, dx = p.com.x - mx / M;
            const float m = p.mass;
            float P[9];
            P[0] = m * (dy * dy + dz * dz);
            P[4] = m * (dz * dz + dx * dx);
            P[8] = m * (dy * dy + dx * dx);
            P[1] = P[3] = dx * m * dy;
            P[5] = P[7] = dy * m * dz;
            P[2] = P[6] = dz * m * dx;
            for (int i = 0; i < 9; ++i) I[i] = I[i] + P[i];
        }
        float rot[9];
        diagonalize(I, rot, 1e-4f, 16);
        equivalent_box = equivalent_box_size(I[0], I[4], I[8], M);

        for (auto& s : shapes)
            for (auto& v : s->poly.vertices) v.position = {v.position.x + c.x, v.position.y + c.y, v.position.z + c.z};

        com_rotation = quat_from_matrix(rot);
        com_position = {c.x + mx / M, c.y + my / M, c.z + mz / M};
        inv_com_position = qi_rotate_inverse(com_rotation, {-com_position.x, -com_position.y, -com_position.z});
        inv_com_rotation = {-com_rotation.x, -com_rotation.y, -com_rotation.z, com_rotation.w};
        solver.inv_mass = 1.0f / M;
        solver.inv_inertia = infinite_inertia ? Vec3{} : Vec3{1.0f / I[0], 1.0f / I[4], 1.0f / I[8]};
    }

    // Collision hulls: polyhedron vertices pulled in along their normals by 0.014, in the solver frame.
    for (auto& s : shapes) {
        s->hull.clear();
        for (const auto& v : s->poly.vertices) {
            const Vec3 d{(v.position.x - v.normal.x * 0.02f * 0.7f) - com_position.x,
                         (v.position.y - v.normal.y * 0.02f * 0.7f) - com_position.y,
                         (v.position.z - v.normal.z * 0.02f * 0.7f) - com_position.z};
            s->hull.push_back(qi_rotate_inverse(com_rotation, d));
        }
    }

    solver_pose_from_entity();
    if (dynamic && equivalent_box.x < 0.1f && equivalent_box.y < 0.1f && equivalent_box.z < 0.1f)
        for (auto& s : shapes) s->material.strength = FLT_MAX;  // tiny bodies never break
    compute_bounds();
}

}  // namespace sh::phys
