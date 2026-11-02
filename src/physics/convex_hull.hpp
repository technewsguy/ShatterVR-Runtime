#pragma once

// The original's QiConvexHull: an incremental hull that starts from the
// first three points as a two-sided triangle, inserts the farthest point above a face by splitting
// that face into three, and then flips edges until the surface is convex again. Bodies use it for
// bevelled boxes and convex shapes; the output order matters because it becomes the polyhedron's
// vertex order (and so the GJK support order).

#include "physics/pmath.hpp"

#include <vector>

namespace sh::phys {

class ConvexHull {
public:
    std::vector<Vec3> points;  // hull vertices, in the order they joined the hull
    std::vector<int> faces;    // triangles, 3 indices into points each

    ConvexHull() = default;
    ConvexHull(const Vec3* pts, int n) { generate(pts, n); }

    // Appends the hull of pts to points/faces (the original does not clear them first).
    // Needs n >= 3; the first three points seed the hull. The result is only a true convex hull for
    // well-behaved inputs (boxes, bevelled boxes, points all on the hull); general clouds can come
    // out folded, exactly as in the original. Throws std::runtime_error on clouds where the
    // original never terminates.
    void generate(const Vec3* pts, int n);

    int face_count() const { return static_cast<int>(faces.size()) / 3; }
};

}  // namespace sh::phys
