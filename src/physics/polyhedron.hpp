#pragma once

// The original's Polyhedron: a half-edge mesh that every body shape
// carries. Its vertices (in order) become the shape's collision hull, its triangles feed the mass
// properties, and breakage later cuts it.

#include "physics/pmath.hpp"

#include <cstdint>
#include <vector>

namespace sh::phys {

class Polyhedron {
public:
    struct Vertex {
        Vec3 position;
        Vec3 normal;  // normalised sum of the adjacent face normals
    };
    struct Edge {  // half-edge
        std::int16_t vertex = 0;
        std::int16_t next = 0;  // next half-edge around the same face
        std::int16_t face = 0;
        std::int16_t twin = 0;  // opposite half-edge
    };
    struct Face {
        std::int16_t edge = 0;  // first half-edge
        Vec3 normal;
        std::uint8_t flag = 0;
    };

    std::vector<Vertex> vertices;
    std::vector<Edge> edges;
    std::vector<Face> faces;
    std::vector<std::int16_t> triangles;  // fan triangulation of each face, 3 indices per triangle

    // Polygons as vertex indices separated by -1. Appends to what is already there, then links
    // twins, computes normals and triangulates. `normals` (optional) seeds the vertex normals.
    void set_polygons(const Vec3* points, int point_count, const int* indices, int index_count,
                      const Vec3* normals = nullptr);
    // Triangles (3 indices each). Appends like set_polygons; vertex normals always recomputed.
    void set_triangles(const Vec3* points, int point_count, const int* indices, int triangle_count);

    // Polyhedron::split: cut by the plane n.p + d = 0 into the part with n.p + d >= 0 (above) and the
    // rest (below), each closed by a cap face (flag 1, normal -n above and n below). Vertex and face
    // order follow the original; edges crossing the plane get a vertex at t clamped to [0.001, 0.999].
    void split(Vec3 n, float d, Polyhedron& above, Polyhedron& below) const;

    void connect_edges();
    void compute_normals(bool vertex_normals);
    void triangulate();
};

}  // namespace sh::phys
