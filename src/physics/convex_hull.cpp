#include "physics/convex_hull.hpp"

#include <cfloat>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace sh::phys {

namespace {

// Working state of one generate call (the original's THull).
class Builder {
public:
    static constexpr long kMaxFlips = 1000000;
    struct Vertex {
        std::int16_t point;  // index into the input points
        std::int16_t edge;   // bookkeeping only; never read back
    };
    struct Edge {  // half-edge; vertex is its origin
        std::int16_t vertex, next, face, twin;
    };
    struct Face {
        bool done;  // no remaining candidate lies above it
        std::int16_t edge;
        float nx, ny, nz, d;  // plane n.p + d = 0
    };
    struct Candidate {
        Vec3 p;
        int index;
    };

    std::vector<Vec3> pts;
    std::vector<Vertex> vertices;
    std::vector<Edge> edges;
    std::vector<Face> faces;
    std::vector<Candidate> candidates;  // points not on the hull yet
    std::vector<int> face_stack, edge_stack;
    long flips = 0;

    Builder(const Vec3* p, int n) : pts(p, p + n) {
        for (int i = 3; i < n; ++i) candidates.push_back({p[i], i});
        vertices = {{0, 0}, {1, 1}, {2, 2}};
        // Triangle 0-1-2 and its back side 0-2-1.
        edges = {{0, 1, 0, 5}, {1, 2, 0, 4}, {2, 0, 0, 3}, {0, 4, 1, 2}, {2, 5, 1, 1}, {1, 3, 1, 0}};
        faces = {{false, 0, 0, 1, 0, 0}, {false, 3, 0, 1, 0, 0}};
        setup_face(0);
        setup_face(1);
    }

    const Vec3& point_of(int edge) const { return pts[vertices[edges[edge].vertex].point]; }

    // Plane through the face's first three vertices; queues the face for a candidate search.
    void setup_face(int f) {
        Face& face = faces[f];
        const int e0 = face.edge, e1 = edges[e0].next, e2 = edges[e1].next;
        const Vec3 p0 = point_of(e0), p1 = point_of(e1), p2 = point_of(e2);
        const float ax = p1.x - p0.x, ay = p1.y - p0.y, az = p1.z - p0.z;
        const float bx = p2.x - p0.x, by = p2.y - p0.y, bz = p2.z - p0.z;
        const float ny = az * bx - ax * bz;
        const float nx = ay * bz - az * by;
        const float nz = ax * by - ay * bx;
        const float len = std::sqrt((ny * ny + nx * nx) + nz * nz);
        if (0 < len) {
            face.nx = nx / len, face.ny = ny / len, face.nz = nz / len;
        } else {
            face.nx = 1, face.ny = 0, face.nz = 0;
        }
        face.d = ((-face.nx) * p0.x - face.ny * p0.y) - face.nz * p0.z;
        face.done = false;
        face_stack.push_back(f);
    }

    static void push_unique(std::vector<int>& stack, int v) {
        for (int s : stack)
            if (s == v) return;
        stack.push_back(v);
    }

    // Insert candidate point p into face f: the face becomes three triangles around the new vertex.
    void expand(int f, int p) {
        const int e0 = faces[f].edge, e1 = edges[e0].next, e2 = edges[e1].next;
        const auto nv = static_cast<std::int16_t>(vertices.size());
        const auto E = static_cast<std::int16_t>(edges.size());
        vertices.push_back({static_cast<std::int16_t>(p), E});
        const auto f1 = static_cast<std::int16_t>(faces.size()), f2 = static_cast<std::int16_t>(f1 + 1);
        faces.push_back({false, 0, 0, 1, 0, 0});
        faces.push_back({false, 0, 0, 1, 0, 0});
        const std::int16_t v0 = edges[e0].vertex, v1 = edges[e1].vertex, v2 = edges[e2].vertex;
        const auto ff = static_cast<std::int16_t>(f);
        faces[f].edge = static_cast<std::int16_t>(e0);
        edges[e0].next = E, edges[e0].face = ff;
        faces[f1].edge = static_cast<std::int16_t>(e1);
        edges[e1].next = static_cast<std::int16_t>(E + 2), edges[e1].face = f1;
        faces[f2].edge = static_cast<std::int16_t>(e2);
        edges[e2].next = static_cast<std::int16_t>(E + 4), edges[e2].face = f2;
        const auto s = [](int v) { return static_cast<std::int16_t>(v); };
        edges.push_back({v1, s(E + 1), ff, s(E + 3)});
        edges.push_back({nv, s(e0), ff, s(E + 4)});
        edges.push_back({v2, s(E + 3), f1, s(E + 5)});
        edges.push_back({nv, s(e1), f1, E});
        edges.push_back({v0, s(E + 5), f2, s(E + 1)});
        edges.push_back({nv, s(e2), f2, s(E + 2)});
        // The point leaves the candidate list (the last candidate takes its slot).
        for (auto& c : candidates) {
            if (c.index == p) {
                c = candidates.back();
                candidates.pop_back();
                break;
            }
        }
        setup_face(f);
        setup_face(f1);
        setup_face(f2);
        push_unique(edge_stack, e0);
        push_unique(edge_stack, e1);
        push_unique(edge_stack, e2);
    }

    // Flip queued edges whose neighbouring triangle's far vertex lies above the edge's face.
    void flip_edges() {
        while (!edge_stack.empty()) {
            const int e = edge_stack.back();
            edge_stack.pop_back();
            const int t = edges[e].twin, f = edges[e].face;
            const int a = edges[t].next, b = edges[a].next;
            const std::int16_t v = edges[b].vertex;
            const Vec3 p = pts[vertices[v].point];
            const Face& pl = faces[f];
            if (!(FLT_EPSILON < ((p.y * pl.ny + p.x * pl.nx) + p.z * pl.nz) + pl.d)) continue;
            // Some clouds make the flips cycle forever; the original then grows its face queue until
            // memory runs out (observed under emulation). Stop with an error instead.
            if (++flips > kMaxFlips) throw std::runtime_error("ConvexHull: edge flips do not converge");
            const int c = edges[e].next, g = edges[t].face, dd = edges[c].next;
            const std::int16_t w = edges[dd].vertex;
            const auto s = [](int x) { return static_cast<std::int16_t>(x); };
            faces[f].edge = s(e);
            edges[e].next = s(dd), edges[e].vertex = v;
            edges[dd].face = s(f), edges[dd].next = s(a);
            edges[a].next = s(e), edges[a].face = s(f);
            faces[g].edge = s(t);
            edges[t].vertex = w, edges[t].face = s(g), edges[t].next = s(b);
            edges[b].next = s(c), edges[b].face = s(g);
            edges[c].next = s(t), edges[c].face = s(g);
            setup_face(f);
            setup_face(g);
            push_unique(edge_stack, dd);
            push_unique(edge_stack, a);
            push_unique(edge_stack, c);
            push_unique(edge_stack, b);
        }
    }

    void run() {
        constexpr float kEpsilon = 0.001f;
        for (;;) {
            int f = -1, best = -1;
            while (!face_stack.empty()) {
                const int k = face_stack.back();
                face_stack.pop_back();
                Face& face = faces[k];
                if (face.done) continue;
                float best_d = 0;
                for (const Candidate& c : candidates) {
                    const float d = ((face.ny * c.p.y + face.nx * c.p.x) + face.nz * c.p.z) + face.d;
                    if (best_d < d && kEpsilon < d) best_d = d, best = c.index;
                }
                if (best != -1) {
                    f = k;
                    break;
                }
                face.done = true;
            }
            if (f == -1) return;
            expand(f, best);
            flip_edges();
        }
    }
};

}  // namespace

void ConvexHull::generate(const Vec3* pts, int n) {
    if (n < 3) throw std::invalid_argument("ConvexHull needs at least three points");
    Builder b(pts, n);
    b.run();
    std::vector<int> remap(n);
    for (const Builder::Vertex& v : b.vertices) {
        remap[v.point] = static_cast<int>(points.size());
        points.push_back(pts[v.point]);
    }
    for (const Builder::Face& f : b.faces) {
        const int e0 = f.edge, e1 = b.edges[e0].next, e2 = b.edges[e1].next;
        faces.push_back(remap[b.vertices[b.edges[e0].vertex].point]);
        faces.push_back(remap[b.vertices[b.edges[e1].vertex].point]);
        faces.push_back(remap[b.vertices[b.edges[e2].vertex].point]);
    }
}

}  // namespace sh::phys
