#include "physics/polyhedron.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace sh::phys {

void Polyhedron::set_polygons(const Vec3* points, int point_count, const int* indices, int index_count,
                              const Vec3* normals) {
    for (int i = 0; i < point_count; ++i) vertices.push_back({points[i], normals ? normals[i] : Vec3{}});
    int start = 0;  // first half-edge of the polygon being read (the original starts at 0)
    for (int k = 0; k < index_count; ++k) {
        const int idx = indices[k];
        if (static_cast<std::int16_t>(idx) == -1) {
            edges.back().next = static_cast<std::int16_t>(start);
            faces.push_back({static_cast<std::int16_t>(start), {}, 0});
            start = static_cast<int>(edges.size());
        } else {
            Edge e;
            e.vertex = static_cast<std::int16_t>(idx);
            e.next = static_cast<std::int16_t>(edges.size() + 1);
            e.face = static_cast<std::int16_t>(faces.size());
            edges.push_back(e);
        }
    }
    connect_edges();
    compute_normals(normals == nullptr);
    triangulate();
}

void Polyhedron::set_triangles(const Vec3* points, int point_count, const int* indices, int triangle_count) {
    for (int i = 0; i < point_count; ++i) vertices.push_back({points[i], {}});
    for (int k = 0; k < triangle_count; ++k) {
        const auto e = static_cast<std::int16_t>(edges.size()), f = static_cast<std::int16_t>(faces.size());
        faces.push_back({e, {}, 0});
        for (int i = 0; i < 3; ++i) {
            Edge edge;
            edge.vertex = static_cast<std::int16_t>(indices[3 * k + i]);
            edge.next = static_cast<std::int16_t>(e + (i + 1) % 3);
            edge.face = f;
            edges.push_back(edge);
        }
    }
    connect_edges();
    compute_normals(true);
    triangulate();
}

namespace {

// The original's in-place quicksort on (key, edge) pairs: first element as pivot, a "hole" moved
// between a right scan (stops below the pivot) and a left scan (stops above it), and an explicit
// range stack with the smaller side on top. Its order among equal keys decides which half-edges
// become twins when an edge occurs more than twice (hulls can produce that).
void sort_edge_keys(std::vector<std::pair<int, int>>& a) {
    std::vector<int> lo{0}, hi{static_cast<int>(a.size())};
    while (!lo.empty()) {
        const int L = lo.back(), H = hi.back();
        if (H - 1 <= L) {
            lo.pop_back(), hi.pop_back();
            continue;
        }
        int i = L, j = H - 1;
        const std::pair<int, int> pivot = a[i];
        for (;;) {
            bool place = false;
            while (!(a[j].first < pivot.first)) {
                if (i >= j) {
                    place = true;
                    break;
                }
                --j;
            }
            if (!place && i < j) {
                a[i] = a[j];
                ++i;
                while (!(pivot.first < a[i].first)) {
                    if (i >= j) {
                        place = true;
                        break;
                    }
                    ++i;
                }
            }
            if (place || j <= i) break;
            a[j] = a[i];
            --j;
            if (i >= j) break;
        }
        a[i] = pivot;
        // [L, i) stays in this slot, [i + 1, H) is pushed; the larger range ends up below.
        hi.back() = i;
        lo.push_back(i + 1), hi.push_back(H);
        const size_t t = lo.size() - 1;
        if (hi[t] - lo[t] > hi[t - 1] - lo[t - 1]) std::swap(lo[t], lo[t - 1]), std::swap(hi[t], hi[t - 1]);
    }
}

}  // namespace

void Polyhedron::connect_edges() {
    // Key each half-edge by its (sorted) vertex pair; the two half-edges with the same key are twins.
    std::vector<std::pair<int, int>> keys(edges.size());
    for (size_t i = 0; i < edges.size(); ++i) {
        const int a = edges[i].vertex, b = edges[edges[i].next].vertex;
        keys[i] = {std::max(a, b) | (std::min(a, b) << 16), static_cast<int>(i)};
    }
    sort_edge_keys(keys);
    for (size_t i = 0; i + 1 < keys.size(); i += 2) {
        edges[keys[i].second].twin = static_cast<std::int16_t>(keys[i + 1].second);
        edges[keys[i + 1].second].twin = static_cast<std::int16_t>(keys[i].second);
    }
}

void Polyhedron::split(Vec3 n, float d, Polyhedron& above, Polyhedron& below) const {
    above = Polyhedron{};
    below = Polyhedron{};
    Polyhedron* out[2] = {&above, &below};
    const int nv = static_cast<int>(vertices.size()), ne = static_cast<int>(edges.size());

    // Vertices go to their side (1: below the plane).
    std::vector<std::uint8_t> side(static_cast<size_t>(nv));
    std::vector<int> vmap[2] = {std::vector<int>(static_cast<size_t>(nv)), std::vector<int>(static_cast<size_t>(nv))};
    for (int v = 0; v < nv; ++v) {
        const Vec3 p = vertices[v].position;
        const int s = ((n.x * p.x + n.y * p.y) + n.z * p.z) + d < 0 ? 1 : 0;
        side[v] = static_cast<std::uint8_t>(s);
        vmap[s][v] = static_cast<int>(out[s]->vertices.size());
        vmap[1 - s][v] = -1;
        out[s]->vertices.push_back(vertices[v]);
    }

    // Each edge crossing the plane gets a new vertex in both halves (shared by the edge and its twin).
    std::vector<int> emap[2] = {std::vector<int>(static_cast<size_t>(ne), -1), std::vector<int>(static_cast<size_t>(ne), -1)};
    for (int e = 0; e < ne; ++e) {
        const int a = edges[e].vertex, b = edges[edges[e].next].vertex;
        if (a > b || side[a] == side[b]) continue;
        const Vertex &va = vertices[a], &vb = vertices[b];
        const Vec3 pa = va.position;
        const Vec3 dv{vb.position.x - pa.x, vb.position.y - pa.y, vb.position.z - pa.z};
        const float denom = (n.x * dv.x + n.y * dv.y) + n.z * dv.z;
        const float hit = denom == 0 ? 0.0f : (-((n.x * pa.x + n.y * pa.y) + n.z * pa.z) - d) / denom;  // QiIntersection
        float t = 0.001f, u = 0.999f;
        if (0.001f < hit) {
            if (hit < 0.999f) t = hit, u = 1.0f - hit;
            else t = 0.999f, u = 0.0009999871f;
        }
        const Vec3 m{t * vb.normal.x + u * va.normal.x, t * vb.normal.y + u * va.normal.y, t * vb.normal.z + u * va.normal.z};
        const float len = std::sqrt((m.y * m.y + m.x * m.x) + m.z * m.z);
        const Vec3 nrm = 0 < len ? Vec3{m.x / len, m.y / len, m.z / len} : Vec3{1, 0, 0};
        const Vertex cut{{pa.x + t * dv.x, pa.y + t * dv.y, pa.z + t * dv.z}, nrm};
        for (int s = 0; s < 2; ++s) {
            emap[s][edges[e].twin] = emap[s][e] = static_cast<int>(out[s]->vertices.size());
            out[s]->vertices.push_back(cut);
        }
    }

    // Faces: walk each face from its first vertex's side; a stretch on the other side becomes a face
    // there. Each crossing leaves an edge on the cut plane, collected for the caps.
    const auto add_edge = [](Polyhedron& p, int vertex, int face) {
        Edge e;
        e.vertex = static_cast<std::int16_t>(vertex);
        e.next = static_cast<std::int16_t>(p.edges.size() + 1);
        e.face = static_cast<std::int16_t>(face);
        p.edges.push_back(e);
    };
    std::vector<int> cap[2];
    for (const Face& f : faces) {
        const int e0 = f.edge;
        const int s = side[edges[e0].vertex];
        Polyhedron &P = *out[s], &Q = *out[1 - s];
        const int face_p = static_cast<int>(P.faces.size()), start_p = static_cast<int>(P.edges.size());
        P.faces.push_back({static_cast<std::int16_t>(start_p), f.normal, f.flag});
        int e = e0;
        for (;;) {
            add_edge(P, vmap[s][edges[e].vertex], face_p);
            if (emap[s][e] != -1) {
                cap[s].push_back(static_cast<int>(P.edges.size()));
                add_edge(P, emap[s][e], face_p);
                const int face_q = static_cast<int>(Q.faces.size()), start_q = static_cast<int>(Q.edges.size());
                Q.faces.push_back({static_cast<std::int16_t>(start_q), f.normal, f.flag});
                add_edge(Q, emap[1 - s][e], face_q);
                int e2 = e;
                do {
                    e2 = edges[e2].next;
                    add_edge(Q, vmap[1 - s][edges[e2].vertex], face_q);
                } while (emap[1 - s][e2] == -1);
                cap[1 - s].push_back(static_cast<int>(Q.edges.size()));
                add_edge(Q, emap[1 - s][e2], face_q);
                Q.edges.back().next = static_cast<std::int16_t>(start_q);
                add_edge(P, emap[s][e2], face_p);
                e = e2;
            }
            e = edges[e].next;
            if (e == e0) break;
        }
        P.edges.back().next = static_cast<std::int16_t>(start_p);
    }

    // Caps: chain the cut-plane edges into one face per half (reversed edges, flag 1).
    for (int i = 0; i < 2; ++i) {
        std::vector<int>& list = cap[i];
        if (list.empty()) continue;
        Polyhedron& P = *out[i];
        const int face = static_cast<int>(P.faces.size()), start = static_cast<int>(P.edges.size());
        P.faces.push_back({static_cast<std::int16_t>(start), i == 1 ? n : Vec3{-n.x, -n.y, -n.z}, 1});
        size_t j = 0;
        int c = list[0];
        for (;;) {
            const std::int16_t from = P.edges[c].vertex, to = P.edges[P.edges[c].next].vertex;
            P.edges[c].twin = static_cast<std::int16_t>(P.edges.size());
            Edge r;
            r.vertex = to;
            r.next = static_cast<std::int16_t>(P.edges.size() + 1);
            r.face = static_cast<std::int16_t>(face);
            r.twin = static_cast<std::int16_t>(c);
            P.edges.push_back(r);
            list[j] = list.back();
            list.pop_back();
            if (list.empty()) break;
            for (j = 0; j < list.size() && P.edges[P.edges[list[j]].next].vertex != from; ++j) {
            }
            if (j == list.size()) break;  // open loop: closed where it stands
            c = list[j];
        }
        P.edges.back().next = static_cast<std::int16_t>(start);
    }
    above.connect_edges();
    below.connect_edges();
    above.triangulate();
    below.triangulate();
}

void Polyhedron::compute_normals(bool vertex_normals) {
    for (Face& f : faces) {
        const Edge& e0 = edges[f.edge];
        const Edge& e1 = edges[e0.next];
        const Vec3 p0 = vertices[e0.vertex].position;
        const Vec3 p1 = vertices[e1.vertex].position;
        const Vec3 p2 = vertices[edges[e1.next].vertex].position;
        const Vec3 a = p1 - p0, b = p2 - p0;
        const Vec3 n{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
        const float len = std::sqrt(n.y * n.y + n.x * n.x + n.z * n.z);
        f.normal = 0 < len ? Vec3{n.x / len, n.y / len, n.z / len} : Vec3{1, 0, 0};
    }
    if (!vertex_normals) return;
    for (Vertex& v : vertices) v.normal = {};
    for (const Face& f : faces) {
        int e = f.edge;
        do {
            Vertex& v = vertices[edges[e].vertex];
            e = edges[e].next;
            v.normal = {v.normal.x + f.normal.x, v.normal.y + f.normal.y, v.normal.z + f.normal.z};
        } while (e != f.edge);
    }
    for (Vertex& v : vertices) {
        const Vec3 n = v.normal;
        const float len = std::sqrt(n.y * n.y + n.x * n.x + n.z * n.z);
        v.normal = 0 < len ? Vec3{n.x / len, n.y / len, n.z / len} : Vec3{1, 0, 0};
    }
}

void Polyhedron::triangulate() {
    triangles.clear();
    for (const Face& f : faces) {
        const std::int16_t v0 = edges[f.edge].vertex;
        int e = edges[f.edge].next;
        while (edges[e].next != f.edge) {
            triangles.push_back(v0);
            triangles.push_back(edges[e].vertex);
            triangles.push_back(edges[edges[e].next].vertex);
            e = edges[e].next;
        }
    }
}

}  // namespace sh::phys
