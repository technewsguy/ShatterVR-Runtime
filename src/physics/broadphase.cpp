#include "physics/broadphase.hpp"

#include <cmath>
#include <limits>

namespace sh::phys {

int AabbTree::alloc() {
    int i;
    if (free_ >= 0) {
        i = free_;
        free_ = nodes_[i].parent;
        nodes_[i] = Node{};
    } else {
        i = static_cast<int>(nodes_.size());
        nodes_.emplace_back();
    }
    return i;
}

void AabbTree::release(int i) {
    nodes_[i].parent = free_;
    free_ = i;
}

bool AabbTree::refit(int i) {
    Node& n = nodes_[i];
    const Node& c0 = nodes_[n.child[0]];
    const Node& c1 = nodes_[n.child[1]];
    const float m0 = margin_of(n.child[0]), m1 = margin_of(n.child[1]);
    auto lo = [&](float a, float b) { return a - m0 < b - m1 ? a - m0 : b - m1; };
    auto hi = [&](float a, float b) { return m1 + b < m0 + a ? m0 + a : m1 + b; };
    const Vec3 mn{lo(c0.min.x, c1.min.x), lo(c0.min.y, c1.min.y), lo(c0.min.z, c1.min.z)};
    const Vec3 mx{hi(c0.max.x, c1.max.x), hi(c0.max.y, c1.max.y), hi(c0.max.z, c1.max.z)};
    const bool same = mn.x == n.min.x && mn.y == n.min.y && mn.z == n.min.z && mx.x == n.max.x && mx.y == n.max.y &&
                      mx.z == n.max.z;
    n.min = mn;
    n.max = mx;
    return !same;
}

std::pair<int, int> AabbTree::remove(int leaf) {
    const int p = nodes_[leaf].parent;
    const int sibling = nodes_[p].child[0] == leaf ? nodes_[p].child[1] : nodes_[p].child[0];
    const int g = nodes_[p].parent;
    nodes_[leaf].parent = -1;
    if (g < 0) {
        nodes_[sibling].parent = -1;
        root_ = sibling;
        return {sibling, p};
    }
    Node& gn = nodes_[g];
    gn.child[gn.child[0] == p ? 0 : 1] = sibling;
    nodes_[sibling].parent = g;
    nodes_[p].parent = -1;
    // Refit upwards until a box no longer changes; reinsertion starts there.
    for (int i = g; i >= 0; i = nodes_[i].parent)
        if (!refit(i)) return {i, p};
    return {root_, p};
}

void AabbTree::insert(int start, int leaf, int parent_node) {
    const Node& l = nodes_[leaf];
    // Descend to the child whose box centre is nearer (L1, sums of min + max); ties go to child 1.
    const Vec3 c{l.max.x + l.min.x, l.max.y + l.min.y, l.max.z + l.min.z};
    auto cost = [&](int i) {
        const Node& n = nodes_[i];
        return std::fabs((n.min.x + n.max.x) - c.x) + std::fabs((n.min.y + n.max.y) - c.y) +
               std::fabs((n.min.z + n.max.z) - c.z);
    };
    int sibling = start;
    while (!nodes_[sibling].leaf()) {
        const int c0 = nodes_[sibling].child[0], c1 = nodes_[sibling].child[1];
        sibling = cost(c1) <= cost(c0) ? c1 : c0;
    }

    // New parent around the sibling and the leaf (both leaves, so both get the margin).
    Node& np = nodes_[parent_node];
    const Node& s = nodes_[sibling];
    const float e = margin_;
    auto lo = [&](float leaf_v, float sib_v) { return sib_v - e < leaf_v - e ? sib_v - e : leaf_v - e; };
    auto hi = [&](float leaf_v, float sib_v) { return e + leaf_v < e + sib_v ? e + sib_v : e + leaf_v; };
    np.min = {lo(l.min.x, s.min.x), lo(l.min.y, s.min.y), lo(l.min.z, s.min.z)};
    np.max = {hi(l.max.x, s.max.x), hi(l.max.y, s.max.y), hi(l.max.z, s.max.z)};

    const int up = s.parent;
    if (up < 0) {
        root_ = parent_node;
        np.parent = -1;
    } else {
        Node& un = nodes_[up];
        un.child[un.child[0] == sibling ? 0 : 1] = parent_node;
        np.parent = up;
        // Refit ancestors until one already contains the new leaf's (tight) box.
        for (int i = up; i >= 0; i = nodes_[i].parent) {
            const Node& a = nodes_[i];
            if (a.min.x <= l.min.x && a.min.y <= l.min.y && a.min.z <= l.min.z && l.max.x <= a.max.x &&
                l.max.y <= a.max.y && l.max.z <= a.max.z)
                break;
            refit(i);
        }
    }
    np.child[0] = sibling;
    np.child[1] = leaf;
    nodes_[sibling].parent = parent_node;
    nodes_[leaf].parent = parent_node;
}

int AabbTree::create(Vec3 min, Vec3 max, void* user) {
    const int leaf = alloc();
    Node& l = nodes_[leaf];
    l.min = min;
    l.max = max;
    l.user = user;
    if (root_ < 0) {
        root_ = leaf;
    } else {
        const int p = alloc();
        insert(root_, leaf, p);
    }
    return leaf;
}

void AabbTree::update(int leaf, Vec3 min, Vec3 max) {
    if (leaf != root_) {
        const Node& p = nodes_[nodes_[leaf].parent];
        if (!(p.min.x <= min.x && p.min.y <= min.y && p.min.z <= min.z && max.x <= p.max.x && max.y <= p.max.y &&
              max.z <= p.max.z)) {
            const auto [start, freed] = remove(leaf);
            nodes_[leaf].min = min;
            nodes_[leaf].max = max;
            insert(start, leaf, freed);
            return;
        }
    }
    nodes_[leaf].min = min;
    nodes_[leaf].max = max;
}

void AabbTree::destroy(int leaf) {
    if (leaf == root_) {
        root_ = -1;
    } else {
        release(remove(leaf).second);
    }
    release(leaf);
}

void AabbTree::fetch_leaves(int i, std::vector<int>& out) const {
    if (nodes_[i].leaf()) {
        out.push_back(i);
        return;
    }
    fetch_leaves(nodes_[i].child[0], out);
    fetch_leaves(nodes_[i].child[1], out);
}

void AabbTree::optimize(int passes, Rand48& rng) {
    std::vector<int> leaves;
    for (int pass = 0; pass < passes; ++pass) {
        if (root_ < 0 || nodes_[root_].leaf()) continue;
        leaves.clear();
        fetch_leaves(root_, leaves);
        const int leaf = leaves[rng.rnd(0, static_cast<int>(leaves.size()))];
        const int freed = remove(leaf).second;
        insert(root_, leaf, freed);  // unlike update(), always from the root
    }
}

void AabbTree::query_aabb(Vec3 min, Vec3 max, std::vector<void*>& out) const {
    if (root_ < 0) return;
    std::vector<int> stack{root_};
    while (!stack.empty()) {
        const Node& n = nodes_[stack.back()];
        stack.pop_back();
        if (!(min.x < n.max.x && min.y < n.max.y && min.z < n.max.z && n.min.x < max.x && n.min.y < max.y &&
              n.min.z < max.z))
            continue;
        if (n.leaf()) {
            out.push_back(n.user);
        } else {
            stack.push_back(n.child[0]);
            stack.push_back(n.child[1]);
        }
    }
}

void AabbTree::query_line(Vec3 from, Vec3 to, std::vector<void*>& out) const {
    constexpr float kBig = std::numeric_limits<float>::max();
    const Vec3 d{to.x - from.x, to.y - from.y, to.z - from.z};
    const float len = std::sqrt(d.y * d.y + d.x * d.x + d.z * d.z);
    Vec3 dir{1, 0, 0};
    if (0 < len) dir = {d.x / len, d.y / len, d.z / len};
    float inv[3];
    int sign[3];
    const float comp[3] = {dir.x, dir.y, dir.z};
    for (int k = 0; k < 3; ++k) {
        if (comp[k] == 0) {
            inv[k] = kBig, sign[k] = 0;
        } else {
            inv[k] = 1.0f / comp[k], sign[k] = inv[k] < 0;
        }
    }
    const float length = d.y * dir.y + d.x * dir.x + d.z * dir.z;

    if (root_ < 0) return;
    std::vector<int> stack{root_};
    while (!stack.empty()) {
        const Node& n = nodes_[stack.back()];
        stack.pop_back();
        // Slab test in the original's order and tie rules.
        const Vec3 b[2] = {n.min, n.max};
        const float tmin = (b[sign[0]].x - from.x) * inv[0];
        const float tymax = inv[1] * (b[1 - sign[1]].y - from.y);
        if (!(tmin <= tymax)) continue;
        const float tmax = inv[0] * (b[1 - sign[0]].x - from.x);
        const float tymin = (b[sign[1]].y - from.y) * inv[1];
        if (!(tymin <= tmax)) continue;
        const float tzmax = inv[2] * (b[1 - sign[2]].z - from.z);
        float t0 = tymin < tmin ? tmin : tymin;
        float t1 = tymax <= tmax ? tymax : tmax;
        if (!(t0 <= tzmax)) continue;
        const float tzmin = (b[sign[2]].z - from.z) * inv[2];
        if (!(tzmin <= t1)) continue;
        if (t0 <= tzmin) t0 = tzmin;
        if (tzmax <= t1) t1 = tzmax;
        if (!(t0 < length && 0 < t1)) continue;
        if (n.leaf()) {
            out.push_back(n.user);
        } else {
            stack.push_back(n.child[0]);
            stack.push_back(n.child[1]);
        }
    }
}

void AabbTree::query_tree(const AabbTree& other, std::vector<std::pair<void*, void*>>& out) const {
    if (root_ < 0 || other.root_ < 0) return;
    const bool self = this == &other;
    // Pairs (node of this tree, node of other); identical nodes only occur when paired with itself.
    std::vector<std::pair<int, int>> stack{{root_, other.root_}};
    while (!stack.empty()) {
        const auto [ia, ib] = stack.back();
        stack.pop_back();
        const Node& a = nodes_[ia];
        const Node& b = other.nodes_[ib];
        if (self && ia == ib) {
            if (!b.leaf()) {
                const int c0 = b.child[0], c1 = b.child[1];
                stack.push_back({c0, c0});
                stack.push_back({c0, c1});
                stack.push_back({c1, c1});
            }
            continue;
        }
        if (!(a.min.x < b.max.x && a.min.y < b.max.y && a.min.z < b.max.z && b.min.x < a.max.x && b.min.y < a.max.y &&
              b.min.z < a.max.z))
            continue;
        if (a.leaf() && b.leaf()) {
            out.push_back({a.user, b.user});
        } else if (b.leaf()) {
            stack.push_back({a.child[0], ib});
            stack.push_back({a.child[1], ib});
        } else if (a.leaf()) {
            stack.push_back({ia, b.child[0]});
            stack.push_back({ia, b.child[1]});
        } else {
            stack.push_back({a.child[0], b.child[0]});
            stack.push_back({a.child[0], b.child[1]});
            stack.push_back({a.child[1], b.child[0]});
            stack.push_back({a.child[1], b.child[1]});
        }
    }
}

}  // namespace sh::phys
