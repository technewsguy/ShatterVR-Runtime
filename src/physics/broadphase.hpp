#pragma once

// Broad phase: the original's dynamic AABB tree (QiDbvt3).
// Leaves hold tight boxes; a fixed margin is added to leaf boxes whenever a parent box is
// built. Query results come out in the original's traversal order.

#include "core/rand48.hpp"
#include "physics/pmath.hpp"

#include <utility>
#include <vector>

namespace sh::phys {

class AabbTree {
public:
    struct Node {
        int parent = -1;
        Vec3 min, max;
        int child[2] = {-1, -1};  // child[1] == -1: leaf
        void* user = nullptr;     // leaves only
        bool leaf() const { return child[1] < 0; }
    };

    explicit AabbTree(float margin) : margin_(margin) {}  // the game uses 0.02 for both trees

    int create(Vec3 min, Vec3 max, void* user);
    // New bounds. The tree only changes shape when the box leaves its parent's box.
    void update(int leaf, Vec3 min, Vec3 max);
    void destroy(int leaf);

    // Each appends the user pointers / pairs it finds to `out`.
    void query_aabb(Vec3 min, Vec3 max, std::vector<void*>& out) const;  // strict overlap
    void query_line(Vec3 from, Vec3 to, std::vector<void*>& out) const;
    // Overlapping leaf pairs (this tree's user, other's user); a tree may be paired with itself.
    void query_tree(const AabbTree& other, std::vector<std::pair<void*, void*>>& out) const;

    // `passes` times: take a random leaf (rng.rnd over the leaves in depth-first order) out and
    // insert it again from the root. Physics::update runs 2 passes on one tree per step.
    void optimize(int passes, Rand48& rng);

    int root() const { return root_; }
    const Node& node(int i) const { return nodes_[i]; }

private:
    int alloc();
    void release(int i);
    float margin_of(int i) const { return nodes_[i].leaf() ? margin_ : 0.0f; }
    bool refit(int i);  // parent box from its children; returns false if unchanged
    // Detaches a leaf; returns the node where refitting stopped (reinsertion start) and the freed parent.
    std::pair<int, int> remove(int leaf);
    void insert(int start, int leaf, int parent_node);
    void fetch_leaves(int i, std::vector<int>& out) const;

    float margin_;
    std::vector<Node> nodes_;
    int free_ = -1;
    int root_ = -1;
};

}  // namespace sh::phys
