#pragma once

// Narrow phase: shapes, support functions and distance queries.
// Separated distance follows the original's GJK variant step by step
//; penetration uses EPA.

#include "physics/pmath.hpp"

#include <cstdint>

namespace sh::phys {

struct Shape {
    enum Type : int { kPoint = 0, kSphere = 1, kBox = 2, kHull = 6 };
    Type type = kPoint;
    float radius = 0;                // sphere
    Vec3 half;                       // box
    const Vec3* vertices = nullptr;  // hull (point cloud; support = farthest vertex)
    int count = 0;

    static Shape sphere(float r) { return {kSphere, r, {}, nullptr, 0}; }
    static Shape box(Vec3 h) { return {kBox, 0, h, nullptr, 0}; }
    static Shape hull(const Vec3* v, int n) { return {kHull, 0, {}, v, n}; }
};

// Farthest point of the shape (local frame) in direction d.
Vec3 support(const Shape& s, Vec3 d);

// Rigid transform stored like the original's (4x4 column-major), built from a position
// and unit quaternion with the original's formulas so results round identically.
struct Transform {
    float m[16];  // m[col * 4 + row]
    static Transform from(Vec3 position, Quat rotation);
    Vec3 apply(Vec3 v) const;             // R v + p
    Vec3 rotate(Vec3 v) const;            // R v
    Vec3 inverse_rotate(Vec3 v) const;    // R^T v
};

struct CollisionContext {
    int max_iterations = 32;  // original defaults
    float tolerance = 0.001f;
};

// Result flags (same bit values as the original).
enum : std::uint32_t {
    kResultValid = 0x01,
    kResultDistance = 0x02,
    kResultPoints = 0x04,
    kResultSeparated = 0x10,
    kResultPenetrating = 0x20,
    kResultBeyondMax = 0x100,  // farther than max_distance (only with kQueryMaxDistance)
    kResultNoHit = 0x200,      // shapecast/raycast: the sweep misses (reported with kResultSeparated)
    kResultIterationLimit = 0x4000,
};

// Query flags. The game passes 0x33 (tdDistance) and 0xb3 (via tdManifold).
enum : std::uint32_t {
    kQueryPoints = 0x01,       // compute witness points
    kQueryPenetration = 0x10,  // shapecast: still report points etc. when the cast starts overlapping
    kQueryMaxDistance = 0x20,  // stop early and report kResultBeyondMax past max_distance
    kQueryWarmStart = 0x40,    // original: read a cached simplex (never used by the game; ignored)
    kQueryWriteCache = 0x80,   // original: write the final simplex to a cache (no effect on results; ignored)
};

struct DistanceQuery {
    const Shape* a = nullptr;
    const Shape* b = nullptr;
    Vec3 b_position;  // B's pose in A's frame
    Quat b_rotation;
    std::uint32_t flags = 0x33;
    float max_distance = 3.402823466e+38f;
};

struct DistanceResult {
    std::uint32_t flags = 0;
    float distance = 0;  // negative when penetrating
    Vec3 normal;         // A's frame, from A towards B
    Vec3 point_a;        // A's frame
    Vec3 point_b;        // B's frame
};

DistanceResult distance(const CollisionContext& ctx, const DistanceQuery& q);

struct OverlapQuery {
    const Shape* a = nullptr;
    const Shape* b = nullptr;
    Vec3 b_position;  // B's pose in A's frame
    Quat b_rotation;
};

// Final GJK simplex of an overlap test, as the original writes it to its cache with query
// flag 0x80: support points on A (A frame) and B (B frame). count 0 when the search stopped
// at a separating direction; 4 when the shapes intersect.
struct OverlapSimplex {
    int count = 0;
    Vec3 a[4];
    Vec3 b[4];
};

// tdOverlap: whether the shapes intersect (the original reports 0x20, else 0x10). Spheres
// keep their radius (no point substitution as in distance()).
bool overlap(const CollisionContext& ctx, const OverlapQuery& q, OverlapSimplex* simplex = nullptr);

// tdShapecast: B (posed in A's frame) moves along -sweep_direction (the original's convention:
// the configuration-space object is extended by +sweep); finds where it first touches A.
// distance is the travel along the (not necessarily unit) direction, negative when the shapes
// already overlap; normal is the touching surface normal in A's frame. Misses report
// kResultSeparated | kResultNoHit. point_a is on A (A frame), point_b on B (B frame) at the hit.
struct ShapecastQuery {
    const Shape* a = nullptr;
    const Shape* b = nullptr;
    Vec3 b_position;
    Quat b_rotation;
    Vec3 sweep_direction;
    std::uint32_t flags = 0x23;
    float max_distance = 3.402823466e+38f;
};
DistanceResult shapecast(const CollisionContext& ctx, const ShapecastQuery& q);

// tdRaycast: a ray from origin along direction (A's frame) against shape A, as a shapecast of a
// point. The game passes flags 0x23, a unit direction and the segment length as max distance.
// point_b is not reported (the original copies only flags, distance, normal and point_a).
struct RaycastQuery {
    const Shape* shape = nullptr;
    Vec3 origin;
    Vec3 direction;
    std::uint32_t flags = 0x23;
    float max_distance = 3.402823466e+38f;
};
DistanceResult raycast(const CollisionContext& ctx, const RaycastQuery& q);

// Persistent contact manifold of one body pair (tdManifold). Same layout as the original's
// 31-word record and the solver's contact input. In the separated state (count 0 and
// normal_a.x == kManifoldSeparated) point_a[0..2] hold A's unit axes and point_b[0..2] the same
// points in B's frame, so the next update can tell whether B has moved.
inline constexpr float kManifoldSeparated = 0.12345f;

struct Manifold {
    int count = 0;
    Vec3 normal_a;    // A's frame, from A towards B
    Vec3 normal_b;    // the same normal in B's frame
    Vec3 point_a[4];  // A's frame
    Vec3 point_b[4];  // B's frame
};

struct ManifoldQuery {
    const Shape* a = nullptr;
    const Shape* b = nullptr;
    Vec3 b_position;  // B's pose in A's frame
    Quat b_rotation;
    float max_distance = 0;
    float tolerance = 0.02f;  // what generateContacts passes
};

// Brings the manifold up to date for B's new pose: keeps it if the stored points still fit,
// otherwise runs a distance query and adds or replaces one point (at most 4).
void update_manifold(const CollisionContext& ctx, Manifold& m, const ManifoldQuery& q);

}  // namespace sh::phys
