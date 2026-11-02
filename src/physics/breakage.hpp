#pragma once

// Breaking bodies apart (the original's breakBody, breakBodySplit, splitShapes, checkConnectivity and
// the joint hand-over after a break;). Shapes are cut with Polyhedron::split;
// pieces near the impact go to a new body cloned from the broken one, and bodies whose shapes no longer
// touch are separated.

#include "core/rand48.hpp"
#include "physics/physics_world.hpp"

#include <memory>
#include <vector>

namespace sh::phys {

// QiPlane: points p with n . p + d = 0; "inside" is n . p + d < 0.
struct Plane {
    Vec3 n;
    float d = 0;
};

// Body::getClosestPoint (without its optional outputs): the closest point on the body's collision hulls
// to p, both in the body's entity frame. (0, 0, 0) when no hull reports one.
Vec3 closest_point(const CollisionContext& ctx, const RigidBody& body, Vec3 p);
// With the outputs: the shape holding the point (left untouched if none reports one), the surface normal there
// (entity frame; the value passed in is turned too if no shape reports one) and whether p is inside a hull.
Vec3 closest_point(const CollisionContext& ctx, const RigidBody& body, Vec3 p, const BodyShape** shape, Vec3* normal,
                   bool* inside);

// computeDistance: distance between two shapes' polyhedra (both in their body's entity frame), or a
// polyhedron and a point. 0 when they overlap; `limit` when they are farther apart than that.
float shape_distance(const CollisionContext& ctx, const BodyShape& a, const BodyShape& b, float limit);
float shape_distance(const CollisionContext& ctx, const BodyShape& a, Vec3 p, float limit);

// splitShapes: cuts the shapes by each plane in turn (only where a shape reaches more than 0.06 past both
// sides, and only shapes not entirely outside some plane). Returns all pieces: a cut shape keeps the part
// above the plane and a new shape (appended to `created`) takes the part below.
std::vector<BodyShape*> split_shapes(World& world, const std::vector<BodyShape*>& shapes,
                                     const std::vector<Plane>& planes,
                                     std::vector<std::unique_ptr<BodyShape>>& created);

// The original's Body::cloneWithoutShapes: a new body in the world's list with the body's entity pose
// and a few of its flags, no shapes.
RigidBody& clone_without_shapes(PhysicsWorld& world, const RigidBody& body);

// Hands each of the body's joints to whichever of `parts` is closest to the joint's anchor (FUN_0014bbcc).
void hand_over_joints(const CollisionContext& ctx, RigidBody& body, const std::vector<RigidBody*>& parts);

// checkConnectivity: groups the body's shapes that touch (within 0.04); every group after the first moves
// to a new body. Mass properties are recomputed.
void check_connectivity(PhysicsWorld& world, RigidBody& body);

// breakBodySplit: one random plane through the point (world); breakable shapes it cuts lose their lower
// part to a new body.
void break_body_split(PhysicsWorld& world, RigidBody& body, Vec3 point, float force, Rand48& rng);

// breakBody: a randomised box of half-size `radius` around the point (world), oriented by `dir`, cuts the
// breakable shapes it reaches whose strength is at most `force`. Pieces containing the point go to a new
// body; then connectivity is checked and one more random split is made.
void break_body(PhysicsWorld& world, RigidBody& body, Vec3 point, Vec3 dir, float force, float radius,
                Rand48& rng);

// Physics::processBreakage, after each solve: every pair with a breakable shape whose summed contact
// impulse reaches the shape's strength breaks that shape's body (once per body, bodies older than 0.5 s),
// with breakBody at the contact (force = strength, radius 0.25), plus a sound, debris and particles.
// Returns whether anything broke (the step then runs another pass).
bool process_breakage(PhysicsWorld& world, PhysicsEvents& events, Rand48& rng);

}  // namespace sh::phys
