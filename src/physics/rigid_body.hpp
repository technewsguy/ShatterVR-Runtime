#pragma once

// The game's bodies (the original's Body and Shape classes): a body owns
// convex shapes, each built from a Polyhedron; mass properties come from the polyhedra, and the
// solver body sits at the centre of mass in the principal-axis frame.

#include "physics/body.hpp"
#include "physics/entity.hpp"
#include "physics/broadphase.hpp"
#include "physics/polyhedron.hpp"

#include <cfloat>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace sh::phys {

// Per-shape material (0x3e bytes in the original, copied whole into the shape).
struct Material {
    enum Type : std::uint8_t { kNormal = 0, kStatic = 1, kInfiniteInertia = 2 };

    float density = 1;
    float friction = 1;     // contact friction = product of the two shapes' values
    float restitution = 0;  // contact restitution = the larger value (the ball sets 0.4)
    float strength = FLT_MAX;  // contact impulse cap = the smaller value; also breakage; FLT_MAX = unbreakable
    float roundness = 0;    // mgRoundness / materials.xml "roundness"; rendering use unverified
    std::int32_t sound_type = 0;  // impact sound bank choice (3: its own bank); other values unverified
    float color[4] = {};    // RGBA: tints breakage debris and particles (sound type 2 shapes tint the particles)
    std::uint8_t group = 1;    // collision group bits (the ball sets 0x40)
    std::uint8_t mask = 0xff;  // groups this shape collides with (both directions must agree)
    std::uint8_t type = kNormal;
    std::uint8_t b2b = 0;
    float f2c = 0, f30 = 0;    // mgTexture: uv offset (added after scaling)
    float f34 = 1, f38 = 1;    // mgTexture: uv scale
    std::uint8_t b3c = 0;      // mgTexture: textured glass (glasstex.glsl, no creases)
    std::uint8_t b3d = 0;      // mgHideEdges: glass without crease lines
};

class RigidBody;
class ConvexHull;
class ShapePair;
struct BodyJoint;  // physics/body_joint.hpp


// Game-layer stand-in for the original Physics object as far as bodies need it: the two broad-phase
// trees and the engine time step (Game +0x100). Grows into the full Physics port.
struct World {
    AabbTree tree_a{0.02f};  // Physics +8 (shapes with in_tree_a set)
    AabbTree tree_b{0.02f};  // Physics +0xc
    float time_step = 1.0f / 60.0f;
    std::uint32_t next_shape_serial = 0;
};

class BodyShape {
public:
    RigidBody* body = nullptr;
    // Creation order. The original orders a pair's shapes by heap address; with fresh allocations
    // (and in the emulator) that is creation order.
    std::uint32_t serial = 0;
    Vec3 box_axes[3];          // centre + axis * equivalent-box size, per principal axis (body frame, recentred)
    Vec3 bounds_min, bounds_max;  // world AABB incl. velocity sweep and 0.02 margin
    float volume = 0;
    int tree_leaf = -1;
    bool in_tree_a = false;
    bool thin = false;         // one equivalent-box side < 0.15 and the others > 0.5
    std::vector<Vec3> hull;    // collision vertices, solver-body frame
    Material material;
    Polyhedron poly;           // body frame (entity), as built
    // Shape +4: set when breakage has cut this shape or made it (unverified use: rendering?).
    bool modified = false;
    // Shape +8: the shape itself when built; pieces cut from a shape copy its value (unverified use).
    const BodyShape* origin = this;

    void compute_bounds();
};

class RigidBody : public Entity {
public:
    explicit RigidBody(World& world) : Entity(Kind::kBody), world_(&world) {}

    bool dynamic = true;
    Body solver;  // at the centre of mass, principal axes
    Vec3 com_position;   // centre-of-mass frame in entity coordinates
    Quat com_rotation;
    Vec3 inv_com_position;  // the inverse of that frame
    Quat inv_com_rotation;
    Vec3 equivalent_box;    // box with the body's principal moments
    std::vector<std::unique_ptr<BodyShape>> shapes;
    bool alive = true;      // cleared when the body leaves the level
    bool round = false;     // the ball: collides as a 0.1 sphere
    int type = 0;           // 1 armed by its first contact pass -> 2 (explodes); unverified beyond that
    float age = 0;          // seconds; two young round bodies (< 0.5) ignore each other
    std::vector<BodyJoint*> joints;
    // Last contact seen by a round body (for sounds): normal in world space, whether the other
    // body was static, whether both were round. Cleared by the step after use.
    Vec3 impact_normal;
    bool impact_static = false;
    bool impact_round = false;
    float prev_speed_sq = 0;  // last step's squared speed (impact sounds)
    // Animation target (body +0x114): the step drives the body toward this pose with velocity rows.
    struct AnimTarget {
        Vec3 position;
        Quat rotation;
        float linear_strength = 0;   // impulse cap per unit mass; 0 = no position drive
        float max_linear_speed = 0;
        float angular_strength = 0;  // 0 = no rotation drive
        float max_angular_speed = 0;
    };
    std::optional<AnimTarget> anim;
    // Attach target (body +0x4c, round bodies): while set the body gets an upward velocity kick of
    // 10 dt per pass; it lets go once the point attach_offset on the target is within 1 unit.
    RigidBody* attach_target = nullptr;
    Vec3 attach_offset;  // in the target's entity frame
    // Obstacle +0xb4 (unverified meaning): when set, breakage debris of this body stays white.
    bool obstacle_plain_debris = false;
    bool broke_something = false;  // body +0x47: set on a round body whose contact broke a shape
    bool broken = false;       // body +0x44: set on a body breakage has touched (and on its pieces)
    bool split_piece = false;  // body +0x11d: set on a body breakBody hands to breakBodySplit (unverified use)
    std::uint32_t f118 = 0;    // body +0x118: copied to pieces; meaning unverified

    // Body::addShape (the shape is appended) and Body::removeShape without the pair removal (the last
    // shape takes the removed one's slot); see breakage.cpp for the full removal.
    void add_shape(std::unique_ptr<BodyShape> shape);
    std::unique_ptr<BodyShape> take_shape(const BodyShape* shape);

    World& world() const { return *world_; }

    // Body::update: age += the game time step.
    void tick_age() { age = age + world_->time_step; }
    // Body::setVelocity / setAngVel / applyImpulse (world point and impulse) on the solver body.
    void set_velocity(Vec3 v) { solver.linear_velocity = v; }
    void set_angular_velocity(Vec3 w) { solver.angular_velocity = w; }
    void apply_impulse(Vec3 point, Vec3 impulse) { solver.apply_impulse(point, impulse); }
    // Body::getPointVelocity: v + w x (p - solver position).
    Vec3 point_velocity(Vec3 p) const {
        const Vec3 r = p - solver.position, w = solver.angular_velocity, v = solver.linear_velocity;
        return {v.x + (r.z * w.y - r.y * w.z), (r.x * w.z - r.z * w.x) + v.y, (r.y * w.x - r.x * w.y) + v.z};
    }

    void set_transform(Vec3 pos, Quat rot);
    // A box (half extents) placed by a column-major 4x4 matrix in entity coordinates. bevel != 0
    // makes a 24-point bevelled box through the convex hull builder.
    BodyShape& add_box(const float matrix[16], Vec3 half_extents, const Material* material, float bevel);
    // A convex shape: the hull's points placed by the matrix, its triangles as faces.
    BodyShape& add_convex(const float matrix[16], const ConvexHull& hull, const Material* material);
    // Body::addCylinder: a 24-sided cylinder along the matrix's x axis (radius, half length), its edges
    // bevelled when bevel > 0. Breakable materials get the convex hull of its points, others its polygons
    // (and material roundness = 1).
    BodyShape& add_cylinder(const float matrix[16], float radius, float half_length, const Material* material,
                            float bevel);
    void compute_mass_properties();
    void compute_bounds();

private:
    void solver_pose_from_entity();
    World* world_;
};

}  // namespace sh::phys
