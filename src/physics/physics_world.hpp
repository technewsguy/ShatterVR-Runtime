#pragma once

// The original's Physics object and its step: Physics::update with
// Physics::animate. Engine effects the step triggers (sounds, particles, explosions, debris,
// breakage) go through PhysicsEvents so the game layer can provide them.

#include "core/rand48.hpp"
#include "physics/body_joint.hpp"
#include "physics/pairs.hpp"
#include "physics/rigid_body.hpp"
#include "physics/solver.hpp"

#include <vector>

namespace sh::phys {

// What Physics::update asks of the engine. Default implementations do nothing.
class PhysicsEvents {
public:
    // The original's sound banks, by their offset in the Audio object.
    enum class Bank : int {
        kBallSoft = 0x518,     // ball hits something dynamic, or nothing recorded
        kBallHard = 0x6f0,     // ball hits a static surface from above (impact normal y > 0.9)
        kBallSide = 0x8c8,     // ball hits a static surface otherwise
        kBallBall = 0xaa0,     // two balls
        kObjectSoft = 0xc78,   // other body, drop in squared speed <= 2
        kObjectHard = 0xe50,   // other body, larger drop
        kObjectType3 = 0x1028, // other body whose first shape has material sound type 3
        kBallGraze = 0x2470,   // fast ball grazing a static surface (random)
        kBreak = 0x1200,       // something broke (processBreakage)
        kBreakThin = 0x1788,   // something broke and one of the breaking shapes is thin
        kExplosionBreakSmall = 0x13d8,  // an explosion broke shapes of total volume in (0.1, 0.5]
        kExplosionBreakBig = 0x15b0,    // an explosion broke shapes of total volume > 0.5
        kExplosion = 0x1b38,            // every explosion
        kShot = 0x1960,                 // a shot (Level::handleInput, not the step)
        kCrash = 0x1ee8,                // the player crashing into something (Level::hitSomething)
        kLaserBall = 0x20c0,            // a laser destroying a ball (Level::processLasers)
    };
    enum class Loop : int { kLaser = 0x2798, kBall = 0x27e8, kObject = 0x2838 };  // kLaser: Level::addLaser
    // Particle systems, in a fixed order (Level::init: effects/default.xml names).
    enum class Effect : int {
        kAttached = 0,     // every 10th frame at an attached round body; 5 per explosion (+0x194 explosion_white)
        kBallBurst = 1,    // 8 per ball-ball impact (+0x1ac spark)
        kBreakShards = 2,  // 10 per broken body (+0x1a8 break_debris)
        kBreakDust = 3,    // 2 per broken body (+0x1a4 break_smoke)
        kExplosionA = 4,   // 3 per explosion (+0x198 explosion_dark)
        kExplosionB = 5,   // 3 per explosion (+0x19c explosion_dust)
        kSpark = 6,        // behind an armed ball every other frame (+0x1a0 trail; Level::update, not the step)
        kLaserSpark = 7,   // where a laser touches something (+0x1b0 laserspark; Level::processLasers)
        kAmbient = 8,      // a room's ambient particles (+0x1b4 ambient; Room::update, mgParticles)
        kBubbles = 9,      // the "bubbles" ambient mode (+0x1b8 bubbles)
        kSmoke = 10,       // mgSmoke (+0x1bc smoke)
    };
    struct Rgba {
        float r = 1, g = 1, b = 1, a = 1;
    };

    virtual ~PhysicsEvents() = default;
    // SoundBank::next: a sound from the bank, 0 for none (the engine draws from the shared rng here).
    virtual int pick_sound(Bank) { return 0; }
    virtual void play_sound(int /*sound*/, Vec3 /*pos*/, float /*volume*/, float /*pitch*/) {}
    virtual void play_loop(Loop, Vec3 /*pos*/, float /*volume*/) {}
    // Audio::playSound(buffer, volume, pitch): a picked sound without a position (the shot), and a fixed sound file
    // (snd/failed.ogg, snd/item.ogg, snd/clickdown0.ogg) from game::Level.
    virtual void play_sound_flat(int /*sound*/, float /*volume*/, float /*pitch*/) {}
    virtual void play_file(const char* /*path*/, float /*volume*/, float /*pitch*/) {}
    // ParticleSystem::spawn(pos, vel, color, flag); the physics passes flag -1 everywhere. (The engine
    // draws from the shared rng in there.)
    virtual void spawn_particle(Effect, Vec3 /*pos*/, Vec3 /*vel*/, Rgba /*color*/, int /*flag*/) {}
    // The Level's particle systems (its +0x1c0 list): ParticleSystem::update on each after the physics step (draws
    // from the shared rng), translate with the floating origin, clear with the level.
    virtual void update_particles() {}
    virtual void translate_particles(float /*dz*/) {}
    virtual void clear_particles() {}
    // Level::explosion at an exploding ball (strength 10; the body is then destroyed), or with no body at a picked-up
    // barrel power-up (strength 20, from game::Level).
    virtual void explosion(RigidBody* /*ball*/, Vec3 /*pos*/, float /*strength*/) {}
    virtual void update_debris() {}
    // Debris::spawn into the level's debris list (pose: pos with identity rotation). The engine draws from
    // the shared rng in there and ignores spawns beyond 400 pieces.
    virtual void spawn_debris(const void* /*obstacle*/, Vec3 /*pos*/, Vec3 /*vel*/, Rgba /*color*/) {}
    // A body breaks for the first time (processBreakage): the level's broken counter (+0x29c),
    // Player::registerBroken and the combo achievement.
    virtual void first_break(RigidBody&) {}
    // A round body is destroyed (Level::destroy): Player::registerBall with the body's +0x47 flag.
    virtual void ball_removed(bool /*broke_something*/) {}
};

class PhysicsWorld : public World {
public:
    PhysicsWorld();

    Solver solver;  // 4 iterations
    CollisionContext context;
    PairList pairs[2];               // 0: tree A vs tree B, 1: tree B vs itself
    std::vector<RigidBody*> bodies;  // the level's body list, in order
    std::vector<Entity*> entities;   // the level's entity list (Level +0x114): bodies and the game's other entities, in order
    std::vector<BodyJoint*> joints;  // the level's joint list, in order

    // Inputs the original reads from the level and game objects.
    Vec3 gravity{0, -10, 0};  // Level::getGravity
    float camera_z = 0;       // level +0xf4: despawn and sound cut-off
    Quat camera_rotation;     // game +0x224: decides the despawn rule
    int frame = 0;            // level +0xe0: attached-body particles every 10th frame

    void update(PhysicsEvents& events, Rand48& rng);
    // Physics::raycast: the segment from -> to against the shapes of both trees whose group shares a bit
    // with mask (and, given an obstacle, whose body belongs to it); round bodies count as 0.2 spheres.
    // Reports the nearest hit (point and normal in world space, the shape). A segment starting inside a
    // shape stops the search: point = from, normal = the unit direction. A zero-length segment never hits.
    bool raycast(Vec3 from, Vec3 to, std::uint8_t mask, Vec3* point, Vec3* normal, const BodyShape** shape,
                 const void* obstacle) const;
    // Physics::collect: shapes of both trees (tree A first) whose box overlaps [min, max], whose group shares a bit
    // with mask and, given an obstacle, whose body belongs to it. Appends them to shapes and their bodies (each once,
    // in order of first appearance) to bodies; either list may be null.
    void collect(std::vector<RigidBody*>* bodies, std::vector<BodyShape*>* shapes, Vec3 min, Vec3 max, std::uint8_t mask,
                 const void* obstacle) const;
    // Physics::getClosestPoint (no callers in the game): the closest point on bodies collected in the box
    // p +- radius (mask; optional obstacle filter). Hit if one is nearer than FLT_MAX. As in the original,
    // the reported point is transformed by the body pose twice (only the distance test uses the right
    // one); the normal is in world space.
    bool closest_point(float radius, Vec3 p, std::uint8_t mask, Vec3* point, Vec3* normal, const BodyShape** shape,
                       const void* obstacle) const;
    // Physics::collideSphere / collideBox: whether a sphere (radius) or a box (half extents) at p overlaps
    // the collision hull of any shape whose tree box meets the query's box (both trees, no mask). The query
    // shape sits at p in each body's solver frame with that frame's rotation, so the box turns with the body.
    // collideBox is behind mgBoxOverlap (no shipped script uses it); collideSphere has no callers.
    bool collide_sphere(Vec3 p, float radius) const;
    bool collide_box(Vec3 p, Vec3 half) const;
    void remove_pairs(const RigidBody& body);   // Physics::remove(Body*)
    void remove_pairs(const BodyShape& shape);  // Physics::remove(Shape*)

    // Level::create for a body: a new body appended to the body list, owned by the world.
    RigidBody& create_body();
    // Level::create / Level::destroy for the game's other entities (power-ups, ...): the entity list and the
    // owning obstacle's list (each removal: every occurrence, the last entry taking the slot).
    void add_entity(Entity& e);
    void remove_entity(Entity& e);
    // Level::destroy for a body (with ~Body): its joints are destroyed (destroy_joint), a round body reports
    // ball_removed, its pairs go, it leaves the body list (the last body takes its slot), then each shape (always
    // the first; the last takes its slot) loses its pairs and its tree leaf. The body is freed if the world
    // owns it.
    void destroy_body(RigidBody& body, PhysicsEvents* events = nullptr);
    // Body::removeShape then Shape::~Shape: the shape loses its pairs, leaves the body (the last shape takes its
    // slot) and its tree leaf, and is freed. The mass properties are not recomputed.
    void destroy_shape(RigidBody& body, const BodyShape* shape);
    // Level::destroyJoint: the joint leaves both bodies' joint lists and the level's list (every occurrence;
    // the last entry takes the slot). A joint made by create_joint is freed; others belong to their creator.
    void destroy_joint(BodyJoint& joint);
    // Level::createJoint: a new joint (type none) appended to body a's joint list, then b's, then the level's.
    BodyJoint& create_joint(RigidBody* a, RigidBody* b);

    std::vector<std::unique_ptr<RigidBody>> owned_bodies;  // bodies made by create_body
    std::vector<std::unique_ptr<BodyJoint>> owned_joints;  // joints made by create_joint
};

// Physics::animate over the level's bodies (velocity rows toward each body's animation target).
void animate(SolverCalls& solver, const std::vector<RigidBody*>& bodies, float dt);

}  // namespace sh::phys
