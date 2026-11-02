#pragma once

// LevelScript: the Lua script of a room or an obstacle. It owns the Lua host, the
// placement parameters (name/value strings), its own QiRandom and the "current shape" state the shape
// bindings read (scale, chamfer, material). Bindings not ported yet are registered as stubs that only
// record the call.

#include "core/qi_random.hpp"
#include "core/rand48.hpp"
#include "physics/convex_hull.hpp"
#include "physics/pmath.hpp"
#include "physics/rigid_body.hpp"
#include "script/script_host.hpp"

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace sh::phys {
class PhysicsWorld;
}

namespace sh::script {

class MaterialTable;

// Script handles: the original passes object pointers to Lua as integers; the port hands out small ids
// (0 = none) that map back to the objects.
class HandleTable {
public:
    // The original's handles are the object pointers, so the same object always gives the same handle.
    int add(void* p) {
        auto [it, inserted] = ids_.try_emplace(p, static_cast<int>(objects_.size()) + 1);
        if (inserted) objects_.push_back(p);
        return it->second;
    }
    template <class T>
    T* get(int handle) const {
        if (handle <= 0 || handle > static_cast<int>(objects_.size())) return nullptr;
        return static_cast<T*>(objects_[static_cast<size_t>(handle - 1)]);
    }

private:
    std::vector<void*> objects_;
    std::map<void*, int> ids_;
};

// An obstacle's streak state (Obstacle +0xb0..+0xc4): mgScore and mgClear write it, the game reads it.
struct ObstacleState {
    float cleared = 0;                   // +0xb0: the level time of mgClear (0: not cleared)
    int score = 0;                       // +0xb4: mgScore: balls for clearing it (0: not part of the streak)
    bool missed = false;                 // +0xb8: the camera passed it while uncleared
    phys::Vec3 score_position{0, 0, 0};  // +0xbc
};

// A sound the game asked for (the audio backend comes later): a script sound by name (mgPlaySound, at the
// obstacle's position) or an entry of a game sound bank (name = bank, index).
struct SoundEvent {
    std::string name;
    int index = -1;
    phys::Vec3 position{0, 0, 0};
    bool positional = false;
    float volume = 1, pitch = 1;
};

// What level scripts reach outside themselves (the original's Level): the physics world bodies are created
// in, the named materials and the handle table, shared by all scripts of a level.
struct ScriptRoom;

struct ScriptLevel {
    phys::PhysicsWorld* world = nullptr;
    const MaterialTable* materials = nullptr;
    HandleTable handles;
    // ResMan::load stand-in for the level's data files (meshes/<name>.xml): the text, or nothing.
    std::function<std::optional<std::string>(const std::string& path)> read_asset;
    // Level::getConvex: the hull of meshes/<name>.xml (its <v>x y z</v> points), built on first use and kept
    // (Level +0x258). Null when the file is missing or has fewer than 3 points; that result is not kept.
    const phys::ConvexHull* get_convex(const std::string& name);
    std::vector<std::pair<std::string, std::unique_ptr<phys::ConvexHull>>> convexes;
    float time = 0;                          // Level +0xd8
    phys::Vec3 camera_position{0, 0, 0};     // Level +0xec
    // mgClear on an obstacle with a score: the game's streak and score update (Level::streakInc, addScore).
    std::function<void(ObstacleState& state, const void* obstacle)> on_clear;
    std::vector<SoundEvent> sound_events;
    std::vector<std::string> ui_commands;    // Script::handleCommand on the UI script (later)
    // Game::getProperty / setProperty ("bag.key"): mgGet / mgSet. Unset: mgGet returns "".
    std::function<std::string(const std::string& key)> get_property;
    std::function<void(const std::string& key, const std::string& value)> set_property;
    float camera_speed = 0;   // Level +0x110 (mgGetCameraSpeed)
    float* camera_shake = nullptr;  // Display +0x8cc (mgCameraShake)
    // Level::create(2) + Entity::setObstacle for mgCreateButton: a button at pos owned by the obstacle.
    std::function<phys::ButtonEntity*(phys::Vec3 pos, std::vector<phys::Entity*>* owner_list, const void* owner)>
        create_button;
    // Level::addLaser for mgLaser: a beam from the point along the direction (100 units).
    std::function<void(phys::Vec3 from, phys::Vec3 dir)> add_laser;
    // ParticleSystem::spawn on the Level's smoke system (+0x1bc) for mgSmoke: world position and velocity.
    std::function<void(phys::Vec3 pos, phys::Vec3 vel)> smoke;
    float time_step = 0;      // Game +0x100 (mgGetTimeStep)
    const ScriptRoom* current_room = nullptr;  // Level +0xd0 (mgGetGravity reads its gravity)
    // Room::getBeat(room, x) for mgGetBeat / mgGetBeatFloat.
    std::function<float(const ScriptRoom& room, float x)> beat;
};

// The room a script belongs to (LevelScript +0x25c, the original's Room): what the room bindings set, with
// Room::Room's defaults. The game layer's Room holds one.
struct ScriptRoom {
    virtual ~ScriptRoom() = default;
    // mgHasBoss: a live obstacle of this room whose type starts with "boss" and that is not cleared.
    virtual bool has_boss() const { return false; }
    // mgIsClosestBoss: of those, the one with the greatest z (its id), or null.
    virtual const void* closest_boss() const { return nullptr; }
    float length = 100;              // +0x30, mgLength
    float start = 0;                 // +0x34: the room spans z = -start .. -(start + length)
    std::string music = "silent";    // +0x38, mgMusic
    std::string particles;           // +0x2f78, mgParticles: the ambient particle mode (Room::ambient)
    float reverb[3] = {0.3f, 0.5f, 0.3f};  // +0x68, +0x64, +0x6c: mgReverb's arguments in order
    float low_pass = 0;              // +0x70, mgLowPass
    float echo[4] = {0.2f, 0.5f, 0.3f, 0.8f};  // +0x74..+0x80, mgEcho
    float speed_multiplier = 1;      // +0x84, mgSpeedMultiplier
    float fog[2][4] = {{1, 1, 1, 1}, {1, 1, 1, 1}};  // +0x2f40, +0x2f50: mgFogColor (rgb; alpha stays 1)
    float gravity = 1;               // +0x2f68, mgGravity
    float difficulty = 0;            // +0x2f6c, the "difficulty" parameter or mgSetDifficulty
    float rotation[2] = {0, 0};      // +0x2f70 (an int argument stored as float), +0x2f74: mgSetRotation
    phys::RigidBody* body = nullptr; // +0x18a4: the room's own body; joint bindings use it for a 0 handle
    std::function<float(const std::string& name, float z)> create_segment_fn;  // Room::createSegment (mgSegment)
    // mgObstacle: an obstacle definition appended to the room (spawned by Room::update like a segment's).
    std::function<void(const std::string& type, phys::Vec3 position, phys::Quat rotation,
                       const std::vector<std::pair<std::string, std::string>>& params)>
        add_obstacle_fn;
};

// The obstacle a script belongs to (LevelScript +0x260): its pose places the script's bodies, and its
// identity becomes the bodies' obstacle (body +0xc).
struct ScriptObstacle {
    phys::Vec3 position;
    phys::Quat rotation;
    const void* id = nullptr;
    std::vector<phys::Entity*>* entities = nullptr;  // Obstacle +0x20: bodies made by the script join it
    ObstacleState* state = nullptr;                     // Obstacle +0xb0
    const float* time = nullptr;                       // Obstacle +0xd4 (mgObstacleTime)
};

class LevelScript {
public:
    struct Param {
        std::string name, value;
    };
    // ResMan::load stand-in: the text of an asset by path ("rooms/include.lua"), or nothing.
    using ReadAsset = std::function<std::optional<std::string>(const std::string& path)>;

    // is_room: the script belongs to a room (Room* set); obstacles get the obstacles include instead.
    // index: the global script counter the original stores at +0x2b8 (staggers garbage collection).
    LevelScript(Rand48& rng, std::vector<Param> params, bool is_room, int index, ScriptLevel* level = nullptr,
                ScriptObstacle obstacle = {});

    // LevelScript::load: the script text plus the include file ("rooms/include.lua" for rooms,
    // "obstacles/include.lua" otherwise) run as one chunk, the bindings registered, the QiRandom seeded
    // (0, or lrand48() when random_seed is set), then init() if it exists.
    bool load(const std::string& path, const ReadAsset& read, bool random_seed);
    // tick() if the script has one, and every 30th frame (index + game frame) a garbage collection step.
    void tick(int game_frame);
    void draw();
    void set_parameter(const std::string& name, const std::string& value);
    void set_index(int index) { index_ = index; }  // the global counter value (LevelScript::load takes it)

    ScriptHost& host() { return host_; }
    const std::vector<Param>& params() const { return params_; }
    // Names of stub bindings the script has called, with counts.
    const std::map<std::string, int>& stub_calls() const { return stub_calls_; }

    QiRandom random{0};                 // +0x2b4
    phys::Vec3 shape_scale{1, 1, 1};    // +0x264
    float chamfer = 0;                  // +0x270
    phys::Material material;            // +0x274
    ScriptRoom* room = nullptr;         // +0x25c
    // The owning obstacle's pose as the script sees it (the original reads the live obstacle; game::Obstacle keeps
    // this copy in step when it moves).
    ScriptObstacle& obstacle() { return obstacle_; }
    bool room_script = false;           // isRoom: this is the room's own script (mgSegment works only there)
    phys::Vec3 glow{0, 0, 0};           // the obstacle's +0xc8 (mgGlow)
    std::vector<std::string> sounds;    // +0x16c: mgCreateSound's resources (by name)

private:
    void register_bindings();
    const std::string* find_param(const std::string& name) const;  // the last match, as the original's scans

    ScriptHost host_;
    std::vector<Param> params_;
    bool is_room_;
    int index_;
    ScriptLevel* level_;
    ScriptObstacle obstacle_;
    std::map<std::string, int> stub_calls_;
};

// The level-script binding names (LevelScript::load registers these 99).
const std::vector<std::string>& level_binding_names();

}  // namespace sh::script
