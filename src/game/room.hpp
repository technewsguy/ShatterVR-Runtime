#pragma once

// A room (the original's Room): a stretch of the level along -z with its own script,
// settings (music, reverb, fog, ...), a static body for its geometry and the obstacles its script places.

#include "core/rand48.hpp"
#include "game/obstacle.hpp"
#include "game/property_bag.hpp"
#include "game/scoring.hpp"
#include "script/level_script.hpp"

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace sh::phys {
class PhysicsEvents;
class PhysicsWorld;
}

namespace sh::game {

// What a room needs from its level and game.
struct RoomContext {
    phys::PhysicsWorld* world = nullptr;
    script::ScriptLevel* level = nullptr;
    Rand48* rng = nullptr;
    std::function<std::optional<std::string>(const std::string& path)> read_asset;
    int player_mode = 0;     // Player +0x63c
    float level_time = 0;    // Level +0xd8
    int script_counter = 0;  // the global LevelScript counter (each script takes the next value)
    const TemplateRegistry* templates = nullptr;  // PropertyBag's template list (templates.xml)
    std::string level_spec;  // Game +0x168: "segment:" / "obstacle:" specs change segment fog
    Scoring* scoring = nullptr;  // balls and streak (Level / Player)
    // Level::destroy for entities other than bodies (power-ups, ...); bodies go to the world's destroy_body.
    std::function<void(phys::Entity&)> destroy_other;
    // Room::createPowerUp (Level::create(1)): a segment's power-up of that type name at pos, owned by the obstacle.
    std::function<void(const std::string& type, phys::Vec3 pos, Obstacle& owner)> create_powerup;
    void destroy_entity(phys::Entity& e);
};

class Room : public script::ScriptRoom {
public:
    using Params = std::vector<std::pair<std::string, std::string>>;

    // Room::Room: settings at their defaults, the "difficulty" parameter, the room body at z = -start,
    // the room's own script (rooms/<type>.lua; "obstacle:<name>" and "segment:<name>" types run the
    // obstacletest / segmenttest scripts) loaded and its init() run, then the fog colours scaled by player mode.
    Room(RoomContext& ctx, const std::string& type, const Params& params, float start);
    ~Room();  // its obstacles go (and with them their bodies, the room body included)
    Room(const Room&) = delete;
    Room& operator=(const Room&) = delete;

    float room_to_z(float x) const { return -start - x; }
    float z_to_room(float z) const { return -z - start; }
    float progress(float camera_z) const { return z_to_room(camera_z) / length; }   // Room::getProgress
    float room_time(float level_time) const { return level_time - time_start; }     // Room::getRoomTime
    bool has_boss() const override;            // mgHasBoss
    const void* closest_boss() const override;  // mgIsClosestBoss

    // Room::createSegment: segments/<name>.xml ("#SEGMENT#": the test segment) placed at room z offset `z`
    // (mgSegment passes -length so far). Its boxes become shapes of the room body, its obstacles (that pass the
    // difficulty and player-mode filters) obstacle definitions; returns the segment's size z (0 if it does not
    // load). Power-ups, decals, water and the render geometry are not built yet.
    float create_segment(const std::string& name, float z);

    struct ObstacleDef {  // +0x8c: 0x5c bytes each
        std::string type;
        phys::Vec3 position;  // room coordinates, z already offset
        phys::Quat rotation;
        std::vector<std::pair<std::string, std::string>> params;  // from param0..param11 "name=value"
        Obstacle* live = nullptr;  // +0x54: the spawned obstacle while it exists
        bool spawned = false;      // +0x58: set once spawned; a definition never spawns twice
    };
    struct SegmentRecord {  // +0x27a4
        float z = 0;
        std::string mesh;  // the segment path with .xml replaced by .mesh
    };
    std::vector<ObstacleDef> obstacle_defs;
    std::vector<SegmentRecord> segments;
    struct Decal {  // +0x27b0: 0x3c bytes each (a segment's <decal>; render only)
        phys::Vec3 pos;      // +0: in the room body's frame (segment z added)
        phys::Quat rot;      // +0xc: rot as X, Y, Z angles
        float size[2] = {1, 1};  // +0x1c: half size
        int cell = 0;        // +0x24: tile, a cell of the decal sheet's 8 x 8 grid
        float color[4] = {1, 1, 1, 1};  // +0x28
        float blend = 1;     // +0x38
    };
    std::vector<Decal> decals;
    // Render data from the boxes (room body frame): the tops of boxes with reflection != "0" (+0x239c, 4 corners
    // each, 0.02 above the top), and the +z, -x and +x faces of visible boxes standing on the floor (+0x2764).
    std::vector<phys::Vec3> reflection_mask;
    struct ReflectionQuad {
        phys::Vec3 p[4];
        float color[4];
    };
    std::vector<ReflectionQuad> reflection_walls;
    int segment_count = 0;  // +0x2f3c: every createSegment call, loaded or not
    int def_index = 0;            // +0x2f60: the RoomDef it was made from (set by the Level)
    bool level_boundary = false;  // +0x2f64: the next RoomDef belongs to another level (a checkpoint)

    std::string type;           // +4
    float time_start = 0;       // +0x88: the level time when the room was made
    std::string test_obstacle;  // +0x18ac: the name after "obstacle:"
    std::string test_segment;   // +0x18d8: the name after "segment:"
    std::unique_ptr<Obstacle> room_obstacle;      // +0x18a8: the room's own script and the room body
    script::LevelScript* script = nullptr;        // room_obstacle's script
    std::vector<std::unique_ptr<Obstacle>> obstacles;  // +0x1798: spawned obstacles

    // Room::createObstacle: obstacles/<name>.lua ("#OBSTACLE#": the test obstacle) at the room body's pose times
    // the given local pose; loaded (init() runs) and appended to `obstacles`.
    Obstacle* create_obstacle(const std::string& name, phys::Vec3 position, phys::Quat rotation,
                              const std::vector<std::pair<std::string, std::string>>& params);
    // Room::update (camera z = Level +0xf4): spawns definitions as the camera nears them (elevators 40 units
    // ahead, the rest 30), updates the room obstacle and the spawned ones, and destroys those that are done.
    // The ambient particles of the current room are not ported yet.
    void update(float camera_z, float dt, int game_frame);
    // The end of Room::update for the level's current room: the ambient particles of the mgParticles mode, before
    // 90% of the room (in slow motion only every eighth frame). camera: Level +0xec; slow: byte 0x226eda.
    void ambient(phys::PhysicsEvents& events, Rand48& rng, phys::Vec3 camera, int game_frame, bool slow);
    bool script_loaded = false;

private:
    RoomContext* ctx_;
};

}  // namespace sh::game
