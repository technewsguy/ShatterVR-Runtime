#pragma once

// An obstacle (the original's Obstacle): a level script placed in a room with a pose, and
// the bodies its script made (its entities). Rooms spawn them from their obstacle definitions as the camera
// approaches and destroy them once everything they made is behind the camera.

#include "script/level_script.hpp"

#include <memory>
#include <string>
#include <vector>

namespace sh::game {

struct RoomContext;
class Room;

class Obstacle {
public:
    // Obstacle::Obstacle: the pose and a LevelScript with the parameters (not loaded yet).
    Obstacle(RoomContext& ctx, Room& room, phys::Vec3 position, phys::Quat rotation,
             std::vector<script::LevelScript::Param> params);
    // Obstacle::~Obstacle: every entity is destroyed (Level::destroy on the first one until none is left; each
    // leaves the list as it goes, the last entry taking its slot), then the script.
    ~Obstacle();
    Obstacle(const Obstacle&) = delete;
    Obstacle& operator=(const Obstacle&) = delete;

    // Obstacle::load = LevelScript::load: the script, the bindings, the seed (lrand48 unless player mode 0),
    // then init().
    bool load(const std::string& path);
    // Obstacle::update: time += dt, the script's tick, and the done flag: set when the obstacle has entities
    // and none of them has bounds min z - 20 < camera z (all are more than 20 units behind the camera).
    // Per entity, a scoring obstacle that is not cleared and not yet missed is missed once the camera is past
    // the entity's bounds min z: "miss left/center/right" (by position x) to the UI and Level::streakMissed.
    void update(float dt, int game_frame, float camera_z);
    // Obstacle::translate: z += dz (centerCamera); the script's view of the pose follows.
    void translate(float dz);

    Room* room;
    phys::Vec3 position;  // +4
    phys::Quat rotation;  // +0x10
    std::vector<phys::Entity*> entities;  // +0x20
    bool done = false;    // +0xac
    float time = 0;       // +0xd4
    script::ObstacleState state;  // +0xb0..+0xc4
    std::unique_ptr<script::LevelScript> script;  // +0xd8

private:
    RoomContext* ctx_;
};

}  // namespace sh::game
