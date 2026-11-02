#pragma once

// The level's entities (the original's Entity): bodies, power-ups, buttons and water share a pose, bounds and
// an owning obstacle, and sit together in the level's entity list (Level +0x114) in creation order.

#include "physics/pmath.hpp"

#include <cfloat>
#include <vector>

namespace sh::phys {

class Entity {
public:
    enum class Kind { kBody = 0, kPowerup = 1, kButton = 2, kWater = 3 };  // Entity +8

    explicit Entity(Kind k) : kind(k) {}
    virtual ~Entity() = default;
    Entity(const Entity&) = delete;
    Entity& operator=(const Entity&) = delete;

    const Kind kind;
    // Entity +0xc: the obstacle the entity belongs to (Entity::setObstacle; null for free entities). A body
    // with one despawns 3 units past the camera instead of 25; free entities are culled by the level.
    const void* near_despawn = nullptr;
    // The obstacle's entity list the entity is in (Obstacle +0x20): ~Entity takes it out (every occurrence,
    // the last entry taking the slot).
    std::vector<Entity*>* obstacle_list = nullptr;
    // Entity pose (what the game moves and draws) and bounds.
    Vec3 position;
    Quat rotation;
    Vec3 bounds_min{FLT_MAX, FLT_MAX, FLT_MAX}, bounds_max{-FLT_MAX, -FLT_MAX, -FLT_MAX};
};

}  // namespace sh::phys

namespace sh::phys {

// A button (the original's Button, entity kind 2): pressed by a ball coming within half a unit (Button::update);
// scripts make them with mgCreateButton and read them with mgIsButtonDown.
class ButtonEntity : public Entity {
public:
    ButtonEntity() : Entity(Kind::kButton) {}
    bool down = false;      // +0x44
    bool was_down = false;  // +0x45: last frame's down
};

}  // namespace sh::phys
