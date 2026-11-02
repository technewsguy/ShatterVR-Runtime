#pragma once

// Power-ups (the original's Powerup): level entities placed by segments, hovering over
// the floor until a ball touches them; then a timer of the level runs for 7 s (or, for the barrel, an
// explosion goes off).

#include "physics/entity.hpp"

#include <string>

namespace sh::game {

class Powerup : public phys::Entity {
public:
    // Powerup::Type: the names segments use, in order; a type's index is also the level timer it starts.
    enum Type {
        kUndefined = 0, kBalls, kBarrel, kSlowMotion, kMultiball, kBallFrenzy, kNitroBalls, kFreebie, kAntiGravity,
        kRewind, kShield, kHoming, kTypeCount
    };
    static const char* const kNames[kTypeCount];
    static int type_id(const std::string& name);  // Powerup::getTypeId: -1 for an unknown name

    Powerup() : Entity(Kind::kPowerup) {}
    void set_type(int t);  // Powerup::setType: the type and its size (+0x4c, +0x50)

    bool picked = false;           // +0x44
    int type = kUndefined;         // +0x48
    float size[2] = {0, 0};        // +0x4c, +0x50: 0.5 0.5 (the barrel 0.5 0.7); for drawing (unverified)
};

}  // namespace sh::game
