#include "game/powerup.hpp"

namespace sh::game {

const char* const Powerup::kNames[kTypeCount] = {"UNDEFINED", "balls",     "barrel",      "slowmotion",
                                                 "multiball", "ballfrenzy", "nitroballs", "freebie",
                                                 "antigravity", "rewind",   "shield",     "homing"};

int Powerup::type_id(const std::string& name) {
    for (int i = 0; i < kTypeCount; ++i)
        if (name == kNames[i]) return i;
    return -1;
}

void Powerup::set_type(int t) {
    type = t;
    size[0] = 0.5f, size[1] = 0.5f;
    // (The pick-up sound is acquired here in the original.)
    if (type == kBarrel) size[0] = 0.5f, size[1] = 0.7f;
}

}  // namespace sh::game
