#pragma once

// Sound banks (the original's SoundBank): sets of interchangeable sound files picked at random
// with a minimum interval. The pick draws from the shared lrand48, so it is part of the game state, not just audio.

#include "core/rand48.hpp"

#include <functional>
#include <string>
#include <vector>

namespace sh::game {

struct SoundBank {
    float min_interval = 0.5f;  // +0: the Audio constructor's 0.5, some banks set their own
    int count = 0;              // +4: files loaded (prefix0.ogg, prefix1.ogg, ... up to 12, stopping at a gap)
    int last = 0;               // +0x1d0: the last pick
    int last_frame = 0;         // +0x1d4: the game frame of the last pick
    // SoundBank::next: the index of a random sound different from the last one (when there are several), or -1 when
    // the bank is empty or played less than min_interval ago (frames since times dt).
    int next(Rand48& rng, int frame, float dt);
    // SoundBank::getSpecific: index itself when it is loaded and the interval has passed (no draw, 'last' kept), else -1.
    int specific(int index, int frame, float dt);
};

// The game's banks, keyed by their offset in the original's Audio object (what PhysicsEvents::Bank names).
struct SoundBankInfo {
    int offset;
    const char* prefix;  // "snd/ball/wall" -> snd/ball/wall0.ogg, wall1.ogg, ...
    float min_interval;
};
extern const SoundBankInfo kSoundBanks[];
extern const int kSoundBankCount;

class SoundBanks {
public:
    // Audio::load1 / load2: counts each bank's files (exists("snd/x0.ogg") ...).
    void load(const std::function<bool(const std::string& path)>& exists);
    SoundBank* find(int offset);
    const SoundBankInfo* info(int offset) const;

private:
    std::vector<std::pair<int, SoundBank>> banks_;
};

}  // namespace sh::game
