#include "game/sound_bank.hpp"

namespace sh::game {

int SoundBank::next(Rand48& rng, int frame, float dt) {
    if (count == 0 || static_cast<float>(frame - last_frame) * dt < min_interval) return -1;
    int i;
    do {
        i = rng.rnd(0, count);
    } while (i == last && count > 1);
    last = i;
    last_frame = frame;
    return i;
}

int SoundBank::specific(int index, int frame, float dt) {
    if (!(min_interval <= static_cast<float>(frame - last_frame) * dt) || index < 0 || index >= count) return -1;
    last_frame = frame;
    return index;
}

// Audio::load1 and Audio::load2, in their loading order.
const SoundBankInfo kSoundBanks[] = {
    {0x168, "snd/clickdown", 0.0f},          {0x340, "snd/clickup", 0.0f},
    {0xc78, "snd/object/hit_s", 0.1f},       {0xe50, "snd/object/hit_l", 0.1f},
    {0x1028, "snd/object/metal", 0.1f},      {0x518, "snd/ball/object", 0.1f},
    {0x6f0, "snd/ball/wall", 0.1f},          {0x8c8, "snd/ball/floor", 0.1f},
    {0xaa0, "snd/ball/ball", 0.1f},          {0x1200, "snd/breakable/glass_s", 0.5f},
    {0x13d8, "snd/breakable/glass_m", 0.5f}, {0x15b0, "snd/breakable/glass_l", 0.5f},
    {0x1788, "snd/breakable/glass_pane", 0.5f}, {0x1b38, "snd/explosion", 0.5f},
    {0x1960, "snd/swoosh", 0.5f},            {0x1d10, "snd/water/splash", 0.2f},
    {0x1ee8, "snd/crash", 0.5f},             {0x20c0, "snd/laser/discharge", 0.5f},
    {0x2298, "snd/cleared", 0.05f},          {0x2470, "snd/ball/ricochet", 0.5f},
};
const int kSoundBankCount = static_cast<int>(sizeof(kSoundBanks) / sizeof(kSoundBanks[0]));

void SoundBanks::load(const std::function<bool(const std::string& path)>& exists) {
    banks_.clear();
    for (const SoundBankInfo& b : kSoundBanks) {
        SoundBank bank;
        bank.min_interval = b.min_interval;
        while (bank.count < 12 && exists(std::string(b.prefix) + std::to_string(bank.count) + ".ogg")) ++bank.count;
        banks_.emplace_back(b.offset, bank);
    }
}

SoundBank* SoundBanks::find(int offset) {
    for (auto& [o, b] : banks_)
        if (o == offset) return &b;
    return nullptr;
}

const SoundBankInfo* SoundBanks::info(int offset) const {
    for (const SoundBankInfo& b : kSoundBanks)
        if (b.offset == offset) return &b;
    return nullptr;
}

}  // namespace sh::game
