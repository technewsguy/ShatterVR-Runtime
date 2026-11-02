#include "audio/level_music.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace sh::audio {

namespace {
constexpr int kBytesPerSecond = kRate * 2 * 2;
constexpr int kTrackBytes = 0x562200;  // 32 s: where the next room's music starts mixing in
}  // namespace

int LevelMusic::Slot::decode(std::int16_t* out, int bytes) {
    if (!track) return 0;
    const int total = static_cast<int>(track->samples.size() * 2);
    const int n = std::max(0, std::min(bytes, total - decoded)) & ~1;
    std::memcpy(out, reinterpret_cast<const char*>(track->samples.data()) + decoded, static_cast<size_t>(n));
    decoded += n;
    return n;
}

void LevelMusic::play(std::shared_ptr<const Pcm> track, bool skip_start) {
    for (Slot& s : slots_) s = Slot{};
    current_ = 0, next_ready_ = false, started_ = true;
    Slot& s = slots_[0];
    s.track = std::move(track);
    if (!s.track) return;
    const int skip = static_cast<int>((skip_start ? 0.0f : 2.0f) * 2 * kRate) * 2;  // channels * seconds * rate, 16-bit
    std::vector<std::int16_t> scratch(0x800);
    for (int left = skip; left > 0;) {
        const int n = s.decode(scratch.data(), std::min(left, 0x1000));
        s.played += n;
        left -= n;
        if (n <= 0) break;
    }
    s.has_data = true;
}

void LevelMusic::set_next(std::shared_ptr<const Pcm> track) {
    Slot& s = slots_[1 - current_];
    s.track = std::move(track);
    s.decoded = 0, s.played = 0;
    next_ready_ = true;
}

void LevelMusic::swap() {
    next_ready_ = false;
    current_ = 1 - current_;
}

void LevelMusic::stop() {
    for (Slot& s : slots_) s = Slot{};
    started_ = false;
}

void LevelMusic::get_data(std::int16_t* out, int bytes) {
    Slot& cur = slots_[current_];
    int got = 0;
    if (cur.has_data) {
        got = cur.decode(out, bytes);
        if (got < bytes) cur.has_data = false;
    }
    if (got < bytes) std::memset(reinterpret_cast<char*>(out) + got, 0, static_cast<size_t>(bytes - got));

    Slot& other = slots_[1 - current_];
    int start = 0, offset = 0;
    if (!other.has_data) {
        const int k = cur.decoded - kTrackBytes;
        if (!next_ready_ || k < 1 || bytes <= k) return;
        offset = got - k;
        start = std::max(offset, 0);
        other.has_data = true;  // mixing in from here
    }
    int want = bytes - offset, at;
    if (want < 1) at = 0x2000, want = 0;
    else if (want < 0x2000) at = 0x2000 - want;
    else at = 0, want = 0x2000;
    at = std::min(start, at);
    std::int16_t temp[0x1000];
    const int got2 = other.decode(temp, want);
    if (got2 < want) other.has_data = false;
    for (int i = 0; i < got2 / 2; ++i) {
        std::int16_t& a = out[at / 2 + i];
        const float fa = static_cast<float>(a) / 32767.0f, fb = static_cast<float>(temp[i]) / 32767.0f;
        const float r = ((fa + fb) - fb * std::fabs(fa)) * 32767.0f;
        a = r > -32767.0f ? (r < 32767.0f ? static_cast<std::int16_t>(static_cast<int>(r)) : std::int16_t{32767})
                          : std::int16_t{-32767};
    }
}

void LevelMusic::report_played(int bytes) {
    Slot& cur = slots_[current_];
    cur.played += bytes;
    if (cur.played > kTrackBytes) {
        const int extra = std::min(cur.played - kTrackBytes, bytes);
        slots_[1 - current_].played += extra;
        cur.played = std::min(cur.played, cur.decoded);
    }
}

float LevelMusic::location() const {
    const Slot& cur = slots_[current_];
    if (!cur.track) return 0.0f;
    return static_cast<float>(cur.played) / static_cast<float>(kBytesPerSecond);
}

}  // namespace sh::audio
