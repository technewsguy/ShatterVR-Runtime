#pragma once

// Decoded sound: 16-bit stereo at 44100 Hz (what the original's level music stream and mixer use).

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sh::audio {

constexpr int kRate = 44100;

struct Pcm {
    std::vector<std::int16_t> samples;  // interleaved left, right
    std::size_t frames() const { return samples.size() / 2; }
};

// Ogg Vorbis bytes to Pcm (mono is duplicated, other rates are resampled linearly). Null on a decode error.
std::shared_ptr<const Pcm> decode_ogg(const std::string& bytes);

}  // namespace sh::audio
