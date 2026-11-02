#pragma once

// QiRandom: the engine's small seeded generator (one 32-bit counter, a hashed output), separate from the
// shared lrand48 stream. Each level script owns one.

#include <cstdint>

namespace sh {

class QiRandom {
public:
    explicit QiRandom(std::uint32_t seed = 0) : seed(seed) {}

    std::uint32_t seed;

    // getNext: the counter advances by 0xdc769 and is hashed.
    std::uint32_t next() {
        seed += 0xdc769u;
        std::uint32_t x = (seed ^ 0x3du ^ (seed >> 16)) * 9u;
        x = (x ^ (x >> 4)) * 0x27d4eb2du;
        return x ^ (x >> 15);
    }
    // getFloat: lo + (hi - lo) * (next * 2^-32).
    float get_float(float lo, float hi) {
        const float u = static_cast<float>(next()) * 2.3283064e-10f;
        return lo + (hi - lo) * u;
    }
    // getInt: next * (hi - lo + 1) as a 64-bit product (the span sign-extended), divided by 0xffffffff,
    // added to lo and clamped to [lo, hi].
    int get_int(int lo, int hi) {
        const std::uint32_t n = next();
        const std::int64_t span = static_cast<std::int64_t>(hi - lo + 1);
        const std::uint64_t prod = static_cast<std::uint64_t>(n) * static_cast<std::uint64_t>(span);
        int r = lo + static_cast<int>(static_cast<std::uint32_t>(prod / 0xffffffffull));
        if (r < lo) r = lo;
        if (r >= hi) r = hi;
        return r;
    }
};

}  // namespace sh
