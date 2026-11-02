#pragma once

// The engine's random numbers: QiRnd draws from the C library's lrand48 (48-bit linear
// congruential generator, as in Android's bionic). Kept exact so that anything seeded the
// same way (e.g. the broad phase's random tree optimisation) makes the same choices.

#include <cstdint>

namespace sh {

class Rand48 {
public:
    // Default lrand48 state when srand48 was never called.
    std::uint64_t state = 0x1234ABCD330Eull;

    void srand48(std::int32_t seed) { state = (std::uint64_t(std::uint32_t(seed)) << 16) | 0x330E; }

    std::int32_t lrand48() {
        state = (0x5DEECE66Dull * state + 0xB) & 0xFFFFFFFFFFFFull;
        return static_cast<std::int32_t>(state >> 17);  // 31 bits, non-negative
    }

    // QiRnd(int lo, int hi): lo + trunc((lrand48 / 2^31) * (hi - lo)) in float, clamped to [lo, hi - 1].
    int rnd(int lo, int hi) {
        const float u = static_cast<float>(lrand48()) * (1.0f / 2147483648.0f);
        int v = static_cast<int>(u * static_cast<float>(hi - lo)) + lo;
        if (v < lo) v = lo;
        if (hi - 1 <= v) v = hi - 1;
        return v;
    }

    // QiRnd(float lo, float hi): lo + (lrand48 / 2^31) * (hi - lo).
    float rnd(float lo, float hi) {
        const float u = static_cast<float>(lrand48()) * (1.0f / 2147483648.0f);
        return lo + u * (hi - lo);
    }

    // QiRndNormal(lo, hi): x = 2u - 1 from one draw; ((hi - lo) * (|x| * x)) * 0.5 + (lo + hi) * 0.5,
    // with |x| x computed as (x * sign(x)) * x (sign 0 at 0). A cheap bell-ish spread around the middle.
    float rnd_normal(float lo, float hi) {
        const float u = static_cast<float>(lrand48()) * (1.0f / 2147483648.0f);
        const float x = (u + u) - 1.0f;
        const float sign = x == 0.0f ? 0.0f : (x < 0.0f ? -1.0f : 1.0f);
        const float t = (x * sign) * x;
        return ((hi - lo) * t) * 0.5f + (lo + hi) * 0.5f;
    }
};

}  // namespace sh
