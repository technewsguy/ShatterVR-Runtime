#pragma once

// QiVec3::random, drawing from the shared lrand48 like the original.

#include "core/rand48.hpp"
#include "physics/pmath.hpp"

#include <cmath>

namespace sh::phys {

// QiVec3::random(scale): three QiRnd(-1, 1) (x, y, z), normalised (X if zero), times scale.
inline Vec3 random_unit(Rand48& rng, float scale) {
    const float x = rng.rnd(-1.0f, 1.0f);
    const float y = rng.rnd(-1.0f, 1.0f);
    const float z = rng.rnd(-1.0f, 1.0f);
    const float len = std::sqrt((y * y + x * x) + z * z);
    Vec3 u{1, 0, 0};
    if (0 < len) u = {x / len, y / len, z / len};
    return {scale * u.x, scale * u.y, scale * u.z};
}

}  // namespace sh::phys
