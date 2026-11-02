#pragma once

// Segment render mesh (.mesh, after zlib inflate). Format:

#include <cstdint>
#include <span>
#include <vector>

namespace sh {

// Matches the original's vertex layout byte for byte, so it can be uploaded as-is
// (position: 3 floats, uv: 2 floats, colour: 4 normalized unsigned bytes; stride 24).
struct MeshVertex {
    float pos[3];
    float uv[2];
    std::uint8_t rgba[4];  // rgb: base colour (shader doubles it), a: 1 - shadow
};
static_assert(sizeof(MeshVertex) == 24);

struct SegmentMesh {
    std::vector<MeshVertex> vertices;
    std::vector<std::uint16_t> indices;  // triangle list
    std::uint32_t dropped_triangles = 0;  // referenced a vertex index >= 0xffff (see notes)
    float min_pos[3] = {0, 0, 0};
    float max_pos[3] = {0, 0, 0};
};

// Parses an inflated .mesh. Throws std::runtime_error on malformed input.
SegmentMesh parse_segment_mesh(std::span<const std::uint8_t> data);

}  // namespace sh
