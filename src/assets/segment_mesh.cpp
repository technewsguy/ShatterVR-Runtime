#include "assets/segment_mesh.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <string>

namespace sh {
namespace {

// The original uses 16-bit index buffers with 0xffff as the exclusive limit;
// triangles touching a vertex at or past it are skipped.
constexpr std::uint32_t kIndexLimit = 0xffff;

std::uint32_t read_u32le(std::span<const std::uint8_t> d, size_t off) {
    return std::uint32_t(d[off]) | (std::uint32_t(d[off + 1]) << 8) | (std::uint32_t(d[off + 2]) << 16) |
           (std::uint32_t(d[off + 3]) << 24);
}

}  // namespace

SegmentMesh parse_segment_mesh(std::span<const std::uint8_t> d) {
    if (d.size() < 8) throw std::runtime_error("mesh: too small");
    const std::uint64_t vertex_count = read_u32le(d, 0);
    const std::uint64_t index_header = 4 + vertex_count * sizeof(MeshVertex);
    if (index_header + 4 > d.size()) throw std::runtime_error("mesh: vertex block exceeds file");
    const std::uint64_t triangle_count = read_u32le(d, static_cast<size_t>(index_header));
    if (index_header + 4 + triangle_count * 12 != d.size())
        throw std::runtime_error("mesh: size does not match vertex/triangle counts");

    SegmentMesh mesh;
    mesh.vertices.resize(static_cast<size_t>(vertex_count));
    // Little-endian floats and bytes in file order are the in-memory layout on x86/ARM.
    if (vertex_count) std::memcpy(mesh.vertices.data(), d.data() + 4, static_cast<size_t>(vertex_count) * sizeof(MeshVertex));

    if (!mesh.vertices.empty()) {
        std::copy_n(mesh.vertices[0].pos, 3, mesh.min_pos);
        std::copy_n(mesh.vertices[0].pos, 3, mesh.max_pos);
        for (const auto& v : mesh.vertices) {
            for (int c = 0; c < 3; ++c) {
                mesh.min_pos[c] = std::min(mesh.min_pos[c], v.pos[c]);
                mesh.max_pos[c] = std::max(mesh.max_pos[c], v.pos[c]);
            }
        }
    }

    mesh.indices.reserve(static_cast<size_t>(triangle_count) * 3);
    size_t p = static_cast<size_t>(index_header) + 4;
    for (std::uint64_t t = 0; t < triangle_count; ++t, p += 12) {
        const std::uint32_t a = read_u32le(d, p), b = read_u32le(d, p + 4), c = read_u32le(d, p + 8);
        if (a >= vertex_count || b >= vertex_count || c >= vertex_count)
            throw std::runtime_error("mesh: index out of range in triangle " + std::to_string(t));
        if (a >= kIndexLimit || b >= kIndexLimit || c >= kIndexLimit) {
            ++mesh.dropped_triangles;
            continue;
        }
        mesh.indices.push_back(static_cast<std::uint16_t>(a));
        mesh.indices.push_back(static_cast<std::uint16_t>(b));
        mesh.indices.push_back(static_cast<std::uint16_t>(c));
    }
    return mesh;
}

}  // namespace sh
