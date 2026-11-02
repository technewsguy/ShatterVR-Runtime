// Tests for the segment mesh parser using synthetic meshes built here.

#include "assets/segment_mesh.hpp"

#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                \
    do {                                                                           \
        if (!(cond)) {                                                             \
            std::fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #cond); \
            ++g_failures;                                                          \
        }                                                                          \
    } while (0)

template <class F>
bool throws(F&& f) {
    try {
        f();
    } catch (const std::exception&) {
        return true;
    }
    return false;
}

using Bytes = std::vector<std::uint8_t>;

void put_u32(Bytes& b, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) b.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
}
void put_f32(Bytes& b, float f) {
    std::uint32_t v;
    std::memcpy(&v, &f, 4);
    put_u32(b, v);
}

// One axis-aligned quad per entry, two triangles each, like the real files.
Bytes make_mesh(int quads, bool reversed_triangles = false) {
    Bytes b;
    put_u32(b, quads * 4);
    for (int q = 0; q < quads; ++q) {
        const float corner[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
        for (auto& c : corner) {
            put_f32(b, c[0] + q);
            put_f32(b, c[1]);
            put_f32(b, -2.0f);
            put_f32(b, c[0] * 0.5f);
            put_f32(b, c[1] * 0.25f);
            b.insert(b.end(), {0x10, 0x20, 0x30, 0xf0});
        }
    }
    put_u32(b, quads * 2);
    for (int i = 0; i < quads; ++i) {
        const int q = reversed_triangles ? quads - 1 - i : i, v = q * 4;
        for (int x : {v, v + 1, v + 2, v, v + 2, v + 3}) put_u32(b, x);
    }
    return b;
}

void test_parse() {
    const auto m = sh::parse_segment_mesh(make_mesh(3));
    CHECK(m.vertices.size() == 12);
    CHECK(m.indices.size() == 18);
    CHECK(m.dropped_triangles == 0);
    CHECK(m.vertices[5].pos[0] == 2.0f && m.vertices[5].pos[1] == 0.0f && m.vertices[5].pos[2] == -2.0f);
    CHECK(m.vertices[6].uv[0] == 0.5f && m.vertices[6].uv[1] == 0.25f);
    CHECK(m.vertices[0].rgba[0] == 0x10 && m.vertices[0].rgba[1] == 0x20 && m.vertices[0].rgba[2] == 0x30 &&
          m.vertices[0].rgba[3] == 0xf0);
    CHECK(m.indices[6] == 4 && m.indices[11] == 7);
    CHECK(m.min_pos[0] == 0.0f && m.max_pos[0] == 3.0f && m.min_pos[2] == -2.0f && m.max_pos[2] == -2.0f);

    // Triangle order is kept as stored.
    const auto r = sh::parse_segment_mesh(make_mesh(3, true));
    CHECK(r.indices[0] == 8);

    const auto empty = sh::parse_segment_mesh(make_mesh(0));
    CHECK(empty.vertices.empty() && empty.indices.empty());
}

void test_errors() {
    Bytes m = make_mesh(2);
    CHECK(throws([&] { sh::parse_segment_mesh(Bytes(m.begin(), m.end() - 1)); }));
    Bytes extra = m;
    extra.push_back(0);
    CHECK(throws([&] { sh::parse_segment_mesh(extra); }));
    Bytes bad_index = m;
    bad_index[bad_index.size() - 4] = 8;  // last index -> vertex 8 of 8
    CHECK(throws([&] { sh::parse_segment_mesh(bad_index); }));
    Bytes huge_count = m;
    huge_count[3] = 0x7f;
    CHECK(throws([&] { sh::parse_segment_mesh(huge_count); }));
    CHECK(throws([&] { sh::parse_segment_mesh(Bytes{1, 0, 0}); }));
}

void test_index_limit() {
    // 0x10000 vertices: triangles touching index >= 0xffff are dropped, like the original.
    const std::uint32_t n = 0x10000;
    Bytes b;
    put_u32(b, n);
    b.resize(b.size() + size_t(n) * 24, 0);
    put_u32(b, 2);
    for (std::uint32_t x : {0u, 1u, 2u, 0xfffdu, 0xfffeu, 0xffffu}) put_u32(b, x);
    const auto m = sh::parse_segment_mesh(b);
    CHECK(m.indices.size() == 3);
    CHECK(m.dropped_triangles == 1);
}

}  // namespace

int main() {
    test_parse();
    test_errors();
    test_index_limit();
    if (g_failures) {
        std::fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("all segment mesh tests passed\n");
    return 0;
}
