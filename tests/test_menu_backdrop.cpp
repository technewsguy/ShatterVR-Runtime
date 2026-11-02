// The menu backdrop's rules: menu.xml items by mode name, the fog blend by the head's
// pitch, and the triangle list without the y = -3 floor. Synthetic XML and meshes; no game data.

#include "assets/segment_mesh.hpp"
#include "game/menu_backdrop.hpp"

#include <cmath>
#include <cstdio>

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("%-64s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) ++g_failures;
}

bool near(float a, float b) { return std::fabs(a - b) <= 1e-5f; }

}  // namespace

int main() {
    using sh::game::MenuBackdrop;
    MenuBackdrop b;
    check(b.parse("<menu>\n<item name=\"classic\" upper=\"-1 -.6 -.5\" lower=\"1.2 1.4 1.5\" tint=\"1 1 1\"/>\n"
                  "<item name=\"zen\" upper=\"1 2 3\" lower=\"4 5 6\" tint=\".5 .5 .5\"/>\n"
                  "<item name=\"training\" upper=\"7 7 7\" lower=\"8 8 8\" tint=\"0 0 0\"/></menu>"),
          "menu.xml parses");
    check(b.modes[1].upper[1] == -0.6f && b.modes[1].lower[2] == 1.5f, "classic is mode 1, leading-dot numbers");
    check(b.modes[3].upper[2] == 3.0f && b.modes[3].tint[0] == 0.5f, "zen is mode 3");
    check(b.modes[0].lower[0] == 8.0f, "training is mode 0");

    float lo[4], up[4];
    b.fog(1, {}, lo, up);  // facing forward: the gradient as given
    check(lo[0] == 1.2f && lo[1] == 1.4f && up[0] == -1.0f && up[2] == -0.5f && lo[3] == 0 && up[3] == 0,
          "level head: lower and upper as listed");
    // Pitched so that +z points straight down (-y): vy = -1, so lower moves half way to upper and upper past it.
    const float s = std::sqrt(0.5f);
    b.fog(3, {s, 0, 0, s}, lo, up);  // 90 degrees about +x takes +z to -y
    check(near(lo[0], 0.5f * 1 + 0.5f * 4) && near(up[0], 1.5f * 1 - 0.5f * 4), "head turned: blend by -vy / 2");

    sh::SegmentMesh m;
    m.vertices.resize(4);
    for (auto& v : m.vertices) v.pos[1] = -3.0f;
    m.vertices[3].pos[1] = 0.0f;
    m.indices = {0, 1, 2, 0, 1, 3, 2, 1, 0};
    const auto tris = sh::game::menu_triangles(m);
    check(tris.size() == 3 && tris[2] == 3, "triangles all at y = -3 are left out");

    std::printf(g_failures ? "%d FAILED\n" : "all passed\n", g_failures);
    return g_failures ? 1 : 0;
}
