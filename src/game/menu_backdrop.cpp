#include "game/menu_backdrop.hpp"

#include "assets/segment_mesh.hpp"
#include "core/qi_string.hpp"

#include <pugixml.hpp>

#include <cstring>

namespace sh::game {

bool MenuBackdrop::parse(const std::string& xml) {
    pugi::xml_document doc;
    if (!doc.load_buffer(xml.data(), xml.size())) return false;
    const pugi::xml_node menu = doc.child("menu");
    if (!menu) return false;
    for (pugi::xml_node item = menu.child("item"); item; item = item.next_sibling("item")) {
        const char* name = item.attribute("name").value();
        int m = std::strcmp(name, "zen") == 0 ? 3 : 1;
        if (std::strcmp(name, "training") == 0) m = 0;
        if (std::strcmp(name, "expert") == 0) m = 2;
        const auto words = [&](const char* attr, float out[3]) {
            const std::string s = item.attribute(attr).value();
            for (int k = 0; k < 3; ++k) out[k] = to_float(get_word(s, k));
        };
        words("upper", modes[m].upper);
        words("lower", modes[m].lower);
        words("tint", modes[m].tint);
    }
    return true;
}

void MenuBackdrop::fog(int mode, phys::Quat q, float lower[4], float upper[4]) const {
    const Mode& g = modes[mode < 0 || mode > 3 ? 1 : mode];
    // The y of the +z axis turned by the head, grouped as the original computes it:
    // 2w (z v.x - x v.z) + (2w^2 - 1) v.y + y (2 q.v), with v = (0, 0, 1).
    const float v[3] = {0, 0, 1};
    const float w2 = q.w + q.w;
    const float d = (q.x * v[0] + q.y * v[1]) + q.z * v[2];
    float vy = w2 * (q.z * v[0] - q.x * v[2]);
    vy = vy + v[1] * (q.w * w2 - 1.0f);
    vy = vy + q.y * (d + d);
    const float f = -(vy * 0.5f), h = f + 1.0f;
    for (int k = 0; k < 3; ++k) {
        lower[k] = f * g.upper[k] + (1.0f - f) * g.lower[k];
        upper[k] = h * g.upper[k] + (1.0f - h) * g.lower[k];
    }
    lower[3] = upper[3] = 0.0f;
}

std::vector<std::uint16_t> menu_triangles(const SegmentMesh& mesh) {
    std::vector<std::uint16_t> out;
    out.reserve(mesh.indices.size());
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const auto y = [&](size_t k) { return mesh.vertices[mesh.indices[i + k]].pos[1]; };
        if (y(0) == -3.0f && y(1) == -3.0f && y(2) == -3.0f) continue;
        out.insert(out.end(), {mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2]});
    }
    return out;
}

}  // namespace sh::game
