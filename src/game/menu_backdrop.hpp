#pragma once

// The menu backdrop (Menu::Menu / Menu::draw): menu.xml gives each player mode a
// fog gradient (upper, lower) and a tint; mgDrawMenuMesh draws segments/menu/zen.mesh in the world camera with the
// room shader, its fog colours blended between the two by where the head's +z axis points vertically.

#include "physics/pmath.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace sh {
struct SegmentMesh;
}

namespace sh::game {

struct MenuBackdrop {
    struct Mode {
        float upper[3] = {0, 0, 0}, lower[3] = {0, 0, 0}, tint[3] = {0, 0, 0};
    };
    Mode modes[4];  // by player mode: 0 training, 1 classic, 2 expert, 3 zen

    // menu.xml: <menu><item name upper lower tint/>; "zen" 3, "training" 0, "expert" 2, any other name 1.
    bool parse(const std::string& xml);
    // Menu::draw's uLowerFog / uUpperFog (alpha 0) for a player mode and the head rotation (Game +0x224).
    void fog(int mode, phys::Quat head, float lower[4], float upper[4]) const;
};

// The mesh Menu::draw shows: the last of the meshes Menu::Menu loads (segments/menu/hidden.mesh, 1.mesh .. 13.mesh,
// zen.mesh), so zen.mesh.
inline constexpr const char* kMenuMeshPath = "segments/menu/zen.mesh";

// Menu::Menu splits the triangles three ways; Menu::draw uses the list of every triangle except those with all
// three vertices at y = -3 exactly.
std::vector<std::uint16_t> menu_triangles(const SegmentMesh& mesh);

}  // namespace sh::game
