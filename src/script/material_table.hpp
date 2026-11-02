#pragma once

// The level's named materials (Level::loadMaterials / Level::getMaterial): read from
// materials.xml, copied whole into a script's current material by mgMaterial.

#include "physics/rigid_body.hpp"

#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace sh::script {

class MaterialTable {
public:
    // Level::loadMaterials on the text of materials.xml: <materials><material name=... .../></materials>.
    // Returns false if the root element is not <materials>.
    bool load_xml(std::string_view text);
    // Level::getMaterial: copies the first material of that name into out; false (out untouched) if none.
    bool get(const std::string& name, phys::Material& out) const;
    void add(const std::string& name, const phys::Material& m) { entries_.emplace_back(name, m); }
    const std::vector<std::pair<std::string, phys::Material>>& entries() const { return entries_; }

private:
    std::vector<std::pair<std::string, phys::Material>> entries_;
};

}  // namespace sh::script
