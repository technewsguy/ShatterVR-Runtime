#include "script/material_table.hpp"

#include "core/qi_string.hpp"

#include <cfloat>
#include <pugixml.hpp>

namespace sh::script {

bool MaterialTable::load_xml(std::string_view text) {
    pugi::xml_document doc;
    if (!doc.load_buffer(text.data(), text.size())) return false;
    const pugi::xml_node root = doc.document_element();
    if (std::string(root.name()) != "materials") return false;
    for (const pugi::xml_node e : root.children()) {
        if (e.type() != pugi::node_element || std::string(e.name()) != "material") continue;
        // Defaults of a fresh entry (not the script defaults): colour 0 with alpha 0 unless given.
        phys::Material m;
        m.density = 1, m.friction = 1, m.restitution = 0, m.strength = FLT_MAX, m.roundness = 0, m.sound_type = 0;
        for (float& c : m.color) c = 0;
        m.group = 1, m.mask = 0xff, m.type = phys::Material::kNormal;
        if (const auto a = e.attribute("color")) {
            const std::string v = a.value();
            m.color[0] = to_float(get_word(v, 0)), m.color[1] = to_float(get_word(v, 1)), m.color[2] = to_float(get_word(v, 2));
            m.color[3] = 1.0f;
        }
        if (const auto a = e.attribute("density")) m.density = to_float(a.value());
        if (const auto a = e.attribute("friction")) m.friction = to_float(a.value());
        if (const auto a = e.attribute("restitution")) m.restitution = to_float(a.value());
        if (const auto a = e.attribute("forcelimit")) m.strength = to_float(a.value());
        // As in the original: roundness is read only when forcelimit is present ("" when it is missing).
        if (e.attribute("forcelimit")) m.roundness = to_float(e.attribute("roundness").value());
        if (const auto a = e.attribute("type")) {
            const std::string t = a.value();
            if (t == "plain") m.sound_type = 0;
            else if (t == "marble") m.sound_type = 1;
            else if (t == "glass") m.sound_type = 2;
            else if (t == "metal") m.sound_type = 3;
        }
        if (const auto a = e.attribute("static")) m.type = static_cast<std::uint8_t>(to_int(a.value()));
        if (const auto a = e.attribute("category")) m.group = static_cast<std::uint8_t>(to_int(a.value()));
        if (const auto a = e.attribute("mask")) m.mask = static_cast<std::uint8_t>(to_int(a.value()));
        entries_.emplace_back(e.attribute("name").value(), m);
    }
    return true;
}

bool MaterialTable::get(const std::string& name, phys::Material& out) const {
    for (const auto& [n, m] : entries_)
        if (n == name) {
            out = m;
            return true;
        }
    return false;
}

}  // namespace sh::script
