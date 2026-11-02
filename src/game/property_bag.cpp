#include "game/property_bag.hpp"

#include "core/qi_string.hpp"
#include "physics/qi_rotate.hpp"

#include <pugixml.hpp>

namespace sh::game {

const PropertySchema::Property* PropertySchema::find(const std::string& key) const {
    for (const Property& p : properties)
        if (p.name == key) return &p;
    return nullptr;
}

const SegmentSchemas& SegmentSchemas::get() {
    static const SegmentSchemas s = [] {
        SegmentSchemas out;
        out.item = {"EItem", {{"template", ""}, {"pos", "0 0 0"}, {"hidden", "0", 1}}};
        auto extend = [&](const char* name, std::vector<PropertySchema::Property> own) {
            PropertySchema schema{name, out.item.properties};
            schema.properties.insert(schema.properties.end(), own.begin(), own.end());
            return schema;
        };
        out.box = extend("EBox", {{"size", "0 0 0"}, {"visible", "1"}, {"color", "1 1 1"}, {"tile", "0"},
                                  {"tileSize", "1"}, {"tileRot", "0"}, {"reflection", "0"}});
        out.power_up = extend("EPowerUp", {{"type", ""}, {"difficulty", "0 1"}});
        std::vector<PropertySchema::Property> obstacle = {{"rot", "0 0 0"}, {"type", ""}};
        for (int i = 0; i < 12; ++i) obstacle.push_back({"param" + std::to_string(i), ""});
        obstacle.push_back({"difficulty", "0 1"});
        obstacle.push_back({"mode", "7"});
        out.obstacle = extend("EObstacle", obstacle);
        out.decal = extend("EDecal", {{"tile", "0"}, {"size", "1 1"}, {"rot", "0 0 0"}, {"color", "1 1 1 1"}, {"blend", "1"},
                                      {"difficulty", "0 1"}});
        out.water = extend("EWater", {{"size", "4 4"}, {"resolution", "32 32"}});
        out.segment = {"Segment",
                       {{"template", ""}, {"size", "20 5 20"}, {"lightLeft", "1"}, {"lightRight", "1"}, {"lightTop", "1"},
                        {"lightBottom", "1"}, {"lightFront", "1"}, {"lightBack", "1"}, {"fogcolor", "1 1 1"},
                        {"softshadow", "0.6"}}};
        return out;
    }();
    return s;
}

void TemplateRegistry::add(const std::string& name, const std::string& key, const std::string& value) {
    Template* t = nullptr;
    for (const auto& p : templates_)
        if (p->name == name) t = p.get();
    if (!t) {
        templates_.push_back(std::make_unique<Template>());
        t = templates_.back().get();
        t->name = name;
    }
    for (auto& [k, v] : t->values)
        if (k == key) {
            v = value;
            return;
        }
    t->values.emplace_back(key, value);
}

const TemplateRegistry::Template* TemplateRegistry::find(const std::string& name) const {
    for (const auto& p : templates_)
        if (p->name == name) return p.get();
    return nullptr;
}

void TemplateRegistry::load_xml(const std::string& xml) {
    pugi::xml_document doc;
    if (!doc.load_buffer(xml.data(), xml.size())) return;
    const pugi::xml_node root = doc.document_element();
    if (std::string(root.name()) != "templates") return;
    for (const pugi::xml_node t : root.children("template")) {
        const std::string name = t.attribute("name").value();
        const pugi::xml_node props = t.first_child();  // QiXmlParser::enter: the first child element
        for (const pugi::xml_attribute a : props.attributes()) add(name, a.name(), a.value());
    }
}

void PropertyBag::set(const std::string& key, const std::string& value) {
    for (auto& [k, v] : values_)
        if (k == key) {
            v = value;
            return;
        }
    if (schema_->find(key)) values_.emplace_back(key, value);
}

void PropertyBag::read_xml(const pugi::xml_node& node) {
    for (const pugi::xml_attribute a : node.attributes())
        if (*a.name() && *a.value()) set(a.name(), a.value());
}

void PropertyBag::set_template(const TemplateRegistry* templates, const std::string& name) {
    template_ = templates ? templates->find(name) : nullptr;
}

void PropertyBag::load(const pugi::xml_node& node, const TemplateRegistry* templates) {
    reset();
    read_xml(node);
    set_template(templates, get("template"));
}

std::string PropertyBag::get(const std::string& key) const {
    for (const auto& [k, v] : values_)
        if (k == key) return v;
    if (template_)
        for (const auto& [k, v] : template_->values)
            if (k == key) return v;
    if (const PropertySchema::Property* p = schema_->find(key)) return p->default_value;
    return "";
}

const std::string* PropertyBag::find_explicit(const std::string& key) const {
    for (const auto& [k, v] : values_)
        if (k == key) return &v;
    return nullptr;
}

int PropertyBag::as_int(const std::string& key) const { return to_int(get(key)); }
float PropertyBag::as_float(const std::string& key) const { return to_float(get(key)); }

phys::Vec3 PropertyBag::as_vec3(const std::string& key) const {
    const std::string v = get(key);
    return {to_float(get_word(v, 0)), to_float(get_word(v, 1)), to_float(get_word(v, 2))};
}

phys::Quat PropertyBag::as_quat(const std::string& key) const {
    const phys::Vec3 a = as_vec3(key);
    return phys::qi_euler(a.x, a.y, a.z);
}

void PropertyBag::as_color(const std::string& key, float out[4]) const {
    const std::string v = get(key);
    out[0] = to_float(get_word(v, 0)), out[1] = to_float(get_word(v, 1)), out[2] = to_float(get_word(v, 2));
    out[3] = word_count(v) < 4 ? 1.0f : to_float(get_word(v, 3));
}

}  // namespace sh::game
