#pragma once

// The original's PropertyBag: typed attribute sets for level data (segment boxes,
// obstacles, ...). A bag has a schema (the properties it accepts, with defaults), an optional template (named
// values from templates.xml) and its explicit values. Lookups go explicit -> template -> default -> "".

#include "physics/pmath.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace pugi {
class xml_node;
}

namespace sh::game {

struct PropertySchema {
    struct Property {
        std::string name, default_value;
        int flags = 0;
    };
    std::string name;
    std::vector<Property> properties;  // a schema made while a bag had another inherits its properties first

    const Property* find(const std::string& key) const;
};

// The schemas Room::createSegment declares, with their inheritance (EBox etc. extend EItem).
struct SegmentSchemas {
    PropertySchema item, box, power_up, obstacle, decal, water, segment;
    static const SegmentSchemas& get();
};

// PropertyBag's global template list (Game::loadTemplates fills it from templates.xml).
class TemplateRegistry {
public:
    struct Template {
        std::string name;
        std::vector<std::pair<std::string, std::string>> values;
    };
    // PropertyBag::addTemplate: the named template is made if missing; the key's value is replaced or appended.
    void add(const std::string& name, const std::string& key, const std::string& value);
    const Template* find(const std::string& name) const;
    // Game::loadTemplates: <templates><template name=".."><properties a="..".../></template>...: every attribute
    // of each template's first child element (empty values included).
    void load_xml(const std::string& xml);

private:
    std::vector<std::unique_ptr<Template>> templates_;
};

class PropertyBag {
public:
    explicit PropertyBag(const PropertySchema& schema) : schema_(&schema) {}

    void reset() { values_.clear(), template_ = nullptr; }
    // PropertyBag::setValue: replaces an explicit value, else appends it if the schema has the key; other
    // keys are dropped.
    void set(const std::string& key, const std::string& value);
    // PropertyBag::readXml: every attribute with a non-empty name and value.
    void read_xml(const pugi::xml_node& node);
    // PropertyBag::setTemplate: the named template, or none.
    void set_template(const TemplateRegistry* templates, const std::string& name);
    // reset, read_xml, then the template named by the bag's own "template" value: what createSegment does per element.
    void load(const pugi::xml_node& node, const TemplateRegistry* templates);

    std::string get(const std::string& key) const;
    const std::string* find_explicit(const std::string& key) const;  // the explicit value, if set
    int as_int(const std::string& key) const;
    float as_float(const std::string& key) const;
    phys::Vec3 as_vec3(const std::string& key) const;       // words 0..2 (missing words are 0)
    phys::Quat as_quat(const std::string& key) const;       // words 0..2 as X, Y, Z angles in radians, qi_euler
    void as_color(const std::string& key, float out[4]) const;  // r g b [a]; alpha 1 with fewer than 4 words

private:
    const PropertySchema* schema_;
    const TemplateRegistry::Template* template_ = nullptr;
    std::vector<std::pair<std::string, std::string>> values_;
};

}  // namespace sh::game
