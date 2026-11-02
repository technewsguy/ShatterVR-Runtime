#include "assets/segment_def.hpp"

#include <pugixml.hpp>

#include <charconv>
#include <stdexcept>

namespace sh {

std::vector<float> parse_floats(std::string_view text, size_t max) {
    std::vector<float> out;
    const char* p = text.data();
    const char* end = p + text.size();
    while (p < end && out.size() < max) {
        while (p < end && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) ++p;
        if (p == end) break;
        float v = 0;
        auto [next, ec] = std::from_chars(p, end, v);
        if (ec != std::errc()) throw std::runtime_error("bad number list: " + std::string(text));
        out.push_back(v);
        p = next;
    }
    return out;
}

namespace {

void read_vec3(const pugi::xml_node& n, const char* attr, float out[3]) {
    const auto v = parse_floats(n.attribute(attr).as_string(), 3);
    for (size_t i = 0; i < v.size(); ++i) out[i] = v[i];
}

pugi::xml_document load(std::string_view xml) {
    pugi::xml_document doc;
    const auto r = doc.load_buffer(xml.data(), xml.size());
    if (!r) throw std::runtime_error(std::string("XML parse error: ") + r.description());
    return doc;
}

}  // namespace

SegmentDef parse_segment_xml(std::string_view xml) {
    const auto doc = load(xml);
    const auto seg = doc.child("segment");
    if (!seg) throw std::runtime_error("segment XML has no <segment> root");
    SegmentDef def;
    read_vec3(seg, "size", def.size);
    def.template_name = seg.attribute("template").as_string();
    for (const auto& b : seg.children("box")) {
        SegmentBox box;
        read_vec3(b, "pos", box.pos);
        read_vec3(b, "size", box.half_size);
        box.hidden = b.attribute("hidden").as_int() != 0;
        box.template_name = b.attribute("template").as_string();
        box.reflection = b.attribute("reflection").as_int() != 0;
        def.boxes.push_back(std::move(box));
    }
    return def;
}

std::optional<std::string> template_property(std::string_view templates_xml, std::string_view template_name,
                                             std::string_view property) {
    const auto doc = load(templates_xml);
    for (const auto& t : doc.select_nodes("//template")) {
        if (template_name != t.node().attribute("name").as_string()) continue;
        const auto attr = t.node().child("properties").attribute(std::string(property).c_str());
        if (attr) return std::string(attr.as_string());
        return std::nullopt;
    }
    return std::nullopt;
}

}  // namespace sh
