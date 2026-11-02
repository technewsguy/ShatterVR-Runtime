#pragma once

// Segment XML (<segment size template><box .../></segment>) and template property lookup.

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace sh {

struct SegmentBox {
    float pos[3] = {0, 0, 0};
    float half_size[3] = {0, 0, 0};  // the XML "size" is a half extent (checked against mesh geometry)
    bool hidden = false;
    std::string template_name;
    bool reflection = false;
};

struct SegmentDef {
    float size[3] = {0, 0, 0};
    std::string template_name;
    std::vector<SegmentBox> boxes;
};

SegmentDef parse_segment_xml(std::string_view xml);

// Reads a property from <template name="..."><properties prop="..."/></template> in templates.xml.
std::optional<std::string> template_property(std::string_view templates_xml, std::string_view template_name,
                                             std::string_view property);

// Parses up to 'max' whitespace-separated floats.
std::vector<float> parse_floats(std::string_view text, size_t max = 16);

}  // namespace sh
