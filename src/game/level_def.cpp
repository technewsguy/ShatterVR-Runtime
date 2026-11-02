#include "game/level_def.hpp"

#include "core/qi_string.hpp"

#include <pugixml.hpp>

namespace sh::game {

void load_level(std::vector<RoomDef>& rooms, int level, const std::string& xml, const LevelLoadOptions& options) {
    const size_t first = rooms.size();
    int total = 0;
    pugi::xml_document doc;
    const pugi::xml_node root = doc.load_buffer(xml.data(), xml.size()) ? doc.document_element() : pugi::xml_node();
    if (root && std::string(root.name()) == "level") {
        for (const pugi::xml_node node : root.children("room")) {
            RoomDef def;
            def.level = level;
            def.type = node.attribute("type").value();
            def.length = to_int(node.attribute("length").value());
            total += def.length;
            for (const pugi::xml_attribute a : node.attributes())
                if (std::string(a.name()) != "type") def.params.emplace_back(a.name(), a.value());
            if (def.type.find("boss") == std::string::npos || options.allow_boss) rooms.push_back(std::move(def));
        }
    }
    if (options.campaign && total != 1000 && level < 12) {
        const float scale = 1000.0f / static_cast<float>(total);
        int sum = 0;
        for (size_t i = first; i < rooms.size(); ++i) {
            const int len = i + 1 == rooms.size() ? 1000 - sum : static_cast<int>(static_cast<float>(rooms[i].length) * scale);
            rooms[i].length = len;
            sum += len;
        }
    }
}

Campaign load_campaign(const ReadAsset& read, int start_level, bool allow_boss) {
    Campaign out;
    const std::optional<std::string> game = read("game.xml");
    if (!game) return out;
    pugi::xml_document doc;
    if (!doc.load_buffer(game->data(), game->size())) return out;
    const LevelLoadOptions options{true, allow_boss};
    int index = 0;
    for (const pugi::xml_node levels : doc.document_element().children("levels")) {
        for (const pugi::xml_node level : levels.children("level")) {
            const std::optional<std::string> text = read(std::string("levels/") + level.attribute("name").value() + ".xml");
            if (!text) return out;
            if (index == start_level) out.start_room = static_cast<int>(out.rooms.size());
            load_level(out.rooms, index, *text, options);
            ++index;
        }
    }
    out.ok = true;
    return out;
}

}  // namespace sh::game
