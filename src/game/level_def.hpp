#pragma once

// The room list a level is played from: Level::loadLevel turns a level file
// (<level><room type=".." length=".." .../></level>) into RoomDefs, and Level::start's campaign branch loads
// every level named in game.xml in order.

#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace sh::game {

struct RoomDef {
    int level = 0;          // +0: index of the level file it came from
    std::string type;       // +4: "basic/dark": rooms/<type>.lua
    int length = 0;         // +0x3c: from the length attribute (toInt), rescaled in the campaign
    std::vector<std::pair<std::string, std::string>> params;  // +0x30: every attribute but type, in order
};

struct LevelLoadOptions {
    bool campaign = false;    // Level +0xe9: rescale each level's lengths to sum to 1000
    bool allow_boss = false;  // player +0x63c == 2, or the level spec (Game +0x168) contains ':'
};

// Level::loadLevel: appends the level's rooms to `rooms`. A room whose type contains "boss" is skipped unless
// allowed (its length still counts towards the total). In the campaign, for level indices below 12 whose
// lengths do not add up to 1000, the rooms this call added get length * (1000 / total) truncated, the last one
// the rest of 1000. Malformed XML or another root element adds nothing.
void load_level(std::vector<RoomDef>& rooms, int level, const std::string& xml, const LevelLoadOptions& options);

using ReadAsset = std::function<std::optional<std::string>(const std::string& path)>;

struct Campaign {
    std::vector<RoomDef> rooms;
    int start_room = 0;  // Player +0x58c: the index of the first room of level `start_level`
    bool ok = false;     // false: game.xml or a level file is missing (the original logs and stops there)
};

// Level::start's campaign branch: game.xml's <levels><level name=".."/> entries, levels/<name>.xml each,
// loaded with campaign rescaling as levels 0, 1, ... start_room is set when level start_level begins
// (quickload leaves the saved checkpoint alone and is not modelled).
Campaign load_campaign(const ReadAsset& read, int start_level, bool allow_boss);

}  // namespace sh::game
