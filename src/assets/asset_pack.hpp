#pragma once

#include "assets/apk_archive.hpp"
#include "assets/asset_decode.hpp"

#include <filesystem>
#include <string>
#include <string_view>
#include <unordered_map>

namespace sh {

// Game assets addressed by their real names relative to assets/, e.g. "shaders/room.glsl".
// Maps to "assets/<name>.mp3" inside the APK and undoes the container formats.
class AssetPack {
public:
    explicit AssetPack(const std::filesystem::path& apk_path);

    bool contains(std::string_view name) const;
    Bytes read_raw(std::string_view name) const;     // stored bytes, still compressed/wrapped
    std::string read_text(std::string_view name) const;
    Bytes read_mesh(std::string_view name) const;     // inflated .mesh
    std::string read_segment_xml(std::string_view name) const;  // name without .gz, e.g. "segments/x/y.xml"
    Image read_texture(std::string_view name) const;  // "gfx/tiles.png" -> RGBA8

private:
    const ApkArchive::Entry& entry(std::string_view name) const;

    ApkArchive apk_;
    std::unordered_map<std::string, const ApkArchive::Entry*> by_name_;
};

}  // namespace sh
