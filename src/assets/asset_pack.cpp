#include "assets/asset_pack.hpp"

#include <stdexcept>

namespace sh {

namespace {
constexpr std::string_view kPrefix = "assets/";
}

AssetPack::AssetPack(const std::filesystem::path& apk_path) : apk_(apk_path) {
    for (const auto& e : apk_.entries()) {
        if (!e.name.starts_with(kPrefix)) continue;
        by_name_.emplace(strip_fake_suffix(e.name).substr(kPrefix.size()), &e);
    }
}

bool AssetPack::contains(std::string_view name) const { return by_name_.contains(std::string(name)); }

const ApkArchive::Entry& AssetPack::entry(std::string_view name) const {
    const auto it = by_name_.find(std::string(name));
    if (it == by_name_.end()) throw std::runtime_error("asset not found: " + std::string(name));
    return *it->second;
}

Bytes AssetPack::read_raw(std::string_view name) const { return apk_.read(entry(name)); }

std::string AssetPack::read_text(std::string_view name) const {
    const Bytes b = read_raw(name);
    return std::string(b.begin(), b.end());
}

Bytes AssetPack::read_mesh(std::string_view name) const { return inflate_zlib(read_raw(name)); }

std::string AssetPack::read_segment_xml(std::string_view name) const {
    const Bytes b = gunzip(read_raw(std::string(name) + ".gz"));
    return std::string(b.begin(), b.end());
}

Image AssetPack::read_texture(std::string_view name) const { return decode_mtx_rgba(read_raw(std::string(name) + ".mtx")); }

}  // namespace sh
