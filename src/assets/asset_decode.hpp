#pragma once

// Decoders for the container formats used by the original game's assets.
// Format descriptions:

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace sh {

using Bytes = std::vector<std::uint8_t>;

// Every asset name in the APK carries a fake ".mp3" suffix.
// "assets/levels/basic.xml.mp3" -> "assets/levels/basic.xml". Names without it are returned as-is.
std::string strip_fake_suffix(std::string_view name);

enum class AssetKind {
    Text,          // .xml .lua .glsl .fnt, plain text
    SegmentXmlGz,  // .xml.gz, gzip-wrapped segment XML
    Mesh,          // .mesh, zlib-wrapped binary
    Texture,       // .png.mtx / .jpg.mtx container
    Ogg,           // Ogg Vorbis, stored as-is
    Signature,     // Oculus store signature blobs, not game data
    Unknown,
};

// Classifies by name (with or without the fake suffix).
AssetKind classify(std::string_view name);
const char* to_string(AssetKind kind);

Bytes inflate_zlib(std::span<const std::uint8_t> data);  // verifies Adler-32
Bytes gunzip(std::span<const std::uint8_t> data);        // verifies CRC-32 and length

// Parsed view of a texture container. 'jpeg' points into the source buffer.
struct MtxTexture {
    std::uint32_t kind = 0;  // 0: JPEG only, 1: JPEG + alpha plane
    int width = 0;           // from the header (kind 1) or the JPEG frame (kind 0)
    int height = 0;
    std::span<const std::uint8_t> jpeg;
    Bytes alpha;  // kind 1 only: width*height bytes, one per pixel
};
MtxTexture parse_mtx(std::span<const std::uint8_t> file);

struct Image {
    int width = 0;
    int height = 0;
    int channels = 0;
    Bytes pixels;  // row-major, top row first
};

// Decodes a texture container to RGBA8. Kind 0 gets alpha = 255.
Image decode_mtx_rgba(std::span<const std::uint8_t> file);
Bytes encode_png(const Image& image);

}  // namespace sh
