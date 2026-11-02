#include "assets/asset_decode.hpp"

#include <miniz.h>
#include <stb_image.h>

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace sh {
namespace {

constexpr std::string_view kFakeSuffix = ".mp3";

std::uint32_t read_u32le(std::span<const std::uint8_t> d, size_t off) {
    if (off + 4 > d.size()) throw std::runtime_error("read past end of buffer");
    return std::uint32_t(d[off]) | (std::uint32_t(d[off + 1]) << 8) | (std::uint32_t(d[off + 2]) << 16) |
           (std::uint32_t(d[off + 3]) << 24);
}

// Inflates one deflate stream. window_bits > 0 expects a zlib wrapper, < 0 raw deflate.
// Returns the output and how many input bytes the stream used.
Bytes inflate_stream(std::span<const std::uint8_t> in, int window_bits, size_t& consumed) {
    mz_stream s{};
    if (mz_inflateInit2(&s, window_bits) != MZ_OK) throw std::runtime_error("inflateInit failed");
    Bytes out(std::max<size_t>(in.size() * 4, 4096));
    s.next_in = in.data();
    s.avail_in = static_cast<unsigned>(in.size());
    for (;;) {
        s.next_out = out.data() + s.total_out;
        s.avail_out = static_cast<unsigned>(out.size() - s.total_out);
        const int r = mz_inflate(&s, MZ_NO_FLUSH);
        if (r == MZ_STREAM_END) break;
        if (r != MZ_OK && r != MZ_BUF_ERROR) {
            mz_inflateEnd(&s);
            throw std::runtime_error(std::string("inflate failed: ") + mz_error(r));
        }
        if (s.avail_out == 0) {
            out.resize(out.size() * 2);
        } else if (s.avail_in == 0) {
            mz_inflateEnd(&s);
            throw std::runtime_error("inflate: truncated stream");
        }
    }
    out.resize(s.total_out);
    consumed = s.total_in;
    mz_inflateEnd(&s);
    return out;
}

bool ends_with(std::string_view s, std::string_view suffix) {
    return s.size() >= suffix.size() && s.substr(s.size() - suffix.size()) == suffix;
}

}  // namespace

std::string strip_fake_suffix(std::string_view name) {
    if (ends_with(name, kFakeSuffix)) name.remove_suffix(kFakeSuffix.size());
    return std::string(name);
}

AssetKind classify(std::string_view name) {
    const std::string n = strip_fake_suffix(name);
    const std::string_view base = std::string_view(n).substr(n.find_last_of('/') + 1);
    if (base == "sig" || base.starts_with("oculussig_")) return AssetKind::Signature;
    if (ends_with(n, ".xml.gz")) return AssetKind::SegmentXmlGz;
    if (ends_with(n, ".mesh")) return AssetKind::Mesh;
    if (ends_with(n, ".png.mtx") || ends_with(n, ".jpg.mtx")) return AssetKind::Texture;
    if (ends_with(n, ".ogg")) return AssetKind::Ogg;
    for (auto ext : {".xml", ".lua", ".glsl", ".fnt"}) {
        if (ends_with(n, ext)) return AssetKind::Text;
    }
    return AssetKind::Unknown;
}

const char* to_string(AssetKind kind) {
    switch (kind) {
        case AssetKind::Text: return "text";
        case AssetKind::SegmentXmlGz: return "segment";
        case AssetKind::Mesh: return "mesh";
        case AssetKind::Texture: return "texture";
        case AssetKind::Ogg: return "ogg";
        case AssetKind::Signature: return "signature";
        case AssetKind::Unknown: return "unknown";
    }
    return "?";
}

Bytes inflate_zlib(std::span<const std::uint8_t> data) {
    size_t used = 0;
    Bytes out = inflate_stream(data, MZ_DEFAULT_WINDOW_BITS, used);
    if (used != data.size()) throw std::runtime_error("zlib: trailing bytes after stream");
    return out;
}

Bytes gunzip(std::span<const std::uint8_t> d) {
    // RFC 1952 member header.
    if (d.size() < 18 || d[0] != 0x1f || d[1] != 0x8b || d[2] != 8) throw std::runtime_error("gzip: bad header");
    const std::uint8_t flags = d[3];
    size_t p = 10;
    auto skip_cstring = [&] {
        while (p < d.size() && d[p] != 0) ++p;
        ++p;
    };
    if (flags & 0x04) p += 2 + (d[p] | (d[p + 1] << 8));  // FEXTRA
    if (flags & 0x08) skip_cstring();                     // FNAME
    if (flags & 0x10) skip_cstring();                     // FCOMMENT
    if (flags & 0x02) p += 2;                             // FHCRC
    if (p + 8 > d.size()) throw std::runtime_error("gzip: truncated header");

    size_t used = 0;
    Bytes out = inflate_stream(d.subspan(p, d.size() - p - 8), -MZ_DEFAULT_WINDOW_BITS, used);
    const size_t trailer = p + used;
    if (trailer + 8 != d.size()) throw std::runtime_error("gzip: unexpected data after member");
    if (read_u32le(d, trailer) != mz_crc32(MZ_CRC32_INIT, out.data(), out.size()))
        throw std::runtime_error("gzip: CRC mismatch");
    if (read_u32le(d, trailer + 4) != static_cast<std::uint32_t>(out.size()))
        throw std::runtime_error("gzip: length mismatch");
    return out;
}

MtxTexture parse_mtx(std::span<const std::uint8_t> d) {
    // File header: kind, payload size (bytes after this 12-byte header), reserved (always 0).
    if (d.size() < 12) throw std::runtime_error("mtx: too small");
    MtxTexture t;
    t.kind = read_u32le(d, 0);
    if (read_u32le(d, 4) != d.size() - 12) throw std::runtime_error("mtx: payload size mismatch");
    if (read_u32le(d, 8) != 0) throw std::runtime_error("mtx: unexpected reserved value");

    int jw = 0, jh = 0, jc = 0;
    auto jpeg_info = [&](std::span<const std::uint8_t> j) {
        if (j.size() < 4 || j[0] != 0xff || j[1] != 0xd8) throw std::runtime_error("mtx: missing JPEG stream");
        if (!stbi_info_from_memory(j.data(), static_cast<int>(j.size()), &jw, &jh, &jc))
            throw std::runtime_error(std::string("mtx: bad JPEG: ") + stbi_failure_reason());
    };

    if (t.kind == 0) {
        t.jpeg = d.subspan(12);
        jpeg_info(t.jpeg);
        t.width = jw;
        t.height = jh;
        return t;
    }
    if (t.kind != 1) throw std::runtime_error("mtx: unknown kind " + std::to_string(t.kind));

    // Kind 1: image header (always 1 in the data), width, height, JPEG size; JPEG; alpha size; zlib alpha.
    if (read_u32le(d, 12) != 1) throw std::runtime_error("mtx: unexpected image header value");
    t.width = static_cast<int>(read_u32le(d, 16));
    t.height = static_cast<int>(read_u32le(d, 20));
    const size_t jpeg_size = read_u32le(d, 24);
    if (28 + jpeg_size + 4 > d.size()) throw std::runtime_error("mtx: JPEG size out of range");
    t.jpeg = d.subspan(28, jpeg_size);
    jpeg_info(t.jpeg);
    if (jw != t.width || jh != t.height) throw std::runtime_error("mtx: JPEG size differs from header");

    const size_t alpha_off = 28 + jpeg_size + 4;
    const size_t alpha_size = read_u32le(d, 28 + jpeg_size);
    if (alpha_off + alpha_size != d.size()) throw std::runtime_error("mtx: alpha stream does not end the file");
    t.alpha = inflate_zlib(d.subspan(alpha_off, alpha_size));
    if (t.alpha.size() != size_t(t.width) * size_t(t.height)) throw std::runtime_error("mtx: alpha plane size mismatch");
    return t;
}

Image decode_mtx_rgba(std::span<const std::uint8_t> file) {
    const MtxTexture t = parse_mtx(file);
    int w = 0, h = 0, c = 0;
    stbi_uc* rgb = stbi_load_from_memory(t.jpeg.data(), static_cast<int>(t.jpeg.size()), &w, &h, &c, 3);
    if (!rgb) throw std::runtime_error(std::string("mtx: JPEG decode failed: ") + stbi_failure_reason());

    Image img{w, h, 4, Bytes(size_t(w) * size_t(h) * 4)};
    const size_t n = size_t(w) * size_t(h);
    for (size_t i = 0; i < n; ++i) {
        img.pixels[i * 4 + 0] = rgb[i * 3 + 0];
        img.pixels[i * 4 + 1] = rgb[i * 3 + 1];
        img.pixels[i * 4 + 2] = rgb[i * 3 + 2];
        img.pixels[i * 4 + 3] = t.alpha.empty() ? 255 : t.alpha[i];
    }
    stbi_image_free(rgb);
    return img;
}

Bytes encode_png(const Image& image) {
    size_t len = 0;
    void* png = tdefl_write_image_to_png_file_in_memory_ex(image.pixels.data(), image.width, image.height,
                                                           image.channels, &len, MZ_DEFAULT_LEVEL, MZ_FALSE);
    if (!png) throw std::runtime_error("PNG encode failed");
    Bytes out(static_cast<std::uint8_t*>(png), static_cast<std::uint8_t*>(png) + len);
    mz_free(png);
    return out;
}

}  // namespace sh
