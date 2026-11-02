// Tests for the asset decoders using synthetic data built here (no game assets in the repo).

#include "assets/asset_decode.hpp"

#include <miniz.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_WRITE_NO_STDIO
#include <stb_image_write.h>

#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace {

int g_failures = 0;

#define CHECK(cond)                                                    \
    do {                                                               \
        if (!(cond)) {                                                 \
            std::fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #cond); \
            ++g_failures;                                              \
        }                                                              \
    } while (0)

template <class F>
bool throws(F&& f) {
    try {
        f();
    } catch (const std::exception&) {
        return true;
    }
    return false;
}

void put_u32(sh::Bytes& b, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) b.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
}

sh::Bytes zlib_compress(const sh::Bytes& in) {
    mz_ulong len = mz_compressBound(static_cast<mz_ulong>(in.size()));
    sh::Bytes out(len);
    if (mz_compress(out.data(), &len, in.data(), static_cast<mz_ulong>(in.size())) != MZ_OK) std::abort();
    out.resize(len);
    return out;
}

sh::Bytes make_jpeg(int w, int h) {
    sh::Bytes rgb(size_t(w) * h * 3);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            auto* p = &rgb[(size_t(y) * w + x) * 3];
            p[0] = 200, p[1] = 100, p[2] = 50;  // flat colour survives JPEG nearly exactly
        }
    sh::Bytes out;
    stbi_write_jpg_to_func(
        [](void* ctx, void* data, int size) {
            auto* o = static_cast<sh::Bytes*>(ctx);
            o->insert(o->end(), static_cast<std::uint8_t*>(data), static_cast<std::uint8_t*>(data) + size);
        },
        &out, w, h, 3, rgb.data(), 95);
    return out;
}

// Kind 1 container: file header, image header, JPEG, alpha size, zlib alpha plane.
sh::Bytes make_mtx_kind1(int w, int h, const sh::Bytes& jpeg, const sh::Bytes& alpha) {
    const sh::Bytes za = zlib_compress(alpha);
    sh::Bytes body;
    put_u32(body, 1);
    put_u32(body, w);
    put_u32(body, h);
    put_u32(body, static_cast<std::uint32_t>(jpeg.size()));
    body.insert(body.end(), jpeg.begin(), jpeg.end());
    put_u32(body, static_cast<std::uint32_t>(za.size()));
    body.insert(body.end(), za.begin(), za.end());
    sh::Bytes file;
    put_u32(file, 1);
    put_u32(file, static_cast<std::uint32_t>(body.size()));
    put_u32(file, 0);
    file.insert(file.end(), body.begin(), body.end());
    return file;
}

void test_names() {
    CHECK(sh::strip_fake_suffix("assets/levels/basic.xml.mp3") == "assets/levels/basic.xml");
    CHECK(sh::strip_fake_suffix("assets/levels/basic.xml") == "assets/levels/basic.xml");
    CHECK(sh::classify("assets/segments/a/b.xml.gz.mp3") == sh::AssetKind::SegmentXmlGz);
    CHECK(sh::classify("assets/segments/a/b.mesh.mp3") == sh::AssetKind::Mesh);
    CHECK(sh::classify("assets/gfx/x.png.mtx.mp3") == sh::AssetKind::Texture);
    CHECK(sh::classify("assets/gfx/x.jpg.mtx.mp3") == sh::AssetKind::Texture);
    CHECK(sh::classify("assets/music/0.ogg.mp3") == sh::AssetKind::Ogg);
    CHECK(sh::classify("assets/levels/basic.xml.mp3") == sh::AssetKind::Text);
    CHECK(sh::classify("assets/obstacles/foo.lua.mp3") == sh::AssetKind::Text);
    CHECK(sh::classify("assets/sig.mp3") == sh::AssetKind::Signature);
    CHECK(sh::classify("assets/oculussig_1234.mp3") == sh::AssetKind::Signature);
    CHECK(sh::classify("assets/whatever.bin.mp3") == sh::AssetKind::Unknown);
}

void test_zlib() {
    const sh::Bytes data(100000, 'x');
    sh::Bytes z = zlib_compress(data);
    CHECK(sh::inflate_zlib(z) == data);
    sh::Bytes trailing = z;
    trailing.push_back(0);
    CHECK(throws([&] { sh::inflate_zlib(trailing); }));
    sh::Bytes corrupt = z;
    corrupt[corrupt.size() - 1] ^= 0xff;  // Adler-32
    CHECK(throws([&] { sh::inflate_zlib(corrupt); }));
    CHECK(throws([&] { sh::inflate_zlib(sh::Bytes(z.begin(), z.begin() + z.size() / 2)); }));
}

void test_gzip() {
    const std::string text = "<segment size=\"12 10 16\"></segment>";
    const sh::Bytes data(text.begin(), text.end());
    // gzip member with FNAME set, deflate body taken from a zlib stream minus its wrapper.
    const sh::Bytes z = zlib_compress(data);
    sh::Bytes gz = {0x1f, 0x8b, 8, 0x08, 0, 0, 0, 0, 0, 0};
    for (char c : std::string("seg.xml")) gz.push_back(static_cast<std::uint8_t>(c));
    gz.push_back(0);
    gz.insert(gz.end(), z.begin() + 2, z.end() - 4);
    put_u32(gz, static_cast<std::uint32_t>(mz_crc32(MZ_CRC32_INIT, data.data(), data.size())));
    put_u32(gz, static_cast<std::uint32_t>(data.size()));
    CHECK(sh::gunzip(gz) == data);

    sh::Bytes bad_crc = gz;
    bad_crc[bad_crc.size() - 8] ^= 1;
    CHECK(throws([&] { sh::gunzip(bad_crc); }));
}

void test_mtx() {
    const int w = 24, h = 16;
    const sh::Bytes jpeg = make_jpeg(w, h);
    sh::Bytes alpha(size_t(w) * h);
    for (size_t i = 0; i < alpha.size(); ++i) alpha[i] = static_cast<std::uint8_t>(i * 7);

    const sh::Bytes file = make_mtx_kind1(w, h, jpeg, alpha);
    const sh::MtxTexture t = sh::parse_mtx(file);
    CHECK(t.kind == 1 && t.width == w && t.height == h);
    CHECK(t.alpha == alpha);
    CHECK(t.jpeg.size() == jpeg.size());

    const sh::Image img = sh::decode_mtx_rgba(file);
    CHECK(img.width == w && img.height == h && img.channels == 4);
    bool alpha_ok = true, colour_ok = true;
    for (size_t i = 0; i < alpha.size(); ++i) {
        alpha_ok &= img.pixels[i * 4 + 3] == alpha[i];
        colour_ok &= std::abs(img.pixels[i * 4 + 0] - 200) < 8 && std::abs(img.pixels[i * 4 + 1] - 100) < 8 &&
                     std::abs(img.pixels[i * 4 + 2] - 50) < 8;
    }
    CHECK(alpha_ok);
    CHECK(colour_ok);

    // Kind 0: bare JPEG after the file header.
    sh::Bytes k0;
    put_u32(k0, 0);
    put_u32(k0, static_cast<std::uint32_t>(jpeg.size()));
    put_u32(k0, 0);
    k0.insert(k0.end(), jpeg.begin(), jpeg.end());
    const sh::MtxTexture t0 = sh::parse_mtx(k0);
    CHECK(t0.kind == 0 && t0.width == w && t0.height == h && t0.alpha.empty());
    CHECK(sh::decode_mtx_rgba(k0).pixels[3] == 255);

    // Structural errors.
    sh::Bytes wrong_size = file;
    wrong_size[4] ^= 1;
    CHECK(throws([&] { sh::parse_mtx(wrong_size); }));
    CHECK(throws([&] { sh::parse_mtx(make_mtx_kind1(w + 1, h, jpeg, sh::Bytes(size_t(w + 1) * h))); }));
    CHECK(throws([&] { sh::parse_mtx(make_mtx_kind1(w, h, jpeg, sh::Bytes(10))); }));

    const sh::Bytes png = sh::encode_png(img);
    CHECK(png.size() > 8 && png[0] == 0x89 && png[1] == 'P' && png[2] == 'N' && png[3] == 'G');
}

}  // namespace

int main() {
    test_names();
    test_zlib();
    test_gzip();
    test_mtx();
    if (g_failures) {
        std::fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("all asset decode tests passed\n");
    return 0;
}
