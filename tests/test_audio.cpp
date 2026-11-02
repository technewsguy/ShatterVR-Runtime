// Tests for the level music stream (src/audio/level_music) on synthetic tracks: the 2 s skip,
// the next track mixing in at 32 s, the soft mix, the played-position hand-over and the swap.

#include "audio/audio_engine.hpp"
#include "audio/level_music.hpp"

#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <memory>
#include <vector>

using namespace sh::audio;

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                \
    do {                                                                           \
        if (!(cond)) {                                                             \
            std::fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #cond); \
            ++g_failures;                                                          \
        }                                                                          \
    } while (0)

std::shared_ptr<const Pcm> constant_track(float seconds, std::int16_t value) {
    auto p = std::make_shared<Pcm>();
    p->samples.assign(static_cast<size_t>(seconds * kRate) * 2, value);
    return p;
}

std::int16_t soft_mix(std::int16_t a, std::int16_t b) {
    const float fa = a / 32767.0f, fb = b / 32767.0f;
    return static_cast<std::int16_t>(static_cast<int>(((fa + fb) - fb * std::fabs(fa)) * 32767.0f));
}

void test_music_chain() {
    LevelMusic m;
    CHECK(m.location() == 0.0f);
    CHECK(!m.active());
    m.play(constant_track(40, 1000));
    CHECK(m.active());
    CHECK(m.location() == 2.0f);  // the first two seconds are skipped and counted as played
    m.set_next(constant_track(40, 2000));

    // 0x2000-byte chunks, as the original asks. Decoding starts at byte 352800 (2 s), so the 32 s mark (byte
    // 0x562200) falls 8160 bytes into chunk 645: the next track starts mixing at that byte.
    std::vector<std::int16_t> buf(0x1000);
    for (int chunk = 0; chunk < 645; ++chunk) {
        m.get_data(buf.data(), 0x2000);
        m.report_played(0x2000);
    }
    CHECK(buf[0] == 1000 && buf[0xfff] == 1000);
    m.get_data(buf.data(), 0x2000);
    CHECK(buf[8160 / 2 - 1] == 1000);
    CHECK(buf[8160 / 2] == soft_mix(1000, 2000));
    CHECK(buf[0xfff] == soft_mix(1000, 2000));
    m.report_played(0x2000);
    // Past 32 s the excess also counts for the next track (32 bytes); the current one is clamped to what was decoded.
    CHECK(m.location() == static_cast<float>(352800 + 646 * 0x2000) / 176400.0f);
    m.get_data(buf.data(), 0x2000);
    CHECK(buf[0] == soft_mix(1000, 2000));
    m.report_played(0x2000);

    // The swap: the next track is current, its played count carried over (32 + 0x2000 bytes); the old track's
    // remaining eight seconds go on mixing in under it.
    m.swap();
    CHECK(m.location() == static_cast<float>(32 + 0x2000) / 176400.0f);
    m.get_data(buf.data(), 0x2000);
    CHECK(buf[0] == soft_mix(2000, 1000));
    m.report_played(0x2000);
    CHECK(m.location() == static_cast<float>(32 + 2 * 0x2000) / 176400.0f);
}

// The boss loop (BossStream::getData through the mixer, no device): a track loops by itself; a queued one takes over
// at the end; queued "once", it plays out and the loop stops. Pitch 0.5 reads the stream at half rate.
void test_boss_loop() {
    AudioEngine e;
    const int chunk = 4410;  // frames per mix call (0.1 s)
    std::vector<std::int16_t> out(static_cast<size_t>(chunk) * 2);
    auto mix = [&] { e.mix(out.data(), chunk * 4); };
    e.boss_play(constant_track(0.25f, 1000), "music/boss_loop0.ogg");
    CHECK(e.boss_playing());
    CHECK(e.boss_name() == "music/boss_loop0.ogg");
    for (int i = 0; i < 10; ++i) mix();  // 1 s: the 0.25 s track has looped
    CHECK(e.boss_playing());
    auto near = [](int a, int b) { return std::abs(a - b) <= 1; };
    CHECK(near(out[0], 700) && near(out[2 * chunk - 1], 700)  /* the music mixer's 0.7 */);
    e.boss_set_next(constant_track(0.25f, 3000), "music/boss_loop1.ogg", false);
    for (int i = 0; i < 4; ++i) mix();
    CHECK(e.boss_name() == "music/boss_loop1.ogg");
    CHECK(near(out[2 * chunk - 1], 2100));
    e.boss_set_next(constant_track(0.15f, 2000), "music/boss_end.ogg", true);
    for (int i = 0; i < 6; ++i) mix();  // loop1 runs out into boss_end, which plays once
    CHECK(e.boss_name() == "music/boss_end.ogg");
    CHECK(!e.boss_playing());
    CHECK(out[0] == 0);

    // Pitch: at 0.5 a 0.2 s track lasts 0.4 s; with the channel off it holds.
    AudioEngine h;
    h.boss_play(constant_track(0.2f, 500), "a");
    h.boss_set_next(nullptr, "", true);
    h.set_music_channel(1, true, 1.0f, 0.5f);
    for (int i = 0; i < 3; ++i) h.mix(out.data(), chunk * 4);
    CHECK(h.boss_playing());
    h.set_music_channel(1, false, 1.0f, 0.5f);
    for (int i = 0; i < 20; ++i) h.mix(out.data(), chunk * 4);
    CHECK(h.boss_playing() && out[0] == 0);
    h.set_music_channel(1, true, 1.0f, 0.5f);
    for (int i = 0; i < 3; ++i) h.mix(out.data(), chunk * 4);
    CHECK(!h.boss_playing());
}

// Room effects: the echo comes back after its delay; the reverb leaves a tail; the blend brings the new effects in
// and only positional sounds go through them. The pools hold 16 and 4 voices (the fifth dry voice drops the first).
void test_room_effects() {
    AudioEngine e;
    const int chunk = 441;  // 10 ms
    std::vector<std::int16_t> out(static_cast<size_t>(chunk) * 2);
    auto loud = [&](AudioEngine& a) {
        a.mix(out.data(), chunk * 4);
        int m = 0;
        for (std::int16_t s : out) m = std::max(m, std::abs(static_cast<int>(s)));
        return m;
    };
    RoomEffectParams p;
    p.echo[0] = 1.0f, p.echo[1] = 0.1f, p.echo[2] = 0.5f, p.echo[3] = 0.0f;  // wet 1, 100 ms
    e.set_room_effects(p);
    for (int i = 0; i < 80; ++i) e.update_room_effects(1.0f / 60.0f);  // fully blended in
    e.play(constant_track(0.01f, 10000), 1.0f, 0.0f, 1.0f, true);
    CHECK(loud(e) >= 9000);
    for (int i = 0; i < 5; ++i) CHECK(loud(e) == 0);  // silence until the delay
    int back = 0;
    for (int i = 0; i < 10; ++i) back = std::max(back, loud(e));
    CHECK(back > 1000);

    // The same sound dry: no echo.
    e.play(constant_track(0.01f, 10000), 1.0f, 0.0f, 1.0f, false);
    for (int i = 0; i < 40; ++i) loud(e);  // (the first echo's repeats die away)
    AudioEngine d;
    d.set_room_effects(p);
    for (int i = 0; i < 80; ++i) d.update_room_effects(1.0f / 60.0f);
    d.play(constant_track(0.01f, 10000), 1.0f, 0.0f, 1.0f, false);
    CHECK(loud(d) >= 9000);
    back = 0;
    for (int i = 1; i < 30; ++i) back = std::max(back, loud(d));
    CHECK(back == 0);

    // The reverb's tail, with the first room's blend: without blending in (mix 0) nothing is added.
    RoomEffectParams r;
    r.reverb[0] = 0.5f, r.reverb[1] = 0.5f, r.reverb[2] = 0.3f;
    AudioEngine v;
    v.set_room_effects(r);
    v.play(constant_track(0.01f, 10000), 1.0f, 0.0f, 1.0f, true);
    loud(v);
    CHECK(loud(v) == 0);
    AudioEngine w;
    w.set_room_effects(r);
    for (int i = 0; i < 80; ++i) w.update_room_effects(1.0f / 60.0f);
    loud(w);  // (the reverb takes a new mix one block late)
    w.play(constant_track(0.01f, 10000), 1.0f, 0.0f, 1.0f, true);
    loud(w);
    int tail = 0;
    for (int i = 0; i < 40; ++i) tail = std::max(tail, loud(w));
    CHECK(tail > 0);

    // The mixer switch and pitch: off holds the voice; pitch 0.5 makes it last twice as long.
    AudioEngine h;
    h.set_mixers(0.7f, 1.0f, 1.0f, false, 1.0f);
    h.play(constant_track(0.02f, 8000), 1.0f, 0.0f, 1.0f, true);
    CHECK(loud(h) == 0);
    h.set_mixers(0.7f, 1.0f, 1.0f, true, 0.5f);
    for (int i = 0; i < 4; ++i) CHECK(loud(h) >= 7000);
    CHECK(loud(h) == 0);

    // Five dry voices at once: only four sound.
    AudioEngine f;
    auto tone = constant_track(0.05f, 3000);
    for (int i = 0; i < 4; ++i) f.play(tone, 1.0f, 0.0f, 1.0f, false);
    const int four = loud(f);
    AudioEngine g;
    for (int i = 0; i < 5; ++i) g.play(tone, 1.0f, 0.0f, 1.0f, false);
    CHECK(loud(g) == four);
}

// The flanger: off it passes the music; on it changes it (a constant track gets the 0.5 + 0.9 ring gain, not 1).
void test_flanger() {
    AudioEngine e;
    e.music_play(constant_track(5.0f, 1000));
    std::vector<std::int16_t> out(882);
    e.set_flanger(false, 0.0f);
    e.mix(out.data(), 882 * 2);
    CHECK(std::abs(out[400] - 700) <= 1);
    e.set_flanger(true, 1.0f);
    e.mix(out.data(), 882 * 2);
    CHECK(std::abs(out[400] - 700) > 10);
}

void test_soft_mix_clamps() {
    // Two loud tracks: the soft mix stays inside +-32767.
    LevelMusic m;
    m.play(constant_track(33, 32000), true);
    m.set_next(constant_track(5, 32000));
    std::vector<std::int16_t> buf(0x1000);
    int max = 0;
    for (int chunk = 0; chunk < 700; ++chunk) {
        m.get_data(buf.data(), 0x2000);
        m.report_played(0x2000);
        for (std::int16_t s : buf) max = std::max(max, static_cast<int>(s));
    }
    CHECK(max == soft_mix(32000, 32000));
    CHECK(max > 32000 && max <= 32767);
}

}  // namespace

int main() {
    test_music_chain();
    test_soft_mix_clamps();
    test_boss_loop();
    test_room_effects();
    test_flanger();
    if (g_failures) {
        std::fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("audio: all checks passed\n");
    return 0;
}
