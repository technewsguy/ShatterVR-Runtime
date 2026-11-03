// sh_play: the game, flat on the PC, read straight from the user's APK.
//
//   sh_play <game.apk> [options]
//     --room N           skip the menu and start at campaign room N (without it: the game's menu)
//     --mode M           player mode (default 1)
//     --balls B          starting balls (default 25)
//     --frames N         run N frames, then exit (with --screenshot: the last frame is saved)
//     --screenshot OUT   save the last frame as PNG (needs --frames)
//     --autofire K       shoot at the screen centre every K frames (for unattended runs)
//     --mute             no sound (the room time then runs on the game clock instead of the music)
//     --click F:X:Y      press the pointer at window pixel (X, Y) on frame F and release it on the next; it rests
//                        there from F - 2 (for unattended runs; repeatable)
//     --back F           the game's back key (Esc) on frame F (for unattended runs; repeatable)
//     --vr               play in an OpenXR headset (the window mirrors the left eye); the trigger throws along the
//                        controller and picks on the menus, the menu / B button is "back"
//     --gaze             with --vr: start with head aiming, as the Gear VR original did (any trigger throws); A / X, grip or G switches
//     --realtime         with --frames: step at 60 Hz of real time (unattended runs otherwise go as fast as they
//                        can, and then the room time does not follow the music)
//
// Controls: left click throws a ball at the pointer (and clicks the menus), F fast forward, S pause, Esc the game's
//           "back" (pause menu in a level, closing menus; at the top of the menu it quits). With --room: after a
//           game over a click restarts, R restarts from the current level's first room.
//
// The game logic is the port's game::Level (checked against the original frame by frame). Rendering is a first
// version: the rooms' segment meshes with the game's room shader and fog, bodies, power-ups, buttons and debris
// with a plain lit shader of our own. Sound: the game's sound banks, loops and level music (src/audio). Particles
// and the original's glass rendering are not there yet.

#include "assets/asset_pack.hpp"
#include "audio/audio_engine.hpp"
#include "assets/segment_mesh.hpp"
#include "common/paths.hpp"
#include "game/display.hpp"
#include "game/effects.hpp"
#include "game/level.hpp"
#include "game/menu_backdrop.hpp"
#include "game/level_def.hpp"
#include "game/player_records.hpp"
#include "game/render_level.hpp"
#include "play/xr.hpp"
#include "ui/ui_script.hpp"
#include "physics/qi_rotate.hpp"
#include "render/gl.hpp"
#include "render/math.hpp"
#include "render/shader_program.hpp"
#include "script/level_script.hpp"
#include "script/material_table.hpp"

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace sh;

namespace {

// Our 2D shader for the UI layer's batches (uMode.x: 0 solid colour, 1 texture * colour, 2 font: the colour with the
// texture's alpha). The game's 2dtex/font shaders take one uniform colour per draw instead of per vertex.
constexpr const char* kUiShader = R"(
uniform mat4 uMvpMatrix;
uniform sampler2D uTexture0;
uniform vec4 uMode;
varying vec2 vTexCoord;
varying vec4 vColor;
#ifdef VERTEX
attribute vec2 aPosition;
attribute vec2 aTexCoord;
attribute vec4 aColor;
void main(void)
{
	vTexCoord = aTexCoord;
	vColor = aColor;
	gl_Position = uMvpMatrix * vec4(aPosition, 0.0, 1.0);
}
#endif
#ifdef FRAGMENT
void main(void)
{
	vec4 t = texture2D(uTexture0, vTexCoord);
	if (uMode.x < 0.5) gl_FragColor = vColor;
	else if (uMode.x < 1.5) gl_FragColor = t * vColor;
	else gl_FragColor = vec4(vColor.rgb, t.a * vColor.a);
}
#endif
)";

struct Options {
    fs::path apk;
    int room = 0, mode = 1, balls = 25, frames = 0, autofire = 0;
    bool mode_given = false;
    bool room_given = false, mute = false, realtime = false, vr = false, gaze = false;
    fs::path screenshot;
    struct Click {
        int frame;
        float x, y;
    };
    std::vector<Click> clicks;  // --click: pointer presses for unattended runs
    struct Hold {
        int from = 0, to = 0;
    };
    std::vector<Hold> holds;     // --hold FROM:TO: the pointer held at the window's centre over these frames
    std::vector<Hold> powerups;  // --powerup F:TYPE: Level::activatePowerup on frame F (from, to = the type)
    std::vector<int> backs;     // --back: the game's back key on these frames
    bool launched_bare = false;  // started with no arguments (double-clicked): game.apk beside the program, VR if it can
};

bool parse_args(int argc, char** argv, Options& o) {
    if (argc < 2) {
        // Double-clicked: the APK named game.apk next to the program, in VR when a headset is ready, else in a window.
        const char* base = SDL_GetBasePath();
        o.apk = std::filesystem::path(base ? base : "") / "game.apk";
        o.vr = true;
        o.launched_bare = true;
        return true;
    }
    o.apk = argv[1];
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        auto next_int = [&](int& v) {
            if (i + 1 >= argc) return false;
            v = std::atoi(argv[++i]);
            return true;
        };
        if (a == "--room") {
            if (!next_int(o.room)) return false;
            o.room_given = true;
        } else if (a == "--mode") {
            if (!next_int(o.mode)) return false;
            o.mode_given = true;
        } else if (a == "--balls") {
            if (!next_int(o.balls)) return false;
        } else if (a == "--frames") {
            if (!next_int(o.frames)) return false;
        } else if (a == "--autofire") {
            if (!next_int(o.autofire)) return false;
        } else if (a == "--mute") {
            o.mute = true;
        } else if (a == "--realtime") {
            o.realtime = true;
        } else if (a == "--vr") {
            o.vr = true;
        } else if (a == "--gaze") {
            o.vr = o.gaze = true;
        } else if (a == "--back" && i + 1 < argc) {
            o.backs.push_back(std::atoi(argv[++i]));
        } else if (a == "--click" && i + 1 < argc) {
            Options::Click c{};
            if (std::sscanf(argv[++i], "%d:%f:%f", &c.frame, &c.x, &c.y) != 3) return false;
            o.clicks.push_back(c);
        } else if ((a == "--hold" || a == "--powerup") && i + 1 < argc) {
            Options::Hold c{};
            if (std::sscanf(argv[++i], "%d:%d", &c.from, &c.to) != 2) return false;
            (a == "--hold" ? o.holds : o.powerups).push_back(c);
        } else if (a == "--screenshot" && i + 1 < argc) {
            o.screenshot = argv[++i];
        } else {
            return false;
        }
    }
    return true;
}

// Assets as the game reads them: segment XML is stored gzipped in the APK.
std::optional<std::string> read_asset(const AssetPack& pack, const std::string& path) {
    try {
        if (path.rfind("segments/", 0) == 0 && path.size() > 4 && path.compare(path.size() - 4, 4, ".xml") == 0) {
            if (!pack.contains(path + ".gz")) return std::nullopt;
            return pack.read_segment_xml(path);
        }
        if (!pack.contains(path)) return std::nullopt;
        return pack.read_text(path);
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

Mat4 to_render(const game::Mat4& m) {
    Mat4 r;
    for (int i = 0; i < 16; ++i) r.m[i] = m.m[i];
    return r;
}

Mat4 translation(phys::Vec3 p) {
    Mat4 r;
    r.m[12] = p.x, r.m[13] = p.y, r.m[14] = p.z;
    return r;
}

// The rotation turning -z towards dir (shortest arc): the pointer's aim for game::GameIO::aim_override.
phys::Quat aim_towards(phys::Vec3 dir) {
    const phys::Vec3 f{0, 0, -1};
    const phys::Vec3 axis = phys::cross(f, dir);
    const float w = 1.0f + phys::dot(f, dir);
    if (w < 1e-6f) return {0, 1, 0, 0};  // straight back
    const float n = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z + w * w);
    return {axis.x / n, axis.y / n, axis.z / n, w / n};
}

// The game's sounds on the PC mixer: sound effects decoded on first use and kept, level music decoded in the
// background (a room's track is about 32 s) and driven by the Level's audio calls (its io.log lines).
class PlayAudio : public game::SoundOut {
public:
    explicit PlayAudio(const AssetPack& pack) : pack_(pack) {}

    bool open() { return engine_.open(); }
    void play(const std::string& path, float volume, float pan, float pitch, bool positional) override {
        if (auto pcm = clip(path)) engine_.play(std::move(pcm), volume, pan, pitch, positional);
    }
    void loop(int channel, const std::string& path, float volume, float pan) override {
        if (volume > 0.0f) engine_.set_loop(channel, clip(path), volume, pan);
        else engine_.stop_loop(channel);
    }

    // One of the Level's audio calls (level music, boss loop, the level music channel's stop).
    void handle(const std::string& line) {
        const size_t sp = line.find(' ');
        const std::string cmd = line.substr(0, sp);
        const std::string arg = sp == std::string::npos ? "" : line.substr(sp + 1, line.find(' ', sp + 1) - sp - 1);
        if (cmd == "setReverb") {  // low pass, echo 4, reverb 3, then the eight comb factors when reverb is on
            audio::RoomEffectParams p;
            std::istringstream in(line.substr(sp + 1));
            in >> p.low_pass;
            for (float& v : p.echo) in >> v;
            for (float& v : p.reverb) in >> v;
            for (float& v : p.combs) in >> v;
            engine_.set_room_effects(p);
        } else if (cmd == "playLevelMusic") {
            background_ = false;
            pending_next_ = {};
            engine_.music_play(decode_music(arg).get());
        } else if (cmd == "setNextLevelMusic") {
            pending_next_ = decode_music(arg);
        } else if (cmd == "swapLevelMusic") {
            background_ = false;
            if (pending_next_.valid()) engine_.music_set_next(pending_next_.get());
            pending_next_ = {};
            engine_.music_swap();
        } else if (cmd == "playBackgroundMusic") {  // the menu's music (Audio::playBackgroundMusic), on the music stream
            pending_next_ = {};
            background_ = true;
            engine_.set_music_channel(0, true, 1.0f, 1.0f);  // (its own channel in the original, never slowed)
            engine_.music_play(decode_music(arg).get());
        } else if (cmd == "stopBackgroundMusic" || cmd == "stopLevelMusic") {
            pending_next_ = {};
            engine_.music_stop();
        } else if (cmd == "stopChannel") {  // QiAudioChannel::stop on the level music (close to a boss)
            engine_.music_stop_channel();
        } else if (cmd == "playBossLoop") {
            boss_next_ = {}, boss_next_name_.clear();
            boss_start_ = boss_track(arg), boss_start_name_ = arg;
        } else if (cmd == "setNextBossLoop") {
            // The Level repeats this every frame; only a change is passed on.
            const bool once = line.size() > 2 && line.compare(line.size() - 2, 2, " 1") == 0;
            if (arg == boss_next_name_ && once == boss_next_once_) return;
            boss_next_name_ = arg, boss_next_once_ = once, boss_next_ = boss_track(arg);
        } else if (cmd == "stopBossLoop") {
            boss_start_ = {}, boss_next_ = {}, boss_next_name_.clear();
            engine_.boss_stop();
        }
    }
    // Hands decoded tracks to the streams once they are ready.
    void poll() {
        auto ready = [](const auto& f) { return f.valid() && f.wait_for(std::chrono::seconds(0)) == std::future_status::ready; };
        if (ready(pending_next_)) {
            engine_.music_set_next(pending_next_.get());
            pending_next_ = {};
        }
        if (ready(boss_start_)) {
            engine_.boss_play(boss_start_.get(), boss_start_name_);
            boss_start_ = {};
        }
        if (!boss_start_.valid() && ready(boss_next_)) {
            engine_.boss_set_next(boss_next_.get(), boss_next_name_, boss_next_once_);
            boss_next_ = {};
        }
    }
    // What the Level reads back (Audio::isBossLoopPlaying, getCurrentBossFileName); a loop still decoding counts.
    bool boss_playing() { return boss_start_.valid() || engine_.boss_playing(); }
    std::string boss_name() { return boss_start_.valid() ? boss_start_name_ : engine_.boss_name(); }
    // The Level's channel state (GameIO channels: on, +0x20 volume, +0x24 pitch) for the level music and boss loop, and
    // the room-effects mixer (+0xcc). The menu's music has a channel of its own in the original: never slowed.
    void set_channels(const game::GameIO::Channel (&ch)[3]) {
        for (int i = background_ ? 1 : 0; i < 2; ++i) engine_.set_music_channel(i, ch[i].on, ch[i].a20, ch[i].a24);
        engine_.set_mixers(music_gain_, flat_gain_, ch[2].a20, ch[2].on, ch[2].a24);
    }
    // Audio::update, once per game frame: the effects' blend; any power-up timer running flanges the level music by
    // sin(pi * the largest timer / 7); music mixer volume = 0.7 * the music setting, the sound mixers = the sound
    // setting (each at most 10, 0 when not above 0).
    void update(float dt, const float* timers, float music, float sound) {
        engine_.update_room_effects(dt);
        float t = 0.0f;
        for (int i = 0; i < 12; ++i) t = std::max(t, timers[i] / 7.0f);
        engine_.set_flanger(t > 0.0f && !background_, std::sin(t * 3.1415927f));
        music_gain_ = std::clamp(music * 0.7f, 0.0f, 10.0f);
        flat_gain_ = std::clamp(sound, 0.0f, 10.0f);
    }

    float music_location() { return engine_.music_location(); }
    bool music_playing() { return engine_.music_active(); }

private:
    std::shared_ptr<const audio::Pcm> clip(const std::string& path) {
        auto it = clips_.find(path);
        if (it != clips_.end()) return it->second;
        std::shared_ptr<const audio::Pcm> pcm;
        if (pack_.contains(path)) pcm = audio::decode_ogg(pack_.read_text(path));
        if (!pcm) std::fprintf(stderr, "sound missing or unreadable: %s\n", path.c_str());
        return clips_[path] = pcm;
    }
    // The boss tracks (about 30 s each) are decoded once and kept: the loop comes back to them.
    std::shared_future<std::shared_ptr<const audio::Pcm>> boss_track(const std::string& path) {
        auto it = boss_tracks_.find(path);
        if (it == boss_tracks_.end()) it = boss_tracks_.emplace(path, decode_music(path).share()).first;
        return it->second;
    }
    std::future<std::shared_ptr<const audio::Pcm>> decode_music(const std::string& path) {
        std::string bytes = pack_.contains(path) ? pack_.read_text(path) : std::string();  // the archive on this thread
        if (bytes.empty()) std::fprintf(stderr, "music missing: %s\n", path.c_str());
        return std::async(std::launch::async, [b = std::move(bytes)] { return audio::decode_ogg(b); });
    }

    const AssetPack& pack_;
    audio::AudioEngine engine_;
    std::map<std::string, std::shared_ptr<const audio::Pcm>> clips_;
    std::future<std::shared_ptr<const audio::Pcm>> pending_next_;
    std::map<std::string, std::shared_future<std::shared_ptr<const audio::Pcm>>> boss_tracks_;
    std::shared_future<std::shared_ptr<const audio::Pcm>> boss_start_, boss_next_;
    std::string boss_start_name_, boss_next_name_;
    bool boss_next_once_ = false;
    bool background_ = false;  // the music stream holds the menu's music
    float music_gain_ = 0.7f, flat_gain_ = 1.0f;
};

// Where the player records live: the per-user preferences folder SDL picks (outside the source tree).
fs::path records_file() {
    char* dir = SDL_GetPrefPath("ShatterVR", "ShatterVR");
    if (!dir) return {};
    fs::path p = fs::path(dir) / "records.txt";
    SDL_free(dir);
    return p;
}

struct GpuMesh {
    gl::GLuint vbo = 0, ibo = 0;
    gl::GLsizei index_count = 0;
};

class Game {
public:
    // Where a draw goes: the framebuffer and its size, the world camera's matrix, the GUI's (GUI units to clip space),
    // and which refraction pair it keeps (0 the window, 1 and 2 the eyes).
    struct View {
        gl::GLuint fbo = 0;
        int w = 0, h = 0, slot = 0;
        Mat4 world, gui;
    };

    Game(const AssetPack& pack, const Options& opt) : pack_(pack), audio_(pack), effects_(world_, rng_), display_(rng_) {
        auto read = [this](const std::string& path) { return read_asset(pack_, path); };
        // Level::loadLevel keeps boss rooms only in expert mode (2); start_level reloads the list when the mode differs.
        campaign_bosses_ = opt.mode == 2;
        const game::Campaign campaign = game::load_campaign(read, 0, campaign_bosses_);
        if (!campaign.ok || campaign.rooms.empty()) throw std::runtime_error("could not load the campaign (game.xml)");
        materials_.load_xml(read("materials.xml").value_or(""));
        templates_.load_xml(read("templates.xml").value_or(""));
        level_script_.world = &world_;
        level_script_.materials = &materials_;
        level_script_.read_asset = read;
        ctx_.world = &world_, ctx_.level = &level_script_, ctx_.rng = &rng_, ctx_.read_asset = read;
        ctx_.templates = &templates_, ctx_.scoring = &scoring_;
        scoring_.balls = opt.balls, scoring_.player_mode = opt.mode;
        level_script_.on_clear = [this](script::ObstacleState& s, const void* o) { scoring_.clear(s, o, level_script_); };
        io_.state = 3, io_.menu_fade = 0, io_.high_score = 1;
        effects_.banks.load([&pack](const std::string& p) { return pack.contains(p); });
        if (opt.mute) {
            std::printf("sound off (--mute)\n");
        } else if (audio_.open()) {
            effects_.out = &audio_;
            // Audio +0x28b1: with music on, the room time follows the music (not for runs faster than real time).
            music_sync_allowed_ = opt.frames == 0 || opt.realtime;
        } else {
            std::fprintf(stderr, "no audio device (%s); playing without sound\n", SDL_GetError());
        }
        level_ = std::make_unique<game::Level>(io_, ctx_, scoring_, effects_, campaign.rooms, true);
        level_->display = &display_;
        level_->debris = &effects_.debris;
        effects_.attach(*level_);
        effects_.scoring = &scoring_;
        scoring_.records = &records_;
        level_->room_index = std::clamp(opt.room, 0, static_cast<int>(campaign.rooms.size()) - 1);
        level_->reset();
        // Without --room the game starts in its menu (state 1) with no level loaded; with it, straight in the level.
        direct_ = opt.room_given;
        state_ = requested_ = direct_ ? 3 : 1;
        level_loaded_ = direct_;
        io_.state = state_;
        io_.properties["loaded"] = direct_ ? "1" : "0";
        io_.properties["premium"] = "1";  // the Gear VR build is the full game (assumed)
        io_.properties["deploy"] = "1";
        io_.properties["platform"] = "android";
        io_.properties["level"] = "0";
        io_.properties["paused"] = "0";
        // Unattended runs (--frames) leave the player's records alone and always start from a fresh install.
        if (opt.frames == 0) records_path_ = records_file();
        if (!records_path_.empty()) {
            std::ifstream in(records_path_, std::ios::binary);
            std::stringstream text;
            text << in.rdbuf();
            if (in && !records_.parse(text.str())) std::fprintf(stderr, "could not read %s\n", records_path_.string().c_str());
        }
        // The menu starts in the saved mode unless --mode is given (--room runs keep --mode's default, 1).
        if (opt.mode_given) records_.current_mode = std::clamp(opt.mode, 0, 3);
        scoring_.player_mode = direct_ ? std::clamp(opt.mode, 0, 3) : records_.current_mode;
        if (direct_)
            std::printf("campaign: %zu rooms; starting at %d (%s)\n", campaign.rooms.size(), level_->room_index,
                        campaign.rooms[static_cast<size_t>(level_->room_index)].type.c_str());
        else
            std::printf("campaign: %zu rooms; starting in the menu (records: %s)\n", campaign.rooms.size(),
                        records_path_.string().c_str());

        room_ = ShaderProgram(pack.read_text("shaders/room.glsl"), "room.glsl");
        if (pack.contains("menu.xml") && !backdrop_.parse(pack.read_text("menu.xml"))) std::fprintf(stderr, "could not read menu.xml\n");
        clear_ = ShaderProgram(pack.read_text("shaders/clear.glsl"), "clear.glsl");
        // Gfx::load1 / load2: these textures repeat.
        tiles_ = upload_texture(pack.read_texture("gfx/tiles.png"), true);
        const Image sprites = pack.read_texture("gfx/sprites.png");
        sprites_ = upload_texture(sprites, true);
        refmap_ = upload_texture(pack.read_texture("gfx/refmap.png"), true);
        glassnormal_ = upload_texture(pack.read_texture("gfx/glassnormal.jpg"), true);
        metalnormal_ = upload_texture(pack.read_texture("gfx/metalnormal.jpg"), true);
        credits_ = upload_texture(pack.read_texture("gfx/credits.png"), true);
        doors_ = upload_texture(pack.read_texture("gfx/doors.png"), true);
        best_distance_tex_ = upload_texture(pack.read_texture("gfx/bestdistance.png"), false);
        tex2d_ = ShaderProgram(pack.read_text("shaders/2dtex.glsl"), "2dtex.glsl");
        basic_ = ShaderProgram(pack.read_text("shaders/basic.glsl"), "basic.glsl");
        basic_glass_ = ShaderProgram(pack.read_text("shaders/basicglass.glsl"), "basicglass.glsl");
        bodytex_ = ShaderProgram(pack.read_text("shaders/bodytex.glsl"), "bodytex.glsl");
        glass_ = ShaderProgram(pack.read_text("shaders/glass.glsl"), "glass.glsl");
        glass_tex_ = ShaderProgram(pack.read_text("shaders/glasstex.glsl"), "glasstex.glsl");
        metal_ = ShaderProgram(pack.read_text("shaders/metal.glsl"), "metal.glsl");
        shadow_ = ShaderProgram(pack.read_text("shaders/softshadow.glsl"), "softshadow.glsl");
        ball_ = ShaderProgram(pack.read_text("shaders/ballreflection.glsl"), "ballreflection.glsl");
        body_depth_ = ShaderProgram(pack.read_text("shaders/bodydepth.glsl"), "bodydepth.glsl");
        body_refl_ = ShaderProgram(pack.read_text("shaders/bodyreflection.glsl"), "bodyreflection.glsl");
        sprite_refl_ = ShaderProgram(pack.read_text("shaders/spritereflection.glsl"), "spritereflection.glsl");
        ball_tex_ = upload_texture(pack.read_texture("gfx/ball.png"));
        blob_ = upload_blob(game::shadow_blob(128), 128);
        // The engine reports twice the decoded size: the APK holds only the low-res images (QiTexture::markDownscaled).
        effects_.load_particles(pack.read_text("effects/default.xml"), sprites.width * 2, sprites.height * 2);
        sprite_ = ShaderProgram(pack.read_text("shaders/sprite.glsl"), "sprite.glsl");
        ui_shader_ = ShaderProgram(kUiShader, "ui (ours)");
        const float quad[] = {0, 0, 1, 0, 0, 1, 1, 1};
        gl::GenBuffers(1, &quad_vbo_);
        gl::BindBuffer(gl::ARRAY_BUFFER, quad_vbo_);
        gl::BufferData(gl::ARRAY_BUFFER, sizeof(quad), quad, gl::STATIC_DRAW);
        gl::GenBuffers(1, &dyn_vbo_);
    }

    void resize(int w, int h) {
        if (w == width_ && h == height_) return;
        width_ = w, height_ = h;
        display_.set_geometry(w, h);
        // The scripts read the visible GUI area once, in their init().
        if (direct_ && !hud_) start_hud();
        if (!direct_ && !menu_) start_menu();
    }

    // Display::setGeometry's GUI space is 2048 units across the screen with a 2048 x 1536 canvas centred vertically. The
    // VR build's scripts lay out for one eye's square screen, so the GUI goes in the window's centred square (as an
    // eye saw it): 2048 units across that square.
    float gui_side() const { return static_cast<float>(std::min(width_, height_)); }
    float gui_scale() const { return gui_side() / 2048.0f; }
    float gui_offset_x() const { return (static_cast<float>(width_) - gui_side()) * 0.5f; }
    float gui_offset() const { return (static_cast<float>(height_) - 1536.0f * gui_scale()) * 0.5f; }

    // The mouse (window pixels) for the UI layer: held, and the press / release since the last use.
    void set_pointer(float x, float y, bool down) {
        if (down && !pointer_.down) pointer_.pressed = true;
        if (!down && pointer_.down) pointer_.released = true;
        pointer_.down = down, pointer_.x = x, pointer_.y = y;
    }
    ui::UiPointer gui_pointer() const {
        ui::UiPointer p = pointer_;
        if (vr_) return p;  // already in GUI units
        if (width_ > 0) p.x = (pointer_.x - gui_offset_x()) / gui_scale(), p.y = (pointer_.y - gui_offset()) / gui_scale();
        return p;
    }

    // mgGet for the UI scripts. Bags the port does not have yet answer like a fresh install, with numbers where the
    // scripts compute with them.
    std::string ui_property(const std::string& k) const {
        if (k == "audio.musicEnabled") return records_.music;
        if (k == "audio.soundEnabled") return records_.sound;
        std::string v = level_->get_property(k);
        if (!v.empty()) return v;
        if (k.rfind("stats.", 0) == 0) return records_.stats[scoring_.player_mode].get(k.substr(6));
        static const std::map<std::string, std::string> defaults = {
            {"game.graphics", "medium"},
            {"game.signedin", "0"},      {"game.purchasing", "0"},    {"player.adssale", "0"},
            {"player.adsshown", "1"},    {"player.adsfront", "0"},    {"player.adsonlyfree", "0"},
            {"player.rated", "1"},       {"player.startcount", "1"},  {"level.score", "0"},
            {"level.distance", "0"},     {"level.credits", "0"}};
        const auto it = defaults.find(k);
        return it == defaults.end() ? v : it->second;
    }

    // What both UI scripts reach (Game, Display, Player, Audio, ResMan).
    ui::UiEnv ui_env() {
        ui::UiEnv env;
        env.read_text = [this](const std::string& p) {
            std::optional<std::string> text = read_asset(pack_, p);
            // Ours: this build's menu has the mode menu but leaves its button undrawn, so turn the button back on.
            if (text && p == "menu/main.lua") {
                const std::string off = "--mgDraw(modeButton)";
                const size_t at = text->find(off);
                if (at != std::string::npos) text->erase(at, 2);
            }
            return text;
        };
        env.texture_size = [this](const std::string& p, int& w, int& h) {
            const UiTexture* t = ui_texture(p);
            if (!t) return false;
            w = t->w, h = t->h;
            return true;
        };
        env.get_property = [this](const std::string& k) { return ui_property(k); };
        env.set_property = [this](const std::string& k, const std::string& v) {
            level_->set_property(k, v);
        };
        env.asset_exists = [this](const std::string& path) { return pack_.contains(path); };
        env.command = [this](const std::string& cmd) { game_command(cmd); };
        env.powerup_fraction = [this](const std::string& name) {  // Level timer[type] / 7
            const int id = game::Powerup::type_id(name);
            return id >= 0 && id < 12 ? level_->timers[id] / 7.0f : 0.0f;
        };
        env.high_score = [this](int checkpoint) {
            const int mode = scoring_.player_mode;
            if (checkpoint < 0) return std::pair<int, int>{records_.best_score[std::clamp(mode, 0, 3)], 0};
            return records_.high_score(mode, checkpoint);
        };
        env.play_sound = [this](const std::string& path, float volume, float pitch) {
            if (effects_.out) audio_.play(path, volume, 0.0f, pitch, false);
        };
        env.select_sound = [this]() {  // GuiBox::select: Audio's clickdown bank, volume 0.5, pitch 1, no pan
            game::SoundBank* bank = effects_.banks.find(0x168);
            if (!bank) return;
            const int i = bank->next(rng_, frame_, 1.0f / 60.0f);
            if (i >= 0 && effects_.out) audio_.play("snd/clickdown" + std::to_string(i) + ".ogg", 0.5f, 0.0f, 1.0f, false);
        };
        const float s = gui_scale(), off = gui_offset();
        env.visible_top = -off / s, env.visible_bottom = (static_cast<float>(height_) - off) / s;
        env.visible_left = 0, env.visible_right = 2048;
        env.screen_left = -gui_offset_x() / s, env.screen_right = (static_cast<float>(width_) - gui_offset_x()) / s;
        if (vr_) {  // the GUI plane as the VR build's eye square saw it
            env.visible_top = -256, env.visible_bottom = 1792;
            env.screen_left = 0, env.screen_right = 2048;
        }
        return env;
    }

    // The HUD script (hud/main.lua): made anew for every level start (Game::startLevel loads its scene).
    void start_hud() {
        hud_.reset();
        hud_ = std::make_unique<ui::UiScript>(ui_rng_, ui_env(), gui_);
        if (!hud_->load("hud/main.lua")) std::fprintf(stderr, "hud/main.lua did not load\n");
    }

    // The menu script (menu/main.lua), loaded once.
    void start_menu() {
        menu_ = std::make_unique<ui::UiScript>(ui_rng_, ui_env(), gui_);
        if (!menu_->load("menu/main.lua")) std::fprintf(stderr, "menu/main.lua did not load\n");
        if (state_ == 1) enter_state(1);
    }

    // Game::handleCommand, as far as the PC build has the pieces.
    void game_command(const std::string& cmd) {
        const size_t sp = cmd.find(' ');
        const std::string word = cmd.substr(0, sp);
        std::string arg = sp == std::string::npos ? "" : cmd.substr(sp + 1);
        while (!arg.empty() && (arg.back() == ' ' || arg.back() == '\t')) arg.pop_back();
        while (!arg.empty() && (arg.front() == ' ' || arg.front() == '\t')) arg.erase(0, 1);
        if (word == "level.start") {
            level_name_ = arg.empty() ? "0" : arg.substr(0, arg.find(' '));
            // In zen mode every level but the credits and moregames ones is the zen level (Game::handleCommand).
            // This APK ships the zen rooms but none of their segments, and their scripts would loop forever building
            // from nothing, so without them zen plays the campaign's rooms under its own rules.
            if (scoring_.player_mode == 3 && level_name_.find("credits") == std::string::npos &&
                level_name_.find("moregames") == std::string::npos && pack_.contains("segments/zen/narrow/start.xml.gz"))
                level_name_ = "level:zen";
            requested_ = 3;
        } else if (word == "game.menu") {
            if (!direct_) requested_ = 1;  // (with --room there is no menu: a click restarts instead)
        } else if (word == "level.restart") {
            restart_level_ = true;
        } else if (word == "game.load") {
            // Game::loadIncremental: eight steps (the menu asks every 10 frames while "loaded" is "0"); what each
            // step loads is not ported, the last one makes the menu build its screens.
            if (++load_steps_ == 8) {
                io_.properties["loaded"] = "1";
                if (menu_) menu_->handle_command("load");
            }
        } else if (word == "game.mode") {
            // Only with the full game (Player +0x588, which this build has): training 0, zen 3, expert 2, anything
            // else 1 (Player::setMode). Then an "evt.mode_<name>" analytics event (not ported), the mode's menu
            // music, and "newmode" to the menu script when the mode changed.
            const std::string name = arg.substr(0, arg.find(' '));
            const int before = scoring_.player_mode;
            scoring_.player_mode = name == "training" ? 0 : name == "zen" ? 3 : name == "expert" ? 2 : 1;
            records_.current_mode = scoring_.player_mode;
            play_menu_music();
            if (scoring_.player_mode != before) {
                save_records();
                if (menu_) menu_->handle_command("newmode");
            }
        } else if (word == "game.quit") {
            quit_ = true;
        } else if (word == "game.resume") {
            io_.properties["paused"] = "0";
            pause_blend_ = 0.0f;
            io_.paused = false;
        } else if (word == "audio.togglemusic" || word == "audio.togglesound") {
            // Game::handleCommand: 0.3 -> 0.7 -> 1 -> 0 -> 0.3; music coming back from 0 restarts the menu's music.
            const bool music = word == "audio.togglemusic";
            std::string& s = music ? records_.music : records_.sound;
            const bool was_off = s != "0.3" && s != "0.7" && s != "1";
            s = s == "0.3" ? "0.7" : s == "0.7" ? "1" : s == "1" ? "0" : "0.3";
            if (music && was_off && state_ == 1) play_menu_music();
        } else if (word == "game.saveConfig") {
            save_records();
        } else if (word == "setproperty") {
            const size_t eq = arg.find('=');
            if (eq != std::string::npos) level_->set_property(arg.substr(0, eq), arg.substr(eq + 1));
        } else if (word == "audio.playBackgroundMusic") {
            if (effects_.out) audio_.handle("playBackgroundMusic " + arg);
        } else if (word == "audio.stopBackgroundMusic") {
            if (effects_.out) audio_.handle("stopBackgroundMusic");
        } else {
            std::printf("game command (not ported): %s\n", cmd.c_str());
        }
    }

    // Game::getMenuMusicPath, through Audio::playBackgroundMusic.
    void play_menu_music() {
        const int mode = scoring_.player_mode;
        const char* music = mode == 3 ? "music/menu_zen.ogg" : mode == 2 ? "music/menu_expert.ogg"
                            : mode == 0 ? "music/menu_training.ogg" : "music/menu.ogg";
        if (effects_.out) audio_.handle(std::string("playBackgroundMusic ") + music);
    }

    // Game::onLeaveState / onEnterState.
    void leave_state(int s) {
        if (s == 3) {  // Level::centerCamera, Player::save
            level_->center_camera();
            player_save();
        }
        if (s == 1 && effects_.out) audio_.handle("stopBackgroundMusic");
    }
    void enter_state(int s) {
        io_.state = s;
        if (s == 1) {
            if (menu_) menu_->handle_command("activate");
            io_.properties["paused"] = "0";
            io_.paused = false;
            play_menu_music();
        } else if (s == 3) {
            start_level();
        }
    }

    // Game::startLevel and Level::start for a checkpoint number ("quickload" keeps the room): the first room of that
    // level of game.xml, with the balls and streak the player had on reaching it (25 and 0 at the start).
    void start_level() {
        io_.properties["level"] = level_name_;
        start_hud();
        // Level::start: "level:<name>" plays that one level file from its start with 25 balls (Player::zero). Its spec
        // has a ':', so its boss rooms stay; it is outside the campaign; "credits" or "moregames" in the name makes it a
        // credits level.
        const bool own_level = level_name_.rfind("level:", 0) == 0;
        io_.spec = level_name_;
        level_->campaign = !own_level;
        level_->credits = level_name_.find("credits") != std::string::npos || level_name_.find("moregames") != std::string::npos;
        if (own_level) {
            std::vector<game::RoomDef> own;
            const std::optional<std::string> xml = read_asset(pack_, "levels/" + level_name_.substr(6) + ".xml");
            if (xml) game::load_level(own, 0, *xml, game::LevelLoadOptions{false, true});
            if (own.empty()) {  // (Game::logE in the original, and no level)
                std::fprintf(stderr, "no level %s\n", level_name_.c_str());
                requested_ = 1;
                return;
            }
            level_->rooms = std::move(own);
            level_->room_index = 0;
            own_rooms_ = true;
        } else if (own_rooms_ || (scoring_.player_mode == 2) != campaign_bosses_) {  // the list follows the player mode (boss rooms)
            own_rooms_ = false;
            campaign_bosses_ = scoring_.player_mode == 2;
            level_->rooms = game::load_campaign([this](const std::string& p) { return read_asset(pack_, p); }, 0,
                                                campaign_bosses_).rooms;
        }
        const std::vector<game::RoomDef>& rooms = level_->rooms;
        if (level_name_ != "quickload" && !own_level) {
            const int n = std::atoi(level_name_.c_str());
            int index = 0;
            for (size_t k = 0; k < rooms.size(); ++k)
                if (rooms[k].level == n) {
                    index = static_cast<int>(k);
                    break;
                }
            level_->room_index = index;
        }
        int balls = 25, streak = 0;
        if (!own_level)
            records_.load_checkpoint(scoring_.player_mode, rooms[static_cast<size_t>(level_->room_index)].level, balls, streak);
        scoring_.balls = balls, scoring_.streak = streak;
        io_.properties["paused"] = "0";
        io_.paused = false;
        level_->reset();
        level_loaded_ = true;
        std::printf("level start %s: room %d (%s), %d balls\n", level_name_.c_str(), level_->room_index,
                    rooms[static_cast<size_t>(level_->room_index)].type.c_str(), balls);
    }

    // Game::stopLevel, once the menu covers the screen.
    void stop_level() {
        level_loaded_ = false;
        if (effects_.out) audio_.handle("stopLevelMusic"), audio_.handle("stopBossLoop");  // Game::stopLevel, Level::stop
        // Game::stopLevel also unloads the HUD scene. Its boxes leave the modal slots with it: a pause menu left modal
        // ("menu" from the pause menu) would otherwise block every menu box.
        hud_.reset();
    }

    // The Escape key: the game's "back" (Game::frame sends it to the menu or the HUD script).
    void back() {
        if (direct_) {
            quit_ = true;
            return;
        }
        if (state_ == 1 && menu_) menu_->handle_command("back");
        if (state_ == 3 && hud_) hud_->handle_command("back");
    }
    bool quit_requested() const { return quit_; }
    // The vibration asked for since the last call (0 none).
    // Ours: aim and pick with the head (the Gear VR original) or with the hand; kept with the records.
    bool head_aim() const { return records_.head_aim; }
    void set_head_aim(bool on) {
        records_.head_aim = on;
        save_records();
        rumble_ = std::max(rumble_, 0.6f);  // a buzz to confirm the switch
    }
    float take_rumble() {
        const float r = rumble_;
        rumble_ = 0.0f;
        return r;
    }

    // --vr. The Gear VR original took the head orientation as Game +0x224 (it aims and steers the despawn rule) and
    // drew the GUI on a plane: GUI space scaled by 0.004 (y flipped) about (1024, 768), 10 units in front of a camera
    // that turns with the head (Display::update). Here the hand that aims replaces the gaze: its ray picks on that
    // plane and its orientation aims the balls (roll * hand, as roll * head was).
    void set_vr(bool on) {
        vr_ = on;
        if (on) pointer_.x = pointer_.y = -1e6f;  // no hand seen yet
    }
    void vr_input(phys::Quat head, const play::VrPose* hand, bool down) {
        io_.camera_rotation = head;
        if (!hand) return;
        vr_aim_ = hand->orientation;
        const phys::Vec3 o = hand->position, d = phys::qi_rotate(hand->orientation, {0, 0, -1});
        if (d.z < -1e-3f) {
            const float t = (-kGuiDistance - o.z) / d.z;
            const float x = 1024.0f + (o.x + t * d.x) / kGuiScale, y = 768.0f - (o.y + t * d.y) / kGuiScale;
            set_pointer(x, y, down);
        } else {
            set_pointer(-1e6f, -1e6f, false);
        }
    }
    // One eye: the world from the level camera moved by the eye's place in the LOCAL space and turned by the level
    // roll (the original Display adds 0.0325 sideways; the runtime's eye poses replace that), the runtime's lens
    // angles with the game's near and far (0.2, 100); the GUI plane in the LOCAL space as above.
    View eye_view(const play::VrEye& e, gl::GLuint fbo, int w, int h, int slot) const {
        View v;
        v.fbo = fbo, v.w = w, v.h = h, v.slot = slot;
        const float n = 0.2f, f = 100.0f;
        const float l = n * std::tan(e.angle_left), r = n * std::tan(e.angle_right);
        const float t = n * std::tan(e.angle_up), b = n * std::tan(e.angle_down);
        Mat4 proj;
        for (float& x : proj.m) x = 0;
        proj.m[0] = 2 * n / (r - l), proj.m[5] = 2 * n / (t - b), proj.m[8] = (r + l) / (r - l), proj.m[9] = (t + b) / (t - b);
        proj.m[10] = -(f + n) / (f - n), proj.m[11] = -1, proj.m[14] = -2 * f * n / (f - n);
        auto inverse_pose = [](phys::Vec3 p, phys::Quat q) {
            game::Mat4 m = game::pose_matrix(p, q);
            game::mat4_invert(m);
            return to_render(m);
        };
        // As Display::update: the level roll only in a level (state 3), then the crash shake (offset, roll about z).
        const phys::Quat roll = io_.state == 3 ? level_->camera_roll : phys::Quat{};
        const phys::Vec3 off = phys::qi_rotate(roll, e.pose.position);
        const phys::Quat rot = phys::qi_mul(phys::axis_angle({0, 0, 1}, -display_.shake_roll), phys::qi_mul(roll, e.pose.orientation));
        v.world = proj * inverse_pose(level_->camera + off + display_.shake_offset, rot);
        Mat4 plane;  // GUI units to the LOCAL space
        plane.m[0] = kGuiScale, plane.m[5] = -kGuiScale, plane.m[10] = kGuiScale;
        plane.m[12] = -1024.0f * kGuiScale, plane.m[13] = 768.0f * kGuiScale, plane.m[14] = -kGuiDistance;
        v.gui = proj * inverse_pose(e.pose.position, e.pose.orientation) * plane;
        return v;
    }
    static constexpr float kGuiScale = 0.004f, kGuiDistance = 10.0f;

    // Player::save: the best distance of this mode becomes max(best, the run's distance score, Player +0x590); then the
    // records go to disk.
    void player_save() {
        records_.report_score(scoring_.player_mode, level_->distance_score);
        records_.stats[scoring_.player_mode].register_distance(records_.best_score[scoring_.player_mode]);
        save_records();
    }
    std::string stats_status() const {
        const game::PlayerStats& stats = records_.stats[scoring_.player_mode];
        std::string s = "stats:";
        for (const auto& kv : stats.values) s += " " + kv.first + "=" + stats.get(kv.first);
        return s;
    }
    void activate_powerup(int type) { level_->activate_powerup(type); }
    void save_records() {
        if (records_path_.empty()) return;
        std::ofstream out(records_path_, std::ios::binary | std::ios::trunc);
        out << records_.serialize();
    }

    // One 60 Hz frame (Game::frame / Game::update): the state change and its fade, the menu transition, the scripts'
    // ticks, Level::handleInput and Level::update in a level, Display::update.
    // throw_ball: a press since the last step; held: the trigger or mouse button is down (rapid fire reads this).
    void step(bool throw_ball, bool held, float px, float py, bool fast, bool pause_pressed) {
        ui_stepped_ = true;
        io_.dt = 1.0f / 60.0f;
        io_.frame = frame_;
        io_.key_fast = fast;
        io_.key_pause_pressed = pause_pressed;
        const bool firing = throw_ball || held;
        io_.touch_pressed = throw_ball;
        io_.has_touch = firing;
        io_.touch_released = touch_held_ && !firing;
        io_.touch_count = firing ? 1 : 0;
        touch_held_ = firing;
        io_.aim_override.reset();
        if (firing && vr_) io_.aim_override = phys::qi_mul(level_->camera_roll, vr_aim_);
        else if (firing && width_ > 0)
            io_.aim_override = aim_towards(throw_ball ? display_.pixel_to_world_dir(px, py) : display_.pixel_to_world_dir(pointer_.x, pointer_.y));
        io_.properties["frame"] = std::to_string(frame_);
        // A requested state fades in over 10 frames, then the states change; the fade then goes out over 20.
        if (requested_ != state_) {
            state_fade_ = std::min(1.0f, state_fade_ + 0.1f);
            if (state_fade_ >= 1.0f) {
                leave_state(state_);
                state_ = requested_;
                enter_state(state_);
            }
        } else {
            state_fade_ = std::max(0.0f, state_fade_ - 0.05f);
        }
        if (state_ == 1 && menu_trans_ > 0.9f && level_loaded_) stop_level();
        if (restart_level_) {
            restart_level_ = false;
            if (state_ == 3) start_level();
        }
        if (effects_.out) {
            audio_.poll();
            if (io_.music_sync) io_.music_location = audio_.music_location();
            io_.music_swap = audio_.music_playing();
            io_.boss_loop_playing = audio_.boss_playing();
            io_.boss_file = audio_.boss_name();
        }
        // Game::frame: fully paused, the room-effects mixer and the level music channel are off; otherwise the mixer is
        // on again. Then Audio::update (its sound volume stands until the Level changes it in slow motion).
        const bool frozen = pause_blend_ == 1.0f;
        const float music_setting = std::strtof(records_.music.c_str(), nullptr);
        const float sound_setting = std::strtof(records_.sound.c_str(), nullptr);
        io_.channels[2].on = !frozen;
        if (frozen) io_.channels[0].on = false;
        io_.channels[2].a20 = std::clamp(sound_setting, 0.0f, 10.0f);
        io_.music_sync = music_sync_allowed_ && music_setting > 0.0f;  // Audio +0x28b1
        if (effects_.out) audio_.update(io_.dt, level_->timers, music_setting, sound_setting);
        // Game::update: the pause blend follows the "paused" property by 0.05 a frame; at 1 the game stops.
        if (io_.properties["paused"] != "0") pause_blend_ = std::min(1.0f, pause_blend_ + 0.05f);
        else pause_blend_ = std::max(0.0f, pause_blend_ - 0.05f);
        io_.pause_blend = pause_blend_;
        const bool stopped = pause_blend_ == 1.0f;
        if (state_ == 3 && level_loaded_ && !frozen && !stopped) {
            const float shake_before = display_.shake;
            level_->handle_input();
            level_->update();
            // (ours, --vr) the controller buzzes: lightly on a throw, hard on a crash (Level::hitSomething shakes).
            if (throw_ball || (held && level_->timers[5] > 0.0f)) rumble_ = std::max(rumble_, 0.2f);
            if (display_.shake > shake_before + 0.01f) rumble_ = 1.0f;
        }
        if (effects_.out) audio_.set_channels(io_.channels);  // pitch, volume, on
        // The frame's sounds and game calls: the Level's log lines, script and scoring sounds, the loops.
        for (const std::string& line : io_.log) {
            if (line.rfind("game ", 0) == 0) {
                game_command(line.substr(5));
            } else if (line.rfind("reportCheckpoint ", 0) == 0) {  // Player::reportCheckpoint, Player::save
                records_.report_checkpoint(scoring_.player_mode, std::atoi(line.c_str() + 17), scoring_.balls, scoring_.streak);
                save_records();
            } else if (line == "setState 1") {  // the credits level's end
                game_command("game.menu");
            } else if (line.rfind("registerScore ", 0) == 0) {
                records_.report_score(scoring_.player_mode, std::atoi(line.c_str() + 14));
            } else if (line.rfind("save ", 0) == 0) {  // Player::save (the Level's checkpoint save)
                player_save();
            } else if (effects_.out) {
                audio_.handle(line);
            }
        }
        io_.log.clear();
        for (const script::SoundEvent& e : level_script_.sound_events) effects_.play_event(e);
        level_script_.sound_events.clear();
        // The Level's UI commands: "gameover" goes to the menu script, the rest to the HUD (Script::handleCommand).
        std::vector<std::pair<float, float>> touches;
        if (throw_ball) touches.emplace_back((px - gui_offset_x()) / gui_scale(), (py - gui_offset()) / gui_scale());
        for (const std::string& cmd : level_script_.ui_commands) {
            if (cmd == "gameover" && menu_) menu_->handle_command(cmd);
            else if (hud_) hud_->handle_command(cmd);
        }
        level_script_.ui_commands.clear();
        // Menu::update: the transition runs at dt * 1000.5, i.e. in one frame in this build; cosine-shaped.
        const float step_t = io_.dt * 1000.5f;
        if (state_ == 1) {
            menu_t_ = menu_t_ + step_t;
            if (menu_t_ > 1.0f) {
                menu_t_ = 1.0f, menu_trans_ = 1.0f;
            } else {
                const float u = 1.0f - menu_t_;
                menu_trans_ = 1.0f - (1.0f - std::cos(u * u * 3.1415927f)) * 0.5f;
            }
        } else {
            menu_t_ = menu_t_ - step_t;
            if (menu_t_ < 0.0f) menu_t_ = 0.0f, menu_trans_ = 0.0f;
            else menu_trans_ = (1.0f - std::cos(menu_t_ * menu_t_ * 3.1415927f)) * 0.5f;
        }
        char trans[32];
        std::snprintf(trans, sizeof trans, "%g", menu_trans_);
        io_.properties["menutransition"] = trans;
        io_.menu_fade = menu_trans_;
        // The scripts: the menu while the transition shows it, the HUD while it does not hide it.
        const ui::UiPointer p = gui_pointer();
        if (menu_ && menu_trans_ > 0.05f) menu_->tick(io_.dt, touches, p);
        if (hud_ && menu_trans_ < 0.95f) hud_->tick(io_.dt, touches, p);
        pointer_.pressed = false;  // a press counts in the first step after it
        effects_.end_frame();
        // RenderLevel::fillBuffers2 draws three random alphas per laser every frame: once per game frame here too.
        laser_verts_.clear();
        if (level_loaded_) game::laser_sprites(rng_, level_->lasers, level_->camera, laser_verts_);
        game::DisplayInput d;
        d.game_state = io_.state, d.camera_rotation = io_.camera_rotation, d.eye = 0;
        d.level_camera = level_->camera, d.level_roll = level_->camera_roll, d.level_time = level_->sim_time;
        d.dt = io_.dt;
        if (!stopped) display_.update(d);
        io_.keys_pressed.clear();
        if (!frozen && !stopped) ++frame_;
    }

    bool game_over() const { return level_->game_over; }
    bool direct() const { return direct_; }

    // With --room: after a game over, back to the first room of the current level with fresh balls.
    void restart(int balls) {
        const std::vector<game::RoomDef>& rooms = level_->rooms;
        int i = level_->room_index;
        while (i > 0 && rooms[static_cast<size_t>(i - 1)].level == rooms[static_cast<size_t>(level_->room_index)].level) --i;
        level_->room_index = i;
        scoring_.balls = balls, scoring_.streak = 0;
        level_->reset();
    }

    std::string status() const {
        char buf[200];
        if (!level_loaded_) return "ShatterVR - menu";
        std::snprintf(buf, sizeof buf, "ShatterVR - balls %d  streak %d  distance %.0f  room %d %s%s", scoring_.balls,
                      scoring_.streak, level_->distance, level_->room_index, level_->current->type.c_str(),
                      level_->game_over ? "  GAME OVER" : "");
        return buf;
    }

    // For unattended runs: the sound side's state.
    std::string effects_status() {
        char buf[200];
        size_t alive = 0;
        for (const game::ParticleSystem& s : effects_.systems) alive += s.particles.size();
        std::snprintf(buf, sizeof buf, "sounds %d, particles %d spawned %zu alive, beams %zu, music %s at %.2f s (synced: %s)",
                      effects_.sounds, effects_.particles, alive, level_->lasers.size(),
                      effects_.out && audio_.music_playing() ? "playing" : "off",
                      effects_.out ? audio_.music_location() : 0.0f, io_.music_sync ? "yes" : "no");
        return buf;
    }

    // RenderLevel::draw, in its order as far as ported: the refraction buffer (quarter size,
    // double-buffered, so glass shows last frame's), then the background, room, bodies, glass, glass creases, metal,
    // and on top our stand-ins for what is not ported yet, then the sprites. Then the UI scripts (Game::draw).
    // The window: the Display's camera, and the GUI in the window's centred square.
    View window_view() const {
        View v;
        v.w = width_, v.h = height_;
        v.world = to_render(display_.main_view.projection) * to_render(display_.main_view.modelview);
        const float s = gui_scale(), W = static_cast<float>(width_), H = static_cast<float>(height_);
        for (float& e : v.gui.m) e = 0;
        v.gui.m[0] = 2.0f * s / W, v.gui.m[5] = -2.0f * s / H, v.gui.m[10] = 1, v.gui.m[15] = 1;
        v.gui.m[12] = -1.0f + 2.0f * gui_offset_x() / W, v.gui.m[13] = 1.0f - 2.0f * gui_offset() / H;
        return v;
    }
    void draw() { draw(window_view()); }
    // reuse_ui: the second eye of a frame (the UI scripts already drew).
    void draw(const View& v, bool reuse_ui = false) {
        view_ = v;
        reuse_ui_ = reuse_ui;
        refr_ = refr_sets_[std::clamp(v.slot, 0, 2)];
        gl::BindFramebuffer(gl::FRAMEBUFFER, view_.fbo);
        if (level_loaded_) {
            draw_level();
        } else {  // no level: black under the menu (its backdrop, mgDrawMenuMesh, draws in the menu script's drawWorld)
            gl::Viewport(0, 0, view_.w, view_.h);
            gl::ClearColor(0, 0, 0, 1);
            gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
        }
        draw_ui();
        gl::UseProgram(0);
    }

    void draw_level() {
        float lower[4], upper[4];
        game::level_fog(*level_, lower, upper);
        game::build_body_buffers(world_, level_->camera, level_->sim_time, bodies_);
        const Mat4& view_proj = view_.world;

        // The refraction buffer: background, room and opaque bodies with basic.glsl, then glass at 0.3 alpha where
        // glass is already behind glass (the stencil keeps the first glass layer out).
        const int rw = std::max(1, view_.w / 4), rh = std::max(1, view_.h / 4);
        refraction_size(rw, rh);
        std::swap(refr_[0], refr_[1]);  // [1] is drawn now; [0] holds the previous frame's
        gl::BindFramebuffer(gl::FRAMEBUFFER, refr_[1].fbo);
        gl::Viewport(0, 0, rw, rh);
        gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT | gl::STENCIL_BUFFER_BIT);
        draw_background(lower, upper);
        gl::Enable(gl::DEPTH_TEST);
        gl::DepthFunc(gl::LEQUAL);
        gl::Enable(gl::POLYGON_OFFSET_FILL);
        gl::PolygonOffset(1.0f, 10.0f);
        draw_room(*level_->current, view_proj, lower, upper, basic_);
        if (level_->next) draw_room(*level_->next, view_proj, lower, upper, basic_);
        gl::Disable(gl::POLYGON_OFFSET_FILL);
        draw_mesh(basic_, bodies_.basic, view_proj, lower, upper, {tiles_});
        gl::Enable(gl::BLEND);
        gl::BlendFunc(gl::SRC_ALPHA, gl::ONE_MINUS_SRC_ALPHA);
        gl::Enable(gl::STENCIL_TEST);
        gl::StencilFunc(gl::EQUAL, 1, 1);
        gl::StencilOp(gl::REPLACE, gl::KEEP, gl::KEEP);
        draw_mesh(basic_glass_, bodies_.glass, view_proj, lower, upper, {});
        draw_mesh(basic_glass_, bodies_.glass_tex, view_proj, lower, upper, {});
        gl::Disable(gl::STENCIL_TEST);
        gl::Disable(gl::BLEND);
        gl::BindFramebuffer(gl::FRAMEBUFFER, view_.fbo);

        gl::Viewport(0, 0, view_.w, view_.h);
        gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
        draw_background(lower, upper);
        gl::Enable(gl::DEPTH_TEST);
        gl::DepthFunc(gl::LEQUAL);
        draw_room(*level_->current, view_proj, lower, upper, room_);
        if (level_->next) draw_room(*level_->next, view_proj, lower, upper, room_);
        draw_mesh(bodytex_, bodies_.basic, view_proj, lower, upper, {tiles_});
        gl::Enable(gl::POLYGON_OFFSET_FILL);
        gl::PolygonOffset(1.0f, 10.0f);
        draw_mesh(glass_, bodies_.glass, view_proj, lower, upper, {glassnormal_, refr_[0].color});
        draw_mesh(glass_tex_, bodies_.glass_tex, view_proj, lower, upper, {credits_, refr_[0].color});
        gl::Disable(gl::POLYGON_OFFSET_FILL);
        // The creases go through whichever glass shader drew last, as 2-pixel lines.
        ShaderProgram& edge_shader = bodies_.glass_tex.indices.empty() ? glass_ : glass_tex_;
        draw_lines(edge_shader, bodies_.glass_edges, view_proj, lower, upper,
                   {bodies_.glass_tex.indices.empty() ? glassnormal_ : credits_, refr_[0].color});
        gl::UseProgram(metal_.id());
        const float cam[3] = {level_->camera.x, level_->camera.y, level_->camera.z};
        gl::Uniform3fv(metal_.uniform("uCameraPos"), 1, cam);
        // The debris shards through the same glass shader as the creases.
        debris_verts_.clear();
        game::debris_triangles(effects_.debris.vertices, level_->camera, debris_verts_);
        draw_triangle_list(edge_shader, debris_verts_, view_proj, lower, upper,
                           {bodies_.glass_tex.indices.empty() ? glassnormal_ : credits_, refr_[0].color});
        draw_mesh(metal_, bodies_.metal, view_proj, lower, upper, {refmap_, metalnormal_});

        sprite_verts_.clear();
        discs_.clear();
        shadow_verts_.clear();
        game::entity_sprites(*level_, world_, display_.rotation, sprite_verts_, discs_, shadow_verts_);
        game::button_sprites(*level_, sprite_verts_);
        // Room::addDecals: most join the sprites; the doors' go to their own pass with doors.png.
        decal_verts_.clear();
        game::decal_sprites(*level_->current, level_->camera, scoring_.player_mode, sprite_verts_, decal_verts_);
        if (level_->next) game::decal_sprites(*level_->next, level_->camera, scoring_.player_mode, sprite_verts_, decal_verts_);
        game::obstacle_score_sprites(*level_->current, world_, level_->camera, level_->sim_time, sprite_verts_, shadow_verts_);
        draw_reflections(view_proj, lower, upper);
        draw_shadows(view_proj);
        draw_discs(view_proj);
        draw_particles(view_proj);
        draw_best_distance(view_proj);
    }

    void draw_triangle_list(ShaderProgram& sh, const std::vector<game::BodyVertex>& tris, const Mat4& view_proj,
                            const float lower[4], const float upper[4], std::initializer_list<gl::GLuint> textures) {
        if (tris.empty()) return;
        use_body_shader(sh, view_proj, lower, upper, textures);
        gl::GLint attrs[4];
        bind_body_vertices(sh, tris.data(), tris.size(), attrs);
        gl::DrawArrays(gl::TRIANGLES, 0, static_cast<gl::GLsizei>(tris.size()));
        for (gl::GLint a : attrs)
            if (a >= 0) gl::DisableVertexAttribArray(a);
    }

    // The floor reflections (RenderLevel::draw): the reflective box tops mark the stencil where they are visible
    // (bodydepth.glsl, no colour), then inside it, without depth testing: the sprites (spritereflection.glsl) and,
    // once per pixel, the body buffers and the rooms' standing walls mirrored at y = 0 (bodyreflection.glsl).
    void draw_reflections(const Mat4& view_proj, const float lower[4], const float upper[4]) {
        refl_mask_.clear(), refl_walls_.clear();
        game::room_reflections(*level_->current, refl_mask_, refl_walls_);
        if (level_->next) game::room_reflections(*level_->next, refl_mask_, refl_walls_);
        if (refl_mask_.empty()) return;
        gl::Clear(gl::STENCIL_BUFFER_BIT);
        gl::Enable(gl::STENCIL_TEST);
        gl::StencilFunc(gl::ALWAYS, 1, 1);
        gl::StencilOp(gl::KEEP, gl::KEEP, gl::REPLACE);
        gl::ColorMask(gl::FALSE_, gl::FALSE_, gl::FALSE_, gl::FALSE_);
        gl::DepthMask(gl::FALSE_);
        draw_triangle_list(body_depth_, refl_mask_, view_proj, lower, upper, {});
        gl::ColorMask(gl::TRUE_, gl::TRUE_, gl::TRUE_, gl::TRUE_);
        gl::Disable(gl::DEPTH_TEST);
        gl::StencilFunc(gl::EQUAL, 1, 1);
        gl::StencilOp(gl::KEEP, gl::KEEP, gl::KEEP);
        draw_sprites(view_proj, sprite_verts_, sprites_, sprite_refl_, gl::SRC_ALPHA);
        gl::StencilOp(gl::KEEP, gl::KEEP, gl::ZERO);
        gl::Enable(gl::BLEND);
        gl::BlendFunc(gl::SRC_ALPHA, gl::ONE_MINUS_SRC_ALPHA);
        gl::DepthMask(gl::FALSE_);
        for (const game::BodyMesh* m : {&bodies_.basic, &bodies_.glass, &bodies_.glass_tex, &bodies_.metal})
            draw_mesh(body_refl_, *m, view_proj, lower, upper, {});
        draw_triangle_list(body_refl_, refl_walls_, view_proj, lower, upper, {});
        gl::Disable(gl::BLEND);
        gl::DepthMask(gl::TRUE_);
        gl::Enable(gl::DEPTH_TEST);
        gl::Disable(gl::STENCIL_TEST);
    }

    // Soft shadows (softshadow.glsl with the generated blob), blended, no depth writes.
    void draw_shadows(const Mat4& view_proj) {
        if (shadow_verts_.empty()) return;
        std::vector<std::uint16_t> idx;
        for (size_t q = 0; q < shadow_verts_.size() / 4; ++q) {
            const auto b = static_cast<std::uint16_t>(q * 4);
            for (int k : {0, 1, 2, 0, 2, 3}) idx.push_back(static_cast<std::uint16_t>(b + k));
        }
        gl::UseProgram(shadow_.id());
        gl::UniformMatrix4fv(shadow_.uniform("uMvpMatrix"), 1, gl::FALSE_, view_proj.m);
        gl::Uniform1i(shadow_.uniform("uTexture0"), 0);
        gl::ActiveTexture(gl::TEXTURE0);
        gl::BindTexture(gl::TEXTURE_2D, blob_);
        gl::Enable(gl::BLEND);
        gl::BlendFunc(gl::SRC_ALPHA, gl::ONE_MINUS_SRC_ALPHA);
        gl::DepthMask(gl::FALSE_);
        gl::BindBuffer(gl::ARRAY_BUFFER, dyn_vbo_);
        gl::BufferData(gl::ARRAY_BUFFER, static_cast<gl::GLsizeiptr>(shadow_verts_.size() * sizeof(game::ShadowVertex)),
                       shadow_verts_.data(), gl::STREAM_DRAW);
        if (!sprite_ibo_) gl::GenBuffers(1, &sprite_ibo_);
        gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, sprite_ibo_);
        gl::BufferData(gl::ELEMENT_ARRAY_BUFFER, static_cast<gl::GLsizeiptr>(idx.size() * sizeof(std::uint16_t)), idx.data(),
                       gl::STREAM_DRAW);
        const auto stride = static_cast<gl::GLsizei>(sizeof(game::ShadowVertex));
        const gl::GLint p = shadow_.attrib("aPosition"), t = shadow_.attrib("aTexCoord"), a = shadow_.attrib("aAlpha");
        gl::EnableVertexAttribArray(p);
        gl::VertexAttribPointer(p, 3, gl::FLOAT, gl::FALSE_, stride, reinterpret_cast<const void*>(offsetof(game::ShadowVertex, pos)));
        gl::EnableVertexAttribArray(t);
        gl::VertexAttribPointer(t, 2, gl::UNSIGNED_BYTE, gl::TRUE_, stride, reinterpret_cast<const void*>(offsetof(game::ShadowVertex, uv)));
        gl::EnableVertexAttribArray(a);
        gl::VertexAttribPointer(a, 1, gl::UNSIGNED_BYTE, gl::TRUE_, stride, reinterpret_cast<const void*>(offsetof(game::ShadowVertex, alpha)));
        gl::DrawElements(gl::TRIANGLES, static_cast<gl::GLsizei>(idx.size()), gl::UNSIGNED_SHORT, nullptr);
        gl::DisableVertexAttribArray(p), gl::DisableVertexAttribArray(t), gl::DisableVertexAttribArray(a);
        gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, 0);
        gl::DepthMask(gl::TRUE_);
        gl::Disable(gl::BLEND);
    }

    // The balls' and power-ups' refractive discs (ballreflection.glsl: ball.png and the refraction texture), blended.
    void draw_discs(const Mat4& view_proj) {
        if (discs_.indices.empty()) return;
        gl::UseProgram(ball_.id());
        gl::UniformMatrix4fv(ball_.uniform("uMvpMatrix"), 1, gl::FALSE_, view_proj.m);
        gl::Uniform1i(ball_.uniform("uTexture0"), 0);
        gl::Uniform1i(ball_.uniform("uTexture1"), 1);
        gl::ActiveTexture(gl::TEXTURE0 + 1);
        gl::BindTexture(gl::TEXTURE_2D, refr_[0].color);
        gl::ActiveTexture(gl::TEXTURE0);
        gl::BindTexture(gl::TEXTURE_2D, ball_tex_);
        gl::Enable(gl::BLEND);
        gl::BlendFunc(gl::SRC_ALPHA, gl::ONE_MINUS_SRC_ALPHA);
        gl::BindBuffer(gl::ARRAY_BUFFER, dyn_vbo_);
        gl::BufferData(gl::ARRAY_BUFFER, static_cast<gl::GLsizeiptr>(discs_.vertices.size() * sizeof(game::ReflectionVertex)),
                       discs_.vertices.data(), gl::STREAM_DRAW);
        if (!body_ibo_) gl::GenBuffers(1, &body_ibo_);
        gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, body_ibo_);
        gl::BufferData(gl::ELEMENT_ARRAY_BUFFER, static_cast<gl::GLsizeiptr>(discs_.indices.size() * sizeof(std::uint32_t)),
                       discs_.indices.data(), gl::STREAM_DRAW);
        const auto stride = static_cast<gl::GLsizei>(sizeof(game::ReflectionVertex));
        const gl::GLint p = ball_.attrib("aPosition"), t = ball_.attrib("aTexCoord"), c = ball_.attrib("aColor");
        gl::EnableVertexAttribArray(p);
        gl::VertexAttribPointer(p, 3, gl::FLOAT, gl::FALSE_, stride, reinterpret_cast<const void*>(offsetof(game::ReflectionVertex, pos)));
        gl::EnableVertexAttribArray(t);
        gl::VertexAttribPointer(t, 3, gl::UNSIGNED_BYTE, gl::TRUE_, stride,
                                reinterpret_cast<const void*>(offsetof(game::ReflectionVertex, normal)));
        gl::EnableVertexAttribArray(c);
        gl::VertexAttribPointer(c, 4, gl::FLOAT, gl::FALSE_, stride, reinterpret_cast<const void*>(offsetof(game::ReflectionVertex, color)));
        gl::DrawElements(gl::TRIANGLES, static_cast<gl::GLsizei>(discs_.indices.size()), gl::UNSIGNED_INT, nullptr);
        gl::DisableVertexAttribArray(p), gl::DisableVertexAttribArray(t), gl::DisableVertexAttribArray(c);
        gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, 0);
        gl::Disable(gl::BLEND);
    }

    // UI textures by asset path, with the size the engine reports (twice the decoded size); null when missing.
    struct UiTexture {
        gl::GLuint tex = 0;
        int w = 0, h = 0;
    };
    const UiTexture* ui_texture(const std::string& path) {
        auto it = ui_textures_.find(path);
        if (it == ui_textures_.end()) {
            UiTexture t;
            if (!path.empty() && pack_.contains(path + ".mtx")) {
                try {
                    const Image img = pack_.read_texture(path);
                    t.tex = upload_texture(img), t.w = img.width * 2, t.h = img.height * 2;
                } catch (const std::exception&) {
                }
            }
            it = ui_textures_.emplace(path, t).first;
        }
        return it->second.tex ? &it->second : nullptr;
    }

    // Game::draw's UI part: the menu's world pass and its screens while the transition shows the menu (mgSetZ depth
    // and the menu backdrop are not ported: the world pass draws flat), the HUD while it is not hidden, then
    // GuiBox::tick, then the state-change fade as black over everything (its look is assumed).
    // The scripts draw once per game step, as the original ticks and draws in lockstep (the menu counts its loading
    // frames in draw()); a second eye, or a display frame without a step, draws the same batches again.
    void draw_ui() {
        const bool fresh = !reuse_ui_ && ui_stepped_;
        if (fresh) {
            ui_stepped_ = false;
            ui_world_.clear(), ui_menu_.clear(), ui_hud_.clear();
            if (menu_ && menu_trans_ > 0.05f) {
                ui_world_ = menu_->draw_world();
                ui_menu_ = menu_->draw();
            }
            if (hud_ && menu_trans_ < 0.95f) ui_hud_ = hud_->draw();
        }
        draw_ui_batches(ui_world_);
        draw_ui_batches(ui_menu_);
        draw_ui_batches(ui_hud_);
        if (vr_ && pointer_.x > -1e5f) {  // ours: where the hand's ray meets the GUI plane (the original had gaze)
            ui::UiBatch dot;
            const float r = pointer_.down ? 10.0f : 14.0f, x = pointer_.x, y = pointer_.y;
            for (const auto& c : {std::array<float, 2>{x - r, y - r}, {x + r, y - r}, {x + r, y + r}, {x - r, y - r},
                                  {x + r, y + r}, {x - r, y + r}})
                dot.vertices.push_back({c[0], c[1], 0, 0, {1, 1, 1, 0.85f}});
            draw_ui_batches({dot});
        }
        if (fresh) {
            gui_.end_frame(pointer_.released);
            pointer_.released = false;
        }
        if (state_fade_ > 0.0f) {
            const float s = gui_scale(), off = gui_offset(), ox = gui_offset_x();
            float t = -off / s, b = (static_cast<float>(height_) - off) / s;
            float l = -ox / s, r = (static_cast<float>(width_) - ox) / s;  // the whole window
            if (vr_) t = l = -40000.0f, b = r = 40000.0f;                   // in the headset: all around the GUI plane
            ui::UiBatch fade;
            const float a = state_fade_;
            for (const auto& c : {std::array<float, 2>{l, t}, {r, t}, {r, b}, {l, t}, {r, b}, {l, b}})
                fade.vertices.push_back({c[0], c[1], 0, 0, {0, 0, 0, a}});
            draw_ui_batches({fade});
        }
    }

    // UI batches in GUI space (our 2D shader: solid, textured, or font alpha), blended over the frame.
    // A menu_mesh marker draws the menu backdrop in between.
    void draw_ui_batches(const std::vector<ui::UiBatch>& batches) {
        size_t start = 0;
        for (size_t i = 0; i <= batches.size(); ++i) {
            if (i < batches.size() && !batches[i].menu_mesh) continue;
            draw_ui_run(std::span(batches).subspan(start, i - start));
            if (i < batches.size()) draw_menu_mesh();
            start = i + 1;
        }
    }
    void draw_ui_run(std::span<const ui::UiBatch> batches) {
        if (batches.empty()) return;
        const Mat4& m = view_.gui;
        const float W = static_cast<float>(view_.w), H = static_cast<float>(view_.h);
        // A GUI point in framebuffer pixels (y up), for the canvas windows' scissor boxes.
        auto pixel = [&](float x, float y, float& px, float& py) {
            const float cx = m.m[0] * x + m.m[4] * y + m.m[12], cy = m.m[1] * x + m.m[5] * y + m.m[13];
            const float cw = m.m[3] * x + m.m[7] * y + m.m[15];
            px = (cx / cw * 0.5f + 0.5f) * W, py = (cy / cw * 0.5f + 0.5f) * H;
        };
        gl::Disable(gl::DEPTH_TEST);
        gl::Enable(gl::BLEND);
        gl::BlendFunc(gl::SRC_ALPHA, gl::ONE_MINUS_SRC_ALPHA);
        gl::UseProgram(ui_shader_.id());
        gl::UniformMatrix4fv(ui_shader_.uniform("uMvpMatrix"), 1, gl::FALSE_, m.m);
        gl::Uniform1i(ui_shader_.uniform("uTexture0"), 0);
        gl::ActiveTexture(gl::TEXTURE0);
        const gl::GLint p = ui_shader_.attrib("aPosition"), t = ui_shader_.attrib("aTexCoord"), c = ui_shader_.attrib("aColor");
        const auto stride = static_cast<gl::GLsizei>(sizeof(ui::UiVertex));
        gl::BindBuffer(gl::ARRAY_BUFFER, dyn_vbo_);
        gl::EnableVertexAttribArray(p), gl::EnableVertexAttribArray(t), gl::EnableVertexAttribArray(c);
        for (const ui::UiBatch& b : batches) {
            const UiTexture* tex = b.texture.empty() ? nullptr : ui_texture(b.texture);
            if (!b.texture.empty() && !tex) continue;
            const float mode[4] = {tex ? (b.font ? 2.0f : 1.0f) : 0.0f, 0, 0, 0};
            gl::Uniform4fv(ui_shader_.uniform("uMode"), 1, mode);
            if (tex) gl::BindTexture(gl::TEXTURE_2D, tex->tex);
            if (b.clip) {  // a canvas window, in GUI units: its projected bounding box
                const auto& k = *b.clip;
                float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
                for (const auto& corner : {std::array<float, 2>{k[0], k[1]}, {k[2], k[1]}, {k[2], k[3]}, {k[0], k[3]}}) {
                    float px, py;
                    pixel(corner[0], corner[1], px, py);
                    x0 = std::min(x0, px), x1 = std::max(x1, px), y0 = std::min(y0, py), y1 = std::max(y1, py);
                }
                gl::Enable(gl::SCISSOR_TEST);
                gl::Scissor(static_cast<gl::GLint>(x0), static_cast<gl::GLint>(y0), static_cast<gl::GLsizei>(x1 - x0),
                            static_cast<gl::GLsizei>(y1 - y0));
            } else {
                gl::Disable(gl::SCISSOR_TEST);
            }
            gl::BufferData(gl::ARRAY_BUFFER, static_cast<gl::GLsizeiptr>(b.vertices.size() * sizeof(ui::UiVertex)),
                           b.vertices.data(), gl::STREAM_DRAW);
            gl::VertexAttribPointer(p, 2, gl::FLOAT, gl::FALSE_, stride, reinterpret_cast<const void*>(offsetof(ui::UiVertex, x)));
            gl::VertexAttribPointer(t, 2, gl::FLOAT, gl::FALSE_, stride, reinterpret_cast<const void*>(offsetof(ui::UiVertex, u)));
            gl::VertexAttribPointer(c, 4, gl::FLOAT, gl::FALSE_, stride, reinterpret_cast<const void*>(offsetof(ui::UiVertex, color)));
            gl::DrawArrays(gl::TRIANGLES, 0, static_cast<gl::GLsizei>(b.vertices.size()));
        }
        gl::DisableVertexAttribArray(p), gl::DisableVertexAttribArray(t), gl::DisableVertexAttribArray(c);
        gl::Disable(gl::SCISSOR_TEST);
        gl::Disable(gl::BLEND);
        gl::Enable(gl::DEPTH_TEST);
    }

    // RenderLevel::clear: the fog gradient over the whole target (clear.glsl).
    void draw_background(const float lower[4], const float upper[4]) {
        gl::Disable(gl::DEPTH_TEST);
        gl::UseProgram(clear_.id());
        gl::Uniform4fv(clear_.uniform("uLowerFog"), 1, lower);
        gl::Uniform4fv(clear_.uniform("uUpperFog"), 1, upper);
        const gl::GLint qpos = clear_.attrib("aPosition");
        gl::BindBuffer(gl::ARRAY_BUFFER, quad_vbo_);
        gl::EnableVertexAttribArray(qpos);
        gl::VertexAttribPointer(qpos, 2, gl::FLOAT, gl::FALSE_, 0, nullptr);
        gl::DrawArrays(gl::TRIANGLE_STRIP, 0, 4);
        gl::DisableVertexAttribArray(qpos);
    }

    // The two refraction targets: colour texture plus depth/stencil, (re)made at this size.
    struct Target {
        gl::GLuint fbo = 0, color = 0, depth = 0;
        int w = 0, h = 0;
    };
    void refraction_size(int w, int h) {
        for (int k = 0; k < 2; ++k) {
            Target& t = refr_[k];
            if (t.w == w && t.h == h) continue;
            if (!t.fbo) gl::GenFramebuffers(1, &t.fbo), gl::GenTextures(1, &t.color), gl::GenRenderbuffers(1, &t.depth);
            gl::BindTexture(gl::TEXTURE_2D, t.color);
            gl::TexImage2D(gl::TEXTURE_2D, 0, gl::RGBA, w, h, 0, gl::RGBA, gl::UNSIGNED_BYTE, nullptr);
            gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, gl::LINEAR);
            gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, gl::LINEAR);
            gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_S, gl::CLAMP_TO_EDGE);
            gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_T, gl::CLAMP_TO_EDGE);
            gl::BindRenderbuffer(gl::RENDERBUFFER, t.depth);
            gl::RenderbufferStorage(gl::RENDERBUFFER, gl::DEPTH24_STENCIL8, w, h);
            gl::BindFramebuffer(gl::FRAMEBUFFER, t.fbo);
            gl::FramebufferTexture2D(gl::FRAMEBUFFER, gl::COLOR_ATTACHMENT0, gl::TEXTURE_2D, t.color, 0);
            gl::FramebufferRenderbuffer(gl::FRAMEBUFFER, gl::DEPTH_STENCIL_ATTACHMENT, gl::RENDERBUFFER, t.depth);
            gl::ClearColor(0, 0, 0, 1);
            gl::Clear(gl::COLOR_BUFFER_BIT);
            gl::BindFramebuffer(gl::FRAMEBUFFER, 0);
            t.w = w, t.h = h;
        }
    }

    // A body mesh with one of the game's body shaders (whatever attributes it declares) and textures on units 0, 1.
    void bind_body_vertices(ShaderProgram& sh, const game::BodyVertex* data, size_t count, gl::GLint attrs[4]) {
        gl::BindBuffer(gl::ARRAY_BUFFER, dyn_vbo_);
        gl::BufferData(gl::ARRAY_BUFFER, static_cast<gl::GLsizeiptr>(count * sizeof(game::BodyVertex)), data, gl::STREAM_DRAW);
        const auto stride = static_cast<gl::GLsizei>(sizeof(game::BodyVertex));
        attrs[0] = sh.attrib("aPosition"), attrs[1] = sh.attrib("aNormal"), attrs[2] = sh.attrib("aTexCoord");
        attrs[3] = sh.attrib("aColor");
        const int sizes[4] = {3, 3, 2, 4};
        const gl::GLenum types[4] = {gl::FLOAT, gl::FLOAT, gl::FLOAT, gl::UNSIGNED_BYTE};
        const size_t offsets[4] = {offsetof(game::BodyVertex, pos), offsetof(game::BodyVertex, normal),
                                   offsetof(game::BodyVertex, uv), offsetof(game::BodyVertex, rgba)};
        for (int i = 0; i < 4; ++i) {
            if (attrs[i] < 0) continue;
            gl::EnableVertexAttribArray(attrs[i]);
            gl::VertexAttribPointer(attrs[i], sizes[i], types[i], i == 3 ? gl::TRUE_ : gl::FALSE_, stride,
                                    reinterpret_cast<const void*>(offsets[i]));
        }
    }
    void use_body_shader(ShaderProgram& sh, const Mat4& view_proj, const float lower[4], const float upper[4],
                         std::initializer_list<gl::GLuint> textures) {
        gl::UseProgram(sh.id());
        gl::UniformMatrix4fv(sh.uniform("uMvpMatrix"), 1, gl::FALSE_, view_proj.m);
        gl::Uniform4fv(sh.uniform("uLowerFog"), 1, lower);
        gl::Uniform4fv(sh.uniform("uUpperFog"), 1, upper);
        int unit = 0;
        for (gl::GLuint t : textures) {
            gl::ActiveTexture(gl::TEXTURE0 + static_cast<gl::GLenum>(unit));
            gl::BindTexture(gl::TEXTURE_2D, t);
            const char* names[2] = {"uTexture0", "uTexture1"};
            if (unit < 2) gl::Uniform1i(sh.uniform(names[unit]), unit);
            ++unit;
        }
        gl::ActiveTexture(gl::TEXTURE0);
    }
    void draw_mesh(ShaderProgram& sh, const game::BodyMesh& mesh, const Mat4& view_proj, const float lower[4],
                   const float upper[4], std::initializer_list<gl::GLuint> textures) {
        if (mesh.indices.empty()) return;
        use_body_shader(sh, view_proj, lower, upper, textures);
        gl::GLint attrs[4];
        bind_body_vertices(sh, mesh.vertices.data(), mesh.vertices.size(), attrs);
        if (!body_ibo_) gl::GenBuffers(1, &body_ibo_);
        gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, body_ibo_);
        gl::BufferData(gl::ELEMENT_ARRAY_BUFFER, static_cast<gl::GLsizeiptr>(mesh.indices.size() * sizeof(std::uint32_t)),
                       mesh.indices.data(), gl::STREAM_DRAW);
        gl::DrawElements(gl::TRIANGLES, static_cast<gl::GLsizei>(mesh.indices.size()), gl::UNSIGNED_INT, nullptr);
        for (gl::GLint a : attrs)
            if (a >= 0) gl::DisableVertexAttribArray(a);
        gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, 0);
    }
    void draw_lines(ShaderProgram& sh, const std::vector<game::BodyVertex>& lines, const Mat4& view_proj,
                    const float lower[4], const float upper[4], std::initializer_list<gl::GLuint> textures) {
        if (lines.empty()) return;
        use_body_shader(sh, view_proj, lower, upper, textures);
        gl::GLint attrs[4];
        bind_body_vertices(sh, lines.data(), lines.size(), attrs);
        gl::LineWidth(2.0f);
        gl::DrawArrays(gl::LINES, 0, static_cast<gl::GLsizei>(lines.size()));
        gl::LineWidth(1.0f);
        for (gl::GLint a : attrs)
            if (a >= 0) gl::DisableVertexAttribArray(a);
    }

    // The Level's particle systems with the game's sprite shader (premultiplied: ONE, ONE_MINUS_SRC_ALPHA), in list
    // order, without depth writes.
    // RenderLevel::draw's best-distance marker (game::best_distance_marker): Gfx::drawRectangle's unit quad (position =
    // uv) through T(0, 0.5, z) S(2.16, -0.8, 0) T(-0.5, -0.5, 0) with 2dtex.glsl, colour (1, 1, 1, alpha), alpha
    // blended, depth test on, depth writes off.
    void draw_best_distance(const Mat4& view_proj) {
        game::BestDistanceMarker mk;
        const int mode = scoring_.player_mode;
        if (!game::best_distance_marker(*level_, mode, records_.best_score[std::clamp(mode, 0, 3)], mk)) return;
        Mat4 model;
        model.m[0] = 2.16f, model.m[5] = -0.8f, model.m[10] = 0.0f;
        model.m[12] = 2.16f * -0.5f, model.m[13] = 0.5f + -0.8f * -0.5f, model.m[14] = mk.z;
        const Mat4 mvp = view_proj * model;
        gl::UseProgram(tex2d_.id());
        gl::UniformMatrix4fv(tex2d_.uniform("uMvpMatrix"), 1, gl::FALSE_, mvp.m);
        const float color[4] = {1, 1, 1, mk.alpha}, zero[4] = {0, 0, 0, 0}, one[4] = {1, 1, 0, 0};
        gl::Uniform4fv(tex2d_.uniform("uColor"), 1, color);
        gl::Uniform2fv(tex2d_.uniform("uTexOffset0"), 1, zero);
        gl::Uniform2fv(tex2d_.uniform("uTexScale0"), 1, one);
        gl::Uniform1i(tex2d_.uniform("uTexture0"), 0);
        gl::ActiveTexture(gl::TEXTURE0);
        gl::BindTexture(gl::TEXTURE_2D, best_distance_tex_);
        gl::Enable(gl::DEPTH_TEST);
        gl::DepthMask(gl::FALSE_);
        gl::Enable(gl::BLEND);
        gl::BlendFunc(gl::SRC_ALPHA, gl::ONE_MINUS_SRC_ALPHA);
        const gl::GLint pos = tex2d_.attrib("aPosition"), uv = tex2d_.attrib("aTexCoord");
        gl::BindBuffer(gl::ARRAY_BUFFER, quad_vbo_);  // (0,0) (1,0) (0,1) (1,1) as a strip
        gl::EnableVertexAttribArray(pos);
        gl::VertexAttribPointer(pos, 2, gl::FLOAT, gl::FALSE_, 0, nullptr);
        if (uv >= 0) {
            gl::EnableVertexAttribArray(uv);
            gl::VertexAttribPointer(uv, 2, gl::FLOAT, gl::FALSE_, 0, nullptr);
        }
        gl::DrawArrays(gl::TRIANGLE_STRIP, 0, 4);
        gl::DisableVertexAttribArray(pos);
        if (uv >= 0) gl::DisableVertexAttribArray(uv);
        gl::Disable(gl::BLEND);
        gl::DepthMask(gl::TRUE_);
    }

    void draw_particles(const Mat4& view_proj) {
        // (sprite_verts_ already holds this frame's entity sprites)
        for (const game::ParticleSystem& s : effects_.systems)
            game::append_sprites(s, level_->camera, level_->speed, sprite_verts_);
        sprite_verts_.insert(sprite_verts_.end(), laser_verts_.begin(), laser_verts_.end());
        draw_sprites(view_proj, sprite_verts_, sprites_);
        draw_sprites(view_proj, decal_verts_, doors_);
    }

    void draw_sprites(const Mat4& view_proj, const std::vector<game::SpriteVertex>& verts, gl::GLuint texture) {
        draw_sprites(view_proj, verts, texture, sprite_, gl::ONE);
    }
    // sh: sprite.glsl (premultiplied: src factor ONE) or spritereflection.glsl (src factor SRC_ALPHA).
    void draw_sprites(const Mat4& view_proj, const std::vector<game::SpriteVertex>& verts, gl::GLuint texture,
                      ShaderProgram& sh, gl::GLenum src_factor) {
        if (verts.empty()) return;
        const size_t quads = std::min<size_t>(verts.size() / 4, 0x4000);
        sprite_index_.clear();
        for (size_t q = 0; q < quads; ++q) {
            const auto b = static_cast<std::uint16_t>(q * 4);
            for (int k : {0, 1, 2, 0, 2, 3}) sprite_index_.push_back(static_cast<std::uint16_t>(b + k));
        }
        if (!sprite_ibo_) gl::GenBuffers(1, &sprite_ibo_);
        gl::UseProgram(sh.id());
        gl::UniformMatrix4fv(sh.uniform("uMvpMatrix"), 1, gl::FALSE_, view_proj.m);
        gl::Uniform1i(sh.uniform("uTexture0"), 0);
        gl::ActiveTexture(gl::TEXTURE0);
        gl::BindTexture(gl::TEXTURE_2D, texture);
        gl::Enable(gl::BLEND);
        gl::BlendFunc(src_factor, gl::ONE_MINUS_SRC_ALPHA);
        gl::DepthMask(gl::FALSE_);
        gl::BindBuffer(gl::ARRAY_BUFFER, dyn_vbo_);
        gl::BufferData(gl::ARRAY_BUFFER, static_cast<gl::GLsizeiptr>(quads * 4 * sizeof(game::SpriteVertex)),
                       verts.data(), gl::STREAM_DRAW);
        gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, sprite_ibo_);
        gl::BufferData(gl::ELEMENT_ARRAY_BUFFER, static_cast<gl::GLsizeiptr>(sprite_index_.size() * sizeof(std::uint16_t)),
                       sprite_index_.data(), gl::STREAM_DRAW);
        const auto stride = static_cast<gl::GLsizei>(sizeof(game::SpriteVertex));
        const gl::GLint attrs[4] = {sh.attrib("aPosition"), sh.attrib("aTexCoord"), sh.attrib("aColor"),
                                    sh.attrib("aAlpha")};
        const int sizes[4] = {3, 2, 4, 1};
        const size_t offsets[4] = {offsetof(game::SpriteVertex, pos), offsetof(game::SpriteVertex, uv),
                                   offsetof(game::SpriteVertex, color), offsetof(game::SpriteVertex, blend)};
        for (int i = 0; i < 4; ++i) {
            if (attrs[i] < 0) continue;
            gl::EnableVertexAttribArray(attrs[i]);
            gl::VertexAttribPointer(attrs[i], sizes[i], gl::FLOAT, gl::FALSE_, stride, reinterpret_cast<const void*>(offsets[i]));
        }
        gl::DrawElements(gl::TRIANGLES, static_cast<gl::GLsizei>(sprite_index_.size()), gl::UNSIGNED_SHORT, nullptr);
        for (int i = 0; i < 4; ++i)
            if (attrs[i] >= 0) gl::DisableVertexAttribArray(attrs[i]);
        gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, 0);
        gl::DepthMask(gl::TRUE_);
        gl::Disable(gl::BLEND);
    }

private:
    // An alpha-only texture (stored as RGBA: white with the alpha), e.g. the shadow blob.
    static gl::GLuint upload_blob(const std::vector<std::uint8_t>& alpha, int size) {
        Image img{size, size, 4, Bytes(alpha.size() * 4)};
        for (size_t i = 0; i < alpha.size(); ++i) {
            img.pixels[4 * i] = img.pixels[4 * i + 1] = img.pixels[4 * i + 2] = 255;
            img.pixels[4 * i + 3] = alpha[i];
        }
        return upload_texture(img);
    }

    static gl::GLuint upload_texture(const Image& img, bool repeat = false) {
        gl::GLuint tex = 0;
        gl::GenTextures(1, &tex);
        gl::BindTexture(gl::TEXTURE_2D, tex);
        gl::PixelStorei(gl::UNPACK_ALIGNMENT, 1);
        gl::TexImage2D(gl::TEXTURE_2D, 0, gl::RGBA, img.width, img.height, 0, gl::RGBA, gl::UNSIGNED_BYTE, img.pixels.data());
        gl::GenerateMipmap(gl::TEXTURE_2D);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, gl::LINEAR_MIPMAP_LINEAR);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, gl::LINEAR);
        const gl::GLint wrap = static_cast<gl::GLint>(repeat ? gl::REPEAT : gl::CLAMP_TO_EDGE);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_S, wrap);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_T, wrap);
        return tex;
    }

    const GpuMesh* mesh(const std::string& path) {
        auto it = meshes_.find(path);
        if (it != meshes_.end()) return it->second.index_count ? &it->second : nullptr;
        GpuMesh g;
        if (pack_.contains(path)) {
            try {
                const SegmentMesh m = parse_segment_mesh(pack_.read_mesh(path));
                gl::GenBuffers(1, &g.vbo);
                gl::BindBuffer(gl::ARRAY_BUFFER, g.vbo);
                gl::BufferData(gl::ARRAY_BUFFER, static_cast<gl::GLsizeiptr>(m.vertices.size() * sizeof(MeshVertex)),
                               m.vertices.data(), gl::STATIC_DRAW);
                gl::GenBuffers(1, &g.ibo);
                gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, g.ibo);
                gl::BufferData(gl::ELEMENT_ARRAY_BUFFER, static_cast<gl::GLsizeiptr>(m.indices.size() * sizeof(std::uint16_t)),
                               m.indices.data(), gl::STATIC_DRAW);
                g.index_count = static_cast<gl::GLsizei>(m.indices.size());
            } catch (const std::exception& e) {
                std::fprintf(stderr, "mesh %s: %s\n", path.c_str(), e.what());
            }
        }
        it = meshes_.emplace(path, g).first;
        return it->second.index_count ? &it->second : nullptr;
    }

    void draw_room(const game::Room& room, const Mat4& view_proj, const float lower[4], const float upper[4],
                   ShaderProgram& sh) {
        gl::UseProgram(sh.id());
        gl::Uniform4fv(sh.uniform("uLowerFog"), 1, lower);
        gl::Uniform4fv(sh.uniform("uUpperFog"), 1, upper);
        gl::Uniform1i(sh.uniform("uTexture0"), 0);
        gl::ActiveTexture(gl::TEXTURE0);
        gl::BindTexture(gl::TEXTURE_2D, tiles_);
        for (const game::Room::SegmentRecord& rec : room.segments) {
            const GpuMesh* g = mesh(rec.mesh);
            if (!g) continue;
            const phys::Vec3 base = room.body->position;  // segment coordinates are in the room body's frame
            draw_segment_mesh(sh, *g, view_proj * translation({base.x, base.y, base.z + rec.z}));
        }
        gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, 0);
    }
    void draw_segment_mesh(ShaderProgram& sh, const GpuMesh& g, const Mat4& mvp) {
        const gl::GLint pos = sh.attrib("aPosition"), uv = sh.attrib("aTexCoord"), col = sh.attrib("aColor");
        const auto stride = static_cast<gl::GLsizei>(sizeof(MeshVertex));
        gl::UniformMatrix4fv(sh.uniform("uMvpMatrix"), 1, gl::FALSE_, mvp.m);
        gl::BindBuffer(gl::ARRAY_BUFFER, g.vbo);
        gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, g.ibo);
        gl::EnableVertexAttribArray(pos);
        gl::VertexAttribPointer(pos, 3, gl::FLOAT, gl::FALSE_, stride, reinterpret_cast<const void*>(offsetof(MeshVertex, pos)));
        gl::EnableVertexAttribArray(uv);
        gl::VertexAttribPointer(uv, 2, gl::FLOAT, gl::FALSE_, stride, reinterpret_cast<const void*>(offsetof(MeshVertex, uv)));
        gl::EnableVertexAttribArray(col);
        gl::VertexAttribPointer(col, 4, gl::UNSIGNED_BYTE, gl::TRUE_, stride,
                                reinterpret_cast<const void*>(offsetof(MeshVertex, rgba)));
        gl::DrawElements(gl::TRIANGLES, g.index_count, gl::UNSIGNED_SHORT, nullptr);
        gl::DisableVertexAttribArray(pos);
        gl::DisableVertexAttribArray(uv);
        gl::DisableVertexAttribArray(col);
    }

    // Menu::draw (mgDrawMenuMesh, in the world viewport): depth cleared, zen.mesh untransformed with room.glsl and
    // tiles (anisotropy 2 in the original), depth test and writes on, no blending, the fog from menu.xml for the
    // player mode turned by the head. (The "low" graphics setting's roommenulow.glsl is not ported.)
    void draw_menu_mesh() {
        if (!menu_mesh_.index_count) {
            if (menu_mesh_.vbo || !pack_.contains(game::kMenuMeshPath)) return;  // tried before / missing
            try {
                const SegmentMesh m = parse_segment_mesh(pack_.read_mesh(game::kMenuMeshPath));
                const std::vector<std::uint16_t> tris = game::menu_triangles(m);
                gl::GenBuffers(1, &menu_mesh_.vbo);
                gl::BindBuffer(gl::ARRAY_BUFFER, menu_mesh_.vbo);
                gl::BufferData(gl::ARRAY_BUFFER, static_cast<gl::GLsizeiptr>(m.vertices.size() * sizeof(MeshVertex)),
                               m.vertices.data(), gl::STATIC_DRAW);
                gl::GenBuffers(1, &menu_mesh_.ibo);
                gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, menu_mesh_.ibo);
                gl::BufferData(gl::ELEMENT_ARRAY_BUFFER, static_cast<gl::GLsizeiptr>(tris.size() * sizeof(std::uint16_t)),
                               tris.data(), gl::STATIC_DRAW);
                menu_mesh_.index_count = static_cast<gl::GLsizei>(tris.size());
            } catch (const std::exception& e) {
                std::fprintf(stderr, "mesh %s: %s\n", game::kMenuMeshPath, e.what());
                if (!menu_mesh_.vbo) gl::GenBuffers(1, &menu_mesh_.vbo);
                return;
            }
        }
        float lower[4], upper[4];
        backdrop_.fog(scoring_.player_mode, io_.camera_rotation, lower, upper);
        gl::Viewport(0, 0, view_.w, view_.h);
        gl::Disable(gl::BLEND);
        gl::Disable(gl::SCISSOR_TEST);
        gl::Enable(gl::DEPTH_TEST);
        gl::DepthFunc(gl::LEQUAL);
        gl::DepthMask(gl::TRUE_);
        gl::Clear(gl::DEPTH_BUFFER_BIT);
        gl::UseProgram(room_.id());
        gl::Uniform4fv(room_.uniform("uLowerFog"), 1, lower);
        gl::Uniform4fv(room_.uniform("uUpperFog"), 1, upper);
        gl::Uniform1i(room_.uniform("uTexture0"), 0);
        gl::ActiveTexture(gl::TEXTURE0);
        gl::BindTexture(gl::TEXTURE_2D, tiles_);
        const Mat4& view_proj = view_.world;
        draw_segment_mesh(room_, menu_mesh_, view_proj);
        gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, 0);
    }


    const AssetPack& pack_;
    phys::PhysicsWorld world_;
    Rand48 rng_;
    script::MaterialTable materials_;
    game::TemplateRegistry templates_;
    script::ScriptLevel level_script_;
    game::RoomContext ctx_;
    game::Scoring scoring_;
    game::GameIO io_;
    PlayAudio audio_;
    game::Effects effects_;
    game::Display display_;
    std::unique_ptr<game::Level> level_;
    int frame_ = 0, width_ = 0, height_ = 0;
    View view_;                  // the view being drawn
    bool reuse_ui_ = false;
    bool ui_stepped_ = true;  // a game step ran since the UI scripts last drew
    float rumble_ = 0.0f;
    float pause_blend_ = 0.0f;         // Game +0x1e8
    bool music_sync_allowed_ = false;  // real-time play with sound
    bool campaign_bosses_ = false;  // the room list includes boss rooms (expert mode)
    std::vector<ui::UiBatch> ui_world_, ui_menu_, ui_hud_;  // this frame's UI batches
    bool vr_ = false;            // --vr: the headset gives the head, the aim and the GUI pointer
    phys::Quat vr_aim_{0, 0, 0, 1};  // the aiming hand's orientation (LOCAL space)

    ShaderProgram room_, clear_, sprite_, basic_, basic_glass_, bodytex_, glass_, glass_tex_, metal_, shadow_, ball_, body_depth_, body_refl_, sprite_refl_,
        ui_shader_;
    ui::GuiState gui_;  // before the scripts: they unregister their modal boxes from it
    ui::UiPointer pointer_;  // window pixels
    std::unique_ptr<ui::UiScript> hud_, menu_;
    // Game states (Game +0x130 current, +0x134 requested, +0x138 fade): 1 menu, 3 level.
    int state_ = 1, requested_ = 1, load_steps_ = 0;
    float state_fade_ = 0, menu_t_ = 0, menu_trans_ = 0;  // Menu +0x39c, +0x3a0 ("menutransition")
    std::string level_name_ = "0";                        // Game +0x168: the level.start argument
    bool touch_held_ = false;  // the input's state in the last step
    bool level_loaded_ = false, direct_ = false, restart_level_ = false, quit_ = false, own_rooms_ = false;
    game::PlayerRecords records_;
    fs::path records_path_;
    Rand48 ui_rng_;  // (the HUD never draws random numbers; math.random would use this)
    std::map<std::string, UiTexture> ui_textures_;
    gl::GLuint tiles_ = 0, sprites_ = 0, refmap_ = 0, glassnormal_ = 0, metalnormal_ = 0, credits_ = 0;
    gl::GLuint best_distance_tex_ = 0;
    ShaderProgram tex2d_;
    gl::GLuint quad_vbo_ = 0, dyn_vbo_ = 0, sprite_ibo_ = 0, body_ibo_ = 0;
    Target refr_sets_[3][2];         // the refraction pair per view: the window, the left eye, the right eye
    Target* refr_ = refr_sets_[0];
    game::BodyBuffers bodies_;
    gl::GLuint ball_tex_ = 0, blob_ = 0, doors_ = 0;
    std::vector<game::BodyVertex> debris_verts_, refl_mask_, refl_walls_;
    game::ReflectionMesh discs_;
    std::vector<game::ShadowVertex> shadow_verts_;
    std::map<std::string, GpuMesh> meshes_;
    game::MenuBackdrop backdrop_;
    GpuMesh menu_mesh_;  // vbo set and no indices: loading failed
    std::vector<game::SpriteVertex> sprite_verts_, laser_verts_, decal_verts_;
    std::vector<std::uint16_t> sprite_index_;
};

bool save_screenshot(const fs::path& out, int width, int height) {
    Image img{width, height, 4, Bytes(size_t(width) * size_t(height) * 4)};
    gl::PixelStorei(gl::PACK_ALIGNMENT, 1);
    gl::ReadPixels(0, 0, width, height, gl::RGBA, gl::UNSIGNED_BYTE, img.pixels.data());
    const size_t row = size_t(width) * 4;
    for (int y = 0; y < height / 2; ++y)
        std::swap_ranges(img.pixels.begin() + y * row, img.pixels.begin() + (y + 1) * row,
                         img.pixels.begin() + (height - 1 - y) * row);
    for (size_t i = 3; i < img.pixels.size(); i += 4) img.pixels[i] = 255;
    const Bytes png = encode_png(img);
    std::ofstream f(out, std::ios::binary);
    f.write(reinterpret_cast<const char*>(png.data()), static_cast<std::streamsize>(png.size()));
    return static_cast<bool>(f);
}

int run(const Options& opt) {
    const AssetPack pack(opt.apk);
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);  // the floor reflections' stencil mask
    SDL_Window* window = SDL_CreateWindow("ShatterVR", 1280, 720, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!window) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        return 1;
    }
    SDL_GLContext context = SDL_GL_CreateContext(window);
    if (!context) {
        std::fprintf(stderr, "SDL_GL_CreateContext failed: %s\n", SDL_GetError());
        return 1;
    }
    SDL_GL_SetSwapInterval(1);
    if (const char* missing = gl::load([](const char* n) { return reinterpret_cast<void*>(SDL_GL_GetProcAddress(n)); })) {
        std::fprintf(stderr, "OpenGL function missing: %s\n", missing);
        return 1;
    }
    int result = 0;
    {
        Game game(pack, opt);
        std::unique_ptr<play::XrRig> xr;
        if (opt.vr) {
            xr = std::make_unique<play::XrRig>();
            std::string error;
            if (!xr->init(error)) {
                std::fprintf(stderr, "OpenXR: %s\n", error.c_str());
                if (!opt.launched_bare) return 1;
                xr.reset();  // double-clicked with no headset ready: play in the window
            }
        }
        if (xr) {
            std::printf("OpenXR runtime: %s\n", xr->runtime_name().c_str());
            game.set_vr(true);
            SDL_GL_SetSwapInterval(0);  // the runtime paces the frames (xrWaitFrame)
        }
        int vr_frames = 0;  // frames handed to the headset
        if (opt.gaze) game.set_head_aim(true);
        int hand = 1;  // the hand that aims: the one that last pulled its trigger (right to start)
        // A press waits for the next game step: the headset's frames come faster than the 60 Hz steps.
        bool running = true, clicked = false, pause = false, xr_throw = false;
        float mx = 0, my = 0;
        Uint64 last = SDL_GetTicksNS();
        double accumulator = 0;
        int frames = 0;
        while (running) {
            SDL_Event e;
            while (SDL_PollEvent(&e)) {
                if (e.type == SDL_EVENT_QUIT) running = false;
                if (e.type == SDL_EVENT_KEY_DOWN && !e.key.repeat) {
                    if (e.key.key == SDLK_ESCAPE) game.back();
                    if (e.key.key == SDLK_G && xr) game.set_head_aim(!game.head_aim());
                    if (e.key.key == SDLK_S) pause = true;
                    if (e.key.key == SDLK_R && game.direct()) game.restart(opt.balls);
                    if (e.key.key == SDLK_C && xr) xr->recenter();
                }
                if (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN && e.button.button == SDL_BUTTON_LEFT) {
                    if (game.direct() && game.game_over()) {
                        game.restart(opt.balls);
                        continue;
                    }
                    clicked = true;
                    mx = e.button.x, my = e.button.y;
                }
            }
            for (int f : opt.backs)
                if (frames == f) game.back();
            int w = 0, h = 0;
            SDL_GetWindowSizeInPixels(window, &w, &h);
            int lw = 0, lh = 0;
            SDL_GetWindowSize(window, &lw, &lh);
            if (w <= 0 || h <= 0) continue;
            game.resize(w, h);
            const float sx = lw > 0 ? float(w) / float(lw) : 1.0f, sy = lh > 0 ? float(h) / float(lh) : 1.0f;
            // The headset's frame: head, hands, buttons.
            play::VrFrame xs;
            bool xr_frame = false, held = false;
            if (xr) {
                if (!xr->poll()) running = false;
                if (xr->session_running()) xr_frame = xr->begin_frame(xs);
                if (xr_frame) {
                    if (xs.throw_pressed[0] && !xs.throw_pressed[1]) hand = 0;
                    if (xs.throw_pressed[1]) hand = 1;
                    if (xs.throw_pressed[hand]) xr_throw = true;
                    held = xs.throw_down[hand];
                    const play::VrPose& aim = xs.aim[hand];
                    // Head aiming, as on the Gear VR (the trigger stands in for its touchpad tap), or hand aiming; the
                    // A / X button (grip on Vive and Windows MR) or G switches.
                    if (xs.aim_toggle_pressed) game.set_head_aim(!game.head_aim());
                    const play::VrPose& pointer = game.head_aim() ? xs.head : aim;
                    game.vr_input(xs.head.valid ? xs.head.orientation : phys::Quat{}, pointer.valid ? &pointer : nullptr,
                                  xs.throw_down[hand]);
                    if (xs.back_pressed) game.back();
                    if (xs.recenter_pressed) xr->recenter();
                }
            } else {
                float cx = 0, cy = 0;
                const SDL_MouseButtonFlags buttons = SDL_GetMouseState(&cx, &cy);
                held = (buttons & SDL_BUTTON_LMASK) != 0;
                game.set_pointer(cx * sx, cy * sy, held);
                for (const Options::Click& c : opt.clicks)
                    if (frames >= c.frame - 2 && frames <= c.frame + 1) game.set_pointer(c.x, c.y, frames == c.frame);
            }
            const bool* keys = SDL_GetKeyboardState(nullptr);
            const Uint64 now = SDL_GetTicksNS();
            accumulator += double(now - last) * 1e-9;
            last = now;
            const bool paced = opt.frames == 0 || opt.realtime;
            int steps = paced ? 0 : 1;  // unattended runs step once per loop
            if (paced) {
                while (accumulator >= 1.0 / 60.0 && steps < 4) accumulator -= 1.0 / 60.0, ++steps;
                if (steps == 4) accumulator = 0;
            }
            for (int i = 0; i < steps; ++i) {
                bool shoot = clicked || xr_throw;
                xr_throw = false;
                float px = mx * sx, py = my * sy;
                if (opt.autofire > 0 && frames % opt.autofire == 0) shoot = true, px = w * 0.5f, py = h * 0.5f;
                for (const Options::Click& c : opt.clicks)
                    if (frames == c.frame) shoot = true, px = c.x, py = c.y;  // as a mouse click: it also throws
                bool hold = held;
                for (const Options::Hold& c : opt.holds)
                    if (frames >= c.from && frames < c.to) hold = true, game.set_pointer(w * 0.5f, h * 0.5f, true);
                for (const Options::Hold& c : opt.powerups)
                    if (frames == c.from) game.activate_powerup(c.to);
                game.step(shoot, hold, px, py, keys[SDL_SCANCODE_F], pause);
                clicked = false, pause = false;
                ++frames;
            }
            if (xr) {
                const float r = game.take_rumble();
                if (r > 0.0f) xr->vibrate(hand, r, r > 0.5f ? 0.25f : 0.04f);
            }
            if (xr) {
                if (xr_frame) {
                    if (xs.render) {
                        for (int eye = 0; eye < 2; ++eye) {
                            int ew = 0, eh = 0;
                            const gl::GLuint fbo = xr->acquire_eye(eye, ew, eh);
                            game.draw(game.eye_view(xs.eyes[eye], fbo, ew, eh, eye + 1), eye == 1);
                            if (eye == 0) {  // the window mirrors the left eye, fitted inside it
                                gl::BindFramebuffer(gl::FRAMEBUFFER, 0);
                                gl::Viewport(0, 0, w, h);
                                gl::ClearColor(0, 0, 0, 1);
                                gl::Clear(gl::COLOR_BUFFER_BIT);
                                const float k = std::min(float(w) / float(ew), float(h) / float(eh));
                                const int dw = int(float(ew) * k), dh = int(float(eh) * k);
                                gl::BindFramebuffer(gl::READ_FRAMEBUFFER, fbo);
                                gl::BindFramebuffer(gl::DRAW_FRAMEBUFFER, 0);
                                gl::BlitFramebuffer(0, 0, ew, eh, (w - dw) / 2, (h - dh) / 2, (w - dw) / 2 + dw, (h - dh) / 2 + dh,
                                                    gl::COLOR_BUFFER_BIT, gl::LINEAR);
                            }
                            xr->release_eye(eye);
                        }
                    }
                    if (xs.render) ++vr_frames;
                    xr->end_frame(xs);
                } else {
                    SDL_Delay(5);  // no session yet (or paused): the runtime is not pacing us
                }
                if (!xr_frame || !xs.render) {
                    // The headset wants no picture (not worn, idle, or not started): the window shows the VR camera
                    // itself, 90 degrees across, from the last head pose when there is one.
                    play::VrEye view_eye;
                    if (xr_frame && xs.head.valid) view_eye.pose = xs.head;
                    view_eye.angle_right = 0.7853982f, view_eye.angle_left = -view_eye.angle_right;
                    view_eye.angle_up = std::atan(float(h) / float(w)), view_eye.angle_down = -view_eye.angle_up;
                    game.draw(game.eye_view(view_eye, 0, w, h, 0));
                }
            } else {
                game.draw();
            }
            if (game.quit_requested()) running = false;
            if (opt.frames > 0 && frames >= opt.frames) {
                running = false;
                if (!opt.screenshot.empty()) {
                    gl::Finish();
                    result = save_screenshot(opt.screenshot, w, h) ? 0 : 1;
                    std::printf("wrote %s (%dx%d) after %d frames\n", opt.screenshot.string().c_str(), w, h, frames);
                }
                std::printf("%s\n%s\n%s\n", game.status().c_str(), game.effects_status().c_str(), game.stats_status().c_str());
                if (xr) std::printf("headset frames rendered: %d\n", vr_frames);
            }
            SDL_SetWindowTitle(window, game.status().c_str());
            SDL_GL_SwapWindow(window);
        }
    }
    SDL_GL_DestroyContext(context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return result;
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parse_args(argc, argv, opt)) {
        std::fprintf(stderr,
                     "usage: %s <game.apk> [--room N] [--mode M] [--balls B] [--frames N] [--screenshot OUT.png] [--autofire K] [--mute] [--realtime]\n",
                     argv[0]);
        return 2;
    }
    if (!opt.screenshot.empty() && tools::refuse_source_tree(opt.screenshot)) {
        std::fprintf(stderr, "refusing to write game-derived images inside the source tree\n");
        return 2;
    }
    // Double-clicked there is no console, so problems go to a message box.
    auto report = [&](const std::string& text) {
        std::fprintf(stderr, "%s\n", text.c_str());
        if (opt.launched_bare) SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "ShatterVR", text.c_str(), nullptr);
    };
    if (opt.launched_bare && !std::filesystem::exists(opt.apk)) {
        report("Put your own Smash Hit Gear VR APK in this folder and name it game.apk:\n\n" + opt.apk.parent_path().string());
        return 1;
    }
    try {
        return run(opt);
    } catch (const std::exception& ex) {
        report(std::string("error: ") + ex.what());
        return 1;
    }
}
