#pragma once

// The UI script layer (the original's Script with its Visual objects): a Lua script (hud/main.lua,
// menu/main.lua) that creates images, texts, UI groups and canvases, animates their position, rotation, scale, alpha
// and colour with tweens, and draws them in GUI space (2048 units across the screen, a 2048 x 1536 canvas centred
// vertically). Drawing produces triangle batches; showing them is the tool's business. UI groups (GuiBox) are picked
// with the pointer while they draw; a pick turns into commands on the next tick.

#include "core/rand48.hpp"
#include "script/script_host.hpp"
#include "ui/font.hpp"

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace sh::ui {

// What the script reaches outside itself (Game, Display, Player, Audio, ResMan).
struct UiEnv {
    std::function<std::optional<std::string>(const std::string& path)> read_text;
    // A texture's reported size (twice the decoded size in this APK), or nothing when missing.
    std::function<bool(const std::string& path, int& w, int& h)> texture_size;
    std::function<std::string(const std::string& key)> get_property;
    std::function<void(const std::string& key, const std::string& value)> set_property;
    std::function<void(const std::string& cmd)> command;           // Game::handleCommand
    std::function<float(const std::string& name)> powerup_fraction;  // Level timer[type] / 7
    // Player::getHighScore for a checkpoint (the second value is the best streak there); -1: the overall high score.
    std::function<std::pair<int, int>(int checkpoint)> high_score;
    std::function<void(const std::string& path, float volume, float pitch)> play_sound;
    std::function<bool(const std::string& path)> asset_exists;  // for sounds named beside the script
    std::function<void()> select_sound;  // GuiBox::select plays the next sound of the UI select bank
    float visible_top = 192, visible_bottom = 1344, visible_left = 0, visible_right = 2048;
    // The whole output in GUI units, for mgFullScreenColor (on the PC the window is wider than the layout square).
    float screen_left = 0, screen_right = 2048;
};

// The pointer in GUI coordinates (the mouse, or the first touch).
struct UiPointer {
    bool down = false, pressed = false, released = false;  // held now, went down / up this frame
    float x = 0, y = 0;
};

// GuiBox's globals, shared by every UI script: the up to five modal UI groups, the rect closest to the pointer this
// frame (the candidate) and last frame's (the hover, which a press selects).
class GuiState {
public:
    // GuiBox::tick, after everything has drawn: the candidate becomes the hover (none on release) and resets.
    void end_frame(bool released);
    void set_modal(const void* box, bool on);
    bool any_modal() const;
    bool is_modal(const void* box) const;
    const void* hover = nullptr;
    const void* candidate = nullptr;
    float best_distance = 3.4028235e38f;

private:
    std::array<const void*, 5> modal_{};
};

struct UiVertex {
    float x, y, u, v;
    float color[4];
};
// Triangles with one texture ("" for a solid colour); font batches take their alpha from the texture. A clip rectangle
// (GUI units: x0, y0, x1, y1) comes from a canvas window. A menu_mesh batch has no vertices: it marks where
// mgDrawMenuMesh asked for the menu backdrop (Menu::draw).
struct UiBatch {
    std::string texture;
    bool font = false;
    std::optional<std::array<float, 4>> clip;
    std::vector<UiVertex> vertices;
    bool menu_mesh = false;
};

class UiScript {
public:
    UiScript(Rand48& rng, UiEnv env, GuiState& gui);
    ~UiScript();
    // Script::load: path like "hud/main.lua" (images and UIs resolve beside it), then init().
    bool load(const std::string& path);
    // Script::tick: touches (GUI coordinates) for mgIsTouched; frame(); tweens advance by dt; picked UI selections run
    // as commands; canvases follow drags; delayed code runs. run_frame false skips frame() (the original's flag).
    void tick(float dt, const std::vector<std::pair<float, float>>& touches, const UiPointer& pointer,
              bool run_frame = true);
    // Script::draw / drawWorld: the script's draw() or drawWorld(); returns this pass's batches in drawing order.
    const std::vector<UiBatch>& draw();
    const std::vector<UiBatch>& draw_world();
    void handle_command(const std::string& cmd);  // Script::handleCommand: the script's handleCommand(cmd)

private:
    struct Tween {
        int type = 0;  // 0 none, 1 linear, 2 easein, 3 easeout, 4 cosine, 5 bounce
        float duration = 0, elapsed = 0;
        float from[3] = {}, to[3] = {};
        float value(int i) const;
        void set(const float* v, int n, int tween_type, float tween_duration);
        void jump(const float* v, int n);  // from the current value straight to v (what a drag does)
    };
    struct Affine {  // x' = a x + c y + tx, y' = b x + d y + ty
        float a = 1, b = 0, c = 0, d = 1, tx = 0, ty = 0;
        Affine operator*(const Affine& o) const;
        void apply(float x, float y, float out[2]) const;
        Affine inverse() const;
    };
    enum class Kind { kImage, kText, kUi, kCanvas };
    struct Rect {  // a GuiBox selection
        float x0, y0, x1, y1;  // texels
        std::string id, cmd;
        int group = 0;
        bool radio = false;    // selected in its radio group
        bool enabled = true;   // mgSetUiSelectionEnabled
    };
    struct Visual {
        Kind kind = Kind::kImage;
        Tween pos, rot, scale, alpha, color;
        float origin[2] = {0, 0};  // set by mgSetOrigo from the size at that moment
        std::string texture;
        float crop[4] = {0, 0, 1, 1};  // uv rectangle
        int tex_w = 0, tex_h = 0;      // reported texture size
        std::string font_name, text;   // texts
        const BitmapFont* font = nullptr;
        // UI groups (GuiBox)
        std::string selected_texture;  // highlight texture (the base texture when shading)
        bool shade = false, overlay = false;
        float region[4] = {0, 0, 0, 0};  // the <ui coords> texel rectangle (default: the whole texture)
        std::vector<Rect> rects;
        std::string outside_cmd, selection;
        bool ui_enabled = true;
        // canvases
        float canvas_w = 0, canvas_h = 0;
        bool canvas_enabled = true;
        int movable = 0;  // 0 none, 1 x, 2 y, 3 xy
        bool has_window = false;
        int window[4] = {0, 0, 0, 0};
        Affine pushed;  // the stack transform when last pushed
        float vel[2] = {0, 0};
        float width() const;
        float height() const;
    };
    void register_bindings();
    Visual* visual(int handle);
    std::string resolve(const std::string& name) const;
    const BitmapFont* font(const std::string& name, bool numbers);
    Affine local_transform(const Visual& v) const;  // pos * rot * scale * -origin
    void box_corners(const Affine& m, float x0, float y0, float x1, float y1, float out[4][2]) const;
    void stack_color(const Visual& v, float out[4]) const;
    void emit(const std::string& texture, bool font, const float corners[4][2], const float uv[4][2], const float color[4]);
    void draw_visual(Visual& v);
    void draw_ui(Visual& v, const Affine& m, const float color[4]);
    void draw_pie(const Visual& v, float fraction);
    void push_canvas(Visual& canvas);
    void pop_canvas();
    void update_canvas_stack();
    void run_selection(const std::string& selection);
    void update_canvas_drag(Visual& canvas);
    const std::vector<UiBatch>& run_draw(const char* fn);
    float tween_arg(const script::ScriptArgs& a, int index, int& type) const;

    UiEnv env_;
    GuiState& gui_;
    script::ScriptHost host_;
    std::string dir_;  // the script's directory with a trailing '/'
    std::vector<std::unique_ptr<Visual>> visuals_;  // handle = index + 1
    std::vector<std::string> sounds_;
    struct Delayed {
        std::string code;
        float time;
    };
    std::vector<Delayed> delayed_;
    std::map<std::string, std::unique_ptr<BitmapFont>> fonts_;
    std::vector<std::pair<float, float>> touches_;
    UiPointer pointer_;
    std::vector<UiBatch> batches_;
    // The canvas stack (Script::updateCanvasStack): combined transform, colour and enabled flag.
    std::vector<Visual*> canvas_stack_;
    Affine stack_;
    float stack_color_[4] = {1, 1, 1, 1};
    bool stack_enabled_ = true;
    std::optional<std::array<float, 4>> clip_;
    // Canvas dragging (Script +0xa8..+0xbc): the canvas, the press point and the canvas position then.
    Visual* drag_ = nullptr;
    float drag_press_[2] = {0, 0}, drag_start_[2] = {0, 0};
    float depth_ = 0, depth_angle_ = 0;  // mgSetZ (Script +0xd4 / +0xd8), zeroed by tick
    float time_ = 0;
    int frame_ = 0;
    bool loaded_ = false;
};

}  // namespace sh::ui
