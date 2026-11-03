#include "ui/ui_script.hpp"

#include <pugixml.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace sh::ui {

namespace {

constexpr float kHalfPi = 1.5707964f, kPi = 3.1415927f;
// GuiBox::render grows each selection rect by this many GUI units for the pick test (90 on the other branch of a
// platform query; which one applies to a mouse is unverified).
constexpr float kPickMargin = 45.0f;

// Tween type 5 ("bounce"): a damped spring from 0 towards 1 sampled over 1024 steps.
const std::array<float, 1024>& bounce_table() {
    static const std::array<float, 1024> t = [] {
        std::array<float, 1024> out{};
        float v = 0.0f, vel = 0.0f;
        for (float& e : out) {
            e = v;
            vel = (1.0f - v) * 0.00035f + vel * 0.98f;
            v = v + vel;
        }
        return out;
    }();
    return t;
}

int tween_type(const std::string& name) {
    if (name == "linear") return 1;
    if (name == "easein") return 2;
    if (name == "easeout") return 3;
    if (name == "cosine") return 4;
    if (name == "bounce") return 5;
    return 0;
}

// Gfx's multiplier colours (Gfx +0, rgba per entry; entries 0, 6, 8 and 9 partly unverified).
constexpr float kMultiplierColor[10][3] = {{0, 0, 1},   {0, 0.5f, 1}, {0, 1, 1},    {0, 1, 0.5f}, {0, 1, 0},
                                           {0.5f, 1, 0}, {1, 1, 0},   {1, 0.5f, 0}, {1, 0, 0},    {1, 0, 1}};

std::string trim(const std::string& s) {
    const size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return {};
    const size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

void read_floats(const char* text, float* out, int n) {
    std::istringstream in(text ? text : "");
    for (int i = 0; i < n; ++i) in >> out[i];
}

}  // namespace

void GuiState::end_frame(bool released) {
    hover = candidate;
    if (released) hover = nullptr;
    candidate = nullptr;
    best_distance = 3.4028235e38f;
}

void GuiState::set_modal(const void* box, bool on) {
    if (on) {
        if (is_modal(box)) return;
        for (const void*& slot : modal_)
            if (!slot) {
                slot = box;
                return;
            }
        return;  // all five slots taken
    }
    for (const void*& slot : modal_)
        if (slot == box) slot = nullptr;
}

bool GuiState::any_modal() const {
    return std::any_of(modal_.begin(), modal_.end(), [](const void* s) { return s != nullptr; });
}

bool GuiState::is_modal(const void* box) const { return std::find(modal_.begin(), modal_.end(), box) != modal_.end(); }

float UiScript::Tween::value(int i) const {
    if (duration == 0.0f) return to[i];
    const float t = std::clamp(elapsed / duration, 0.0f, 1.0f);
    float f = 1.0f;
    switch (type) {
    case 1: f = t; break;
    case 2: f = 1.0f - std::cos(t * kHalfPi); break;
    case 3: f = std::sin(t * kHalfPi); break;
    case 4: f = (1.0f - std::cos(t * kPi)) * 0.5f; break;
    case 5: f = bounce_table()[static_cast<size_t>(t * 1023.0f)]; break;
    default: break;
    }
    return from[i] + (to[i] - from[i]) * f;
}

void UiScript::Tween::set(const float* v, int n, int tween_type, float tween_duration) {
    for (int i = 0; i < n; ++i) from[i] = value(i);
    for (int i = 0; i < n; ++i) to[i] = v[i];
    if (tween_type == 0 || !(tween_duration > 0.0f)) {
        for (int i = 0; i < n; ++i) from[i] = v[i];
        type = 0, duration = 0, elapsed = 0;
        return;
    }
    type = tween_type, duration = tween_duration, elapsed = 0;
}

void UiScript::Tween::jump(const float* v, int n) {
    for (int i = 0; i < n; ++i) from[i] = value(i);
    for (int i = 0; i < n; ++i) to[i] = v[i];
    type = 0, duration = 0, elapsed = 0;
}

UiScript::Affine UiScript::Affine::operator*(const Affine& o) const {
    Affine r;
    r.a = a * o.a + c * o.b, r.b = b * o.a + d * o.b;
    r.c = a * o.c + c * o.d, r.d = b * o.c + d * o.d;
    r.tx = a * o.tx + c * o.ty + tx, r.ty = b * o.tx + d * o.ty + ty;
    return r;
}

void UiScript::Affine::apply(float x, float y, float out[2]) const {
    out[0] = a * x + c * y + tx, out[1] = b * x + d * y + ty;
}

UiScript::Affine UiScript::Affine::inverse() const {
    const float det = a * d - b * c;
    Affine r;
    if (det == 0.0f) return r;
    const float k = 1.0f / det;
    r.a = d * k, r.b = -b * k, r.c = -c * k, r.d = a * k;
    r.tx = -(r.a * tx + r.c * ty), r.ty = -(r.b * tx + r.d * ty);
    return r;
}

float UiScript::Visual::width() const {
    switch (kind) {
    case Kind::kText: {
        if (!font) return 0;
        float w = 0;
        font->layout(text, &w);
        return w;
    }
    case Kind::kUi: return region[2] - region[0];
    case Kind::kCanvas: return canvas_w;
    default: return (crop[2] - crop[0]) * static_cast<float>(tex_w);
    }
}

float UiScript::Visual::height() const {
    switch (kind) {
    case Kind::kText: return font ? font->cell_height() : 0.0f;
    case Kind::kUi: return region[3] - region[1];
    case Kind::kCanvas: return canvas_h;
    default: return (crop[3] - crop[1]) * static_cast<float>(tex_h);
    }
}

UiScript::UiScript(Rand48& rng, UiEnv env, GuiState& gui) : env_(std::move(env)), gui_(gui), host_(rng) {
    host_.on_error = [](const std::string& msg) { std::fprintf(stderr, "ui script: %s\n", msg.c_str()); };
    register_bindings();
}

UiScript::~UiScript() {
    for (auto& v : visuals_) gui_.set_modal(v.get(), false);
}

bool UiScript::load(const std::string& path) {
    const size_t slash = path.rfind('/');
    dir_ = slash == std::string::npos ? std::string() : path.substr(0, slash + 1);
    const std::optional<std::string> src = env_.read_text ? env_.read_text(path) : std::nullopt;
    if (!src || !host_.load(path, *src)) return false;
    loaded_ = true;
    if (host_.has_function("init")) host_.call("init");
    return true;
}

void UiScript::tick(float dt, const std::vector<std::pair<float, float>>& touches, const UiPointer& pointer,
                    bool run_frame) {
    touches_ = touches;
    pointer_ = pointer;
    depth_ = 0, depth_angle_ = 0;
    if (!loaded_) return;
    host_.collect_garbage(1);
    if (run_frame && host_.has_function("frame")) host_.call("frame");
    // Every visual's tweens advance; the first UI group holding a pick runs it and ends the pass (the visuals after it
    // keep their tweens this frame, as in the original); movable canvases follow the pointer.
    for (size_t i = 0; i < visuals_.size(); ++i) {
        Visual& v = *visuals_[i];
        for (Tween* t : {&v.pos, &v.rot, &v.scale, &v.alpha, &v.color}) t->elapsed = t->elapsed + dt;
        if (v.kind == Kind::kUi && !v.selection.empty()) {
            const std::string sel = v.selection;
            v.selection.clear();
            run_selection(sel);
            break;
        }
        if (v.kind == Kind::kCanvas && v.movable != 0 && v.canvas_enabled) update_canvas_drag(v);
    }
    // Delayed code: counted down, run once below zero (the last entry takes the slot).
    for (size_t i = 0; i < delayed_.size();) {
        delayed_[i].time = delayed_[i].time - dt;
        if (delayed_[i].time >= 0.0f) {
            ++i;
            continue;
        }
        const std::string code = delayed_[i].code;
        delayed_[i] = delayed_.back();
        delayed_.pop_back();
        host_.execute(code);
    }
    ++frame_;
    time_ = time_ + dt;
}

// A selection is ";"-separated commands: "script:x" goes to the script's handleCommand, others to the game.
void UiScript::run_selection(const std::string& selection) {
    std::string rest = selection;
    for (;;) {
        const size_t semi = rest.find(';');
        const std::string cmd = trim(semi == std::string::npos ? rest : rest.substr(0, semi));
        if (cmd.rfind("script:", 0) == 0) handle_command(cmd.substr(7));
        else if (env_.command) env_.command(cmd);
        if (semi == std::string::npos) break;
        rest = rest.substr(semi + 1);
    }
}

// Script::tick's canvas drag: a press inside the window grabs the canvas; while held it follows the pointer, giving
// way past the window edges; released, it coasts on its velocity and settles back inside.
void UiScript::update_canvas_drag(Visual& c) {
    const float lo[2] = {static_cast<float>(c.window[2]) - c.canvas_w, static_cast<float>(c.window[3]) - c.canvas_h};
    const float hi[2] = {static_cast<float>(c.window[0]), static_cast<float>(c.window[1])};
    const float cur[2] = {c.pos.value(0), c.pos.value(1)};
    if (drag_ == &c) {
        if (!pointer_.down) {
            drag_ = nullptr;
            return;
        }
        const float dx = pointer_.x - drag_press_[0], dy = pointer_.y - drag_press_[1];
        if (dx * dx + dy * dy > 900.0f) gui_.hover = nullptr;  // a drag, not a tap: GuiBox::abortSelect
        const Affine inv = c.pushed.inverse();
        const float d[2] = {inv.a * dx + inv.c * dy, inv.b * dx + inv.d * dy};
        float n[2];
        for (int k = 0; k < 2; ++k) {
            const float clamped = std::min(std::max(d[k], lo[k] - drag_start_[k]), hi[k] - drag_start_[k]);
            const float f = 1.0f / (std::fabs(d[k] - clamped) * 0.01f + 1.0f);
            n[k] = (1.0f - f) * clamped + d[k] * f;
        }
        if (c.movable == 1) n[1] = 0.0f;
        else if (c.movable == 2) n[0] = 0.0f;
        const float to[2] = {drag_start_[0] + n[0], drag_start_[1] + n[1]};
        for (int k = 0; k < 2; ++k) c.vel[k] = c.vel[k] * 0.8f + (to[k] - cur[k]) * 0.2f;
        c.pos.jump(to, 2);
        return;
    }
    if (pointer_.pressed) {
        float w0[2], w1[2];
        c.pushed.apply(static_cast<float>(c.window[0]), static_cast<float>(c.window[1]), w0);
        c.pushed.apply(static_cast<float>(c.window[2]), static_cast<float>(c.window[3]), w1);
        if (w0[0] < pointer_.x && pointer_.x < w1[0] && w0[1] < pointer_.y && pointer_.y < w1[1]) {
            drag_ = &c;
            drag_press_[0] = pointer_.x, drag_press_[1] = pointer_.y;
            drag_start_[0] = cur[0], drag_start_[1] = cur[1];
        }
        return;
    }
    float to[2];
    for (int k = 0; k < 2; ++k) {
        const float p = cur[k] + c.vel[k];
        const float clamped = std::min(std::max(p, lo[k]), hi[k]);
        c.vel[k] = c.vel[k] * 0.9f;
        to[k] = p * 0.7f + clamped * 0.3f;
    }
    c.pos.jump(to, 2);
}

const std::vector<UiBatch>& UiScript::run_draw(const char* fn) {
    batches_.clear();
    canvas_stack_.clear();
    update_canvas_stack();
    clip_.reset();
    if (loaded_ && host_.has_function(fn)) host_.call(fn);
    return batches_;
}

const std::vector<UiBatch>& UiScript::draw() { return run_draw("draw"); }
const std::vector<UiBatch>& UiScript::draw_world() { return run_draw("drawWorld"); }

void UiScript::handle_command(const std::string& cmd) {
    if (loaded_ && host_.has_function("handleCommand")) host_.call("handleCommand", cmd);
}

UiScript::Visual* UiScript::visual(int handle) {
    return handle >= 1 && handle <= static_cast<int>(visuals_.size()) ? visuals_[static_cast<size_t>(handle - 1)].get()
                                                                       : nullptr;
}

std::string UiScript::resolve(const std::string& name) const {
    int w, h;
    if (!dir_.empty() && env_.texture_size && env_.texture_size(dir_ + name, w, h)) return dir_ + name;
    return name;
}

const BitmapFont* UiScript::font(const std::string& name, bool numbers) {
    auto it = fonts_.find(name);
    if (it != fonts_.end()) return it->second.get();
    auto f = std::make_unique<BitmapFont>();
    int w = 0, h = 0;
    const std::optional<std::string> fnt = env_.read_text ? env_.read_text("fonts/" + name + ".fnt") : std::nullopt;
    if (!fnt || !env_.texture_size || !env_.texture_size("fonts/" + name + ".png", w, h) || !f->load(*fnt, numbers, w, h))
        f.reset();
    const BitmapFont* raw = f.get();
    fonts_[name] = std::move(f);
    return raw;
}

// Visual::getTransform: translate (position, mgSetZ depth), turn about Y by the mgSetZ angle, rotate, scale, move by
// -origin. In the flat GUI viewport the depth has no visible effect and the turn narrows x by its cosine.
UiScript::Affine UiScript::local_transform(const Visual& v) const {
    const float a = v.rot.value(0), co = std::cos(a), si = std::sin(a);
    const float sx = v.scale.value(0), sy = v.scale.value(1);
    const float turn = v.kind == Kind::kCanvas ? 1.0f : std::cos(depth_angle_);  // (canvases do not use it)
    Affine m;
    m.a = co * sx * turn, m.b = si * sx, m.c = -si * sy * turn, m.d = co * sy;
    m.tx = v.pos.value(0) - (m.a * v.origin[0] + m.c * v.origin[1]);
    m.ty = v.pos.value(1) - (m.b * v.origin[0] + m.d * v.origin[1]);
    return m;
}

void UiScript::box_corners(const Affine& m, float x0, float y0, float x1, float y1, float out[4][2]) const {
    m.apply(x0, y0, out[0]), m.apply(x1, y0, out[1]), m.apply(x1, y1, out[2]), m.apply(x0, y1, out[3]);
}

// Visual::getColor: the visual's colour and alpha times the canvas stack's.
void UiScript::stack_color(const Visual& v, float out[4]) const {
    out[0] = v.color.value(0) * stack_color_[0], out[1] = v.color.value(1) * stack_color_[1];
    out[2] = v.color.value(2) * stack_color_[2], out[3] = v.alpha.value(0) * stack_color_[3];
}

void UiScript::emit(const std::string& texture, bool is_font, const float corners[4][2], const float uv[4][2],
                    const float color[4]) {
    if (batches_.empty() || batches_.back().texture != texture || batches_.back().font != is_font ||
        batches_.back().clip != clip_ || batches_.back().menu_mesh)
        batches_.push_back({texture, is_font, clip_, {}});
    auto& out = batches_.back().vertices;
    for (int k : {0, 1, 2, 0, 2, 3})
        out.push_back({corners[k][0], corners[k][1], uv[k][0], uv[k][1], {color[0], color[1], color[2], color[3]}});
}

void UiScript::draw_visual(Visual& v) {
    if (v.kind == Kind::kCanvas) return;  // Canvas::draw draws nothing
    float color[4];
    stack_color(v, color);
    if (!(color[3] > 0.0f)) return;
    const float sx = v.scale.value(0), sy = v.scale.value(1);
    if (sx * sx + sy * sy == 0.0f) return;
    const Affine m = stack_ * local_transform(v);
    if (v.kind == Kind::kText) {
        if (!v.font) return;
        float total = 0;
        const std::vector<Glyph> glyphs = v.font->layout(v.text, &total);
        const float h = v.font->cell_height();
        for (const Glyph& g : glyphs) {
            float c[4][2];
            box_corners(m, g.x0, 0.0f, g.x1, h, c);
            const float uv[4][2] = {{g.u0, g.v0}, {g.u1, g.v0}, {g.u1, g.v1}, {g.u0, g.v1}};
            emit("fonts/" + v.font_name + ".png", true, c, uv, color);
        }
        return;
    }
    if (v.kind == Kind::kUi) {
        draw_ui(v, m, color);
        return;
    }
    if (v.texture.empty()) return;
    float c[4][2];
    box_corners(m, 0.0f, 0.0f, v.width(), v.height(), c);
    const float uv[4][2] = {{v.crop[0], v.crop[1]}, {v.crop[2], v.crop[1]}, {v.crop[2], v.crop[3]}, {v.crop[0], v.crop[3]}};
    emit(v.texture, false, c, uv, color);
}

// GuiBox::render: the region of the texture, then per selection rect the pick test against the pointer, the highlight
// (the hovered rect, or a radio group's selected one) and the select on a press. A press that picks nothing while the
// box takes input selects its <outside> command.
void UiScript::draw_ui(Visual& v, const Affine& m, const float color[4]) {
    if (v.texture.empty() || v.tex_w <= 0 || v.tex_h <= 0) return;
    const float tw = static_cast<float>(v.tex_w), th = static_cast<float>(v.tex_h);
    // A crop shows that part of the group at the group's position, its rects moving with it (ours: the only cropped
    // group is the mode button, which this build's script does not draw).
    const bool cropped = v.crop[0] != 0.0f || v.crop[1] != 0.0f || v.crop[2] != 1.0f || v.crop[3] != 1.0f;
    const float rx0 = cropped ? v.crop[0] * tw : v.region[0], ry0 = cropped ? v.crop[1] * th : v.region[1];
    const float rx1 = cropped ? v.crop[2] * tw : v.region[2], ry1 = cropped ? v.crop[3] * th : v.region[3];
    const float rw = rx1 - rx0, rh = ry1 - ry0;
    float c[4][2];
    box_corners(m, 0.0f, 0.0f, rw, rh, c);
    const float u0 = rx0 / tw, v0 = ry0 / th, u1 = rx1 / tw, v1 = ry1 / th;
    const float uv[4][2] = {{u0, v0}, {u1, v0}, {u1, v1}, {u0, v1}};
    emit(v.texture, false, c, uv, color);

    const bool takes_input = stack_enabled_;
    if (takes_input) v.selection.clear();
    const bool box_enabled = v.ui_enabled && (!gui_.any_modal() || gui_.is_modal(&v));
    for (size_t i = 0; i < v.rects.size(); ++i) {
        Rect& r = v.rects[i];
        if (!r.enabled) continue;
        float p0[2], p1[2];
        m.apply(r.x0 - rx0, r.y0 - ry0, p0);
        m.apply(r.x1 - rx0, r.y1 - ry0, p1);
        const bool hit = p0[0] - kPickMargin <= pointer_.x && pointer_.x <= p1[0] + kPickMargin &&
                         p0[1] - kPickMargin <= pointer_.y && pointer_.y <= p1[1] + kPickMargin;
        bool highlight = r.radio && !v.selected_texture.empty();
        if (hit && box_enabled) {
            if (takes_input) {
                // boxDistance: how far the pointer is outside the rect (0 inside); the closest rect is the candidate.
                const float ex = std::max({p0[0] - pointer_.x, 0.0f, pointer_.x - p1[0]});
                const float ey = std::max({p0[1] - pointer_.y, 0.0f, pointer_.y - p1[1]});
                const float dist = std::sqrt(ex * ex + ey * ey);
                if (dist < gui_.best_distance) gui_.best_distance = dist, gui_.candidate = &r;
            }
            if (gui_.hover == &r) highlight = true;
        }
        if (highlight && !v.selected_texture.empty()) {
            float hc[4][2];
            box_corners(m, r.x0 - rx0, r.y0 - ry0, r.x1 - rx0, r.y1 - ry0, hc);
            const float grey[4] = {0.5f, 0.5f, 0.5f, 1.0f};
            if (v.overlay) {
                const float full[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
                emit(v.selected_texture, false, hc, full, v.shade ? grey : color);
            } else {
                const float ru0 = r.x0 / tw, rv0 = r.y0 / th, ru1 = r.x1 / tw, rv1 = r.y1 / th;
                const float ruv[4][2] = {{ru0, rv0}, {ru1, rv0}, {ru1, rv1}, {ru0, rv1}};
                emit(v.selected_texture, false, hc, ruv, v.shade ? grey : color);
            }
        }
        if (takes_input && hit && gui_.hover == &r && pointer_.pressed) {
            // GuiBox::select
            if (env_.select_sound) env_.select_sound();
            v.selection = r.cmd;
            if (r.group != 0) {
                for (Rect& o : v.rects)
                    if (o.group == r.group) o.radio = false;
                r.radio = true;
            }
        }
    }
    if (takes_input && pointer_.pressed && v.selection.empty() && !gui_.hover && !gui_.candidate && box_enabled)
        v.selection = v.outside_cmd;
}

void UiScript::draw_pie(const Visual& v, float fraction) {
    float color[4];
    stack_color(v, color);
    if (!(color[3] > 0.0f) || v.texture.empty() || !(fraction > 0.0f)) return;
    float box[4][2];
    box_corners(stack_ * local_transform(v), 0.0f, 0.0f, v.width(), v.height(), box);
    // Gfx::drawPie: a fan in the unit square around (0.5, 0.5), up to 30 segments; uv follows position. The start
    // angle and direction are unverified (12 o'clock, clockwise here).
    const int segments = fraction < 1.0f ? static_cast<int>(fraction * 29.0f) + 1 : 30;
    auto at = [&](float s, float t, float out[2]) {
        out[0] = box[0][0] + (box[1][0] - box[0][0]) * s + (box[3][0] - box[0][0]) * t;
        out[1] = box[0][1] + (box[1][1] - box[0][1]) * s + (box[3][1] - box[0][1]) * t;
    };
    auto tex = [&](float s, float t, float out[2]) {
        out[0] = v.crop[0] + (v.crop[2] - v.crop[0]) * s, out[1] = v.crop[1] + (v.crop[3] - v.crop[1]) * t;
    };
    if (batches_.empty() || batches_.back().texture != v.texture || batches_.back().font || batches_.back().clip != clip_ ||
        batches_.back().menu_mesh)
        batches_.push_back({v.texture, false, clip_, {}});
    auto& out = batches_.back().vertices;
    const float sweep = std::min(fraction, 1.0f) * 2.0f * kPi;
    for (int i = 0; i < segments; ++i) {
        const float a0 = -kHalfPi + sweep * static_cast<float>(i) / static_cast<float>(segments);
        const float a1 = -kHalfPi + sweep * static_cast<float>(i + 1) / static_cast<float>(segments);
        const float st[3][2] = {{0.5f, 0.5f},
                                {0.5f + 0.5f * std::cos(a0), 0.5f + 0.5f * std::sin(a0)},
                                {0.5f + 0.5f * std::cos(a1), 0.5f + 0.5f * std::sin(a1)}};
        for (const auto& p : st) {
            float pos[2], uv[2];
            at(p[0], p[1], pos), tex(p[0], p[1], uv);
            out.push_back({pos[0], pos[1], uv[0], uv[1], {color[0], color[1], color[2], color[3]}});
        }
    }
}

// Script::pushCanvas: remember the stack transform (drags and the window use it), clip to the window, push.
void UiScript::push_canvas(Visual& canvas) {
    canvas.pushed = stack_;
    if (canvas.has_window) {
        float w0[2], w1[2];
        stack_.apply(static_cast<float>(canvas.window[0]), static_cast<float>(canvas.window[1]), w0);
        stack_.apply(static_cast<float>(canvas.window[2]), static_cast<float>(canvas.window[3]), w1);
        clip_ = std::array<float, 4>{w0[0], w0[1], w1[0], w1[1]};
    }
    canvas_stack_.push_back(&canvas);
    update_canvas_stack();
}

void UiScript::pop_canvas() {
    if (canvas_stack_.empty()) return;
    if (canvas_stack_.back()->has_window) clip_.reset();
    canvas_stack_.pop_back();
    update_canvas_stack();
}

// Script::updateCanvasStack: the product of the canvases' transforms, their colours times alphas, and whether all of
// them are enabled (UI groups take input only then).
void UiScript::update_canvas_stack() {
    stack_ = Affine{};
    for (float& c : stack_color_) c = 1.0f;
    stack_enabled_ = true;
    for (const Visual* c : canvas_stack_) {
        stack_ = stack_ * local_transform(*c);
        for (int k = 0; k < 3; ++k) stack_color_[k] = stack_color_[k] * c->color.value(k);
        stack_color_[3] = stack_color_[3] * c->alpha.value(0);
        stack_enabled_ = stack_enabled_ && c->canvas_enabled;
    }
}

float UiScript::tween_arg(const script::ScriptArgs& a, int index, int& type) const {
    type = a.size() > index ? tween_type(a.as_string(index)) : 0;
    return a.size() > index + 1 ? a.as_float(index + 1) : 0.0f;
}

void UiScript::register_bindings() {
    using script::ScriptArgs;
    using script::ScriptRet;
    auto reg = [&](const char* name, script::ScriptHost::Fn fn) { host_.register_function(name, std::move(fn)); };
    auto add_visual = [this](std::unique_ptr<Visual> v, ScriptRet& r) {
        const float one[3] = {1, 1, 1};
        v->scale.set(one, 2, 0, 0), v->alpha.set(one, 1, 0, 0), v->color.set(one, 3, 0, 0);
        visuals_.push_back(std::move(v));
        r.add_handle(static_cast<int>(visuals_.size()));
    };
    auto ui_of = [this](int handle) -> Visual* {
        Visual* v = visual(handle);
        return v && v->kind == Kind::kUi ? v : nullptr;
    };
    auto canvas_of = [this](int handle) -> Visual* {
        Visual* v = visual(handle);
        return v && v->kind == Kind::kCanvas ? v : nullptr;
    };
    reg("mgCreateImage", [this, add_visual](const ScriptArgs& a, ScriptRet& r) {
        auto v = std::make_unique<Visual>();
        v->kind = Kind::kImage;
        v->texture = resolve(a.as_string(0));
        if (env_.texture_size) env_.texture_size(v->texture, v->tex_w, v->tex_h);
        if (a.size() > 1 && v->tex_w > 0 && v->tex_h > 0) {
            const float w = static_cast<float>(v->tex_w), h = static_cast<float>(v->tex_h);
            v->crop[0] = a.as_float(1) / w, v->crop[1] = a.as_float(2) / h;
            v->crop[2] = a.as_float(3) / w, v->crop[3] = a.as_float(4) / h;
        }
        add_visual(std::move(v), r);
    });
    reg("mgCreateText", [this, add_visual](const ScriptArgs& a, ScriptRet& r) {
        auto v = std::make_unique<Visual>();
        v->kind = Kind::kText;
        v->font_name = a.as_string(0);
        v->font = font(v->font_name, a.as_bool(1));
        add_visual(std::move(v), r);
    });
    // GuiBox::init: <ui texture shade|select_overlay|selected coords> with <rect id cmd coords group selected/> and
    // <outside cmd/>.
    reg("mgCreateUi", [this, add_visual](const ScriptArgs& a, ScriptRet& r) {
        auto v = std::make_unique<Visual>();
        v->kind = Kind::kUi;
        const std::optional<std::string> xml = env_.read_text ? env_.read_text(dir_ + a.as_string(0)) : std::nullopt;
        pugi::xml_document doc;
        if (xml && doc.load_buffer(xml->data(), xml->size())) {
            const pugi::xml_node ui = doc.child("ui");
            v->texture = resolve(ui.attribute("texture").as_string(""));
            if (std::string(ui.attribute("shade").as_string("")) == "true") {
                v->shade = true;
                v->selected_texture = v->texture;
            } else if (*ui.attribute("select_overlay").as_string("")) {
                v->overlay = true;
                v->selected_texture = resolve(ui.attribute("select_overlay").as_string(""));
            } else if (*ui.attribute("selected").as_string("")) {
                v->selected_texture = resolve(ui.attribute("selected").as_string(""));
            }
            if (env_.texture_size) env_.texture_size(v->texture, v->tex_w, v->tex_h);
            v->region[2] = static_cast<float>(v->tex_w), v->region[3] = static_cast<float>(v->tex_h);
            if (ui.attribute("coords")) read_floats(ui.attribute("coords").as_string(""), v->region, 4);
            for (const pugi::xml_node node : ui.children()) {
                const std::string name = node.name();
                if (name == "rect") {
                    Rect rect{};
                    float c[4] = {0, 0, 0, 0};
                    read_floats(node.attribute("coords").as_string(""), c, 4);
                    rect.x0 = c[0], rect.y0 = c[1], rect.x1 = c[2], rect.y1 = c[3];
                    rect.id = node.attribute("id").as_string("");
                    rect.cmd = node.attribute("cmd").as_string("");
                    v->rects.push_back(rect);
                    // GuiBox::setRadio: the group and the selected flag go to every rect with this id.
                    const int group = node.attribute("group").as_int(0);
                    if (group > 0) {
                        const bool selected = std::string(node.attribute("selected").as_string("")) == "true";
                        for (Rect& o : v->rects)
                            if (o.id == rect.id) o.group = group, o.radio = selected;
                    }
                } else if (name == "outside") {
                    v->outside_cmd = node.attribute("cmd").as_string("");
                }
            }
        }
        add_visual(std::move(v), r);
    });
    reg("mgCreateCanvas", [add_visual](const ScriptArgs& a, ScriptRet& r) {
        auto v = std::make_unique<Visual>();
        v->kind = Kind::kCanvas;
        v->canvas_w = a.as_float(0), v->canvas_h = a.as_float(1);
        add_visual(std::move(v), r);
    });
    // mgSetOrigo fixes the origin from the visual's size at the time of the call; an unknown name changes nothing.
    reg("mgSetOrigo", [this](const ScriptArgs& a, ScriptRet&) {
        Visual* v = visual(a.as_handle(0));
        if (!v) return;
        const std::string o = a.as_string(1);
        const float w = v->width(), h = v->height();
        if (o == "center") v->origin[0] = w * 0.5f, v->origin[1] = h * 0.5f;
        else if (o == "topleft") v->origin[0] = 0, v->origin[1] = 0;
        else if (o == "topright") v->origin[0] = w, v->origin[1] = 0;
        else if (o == "bottomleft") v->origin[0] = 0, v->origin[1] = h;
        else if (o == "bottomright") v->origin[0] = w, v->origin[1] = h;
        else if (o == "pixel") v->origin[0] = a.as_float(2), v->origin[1] = a.as_float(3);
    });
    reg("mgSetCrop", [this](const ScriptArgs& a, ScriptRet&) {
        Visual* v = visual(a.as_handle(0));
        if (!v || v->tex_w <= 0 || v->tex_h <= 0) return;
        const float w = static_cast<float>(v->tex_w), h = static_cast<float>(v->tex_h);
        v->crop[0] = a.as_float(1) / w, v->crop[1] = a.as_float(2) / h, v->crop[2] = a.as_float(3) / w, v->crop[3] = a.as_float(4) / h;
    });
    reg("mgSetText", [this](const ScriptArgs& a, ScriptRet&) {
        if (Visual* v = visual(a.as_handle(0))) v->text = a.as_string(1);
    });
    reg("mgSetPos", [this](const ScriptArgs& a, ScriptRet&) {
        Visual* v = visual(a.as_handle(0));
        if (!v) return;
        int type;
        const float d = tween_arg(a, 3, type);
        const float p[2] = {a.as_float(1), a.as_float(2)};
        v->pos.set(p, 2, type, d);
    });
    reg("mgSetRot", [this](const ScriptArgs& a, ScriptRet&) {
        Visual* v = visual(a.as_handle(0));
        if (!v) return;
        int type;
        const float d = tween_arg(a, 2, type);
        const float p[1] = {a.as_float(1)};
        v->rot.set(p, 1, type, d);
    });
    reg("mgSetScale", [this](const ScriptArgs& a, ScriptRet&) {
        Visual* v = visual(a.as_handle(0));
        if (!v) return;
        int type;
        const float d = tween_arg(a, 3, type);
        const float sx = a.as_float(1);
        const float p[2] = {sx, a.size() > 2 ? a.as_float(2) : sx};
        v->scale.set(p, 2, type, d);
    });
    reg("mgSetAlpha", [this](const ScriptArgs& a, ScriptRet&) {
        Visual* v = visual(a.as_handle(0));
        if (!v) return;
        int type;
        const float d = tween_arg(a, 2, type);
        const float p[1] = {a.as_float(1)};
        v->alpha.set(p, 1, type, d);
    });
    reg("mgSetColor", [this](const ScriptArgs& a, ScriptRet&) {
        Visual* v = visual(a.as_handle(0));
        if (!v) return;
        int type;
        const float d = tween_arg(a, 4, type);
        const float p[3] = {a.as_float(1), a.as_float(2), a.as_float(3)};
        v->color.set(p, 3, type, d);
    });
    reg("mgGetPos", [this](const ScriptArgs& a, ScriptRet& r) {
        const Visual* v = visual(a.as_handle(0));
        r.add_float(v ? v->pos.value(0) : 0.0f), r.add_float(v ? v->pos.value(1) : 0.0f);
    });
    reg("mgGetRot", [this](const ScriptArgs& a, ScriptRet& r) {
        const Visual* v = visual(a.as_handle(0));
        r.add_float(v ? v->rot.value(0) : 0.0f);
    });
    reg("mgGetScale", [this](const ScriptArgs& a, ScriptRet& r) {
        const Visual* v = visual(a.as_handle(0));
        r.add_float(v ? v->scale.value(0) : 0.0f), r.add_float(v ? v->scale.value(1) : 0.0f);
    });
    reg("mgGetAlpha", [this](const ScriptArgs& a, ScriptRet& r) {
        const Visual* v = visual(a.as_handle(0));
        r.add_float(v ? v->alpha.value(0) : 0.0f);
    });
    reg("mgGetColor", [this](const ScriptArgs& a, ScriptRet& r) {
        const Visual* v = visual(a.as_handle(0));
        for (int i = 0; i < 3; ++i) r.add_float(v ? v->color.value(i) : 0.0f);
    });
    reg("mgGetSize", [this](const ScriptArgs& a, ScriptRet& r) {
        const Visual* v = visual(a.as_handle(0));
        r.add_float(v ? v->width() : 0.0f), r.add_float(v ? v->height() : 0.0f);
    });
    reg("mgIsVisible", [this](const ScriptArgs& a, ScriptRet& r) {
        const Visual* v = visual(a.as_handle(0));
        r.add_bool(v && v->alpha.value(0) > 0.0f && v->scale.value(0) != 0.0f && v->scale.value(1) != 0.0f);
    });
    reg("mgDraw", [this](const ScriptArgs& a, ScriptRet&) {
        if (Visual* v = visual(a.as_handle(0))) draw_visual(*v);
    });
    reg("mgDrawPie", [this](const ScriptArgs& a, ScriptRet&) {
        if (const Visual* v = visual(a.as_handle(0))) draw_pie(*v, a.as_float(1));
    });
    reg("mgDrawBorder", [this](const ScriptArgs& a, ScriptRet&) {
        if (Visual* v = visual(a.as_handle(0))) draw_visual(*v);  // (the border tiling is not ported)
    });
    reg("mgGet", [this](const ScriptArgs& a, ScriptRet& r) {
        const std::string k = a.as_string(0);
        auto num = [](float f) {
            char b[32];
            std::snprintf(b, sizeof b, "%g", f);
            return std::string(b);
        };
        if (k == "display.visibleTop") r.add_string(num(env_.visible_top));
        else if (k == "display.visibleBottom") r.add_string(num(env_.visible_bottom));
        else if (k == "display.visibleLeft") r.add_string(num(env_.visible_left));
        else if (k == "display.visibleRight") r.add_string(num(env_.visible_right));
        else r.add_string(env_.get_property ? env_.get_property(k) : std::string());
    });
    reg("mgSet", [this](const ScriptArgs& a, ScriptRet&) {
        if (env_.set_property) env_.set_property(a.as_string(0), a.as_string(1));
    });
    reg("mgCommand", [this](const ScriptArgs& a, ScriptRet&) {
        if (env_.command) env_.command(a.as_string(0));
    });
    reg("mgScriptTime", [this](const ScriptArgs&, ScriptRet& r) { r.add_float(time_); });
    reg("mgScriptFrame", [this](const ScriptArgs&, ScriptRet& r) { r.add_int(frame_); });
    reg("mgFullScreenColor", [this](const ScriptArgs& a, ScriptRet&) {
        const float color[4] = {a.as_float(0), a.as_float(1), a.as_float(2), a.as_float(3)};
        if (!(color[3] > 0.0f)) return;
        const float l = env_.screen_left, rgt = env_.screen_right, t = env_.visible_top, b = env_.visible_bottom;
        const float c[4][2] = {{l, t}, {rgt, t}, {rgt, b}, {l, b}};
        const float uv[4][2] = {};
        emit("", false, c, uv, color);
    });
    reg("mgCreateSound", [this](const ScriptArgs& a, ScriptRet& r) {
        // Beside the script when it is there (menu/showmenu.ogg), else from the asset root (snd/...), as for images.
        const std::string name = a.as_string(0);
        sounds_.push_back(!dir_.empty() && env_.asset_exists && env_.asset_exists(dir_ + name) ? dir_ + name : name);
        r.add_int(static_cast<int>(sounds_.size()) - 1);
    });
    reg("mgPlaySound", [this](const ScriptArgs& a, ScriptRet&) {
        const int i = a.as_int(0);
        if (i < 0 || i >= static_cast<int>(sounds_.size()) || !env_.play_sound) return;
        const float vol = a.size() > 1 ? a.as_float(1) : 1.0f, pitch = a.size() > 2 ? a.as_float(2) : 1.0f;
        env_.play_sound(sounds_[static_cast<size_t>(i)], vol, pitch);
    });
    reg("mgIsTouched", [this](const ScriptArgs& a, ScriptRet& r) {
        const float x = a.as_float(0), y = a.as_float(1), rad = a.as_float(2);
        bool hit = false;
        for (const auto& t : touches_)
            if ((t.first - x) * (t.first - x) + (t.second - y) * (t.second - y) <= rad * rad) hit = true;
        r.add_bool(hit);
    });
    reg("mgGetLastTouch", [this](const ScriptArgs&, ScriptRet& r) {
        r.add_float(touches_.empty() ? 0.0f : touches_.back().first);
        r.add_float(touches_.empty() ? 0.0f : touches_.back().second);
    });
    reg("mgRunDelayed", [this](const ScriptArgs& a, ScriptRet&) { delayed_.push_back({a.as_string(0), a.as_float(1)}); });
    reg("mgClearRunDelayed", [this](const ScriptArgs&, ScriptRet&) { delayed_.clear(); });
    reg("mgGetHighScore", [this](const ScriptArgs& a, ScriptRet& r) {
        // No argument: the overall high score (asked for as checkpoint -1; what the original returns is unverified).
        const int checkpoint = a.size() > 0 ? a.as_int(0) : -1;
        const std::pair<int, int> s = env_.high_score ? env_.high_score(checkpoint) : std::pair<int, int>{0, 0};
        r.add_int(s.first), r.add_int(s.second);
    });
    reg("mgGetPowerupFraction", [this](const ScriptArgs& a, ScriptRet& r) {
        r.add_float(env_.powerup_fraction ? env_.powerup_fraction(a.as_string(0)) : 0.0f);
    });
    reg("mgGetMultiplierColor", [](const ScriptArgs& a, ScriptRet& r) {
        const int n = std::clamp(a.as_int(0), 0, 9);
        for (int i = 0; i < 3; ++i) r.add_float(kMultiplierColor[n][i]);
    });
    // UI groups.
    reg("mgSetUiModal", [this, ui_of](const ScriptArgs& a, ScriptRet&) {
        if (Visual* v = ui_of(a.as_handle(0))) gui_.set_modal(v, a.as_bool(1));
    });
    reg("mgSetUiEnabled", [ui_of](const ScriptArgs& a, ScriptRet&) {
        if (Visual* v = ui_of(a.as_handle(0))) v->ui_enabled = a.as_bool(1);
    });
    reg("mgSetUiSelectionEnabled", [ui_of](const ScriptArgs& a, ScriptRet&) {
        Visual* v = ui_of(a.as_handle(0));
        if (!v) return;
        const std::string id = a.as_string(1);
        const bool on = a.as_bool(2);
        for (Rect& r : v->rects)
            if (r.id == id) r.enabled = on;
    });
    // GuiBox::selectRadio: the group of the rect with this id (0 if none) gets this rect as its selected one.
    reg("mgRadioSelect", [ui_of](const ScriptArgs& a, ScriptRet&) {
        Visual* v = ui_of(a.as_handle(0));
        if (!v) return;
        const std::string id = a.as_string(1);
        int group = 0;
        for (const Rect& r : v->rects)
            if (r.id == id) group = r.group;
        for (Rect& r : v->rects)
            if (r.group == group) r.radio = r.id == id;
    });
    // Canvases.
    reg("mgPushCanvas", [this, canvas_of](const ScriptArgs& a, ScriptRet&) {
        if (Visual* c = canvas_of(a.as_handle(0))) push_canvas(*c);
    });
    reg("mgPopCanvas", [this](const ScriptArgs&, ScriptRet&) { pop_canvas(); });
    reg("mgSetCanvasEnabled", [canvas_of](const ScriptArgs& a, ScriptRet&) {
        if (Visual* c = canvas_of(a.as_handle(0))) c->canvas_enabled = a.as_bool(1);
    });
    reg("mgSetCanvasSize", [canvas_of](const ScriptArgs& a, ScriptRet&) {
        if (Visual* c = canvas_of(a.as_handle(0))) c->canvas_w = a.as_float(1), c->canvas_h = a.as_float(2);
    });
    reg("mgSetCanvasWindow", [canvas_of](const ScriptArgs& a, ScriptRet&) {
        Visual* c = canvas_of(a.as_handle(0));
        if (!c) return;
        for (int k = 0; k < 4; ++k) c->window[k] = a.as_int(k + 1);
        c->has_window = c->window[0] != 0 || c->window[1] != 0 || c->window[2] != 0 || c->window[3] != 0;
    });
    reg("mgSetCanvasMovable", [canvas_of](const ScriptArgs& a, ScriptRet&) {
        Visual* c = canvas_of(a.as_handle(0));
        if (!c) return;
        const std::string m = a.as_string(1);
        c->movable = m == "x" ? 1 : m == "y" ? 2 : m == "xy" ? 3 : 0;
    });
    // mgSetZ(z [, angle]): the depth and turn of what is drawn next (a missing angle is 0).
    reg("mgSetZ", [this](const ScriptArgs& a, ScriptRet&) {
        depth_ = a.as_float(0);
        depth_angle_ = a.size() > 1 ? a.as_float(1) : 0.0f;
    });
    // Menu::draw (the menu backdrop mesh in the world camera): a marker in the batch list; the tool draws it there.
    reg("mgDrawMenuMesh", [this](const ScriptArgs&, ScriptRet&) {
        UiBatch b;
        b.menu_mesh = true;
        batches_.push_back(std::move(b));
    });
    // Not ported yet: game modes, the menu grid (Menu::drawGrid; no shipped script calls it).
    for (const char* name : {"mgCameraShake", "mgSetMode", "mgDrawMenuGrid"})
        reg(name, [](const ScriptArgs&, ScriptRet&) {});
    reg("mgGetUiSelectionCount", [](const ScriptArgs&, ScriptRet& r) { r.add_int(0); });
    reg("mgGetUiSelectionInfo", [](const ScriptArgs&, ScriptRet& r) { r.add_string(""); });
    reg("mgGetCheckpointPos", [](const ScriptArgs&, ScriptRet& r) { r.add_float(0), r.add_float(0); });
    reg("mgGetScreenCoord", [](const ScriptArgs& a, ScriptRet& r) { r.add_float(a.as_float(0)), r.add_float(a.as_float(1)); });
}

}  // namespace sh::ui
