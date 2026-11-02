// The UI script layer's interaction rules: canvas transforms, GuiBox picking with the hover handed over
// at the end of each frame, modal boxes and <outside> commands, radio groups, and canvas dragging with its rubber band
// and coasting. Synthetic scripts and UI files; no game data.

#include "core/rand48.hpp"
#include "ui/ui_script.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

using sh::ui::GuiState;
using sh::ui::UiBatch;
using sh::ui::UiEnv;
using sh::ui::UiPointer;
using sh::ui::UiScript;

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("%-64s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) ++g_failures;
}

bool near(float a, float b, float eps = 1e-3f) { return std::fabs(a - b) <= eps; }

struct Harness {
    std::map<std::string, std::string> files;
    std::vector<std::string> commands;
    GuiState gui;
    sh::Rand48 rng;
    std::unique_ptr<UiScript> script;

    void start(const std::string& path) {
        UiEnv env;
        env.read_text = [this](const std::string& p) -> std::optional<std::string> {
            auto it = files.find(p);
            if (it == files.end()) return std::nullopt;
            return it->second;
        };
        env.texture_size = [](const std::string& p, int& w, int& h) {
            if (p != "ui/t.png" && p != "ui/s.png") return false;
            w = 200, h = 100;
            return true;
        };
        env.command = [this](const std::string& c) { commands.push_back(c); };
        script = std::make_unique<UiScript>(rng, std::move(env), gui);
        if (!script->load(path)) std::printf("load failed: %s\n", path.c_str());
    }
    // One frame as the game runs it: tick, draw, then GuiBox::tick.
    const std::vector<UiBatch>& frame(const UiPointer& p) {
        script->tick(1.0f / 60.0f, {}, p);
        const std::vector<UiBatch>& b = script->draw();
        gui.end_frame(p.released);
        return b;
    }
};

UiPointer at(float x, float y, bool down = false, bool pressed = false, bool released = false) {
    UiPointer p;
    p.x = x, p.y = y, p.down = down, p.pressed = pressed, p.released = released;
    return p;
}

void test_picking() {
    Harness h;
    h.files["ui/box.xml"] =
        "<ui texture=\"t.png\" selected=\"s.png\">"
        "<rect id=\"a\" coords=\"0 0 100 100\" cmd=\"hitA\"/>"
        "<rect id=\"b\" group=\"1\" coords=\"100 0 150 100\" cmd=\"hitB ; script:fromB\"/>"
        "<rect id=\"c\" group=\"1\" selected=\"true\" coords=\"150 0 200 100\" cmd=\"hitC\"/>"
        "<outside cmd=\"out\"/></ui>";
    h.files["ui/modal.xml"] = "<ui texture=\"t.png\"><rect coords=\"0 0 200 100\" cmd=\"m\"/><outside cmd=\"mout\"/></ui>";
    h.files["ui/main.lua"] = R"(
function init()
  c = mgCreateCanvas(1000, 500)
  mgSetOrigo(c, "center")
  mgSetPos(c, 1024, 768)
  mgSetScale(c, 2, 2)
  u = mgCreateUi("box.xml")
  mgSetPos(u, 100, 100)
  m = mgCreateUi("modal.xml")
  mgSetPos(m, 1500, 100)
end
function draw()
  mgPushCanvas(c)
  mgDraw(u)
  mgPopCanvas()
  mgDraw(m)
end
function handleCommand(cmd)
  mgCommand("script " .. cmd)
  if cmd == "modal" then mgSetUiModal(m, true) end
end
)";
    h.start("ui/main.lua");

    // The UI inside the canvas: canvas-local (100, 100) lands at 1024 + (100 - 500) * 2 = 224, 768 + (100 - 250) * 2.
    const std::vector<UiBatch>& b = h.frame(at(300, 500));
    check(!b.empty() && b[0].texture == "ui/t.png" && near(b[0].vertices[0].x, 224) && near(b[0].vertices[0].y, 468) &&
              near(b[0].vertices[1].x, 624) && near(b[0].vertices[2].y, 668),
          "canvas transform: origin, scale, position");
    // The radio group's selected rect draws its highlight without a hover.
    bool radio_drawn = false;
    for (const UiBatch& x : b)
        if (x.texture == "ui/s.png" && near(x.vertices[0].x, 224 + 300) && near(x.vertices[0].u, 0.75f)) radio_drawn = true;
    check(radio_drawn, "radio group: selected rect highlighted");
    check(h.gui.hover != nullptr, "the rect under the pointer becomes the hover");

    // A press on the hovered rect selects it; the next tick runs its command.
    h.frame(at(300, 500, true, true));
    check(h.commands.empty(), "a selection waits for the next tick");
    h.frame(at(300, 500, true));
    check(h.commands.size() == 1 && h.commands[0] == "hitA", "selection runs as a game command");
    h.frame(at(300, 500, false, false, true));
    check(h.gui.hover == nullptr, "release clears the hover");

    // Rect b: two commands, one for the script; it becomes its group's selected rect.
    h.commands.clear();
    const float bx = 224 + 250, by = 520;  // inside rect b (texels 100..150 -> 424..524)
    h.frame(at(bx, by));
    h.frame(at(bx, by, true, true));
    h.frame(at(bx, by, true));
    check(h.commands.size() == 2 && h.commands[0] == "hitB" && h.commands[1] == "script fromB",
          "';' commands, 'script:' to handleCommand");
    const std::vector<UiBatch>& b2 = h.frame(at(1900, 1300, false, false, true));
    bool c_drawn = false, b_drawn = false;
    for (const UiBatch& x : b2)
        if (x.texture == "ui/s.png") {
            if (near(x.vertices[0].u, 0.75f)) c_drawn = true;
            if (near(x.vertices[0].u, 0.5f)) b_drawn = true;
        }
    check(b_drawn && !c_drawn, "selecting b moves the radio selection from c");

    // A press away from every rect selects the <outside> command of each box that takes input.
    h.commands.clear();
    h.frame(at(1900, 1300, true, true));
    h.frame(at(1900, 1300, true));
    check(h.commands.size() == 1 && h.commands[0] == "out", "outside command (first box with one)");

    // With a modal box, the others take no input: their rects are not picked and their outside is silent.
    h.frame(at(1900, 1300, false, false, true));
    h.script->handle_command("modal");
    h.commands.clear();
    h.frame(at(300, 500));
    check(h.gui.hover == nullptr, "modal: other boxes are not picked");
    h.frame(at(300, 500, true, true));
    h.frame(at(300, 500, true));
    bool only_mout = !h.commands.empty();
    for (const std::string& c : h.commands)
        if (c != "mout" && c.rfind("script", 0) != 0) only_mout = false;
    check(only_mout, "modal: a press elsewhere reaches only the modal box's outside");
}

void test_drag() {
    Harness h;
    h.files["ui/main.lua"] = R"(
function init()
  lc = mgCreateCanvas(4000, 1536)
  mgSetCanvasWindow(lc, 0, 0, 2048, 1536)
  mgSetCanvasMovable(lc, "x")
end
function draw()
  local x, y = mgGetPos(lc)
  mgCommand(string.format("%.6f %.6f", x, y))
end
)";
    h.start("ui/main.lua");
    auto pos = [&](float& x, float& y) {
        x = y = -9999;
        if (!h.commands.empty()) std::sscanf(h.commands.back().c_str(), "%f %f", &x, &y);
    };
    float x, y;
    h.frame(at(1000, 700, true, true));  // grab
    h.frame(at(900, 650, true));         // 100 left (and 50 up: movable "x" only)
    pos(x, y);
    check(near(x, -100) && near(y, 0), "drag follows the pointer along x");
    h.frame(at(1100, 700, true));  // 100 right of the press: past the right edge (0), half of it given
    pos(x, y);
    check(near(x, 50), "rubber band past the window edge");
    h.frame(at(1100, 700, false, false, true));  // release: the drag ends, nothing moves this tick
    pos(x, y);
    check(near(x, 50), "release ends the drag");
    // Coasting: velocity -20 * 0.8 + 150 * 0.2 = 14; p = 64, clamped 0, to 64 * 0.7 = 44.8.
    h.frame(at(1100, 700));
    pos(x, y);
    check(near(x, 44.8f), "coasts with its velocity and settles back inside");
    for (int i = 0; i < 300; ++i) h.frame(at(1100, 700));
    pos(x, y);
    check(std::fabs(x) < 0.01f, "comes to rest at the edge");
}

}  // namespace

int main() {
    test_picking();
    test_drag();
    if (g_failures) std::printf("%d FAILED\n", g_failures);
    return g_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
