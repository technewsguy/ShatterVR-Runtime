#pragma once

// The game's camera (the original's Display): the main 3D viewport placed at the level's
// camera with the head rotation and the level roll, one eye's offset, the crash shake, the field of view, and
// the screen-to-world helpers the game logic uses (pixelToWorldDir for the crash spray).

#include "core/rand48.hpp"
#include "game/viewport.hpp"

namespace sh::game {

// What Display::update reads from the game and the level.
struct DisplayInput {
    int game_state = 3;               // Game +0x130 (3 = playing: the level roll joins the head rotation)
    phys::Quat camera_rotation;       // Game +0x224: the head
    int eye = 0;                      // Game +0x234: 0 left (minus the eye offset), else right (plus)
    phys::Vec3 level_camera{0, 1, 0}; // Level +0xec
    phys::Quat level_roll;            // Level +0xf8
    float level_time = 0;             // Level +0xd8 (the shake's wobble)
    float dt = 1.0f / 60.0f;          // Game +0x100
    float edit_zoom = 0;              // the level's "editZoom" property (toFloat)
    float edit_pan[2] = {};           // the level's "editPan" property, two words
};

class Display {
public:
    explicit Display(Rand48& rng) : rng_(&rng) {}
    void set_geometry(int w, int h);    // Display::setGeometry (the main viewport's part)
    void update(const DisplayInput& in);  // Display::update (game states other than 4)
    phys::Vec3 pixel_to_world_dir(float px, float py) const;  // Display::pixelToWorldDir
    // Display::enterMenuMode's GUI viewport (the loading screen's world pass): the main viewport, then translate
    // (0, 4, 0), rotate pi/2 about +y, scale (0.029, -0.029, 1), translate (-1024, -768, 0). GUI points with a depth
    // (mgSetZ) go through it with project (Display::guiToPixel).
    Viewport menu_view() const;

    int width = 0, height = 0;          // +0, +4
    Viewport main_view;                 // +8: the main 3D viewport
    float edit_zoom = 1;                // +0x828
    float edit_pan[2] = {};             // +0x82c
    phys::Vec3 position;                // +0x834
    phys::Quat rotation;                // +0x840 (where a new ball appears from: Level::createBall)
    int fov_ease = 0;                   // +0x850: the easing of a field-of-view change (1-5)
    float fov_duration = 0, fov_elapsed = 0;  // +0x854, +0x858
    float fov = 0, fov_target = 0;      // +0x85c, +0x860
    phys::Vec3 corners[4];              // +0x874: the view rays through the four screen corners, divided by -z
    float shake = 0;                    // +0x8cc: set by a crash (at least 1), at most 2, counts down by dt
    // This update's shake as applied above (position += offset, rotation = roll about z by -roll first); sh_play --vr
    // puts it on the headset's eyes.
    phys::Vec3 shake_offset;
    float shake_roll = 0;
    bool wide_fov = true;               // a global (1 in the shipped data): the target field of view is 90, else 60

private:
    float eased_fov();
    Rand48* rng_;
    float ease_table_[1024] = {};
    bool ease_table_ready_ = false;
};

}  // namespace sh::game
