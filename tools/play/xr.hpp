#pragma once

// sh_play's OpenXR side (--vr): an OpenGL session on the window's context, one swapchain per eye with a depth/stencil
// buffer of our own, the head and both hands' aim poses in the LOCAL space (origin where the head started, -z ahead,
// y up, metres), and two actions: "throw" (trigger / select) and "back" (menu / B). The game turns these into its
// own inputs; nothing here knows about the game.

#include "physics/pmath.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace sh::play {

struct VrPose {
    bool valid = false;
    phys::Vec3 position;
    phys::Quat orientation;
};

struct VrEye {
    VrPose pose;
    float angle_left = 0, angle_right = 0, angle_up = 0, angle_down = 0;  // radians (left and down negative)
};

struct VrFrame {
    bool render = false;  // the runtime wants this frame drawn
    VrEye eyes[2];
    VrPose head;
    VrPose aim[2];              // 0 left, 1 right hand
    bool throw_pressed[2] = {}; // went down since the last frame
    bool throw_down[2] = {};
    bool back_pressed = false;
    bool recenter_pressed = false;  // thumbstick / trackpad click
    bool aim_toggle_pressed = false;  // A / X (grip on Vive and Windows MR): head or hand aiming
};

class XrRig {
public:
    XrRig();
    ~XrRig();
    XrRig(const XrRig&) = delete;
    XrRig& operator=(const XrRig&) = delete;

    // Needs the window's OpenGL context current. False with a reason (no runtime, no headset, ...).
    bool init(std::string& error);
    // Runtime events; false once the session is over (exit requested, lost, or the instance went away).
    bool poll();
    bool session_running() const;
    // xrWaitFrame / xrBeginFrame, then the views and actions. Only while the session runs.
    bool begin_frame(VrFrame& state);
    // The framebuffer for one eye this frame (the swapchain image plus our depth/stencil buffer), and its size.
    unsigned acquire_eye(int eye, int& width, int& height);
    void release_eye(int eye);
    // xrEndFrame with the two views as a projection layer (or no layer when the frame was not rendered).
    void end_frame(const VrFrame& state);
    std::string runtime_name() const;
    // A vibration on one hand (0 left, 1 right): amplitude 0..1 for a duration in seconds.
    void vibrate(int hand, float amplitude, float seconds);
    // From now on poses are given relative to the head as it is now: its place becomes the origin and its heading
    // (yaw only) -z. The runtime's own LOCAL space is unchanged (the composition layer still uses it).
    void recenter();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sh::play
