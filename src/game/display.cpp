#include "game/display.hpp"

#include "physics/qi_rotate.hpp"

#include <cmath>

namespace sh::game {

void Display::set_geometry(int w, int h) {
    width = w, height = h;
    main_view.init(0, 0, w, h);
    // (The pixel, GUI and ortho viewports, the reflection FBO and the gui size properties are not ported.)
}

float Display::eased_fov() {
    if (fov_duration == 0.0f) return fov_target;
    float t = fov_elapsed / fov_duration;
    if (!(t > 0.0f)) t = 0.0f;
    else if (!(t < 1.0f)) t = 1.0f;
    const float pi = 3.14159274f;
    float e;
    switch (fov_ease) {
        case 1: e = t; break;
        case 2: e = 1.0f - std::cos((t * pi) * 0.5f); break;
        case 3: e = std::sin((t * pi) * 0.5f); break;
        case 4:
            e = (1.0f - std::cos(t * pi)) * 0.5f;
            e = e + (1.0f - e) * 0.0f;
            break;
        case 5: e = ease_table_[static_cast<int>(t * 1023.0f)]; break;
        default: e = 1.0f;
    }
    return fov + (fov_target - fov) * e;
}

void Display::update(const DisplayInput& in) {
    shake_offset = {0, 0, 0}, shake_roll = 0.0f;
    edit_zoom = in.edit_zoom;
    edit_pan[0] = in.edit_pan[0], edit_pan[1] = in.edit_pan[1];
    // (Game state 4, a camera of its own, is not ported.)
    position = in.level_camera;
    rotation = phys::qi_mul(in.level_roll, in.camera_rotation);
    if (!ease_table_ready_) {  // the 1024-step ease curve, made once
        float v = 0.0f, d = 0.0f;
        for (float& e : ease_table_) {
            e = v;
            d = (1.0f - v) * 0x1.6f0068p-12f + d * 0.98f;  // 0.00035
            v = v + d;
        }
        ease_table_ready_ = true;
    }
    fov = eased_fov();
    fov_target = wide_fov ? 90.0f : 60.0f;
    fov_ease = 0, fov_duration = 0.0f, fov_elapsed = 0.0f;
    rotation = in.game_state == 3 ? phys::qi_mul(in.level_roll, in.camera_rotation) : in.camera_rotation;
    // This eye's half of the 6.5 cm eye distance, along the view's x axis.
    const float k = 0.0325f;
    const phys::Vec3 off = phys::qi_rotate(rotation, {1.0f * k, 0.0f * k, 0.0f * k});
    if (in.eye == 0) position = {position.x - off.x, position.y - off.y, position.z - off.z};
    else position = {position.x + off.x, position.y + off.y, position.z + off.z};
    // (The GUI viewport at +0x180 is not ported.)

    // The crash shake: a wobble and two random draws, scaled by the frame time; a roll by the same x offset.
    if (shake > 0.0f) {
        const float steps = in.dt / 0x1.111f0cp-6f;  // 0.01667
        if (!(shake < 2.0f)) shake = 2.0f;
        const float s40 = std::sin(in.level_time * 40.0f);
        const float s33 = std::sin(in.level_time * 33.0f);
        const float lo = steps * -0.1f, hi = steps * 0.1f;
        float r1 = rng_->rnd(lo, hi);
        float r2 = rng_->rnd(lo, hi);
        const float a = shake * 0.2f;
        r1 = r1 + s40 * 0.3f;
        r2 = r2 + s33 * 0.3f;
        const float dx = a * r1;
        shake_offset = {dx, a * r2, 0.0f}, shake_roll = dx;  // (kept for the VR eyes)
        position.y = position.y + a * r2;
        position.x = dx + position.x;
        position.z = position.z + 0.0f;
        rotation = phys::qi_mul(phys::axis_angle({0, 0, 1}, -dx), rotation);
        shake = shake - in.dt;
    }

    main_view.f14 = 0.0f;
    main_view.update_projection();
    main_view.f18 = 0.0f;
    main_view.update_projection();
    main_view.set_bounds(0, 0, width, height);
    main_view.set_mode_3d(eased_fov(), 0.2f, 100.0f);
    main_view.set_camera_pos(position);
    main_view.set_camera_rot(rotation);
    const float w = static_cast<float>(main_view.x1 - main_view.x0), h = static_cast<float>(main_view.y1 - main_view.y0);
    corners[0] = main_view.pixel_direction(0.0f, 0.0f);
    corners[1] = main_view.pixel_direction(0.0f, h);
    corners[2] = main_view.pixel_direction(w, h);
    corners[3] = main_view.pixel_direction(w, 0.0f);
    for (phys::Vec3& c : corners) {
        const float nz = -c.z;
        c = {c.x / nz, c.y / nz, c.z / nz};
    }
}

phys::Vec3 Display::pixel_to_world_dir(float px, float py) const {
    const phys::Vec3 a = main_view.unproject({px, py, 0.0f});
    const phys::Vec3 b = main_view.unproject({px, py, 1.0f});
    const float dy = b.y - a.y, dx = b.x - a.x, dz = b.z - a.z;
    const float len = std::sqrt((dy * dy + dx * dx) + dz * dz);
    if (len > 0.0f) return {dx / len, dy / len, dz / len};
    return {1.0f, 0.0f, 0.0f};
}

}  // namespace sh::game

namespace sh::game {

Viewport Display::menu_view() const {
    Viewport v = main_view;
    v.translate({0.0f, 4.0f, 0.0f});
    v.rotate(phys::axis_angle({0, 1, 0}, 1.5707964f));
    v.scale({0.029f, -0.029f, 1.0f});
    v.translate({-1024.0f, -768.0f, 0.0f});
    return v;
}

}  // namespace sh::game
