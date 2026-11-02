#include "game/effects.hpp"

#include "game/level.hpp"
#include "physics/explosion.hpp"

#include <cmath>

namespace sh::game {

namespace {
constexpr int kStreakBank = 0x2298;  // Audio +0x2298: snd/cleared0..11
}

void Effects::explosion(phys::RigidBody*, phys::Vec3 pos, float strength) {
    phys::ExplosionEnv env;
    env.camera_position = level_->camera;
    env.camera_shake = level_->display ? &level_->display->shake : nullptr;
    env.debris = &debris;
    phys::explosion(*world_, *this, *rng_, env, pos, strength);
}

void Effects::update_debris() {
    phys::DebrisEnv env;
    env.world = world_;
    env.gravity = phys::level_gravity(&level_->current->gravity, level_->camera_roll);  // Level::getGravity
    env.dt = world_->time_step;
    env.frame = world_->frame;
    env.camera_z = level_->camera.z;
    debris.update(env, *rng_);
}

void Effects::spawn_debris(const void* obstacle, phys::Vec3 pos, phys::Vec3 vel, Rgba color) {
    const float rgba[4] = {color.r, color.g, color.b, color.a};
    debris.spawn(obstacle, pos, {0, 0, 0, 1}, vel, rgba, *rng_);
}

void Effects::load_particles(const std::string& xml, int texture_w, int texture_h) {
    for (int i = 0; i < 11; ++i) systems[i].load(xml, kSystemNames[i], texture_w, texture_h);
}

void Effects::spawn_particle(Effect e, phys::Vec3 pos, phys::Vec3 vel, Rgba color, int sprite) {
    // Effect -> the Level's system (index into kSystemNames).
    static constexpr int kSystem[11] = {0 /*explosion_white*/, 7 /*spark*/,         6 /*break_debris*/, 5 /*break_smoke*/,
                                        2 /*explosion_dark*/,  3 /*explosion_dust*/, 4 /*trail*/,       8 /*laserspark*/,
                                        9 /*ambient*/,         10 /*bubbles*/,     1 /*smoke*/};
    ++particles;
    const float rgba[4] = {color.r, color.g, color.b, color.a};
    systems[kSystem[static_cast<int>(e)]].spawn(*rng_, pos, vel, rgba, sprite);
}

void Effects::update_particles() {
    const GameIO& io = level_->io();
    for (ParticleSystem& s : systems) s.update(*rng_, io.dt, io.frame, io.time_scale_flag);
}

void Effects::translate_particles(float dz) {
    for (ParticleSystem& s : systems) s.translate(dz);
}

void Effects::clear_particles() {
    for (ParticleSystem& s : systems) s.clear();
}

phys::Vec3 Effects::listener() const {
    return level_ && level_->display ? level_->display->position : phys::Vec3{0, 0, 0};  // Display +0x834
}

float Effects::volume_at(phys::Vec3 pos) const {
    const phys::Vec3 l = listener();
    const float dy = pos.y - l.y, dx = pos.x - l.x, dz = pos.z - l.z;
    float d = std::sqrt((dy * dy + dx * dx) + dz * dz);
    if (l.z < pos.z) d = d * 3.0f + 3.0f;  // behind the camera: much quieter
    return 1.0f / (d * d * 0.03f + 1.0f);
}

float Effects::pan_at(phys::Vec3 pos) const {
    const phys::Vec3 l = listener();
    const float dy = pos.y - l.y, dx = pos.x - l.x, dz = pos.z - l.z;
    const float p = pos.x * 1.5f / std::sqrt((dy * dy + dx * dx) + dz * dz);  // absolute x, as the original
    return !(p > -1.0f) ? -1.0f : (p < 1.0f ? p : 1.0f);
}

void Effects::first_break(phys::RigidBody&) {
    if (level_) ++level_->f29c;
    if (PlayerStats* s = scoring ? scoring->stats() : nullptr) s->register_broken();
}

int Effects::pick_sound(Bank b) {
    SoundBank* bank = banks.find(static_cast<int>(b));
    if (!bank || !level_) return 0;
    const int i = bank->next(*rng_, level_->io().frame, level_->io().dt);
    return i < 0 ? 0 : static_cast<int>(b) * 16 + i + 1;
}

std::string Effects::sound_path(int sound) const {
    const SoundBankInfo* info = banks.info((sound - 1) / 16);
    return info ? std::string(info->prefix) + std::to_string((sound - 1) % 16) + ".ogg" : std::string();
}

void Effects::play_at(const std::string& path, phys::Vec3 pos, float volume, float pitch) {
    ++sounds;
    if (out) out->play(path, volume_at(pos) * volume, pan_at(pos), pitch, true);
}

void Effects::play_sound(int sound, phys::Vec3 pos, float volume, float pitch) {
    if (sound) play_at(sound_path(sound), pos, volume, pitch);
}

void Effects::play_sound_flat(int sound, float volume, float pitch) {
    if (!sound) return;
    ++sounds;
    if (out) out->play(sound_path(sound), volume, 0.0f, pitch, false);
}

void Effects::play_file(const char* path, float volume, float pitch) {
    ++sounds;
    if (out) out->play(path, volume, 0.0f, pitch, false);
}

void Effects::play_event(const script::SoundEvent& e) {
    std::string path = e.name;
    if (e.index >= 0) {
        SoundBank* bank = banks.find(kStreakBank);
        const int i = bank && level_ ? bank->specific(e.index, level_->io().frame, level_->io().dt) : -1;
        if (i < 0) return;
        path = banks.info(kStreakBank)->prefix + std::to_string(i) + ".ogg";
    }
    if (e.positional) {
        play_at(path, e.position, e.volume, e.pitch);
    } else {
        ++sounds;
        if (out) out->play(path, e.volume, 0.0f, e.pitch, false);
    }
}

void Effects::play_loop(Loop loop, phys::Vec3 pos, float volume) {
    for (LoopState& s : loops_) {
        if (s.loop != loop) continue;
        const float v = volume_at(pos) * volume;
        s.sum.y += v * pos.y, s.sum.x += v * pos.x, s.sum.z += v * pos.z;
        s.weight += v;
        if (v > s.loudest) s.loudest = v;
    }
}

void Effects::end_frame() {
    for (int i = 0; i < 3; ++i) {
        LoopState& s = loops_[i];
        if (out) {
            if (s.loudest == 0.0f || s.weight == 0.0f) {
                out->loop(i, s.path, 0.0f, 0.0f);
            } else {
                const phys::Vec3 avg{s.sum.x / s.weight, s.sum.y / s.weight, s.sum.z / s.weight};
                const float volume = !(s.loudest > 0.0f) ? 0.0f : (s.loudest < 10.0f ? s.loudest : 10.0f);
                out->loop(i, s.path, volume, pan_at(avg));
            }
        }
        s.sum = {0, 0, 0}, s.weight = 0, s.loudest = 0;
    }
}

}  // namespace sh::game
