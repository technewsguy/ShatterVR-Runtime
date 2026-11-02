#pragma once

// The game's side of the physics events (the original's Level / Audio / ParticleSystem / Debris callees): explosions
// and breakage debris run for real; sounds are picked from the banks (SoundBank::next draws from the shared lrand48,
// as in the original), given the original's distance volume and pan (Audio::getVolume, Audio::playSound,
// SoundLoop) and handed to a SoundOut. The Level's particle systems spawn and update for real (their lrand48 draws
// included).

#include "core/rand48.hpp"
#include "game/particles.hpp"
#include "game/scoring.hpp"
#include "game/sound_bank.hpp"
#include "physics/debris.hpp"
#include "physics/physics_world.hpp"
#include "script/level_script.hpp"

#include <string>

namespace sh::game {

class Level;

// Where the picked sounds go (the PC mixer; nothing when unset). Volume 0..10, pan -1..1.
struct SoundOut {
    virtual ~SoundOut() = default;
    // positional: Audio::playSound's positional pool (with the room's effects); else the dry one.
    virtual void play(const std::string& path, float volume, float pan, float pitch, bool positional) = 0;
    // A looping channel's state for this frame (volume 0 stops it).
    virtual void loop(int channel, const std::string& path, float volume, float pan) = 0;
};

class Effects : public phys::PhysicsEvents {
public:
    Effects(phys::PhysicsWorld& world, Rand48& rng) : world_(&world), rng_(&rng) {}
    void attach(Level& level) { level_ = &level; }

    phys::Debris debris;  // Level +0xcc
    SoundBanks banks;     // the Audio object's banks (load them first)
    SoundOut* out = nullptr;
    Scoring* scoring = nullptr;  // for the stats bag
    int sounds = 0, particles = 0;

    // A sound id: 0 for none, else bank offset * 16 + file index + 1.
    int pick_sound(Bank b) override;
    void play_sound(int sound, phys::Vec3 pos, float volume, float pitch) override;
    void play_sound_flat(int sound, float volume, float pitch) override;
    void play_file(const char* path, float volume, float pitch) override;
    void play_loop(Loop, phys::Vec3 pos, float volume) override;
    void spawn_particle(Effect e, phys::Vec3 pos, phys::Vec3 vel, Rgba color, int sprite) override;
    void update_particles() override;
    void translate_particles(float dz) override;
    void clear_particles() override;
    // processBreakage's first break: Level +0x29c += 1, Player::registerBroken. Player::registerBall.
    void first_break(phys::RigidBody&) override;
    void ball_removed(bool broke_something) override {
        if (PlayerStats* s = scoring ? scoring->stats() : nullptr) s->register_ball(broke_something);
    }

    // Level::init: the systems from effects/default.xml (texture: gfx/sprites.png, for the sprite coords).
    void load_particles(const std::string& xml, int texture_w, int texture_h);
    // The Level's list (+0x1c0) in its order: explosion_white, smoke, explosion_dark, explosion_dust, trail,
    // break_smoke, break_debris, spark, laserspark, ambient, bubbles (Level +0x194, +0x1bc, +0x198 ... +0x1b8).
    static constexpr const char* kSystemNames[11] = {"explosion_white", "smoke",       "explosion_dark", "explosion_dust",
                                                     "trail",           "break_smoke", "break_debris",   "spark",
                                                     "laserspark",      "ambient",     "bubbles"};
    ParticleSystem systems[11];

    void explosion(phys::RigidBody* ball, phys::Vec3 pos, float strength) override;
    void update_debris() override;
    void spawn_debris(const void* obstacle, phys::Vec3 pos, phys::Vec3 vel, Rgba color) override;

    // A script or scoring sound (mgPlaySound at the obstacle; "streak": the cleared bank's getSpecific).
    void play_event(const script::SoundEvent& e);
    // SoundLoop::end then begin for every loop: once per frame, after the update.
    void end_frame();

    float volume_at(phys::Vec3 pos) const;  // Audio::getVolume
    float pan_at(phys::Vec3 pos) const;     // the pan of Audio::playSound(sound, pos, ...)

private:
    std::string sound_path(int sound) const;
    void play_at(const std::string& path, phys::Vec3 pos, float volume, float pitch);
    phys::Vec3 listener() const;

    struct LoopState {  // SoundLoop +0x3c..+0x4c
        Loop loop;
        const char* path;
        phys::Vec3 sum{0, 0, 0};
        float weight = 0, loudest = 0;
    };
    LoopState loops_[3] = {{Loop::kLaser, "snd/laser/loop.ogg"},
                           {Loop::kBall, "snd/ball/rollloop.ogg"},
                           {Loop::kObject, "snd/object/slideloop.ogg"}};
    phys::PhysicsWorld* world_;
    Rand48* rng_;
    Level* level_ = nullptr;
};

}  // namespace sh::game
