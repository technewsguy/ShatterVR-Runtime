#pragma once

// The level being played (the original's Level): the room list, the current and next
// room, the camera's run down -z, the frame update (time scale, camera speed, room hand-off, entity culling,
// physics, the crash test, the UI properties) and the floating origin (centerCamera).

#include "game/display.hpp"
#include "game/level_def.hpp"
#include "game/powerup.hpp"
#include "game/room.hpp"
#include "game/scoring.hpp"
#include "physics/debris.hpp"
#include "physics/physics_world.hpp"

#include <map>
#include <optional>
#include <memory>
#include <string>
#include <vector>

namespace sh::game {

// What Level reads from and writes to the rest of the game (the original's Game, Audio, Input, Player and a few
// global flags). Plain values in, logs out, so a frame can be driven and checked before those systems exist.
struct GameIO {
    // Game
    float dt = 1.0f / 60.0f;  // +0x100: the frame time; Level::update scales it in place (slow motion, pause)
    int frame = 0;            // +0x104
    float extra_time = 0;     // +0x108: added to Level +0x2a0 every frame (unverified meaning)
    float pause_blend = 0;    // +0x1e8: 1 stops the game clock
    int state = 0;            // +0x130: 3 lets the level drive the music pitch
    float menu_fade = 0;      // +0x1f0 -> +0x3a0: the level music starts once it is below 0.1
    std::string spec;         // +0x168: the level spec (campaign room index, "level:..", ...)
    std::map<std::string, std::string> properties;  // Game +0x3c bag ("lastdistance")
    // Input
    bool key_fast = false;          // QiInput::isKeyDown(0x66): fast forward
    bool key_pause_pressed = false; // QiInput::wasKeyPressed(0x73): toggles the pause
    // Shooting input (Level::handleInput): touch 0 and button 8, and the debug keys pressed this frame.
    int touch_count = 0;
    bool touch_pressed = false, touch_released = false, has_touch = false;
    bool button_pressed = false, button_released = false, button_down = false;
    std::vector<int> keys_pressed;  // ' ', 'n', 'b', 'd', '1'..'4'
    int burst_flag = 0;             // a global handleInput clears after a rapid-fire burst of 3+ (use unverified)
    // Orientation
    phys::Quat camera_rotation{0, 0, 0, 1};   // Game +0x224: the head; aims the balls, steers the despawn rule
    // The port's pointer aiming (flat PC build): when set, shots are aimed with this rotation instead of the level
    // roll times the head (it should turn -z towards the pointer). Not in the original.
    std::optional<phys::Quat> aim_override;
    // Global flags
    bool paused = false;         // the 0x73 toggle
    bool hints_shown[2] = {};    // "hint1" (75 m in basic/basic), "hint2" (150 m)
    bool editor_flag = false;    // byte 0x226ed9 (also the script seed flag)
    bool time_scale_flag = false;  // byte 0x226eda
    float time_scale = 1;        // float 0x216a14
    // Audio
    bool music_sync = false;     // Audio +0x28b1: the room time follows the music position
    float music_location = 0;    // Audio::getLevelMusicLocation (seconds)
    bool music_swap = false;     // Audio +0x15c -> +0x38: swapLevelMusic instead of playLevelMusic
    struct Channel {
        bool on = false;         // +0x2c
        float a20 = 0, a24 = 0;  // +0x20, +0x24 (pitch)
    } channels[3];               // Audio +0x15c, +0x28b8, +0xcc
    float audio_e8 = 0;          // Audio +0xe8
    bool boss_loop_playing = false;
    std::string boss_file;       // Audio::getCurrentBossFileName
    // Player
    int high_score = 1;          // Player::getHighScore(1)
    // Calls into audio, player stats/saves, the scripts and the game, in order.
    std::vector<std::string> log;
};

class Level {
public:
    Level(GameIO& io, RoomContext& ctx, Scoring& scoring, phys::PhysicsEvents& events, std::vector<RoomDef> rooms,
          bool campaign);
    ~Level();

    void reset();         // Level::reset
    void load_next_room();  // Level::loadNextRoom
    void enter_room(Room& room);  // Level::enterRoom
    int level_distance(int level) const;  // Level::getLevelDistance
    void update();        // Level::update
    void center_camera(); // Level::centerCamera
    void loop_camera(float dz);  // Level::loopCamera
    void handle_input();  // Level::handleInput (the gaze-aimed shots and the debug keys)
    phys::RigidBody& create_ball();  // Level::createBall
    void shoot(phys::Vec3 direction, float ball_speed, bool armed);  // Level::shoot
    void hit_something();  // Level::hitSomething: the crash (balls lost, shake, the streak, a spray of balls)
    // Room::createPowerUp / Level::create(1): a power-up of that type name at pos, owned by the obstacle.
    void create_powerup(const std::string& type, phys::Vec3 pos, Obstacle& owner);
    void update_powerup(Powerup& p);    // Powerup::update
    void pick_up(Powerup& p);           // Powerup::pickUp
    void activate_powerup(int type);    // Level::activatePowerup: every timer 0, then the type's timer 7 s
    phys::ButtonEntity* create_button(phys::Vec3 pos, std::vector<phys::Entity*>* owner_list, const void* owner);
    void update_button(phys::ButtonEntity& b);  // Button::update
    void add_laser(phys::Vec3 from, phys::Vec3 dir);  // Level::addLaser (mgLaser)
    void process_lasers();                             // Level::processLasers
    void destroy_entity(phys::Entity& e);  // Level::destroy

    std::vector<RoomDef> rooms;  // +0x1cc
    int room_index = 0;          // Player +0x58c
    int distance_score = 0;      // Player +0x590
    bool campaign = false;       // +0xe9
    std::unique_ptr<Room> current, next;  // +0xd0, +0xd4
    phys::Vec3 camera{0, 1, 0};  // +0xec
    phys::Quat camera_roll{0, 0, 0, 1};  // +0x270 (copied to +0xf8)
    float speed = 0;             // +0x110
    float distance = 0;          // +0x144
    float invulnerable = 0;      // +0x148
    float f14c = 0, f150 = 0;    // +0x14c, +0x150
    float time_scale = 1;        // +0x154
    float timers[12] = {};       // +0x158..+0x184 (+0x164 = slow motion)
    float f188 = 0;              // +0x188: the rapid-fire cooldown
    int burst = 0;               // +0x18c: shots in the current rapid-fire burst
    int f238 = -1, f23c = 0;     // +0x238, +0x23c (set by reset and handleInput; use unverified)
    float sim_time = 0;          // +0xd8
    int frames = 0;              // +0xe0
    int distance_base = 0;       // +0x218: the def lengths of the rooms already left
    bool game_over = false;      // +0x214
    float fade = 0;              // +0x21c: rises to 1 while the player has no balls (game over at 1)
    bool end_transition = false; // +0x224
    float end_t = 0, end_from = 0;  // +0x228, +0x22c
    bool scripted_move = false;  // +0x230
    float scripted_speed = 0;    // +0x234
    float room_time = 0;         // +0x250
    float room_pace = 0;         // +0x254
    float pain = 0;              // +0x264
    bool endless = false;        // +0x268
    bool credits = false;        // +0x269
    int quicksave_in = 0;        // +0x26c
    int stat_streak = 0, stat_balls = 0, stat_frames = 0;  // +0x280, +0x284, +0x288
    float dist_ach = 0, streak_dist_ach = 0;  // +0x290, +0x294
    bool music_started = false;  // +0x298
    int f29c = 0;                // +0x29c: bodies broken this level
    float f2a0 = 0, f2a4 = 0;    // +0x2a0, +0x2a4
    static const PropertySchema& schema();  // the "Level" property schema (Level::Level)
    PropertyBag properties{schema()};       // the Level's own PropertyBag (+0)
    // Game::getProperty / setProperty for the scripts (mgGet / mgSet): "bag.key" with bag level, player or game.
    std::string get_property(const std::string& key) const;
    void set_property(const std::string& key, const std::string& value);
    Display* display = nullptr;  // Game +8: ball placement (its rotation), the crash shake and spray
    phys::Debris* debris = nullptr;  // Level +0xcc: shifted by centerCamera / loopCamera (owned by the effects)
    std::vector<std::unique_ptr<Powerup>> powerups;  // +0x12c, in list order
    std::vector<std::unique_ptr<phys::ButtonEntity>> buttons;  // +0x1fc, in list order
    struct Laser {
        phys::Vec3 from, to;
    };
    std::vector<Laser> lasers;  // +0x208: this frame's beams (cleared before the rooms update)
    const GameIO& io() const { return *io_; }

private:
    void log(std::string s) { io_->log.push_back(std::move(s)); }
    std::string music_path(const Room& room) const;
    std::unique_ptr<Room> make_room(const RoomDef& def, float start);

    GameIO* io_;
    RoomContext* ctx_;
    Scoring* scoring_;
    phys::PhysicsEvents* events_;
};

}  // namespace sh::game
