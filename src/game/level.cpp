#include "game/level.hpp"

#include "core/qi_string.hpp"
#include "game/obstacle.hpp"
#include "physics/debris.hpp"
#include "physics/qi_random.hpp"
#include "physics/qi_rotate.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace sh::game {

namespace {

std::string fmt(float v) { return format_float(v, 5); }

}  // namespace

Level::Level(GameIO& io, RoomContext& ctx, Scoring& scoring, phys::PhysicsEvents& events, std::vector<RoomDef> defs,
             bool is_campaign)
    : rooms(std::move(defs)), campaign(is_campaign), io_(&io), ctx_(&ctx), scoring_(&scoring), events_(&events) {
    ctx.destroy_other = [this](phys::Entity& e) { destroy_entity(e); };
    ctx.create_powerup = [this](const std::string& type, phys::Vec3 pos, Obstacle& owner) { create_powerup(type, pos, owner); };
    ctx.level->get_property = [this](const std::string& k) { return get_property(k); };
    ctx.level->set_property = [this](const std::string& k, const std::string& v) { set_property(k, v); };
    ctx.level->add_laser = [this](phys::Vec3 from, phys::Vec3 dir) { add_laser(from, dir); };
    ctx.level->smoke = [this](phys::Vec3 pos, phys::Vec3 vel) {  // mgSmoke: the smoke system, white, a random cell
        events_->spawn_particle(phys::PhysicsEvents::Effect::kSmoke, pos, vel, {1, 1, 1, 1}, -1);
    };
    ctx.level->create_button = [this](phys::Vec3 p, std::vector<phys::Entity*>* list, const void* owner) {
        return create_button(p, list, owner);
    };
    ctx.level->beat = [this](const script::ScriptRoom& r, float x) {  // Room::getBeat: two beats a second
        float t = io_->music_sync ? io_->music_location : static_cast<const Room&>(r).room_time(sim_time);
        t = t + x * time_scale;
        float b = (t * 0.03125f) * 64.0f;
        if (&r != current.get()) b = b - 64.0f;
        return b;
    };
}

Level::~Level() {
    current.reset();
    next.reset();
    while (!powerups.empty()) destroy_entity(*powerups.front());
    while (!buttons.empty()) destroy_entity(*buttons.front());
    ctx_->destroy_other = nullptr;
    ctx_->create_powerup = nullptr;
    ctx_->level->get_property = nullptr;
    ctx_->level->set_property = nullptr;
    ctx_->level->beat = nullptr;
    ctx_->level->create_button = nullptr;
    ctx_->level->add_laser = nullptr;
    ctx_->level->smoke = nullptr;
}

std::string Level::music_path(const Room& room) const { return "music/" + room.music + ".ogg"; }

int Level::level_distance(int level) const {
    int sum = 0;
    if (!rooms.empty() && rooms[0].level < level) {
        sum = rooms[0].length;
        for (size_t i = 1; i < rooms.size() && rooms[i].level < level; ++i) sum += rooms[i].length;
    }
    return sum;
}

const PropertySchema& Level::schema() {
    static const PropertySchema s{"Level",
                                  {{"simtime", "0.0", 3},     {"state", "", 3},        {"score", "0", 3},
                                   {"ballcount", "0", 3},     {"distance", "0", 3},    {"progress", "0", 3},
                                   {"editZoom", "1", 1},      {"editPan", "0 0", 1},   {"editWorkMode", "0", 1},
                                   {"powerup", "", 3},        {"xp", "0", 3},          {"multiplier", "0", 3},
                                   {"streak", "0", 3},        {"pain", "0", 3},        {"credits", "0", 3},
                                   {"egg", "0", 3},           {"bossattack", "0", 3}}};
    return s;
}

std::string Level::get_property(const std::string& key) const {
    const size_t dot = key.find('.');
    if (dot == std::string::npos) return "";
    const std::string bag = key.substr(0, dot), name = key.substr(dot + 1);
    if (bag == "level") return properties.get(name);
    if (bag == "player") return name == "mode" ? std::to_string(scoring_->player_mode) : "";  // (the rest of the bag: not ported)
    if (bag == "game") {
        const auto it = io_->properties.find(name);
        return it == io_->properties.end() ? "" : it->second;
    }
    return "";  // display, audio, stats: not ported
}

void Level::set_property(const std::string& key, const std::string& value) {
    const size_t dot = key.find('.');
    if (dot == std::string::npos) return;
    const std::string bag = key.substr(0, dot), name = key.substr(dot + 1);
    if (bag == "level") properties.set(name, value);
    else if (bag == "game") io_->properties[name] = value;
}

std::unique_ptr<Room> Level::make_room(const RoomDef& def, float start) {
    // Room::Room reads the level time and camera, the level spec (Game +0x168) and the player mode.
    ctx_->level_time = sim_time;
    ctx_->level->time = sim_time;
    ctx_->level->camera_position = camera;
    ctx_->level->camera_speed = speed, ctx_->level->time_step = io_->dt;
    ctx_->level->current_room = current.get();  // (null while the first room is being made)
    ctx_->level->camera_shake = display ? &display->shake : nullptr;
    ctx_->level_spec = io_->spec;
    ctx_->player_mode = scoring_->player_mode;
    return std::make_unique<Room>(*ctx_, def.type, def.params, start);
}

void Level::load_next_room() {
    const int index = std::min(room_index + 1, static_cast<int>(rooms.size()) - 1);
    next = make_room(rooms[static_cast<size_t>(index)], current->length + current->start);
    log("setNextLevelMusic " + music_path(*next));
    next->def_index = index;
}

void Level::enter_room(Room& room) {
    endless = rooms[static_cast<size_t>(room_index)].type.find("endless") != std::string::npos;
    if (endless) {
        scoring_->log.push_back("achievement 2");
        if (scoring_->balls > 199) scoring_->log.push_back("achievement 12");
    }
    // Audio::setReverb: its two QiMonoReverbs (left, right) each draw QiRnd(0.8, 1.2) four times for their comb
    // lengths when the first reverb argument is above 0; those draws come from the shared lrand48 here, mid-frame.
    // The values go with the line for the sound side.
    std::string combs;
    if (room.reverb[0] > 0.0f)
        for (int k = 0; k < 8; ++k) combs += " " + fmt(ctx_->rng->rnd(0.8f, 1.2f));
    log("setReverb " + fmt(room.low_pass) + " " + fmt(room.echo[0]) + " " + fmt(room.echo[1]) + " " + fmt(room.echo[2]) +
        " " + fmt(room.echo[3]) + " " + fmt(room.reverb[0]) + " " + fmt(room.reverb[1]) + " " + fmt(room.reverb[2]) + combs);
    const size_t i = static_cast<size_t>(room.def_index);
    if (i + 1 < rooms.size() && rooms[i].level != rooms[i + 1].level) room.level_boundary = true;
    if (!campaign || scripted_move) log("clearQuickSave");
    else quicksave_in = 3;
    room_time = 0;
    if (campaign && !scripted_move) io_->spec = std::to_string(rooms[static_cast<size_t>(room_index)].level);
    f2a0 = 0;
}

void Level::reset() {
    // Level::clear
    current.reset();
    next.reset();
    while (!ctx_->world->entities.empty()) destroy_entity(*ctx_->world->entities.front());
    properties.reset();
    if (debris) debris->clear();
    events_->clear_particles();
    properties.set("egg", "0");
    current = make_room(rooms[static_cast<size_t>(room_index)], 0.0f);
    current->def_index = room_index;
    camera = {0, 1, 0};
    music_started = false;
    distance_base = 0;
    for (int k = 0; k < room_index; ++k) distance_base += rooms[static_cast<size_t>(k)].length;
    distance = room_index > 0 ? static_cast<float>(distance_base) : 0.0f;
    f150 = 0, invulnerable = 0, sim_time = 0, f14c = 0, time_scale = 1;
    for (float& t : timers) t = 0;
    game_over = false;
    f188 = 0, burst = 0, fade = 0, f238 = -1, f23c = 0;
    scoring_->streak_level = 0;
    end_t = 0, end_transition = false, room_pace = 0, pain = 0, quicksave_in = 0, dist_ach = 0, endless = false;
    streak_dist_ach = 0, stat_streak = 0, stat_balls = 0, stat_frames = 0;
    scoring_->first_scores = 0;
    enter_room(*current);
    io_->audio_e8 = 1;
    f29c = 0, speed = 0, f2a4 = 0;
    scoring_->saved_streak = 0;
}

void Level::destroy_entity(phys::Entity& e) {
    phys::PhysicsWorld& world = *ctx_->world;
    if (e.kind == phys::Entity::Kind::kBody) {
        world.destroy_body(static_cast<phys::RigidBody&>(e), events_);
        return;
    }
    if (e.kind == phys::Entity::Kind::kPowerup) {
        auto& p = static_cast<Powerup&>(e);
        scoring_->log.push_back(std::string("registerPowerup ") + (p.picked ? "1" : "0"));
        if (PlayerStats* s = scoring_->stats()) s->register_powerup(p.picked);
        world.remove_entity(p);
        for (size_t i = 0; i < powerups.size(); ++i) {
            if (powerups[i].get() != &p) continue;
            std::swap(powerups[i], powerups.back());  // the last entry takes the slot
            powerups.pop_back();
            break;
        }
    }
    if (e.kind == phys::Entity::Kind::kButton) {
        world.remove_entity(e);
        for (size_t i = 0; i < buttons.size(); ++i) {
            if (buttons[i].get() != &e) continue;
            std::swap(buttons[i], buttons.back());  // the last entry takes the slot
            buttons.pop_back();
            break;
        }
    }
}

void Level::create_powerup(const std::string& type, phys::Vec3 pos, Obstacle& owner) {
    powerups.push_back(std::make_unique<Powerup>());
    Powerup& p = *powerups.back();
    ctx_->world->add_entity(p);
    const int t = Powerup::type_id(type);
    if (t >= 0) p.set_type(t);
    p.position = pos, p.rotation = {0, 0, 0, 1};
    p.near_despawn = &owner;  // Entity::setObstacle
    p.obstacle_list = &owner.entities;
    owner.entities.push_back(&p);
}

void Level::activate_powerup(int type) {
    for (float& t : timers) t = 0.0f;
    timers[type] = 7.0f;
}

void Level::pick_up(Powerup& p) {
    if (p.picked) return;
    log("playSound pickup " + fmt(0.7f) + " " + fmt(1.0f));
    events_->play_file("snd/item.ogg", 0.7f, 1.0f);
    for (int i = 0; i < 5; ++i) {
        const phys::Vec3 v = phys::random_unit(*ctx_->rng, 0.5f);
        events_->spawn_particle(phys::PhysicsEvents::Effect::kAttached, p.position, v, {1, 1, 1, 1}, -1);
    }
    p.picked = true;
    if (p.type != Powerup::kBarrel) activate_powerup(p.type);
    else events_->explosion(nullptr, p.position, 20.0f);
}

void Level::update_powerup(Powerup& p) {
    const phys::Vec3 c = p.position;
    p.bounds_min = {c.x - 0.5f, c.y - 0.5f, c.z - 0.5f};
    p.bounds_max = {c.x + 0.5f, c.y + 0.5f, c.z + 0.5f};
    if (p.picked || !(camera.z - c.z < 30.0f)) return;
    // Settle to half a unit over the floor (room geometry, mask 0x80), at most 0.05 a frame.
    phys::PhysicsWorld& world = *ctx_->world;
    phys::Vec3 hit{0, 0, 0};
    const bool found = world.raycast({c.x + 0.0f, c.y + 0.5f, c.z + 0.0f}, {c.x + 0.0f, c.y - 1.0f, c.z + 0.0f}, 0x80, &hit,
                                     nullptr, nullptr, nullptr);
    float step = -0.05f;
    if (found) {
        const float d = (hit.y + 0.5f) - p.position.y;
        if (d > -0.05f) step = d < 0.05f ? d : 0.05f;
    }
    p.position.y = step + p.position.y;
    // A ball within half a unit picks it up (and counts as having broken something).
    for (size_t i = 0; i < world.bodies.size(); ++i) {
        phys::RigidBody* b = world.bodies[i];
        if (!b->round) continue;
        const float dy = b->position.y - p.position.y, dx = b->position.x - p.position.x,
                    dz = b->position.z - p.position.z;
        if (std::sqrt((dy * dy + dx * dx) + dz * dz) < 0.5f) {
            b->broke_something = true;
            pick_up(p);
        }
    }
}

phys::ButtonEntity* Level::create_button(phys::Vec3 pos, std::vector<phys::Entity*>* owner_list, const void* owner) {
    buttons.push_back(std::make_unique<phys::ButtonEntity>());
    phys::ButtonEntity& b = *buttons.back();
    ctx_->world->add_entity(b);
    b.near_despawn = owner;  // Entity::setObstacle
    if (owner && owner_list) b.obstacle_list = owner_list, owner_list->push_back(&b);
    b.position = pos, b.rotation = {0, 0, 0, 1};
    return &b;
}

void Level::update_button(phys::ButtonEntity& b) {
    const phys::Vec3 c = b.position;
    b.bounds_min = {c.x - 0.5f, c.y - 0.5f, c.z};
    b.bounds_max = {c.x + 0.5f, c.y + 0.5f, c.z + 0.0f};
    b.was_down = b.down;
    if (b.down) return;
    const phys::PhysicsWorld& world = *ctx_->world;
    for (phys::RigidBody* ball : world.bodies) {  // the first ball within half a unit presses it
        if (!ball->round) continue;
        const float dy = ball->position.y - b.position.y, dx = ball->position.x - b.position.x,
                    dz = ball->position.z - b.position.z;
        if (std::sqrt((dy * dy + dx * dx) + dz * dz) < 0.5f) {
            ball->broke_something = true;
            log("playSound button " + fmt(1.0f) + " " + fmt(1.0f));
            events_->play_file("snd/clickdown0.ogg", 1.0f, 1.0f);
            b.down = true;
            return;
        }
    }
}

namespace {

// QiDistanceSq(point, segment): the squared distance from p to the segment o .. o + d.
float distance_sq_segment(phys::Vec3 p, phys::Vec3 o, phys::Vec3 d) {
    const float len = std::sqrt((d.y * d.y + d.x * d.x) + d.z * d.z);
    auto to_origin = [&] {
        const float y = p.y - o.y, x = p.x - o.x, z = p.z - o.z;
        return (y * y + x * x) + z * z;
    };
    if (len == 0.0f) return to_origin();
    const float k = len + FLT_EPSILON;
    const phys::Vec3 u{d.x / k, d.y / k, d.z / k};
    float ay = p.y - o.y, ax = p.x - o.x, az = p.z - o.z;
    const float proj = (ay * u.y + ax * u.x) + az * u.z;
    if (!(proj > 0.0f)) return to_origin();
    if (proj < len) {
        ay = ay - u.y * proj, ax = ax - u.x * proj, az = az - u.z * proj;
        return (ay * ay + ax * ax) + az * az;
    }
    const float y = (d.y + o.y) - p.y, x = (d.x + o.x) - p.x, z = (d.z + o.z) - p.z;
    return (y * y + x * x) + z * z;
}

}  // namespace

void Level::add_laser(phys::Vec3 from, phys::Vec3 dir) {
    const float l = 100.0f;
    lasers.push_back({from, {from.x + dir.x * l, from.y + dir.y * l, from.z + dir.z * l}});
    events_->play_loop(phys::PhysicsEvents::Loop::kLaser, from, ctx_->rng->rnd(0.1f, 0.2f));  // the hum
}

void Level::process_lasers() {
    using PE = phys::PhysicsEvents;
    phys::PhysicsWorld& world = *ctx_->world;
    Rand48& rng = *ctx_->rng;
    const PE::Rgba white{1, 1, 1, 1};
    for (size_t i = 0; i < lasers.size(); ++i) {
        phys::Vec3 point{0, 0, 0}, normal{0, 0, 0};
        const phys::BodyShape* shape = nullptr;
        if (world.raycast(lasers[i].from, lasers[i].to, 0xff, &point, &normal, &shape, nullptr)) {
            phys::RigidBody* body = shape->body;
            if (body->round) {  // a ball in the beam is destroyed in a burst
                events_->play_sound(events_->pick_sound(PE::Bank::kLaserBall), point, 1.0f, 1.0f);
                for (int k = 0; k < 25; ++k) {
                    const phys::Vec3 r = phys::random_unit(rng, 2.0f);
                    events_->spawn_particle(PE::Effect::kBallBurst, point, {r.x + 0.0f, r.y + 1.0f, r.z + 0.0f}, white, -1);
                }
                for (int k = 0; k < 5; ++k)
                    events_->spawn_particle(PE::Effect::kAttached, point, phys::random_unit(rng, 1.0f), white, -1);
                for (int k = 0; k < 20; ++k) {
                    const phys::Vec3 r = phys::random_unit(rng, 2.0f);
                    events_->spawn_debris(body->near_despawn, point, {r.x + 0.0f, r.y + 1.0f, r.z + 0.0f}, white);
                }
                destroy_entity(*body);
            } else if (rng.rnd(0.0f, 1.0f) < 0.1f) {  // now and then a spark off the surface
                const phys::Vec3 n2{normal.x + normal.x, normal.y + normal.y, normal.z + normal.z};
                const phys::Vec3 r = phys::random_unit(rng, 1.0f);
                events_->spawn_particle(PE::Effect::kLaserSpark, point, {n2.x + r.x, n2.y + r.y, n2.z + r.z}, white, -1);
            }
            lasers[i].to = point;
        }
        // A beam reaching past the camera that passes within 0.1 of it is a crash.
        const Laser& l = lasers[i];
        const float dy = l.from.y - l.to.y, dx = l.from.x - l.to.x, dz = l.from.z - l.to.z;
        const float beam = std::sqrt((dy * dy + dx * dx) + dz * dz);
        const float cy = l.from.y - camera.y, cx = l.from.x - camera.x, cz = l.from.z - camera.z;
        const float to_camera = std::sqrt((cy * cy + cx * cx) + cz * cz);
        if (to_camera + 0.1f < beam) {
            const phys::Vec3 d{l.to.x - l.from.x, l.to.y - l.from.y, l.to.z - l.from.z};
            if (distance_sq_segment(camera, l.from, d) < 0x1.47ae16p-7f && invulnerable <= 0.0f) hit_something();  // 0.01 + 1 ulp
        }
    }
}

void Level::center_camera() {
    const float d = camera.z;
    camera.z = d - d;
    for (phys::Entity* e : ctx_->world->entities) {
        if (e->kind == phys::Entity::Kind::kBody) {
            auto* b = static_cast<phys::RigidBody*>(e);
            b->set_transform({b->position.x, b->position.y, b->position.z - d}, b->rotation);
            if (b->anim) b->anim->position.z = b->anim->position.z - d;
        } else {
            e->position = {e->position.x, e->position.y, e->position.z - d};  // Entity::setTransform
        }
    }
    if (debris) debris->translate(-d);  // (RenderLevel translates too: not ported.)
    events_->translate_particles(-d);
    current->start = current->start - (-d);
    auto shift = [&](Room& r) {
        r.room_obstacle->translate(-d);
        for (auto& o : r.obstacles) o->translate(-d);
    };
    shift(*current);
    if (next) {
        next->start = next->start - (-d);
        shift(*next);
    }
}

void Level::loop_camera(float dz) {
    f2a0 = 5.0f;
    camera.z = camera.z + dz;
    for (phys::Entity* e : ctx_->world->entities) {
        if (e->kind != phys::Entity::Kind::kBody) continue;
        auto* b = static_cast<phys::RigidBody*>(e);
        if (!b->dynamic) continue;
        b->set_transform({b->position.x, b->position.y, b->position.z + dz}, b->rotation);
        if (b->anim) b->anim->position.z = b->anim->position.z + dz;
    }
    if (debris) debris->translate(dz);  // (RenderLevel translates too: not ported.)
    events_->translate_particles(dz);
}

phys::RigidBody& Level::create_ball() {
    phys::RigidBody& b = ctx_->world->create_body();  // Level::create
    b.round = true;
    const phys::Vec3 o = phys::qi_rotate(display->rotation, {0.0f, -0.1f, -0.5f});  // Display +0x840
    b.set_transform({camera.x + o.x, camera.y + o.y, camera.z + o.z}, {0, 0, 0, 1});
    const float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    phys::BodyShape& s = b.add_box(identity, {0.1f, 0.1f, 0.1f}, nullptr, 0.0f);
    s.material.restitution = 0.4f;
    s.material.group = 0x40;
    s.material.density = 5.0f;
    b.compute_mass_properties();
    return b;
}

void Level::shoot(phys::Vec3 direction, float ball_speed, bool armed) {
    phys::RigidBody& b = create_ball();
    if (credits && distance > 100.0f) {  // credits balls: slower and heavier
        ball_speed = ball_speed * 0.4f;
        b.shapes[0]->material.friction = 0.0f;
        b.shapes[0]->material.density = 10.0f;
    }
    const phys::Vec3 g = phys::level_gravity(&current->gravity, camera_roll);  // Level::getGravity
    const float c = 0.01f;
    phys::Vec3 v{ball_speed * (direction.x - g.x * c), ball_speed * (direction.y - g.y * c),
                 ball_speed * (direction.z - g.z * c)};
    if (speed > 0.0f) {  // minus the camera's own motion along +z
        const phys::Vec3 axis{0.0f, 0.0f, 1.0f};
        v = {v.x - axis.x * speed, v.y - axis.y * speed, v.z - axis.z * speed};
    }
    b.set_velocity(v);
    if (armed && timers[6] > 0.0f) b.type = 1;  // explosive balls while +0x170 runs
}

void Level::hit_something() {
    Scoring& sc = *scoring_;
    if (sc.player_mode == 3) return;
    const int before = sc.balls;
    dist_ach = 0;
    if (display->shake <= 1.0f) display->shake = 1.0f;
    sc.balls = before - 10 < 0 ? 0 : before - 10;
    invulnerable = 1.0f;
    sc.streak_abort(true);
    int lost = before - sc.balls;
    events_->play_sound(events_->pick_sound(phys::PhysicsEvents::Bank::kCrash), camera, 1.0f, 1.0f);
    if (lost >= 10) lost = 10;
    // The lost balls fly out through random points of the lower middle of the screen.
    Rand48& rng = *ctx_->rng;
    for (int i = 0; i < lost; ++i) {
        const int w = display->width, h = display->height;
        const int x = rng.rnd(-(w / 3), w / 3) + w / 2;
        const int y = rng.rnd(-(h / 4), h / 4) + (2 * h) / 3;
        const phys::Vec3 dir = display->pixel_to_world_dir(static_cast<float>(x), static_cast<float>(y));
        const float base = speed;
        shoot(dir, base + rng.rnd(4.0f, 6.0f), false);
    }
    pain = 1.0f;
}

void Level::handle_input() {
    GameIO& io = *io_;
    Scoring& sc = *scoring_;
    auto key = [&](int k) { return std::find(io.keys_pressed.begin(), io.keys_pressed.end(), k) != io.keys_pressed.end(); };
    if (key(' ')) log("game player.activatepowerup");
    if (key('n')) {
        if (room_index < static_cast<int>(rooms.size()) - 1) room_index += 1;
        reset();
    }
    if (key('b')) {
        if (room_index > 0) room_index -= 1;
        reset();
    }
    if (key('d')) sc.streak += 10;
    if (key('1')) sc.balls += 10;
    if (key('2')) log("pushPowerup 5");
    if (key('3')) log("pushPowerup 6");
    if (key('4')) log("pushPowerup 3");
    if (scripted_move || end_transition) return;
    f23c = 0, f238 = -1;
    const float rapid = timers[5];  // +0x16c: rapid fire
    const int count = io.button_down ? 1 : io.touch_count;
    if (rapid > 0.0f) {
        if (io.button_released || io.touch_released) {
            if (burst > 2) io.burst_flag = 0;
            burst = 0;
        }
        if (!io.button_down && !io.has_touch) return;
        if (!(f188 <= 0.0f)) return;  // the cooldown runs (or is NaN)
        if (burst % count != 0) return;
        f188 = 0.1f;
        burst += 1;
    } else {
        if (!io.button_pressed && !io.touch_pressed) return;
        if (sc.balls <= 0) return;
        if (sc.player_mode != 3) sc.balls -= 1;
    }
    const bool single = timers[6] > 0.0f || timers[5] > 0.0f;
    const phys::Quat aim = io.aim_override ? *io.aim_override : phys::qi_mul(camera_roll, io.camera_rotation);  // roll (+0xf8) * head
    const int streak = sc.streak;
    auto fire = [&](float x, float y) { shoot(phys::qi_rotate(aim, {x, y, -1.0f}), 20.0f, true); };
    // The original's 0.02 and 0.04 are one ulp above the nearest floats (0x3ca3d70b, 0x3d23d70b).
    const float k02 = 0x1.47ae16p-6f, k04 = 0x1.47ae16p-5f;
    if (streak <= 9 || single) {
        fire(0.0f, 0.0f);
    } else if (streak <= 19) {
        fire(-k02, 0.0f), fire(k02, 0.0f);
    } else if (streak <= 29) {
        fire(-0.025f, 0.025f), fire(0.025f, 0.025f), fire(0.0f, -0.025f);
    } else if (streak <= 39) {
        fire(-0.035f, 0.0f), fire(0.035f, 0.0f), fire(0.0f, 0.035f), fire(0.0f, -0.035f);
    } else {
        fire(0.0f, 0.0f), fire(-k04, -0.05f), fire(k04, -0.05f), fire(-k04, 0.05f), fire(k04, 0.05f);
    }
    const int shot = events_->pick_sound(phys::PhysicsEvents::Bank::kShot);  // SoundBank::next
    log("playSound shot " + fmt(0.4f) + " " + fmt(1.0f));
    events_->play_sound_flat(shot, 0.4f, 1.0f);
}

void Level::update() {
    GameIO& io = *io_;
    Scoring& sc = *scoring_;
    script::ScriptLevel& sl = *ctx_->level;

    // The level music, once the menu has faded.
    if (!music_started && io.menu_fade < 0.1f) {
        music_started = true;
        log("playLevelMusic " + music_path(*current) + " 0");
    }

    // Camera roll: eased progress through the room, mgSetRotation's frequency and amplitude, about Z.
    {
        float p = current->progress(camera.z);
        if (p < 0.1f) {
            const float a = p * 10.0f;
            p = p * a + (p * p * 10.0f) * (1.0f - a);
        }
        if (p > 0.9f) {
            float q = 1.0f - p;
            q = q * q;
            q = q * 100.0f;
            const float w = 1.0f - q * 0.1f;
            const float t = (p - 0.9f) * 10.0f;
            p = w * t + p * (1.0f - t);
        }
        const float angle = p * 6.28318548f;
        const float amp = current->rotation[1];
        const float a = angle * current->rotation[0];
        camera_roll = amp == 0.0f ? phys::axis_angle({0, 0, 1}, a) : phys::axis_angle({0, 0, 1}, amp * std::sin(a));
    }
    stat_streak += sc.streak, stat_balls += sc.balls, stat_frames += 1;

    // Hints in the first room.
    if (!io.hints_shown[0] && current->type == "basic/basic" && distance > 75.0f && io.high_score == 0) {
        sl.ui_commands.push_back("hint1");
        io.hints_shown[0] = true;
    }
    if (!io.hints_shown[1] && current->type == "basic/basic" && distance > 150.0f && io.high_score == 0) {
        sl.ui_commands.push_back("hint2");
        io.hints_shown[1] = true;
    }
    distance_score = static_cast<int>(distance);
    if (quicksave_in > 0 && --quicksave_in == 0) log("quickSave");

    // Pain flash and the no-balls fade.
    {
        const float v = pain - io.dt;
        pain = !(v > 0.0f) ? 0.0f : v;
    }
    if (sc.balls != 0 || !(timers[5] <= 0.0f)) {
        const float v = fade - 0.005f;
        fade = !(v > 0.0f) ? 0.0f : v;
    } else {
        if (fade <= 0.0f) {
            log("playSound noballs " + fmt(0.7f) + " " + fmt(1.0f));
            events_->play_file("snd/failed.ogg", 0.7f, 1.0f);
        }
        const float v = fade + 0.005f;
        fade = !(v < 1.0f) ? 1.0f : v;
    }

    // Time scale: slow motion (+0x164), recovery, fast forward on key 0x66.
    const float base = io.time_scale_flag ? io.time_scale * 0.01f : 0.01f;
    if (timers[3] > 0.0f) {
        const float v = time_scale - base;
        time_scale = v > 0.3333f ? v : 0.3333f;
    } else if (time_scale < 1.0f) {
        const float v = base + time_scale;
        time_scale = v < 1.0f ? v : 1.0f;
    }
    if (io.key_fast) {
        const float v = time_scale + 0.1f;
        time_scale = !(v < 3.0f) ? 3.0f : v;
    } else if (time_scale > 1.0f) {
        const float v = time_scale - 0.1f;
        time_scale = !(v > 1.0f) ? 1.0f : v;
    }
    const float k = (1.0f - io.pause_blend) * time_scale;
    const float dt = k * io.dt;
    io.dt = dt;
    ctx_->world->time_step = dt;
    {
        const float slow = dt - io.time_scale * 0.01667f;
        for (int i = 0; i < 12; ++i)
            if (i != 3) timers[i] = timers[i] - dt;
        timers[3] = (timers[3] - dt) + slow;
    }
    const bool boss = current->type.find("boss") != std::string::npos;
    {
        float v = f14c + dt * 0.01f;
        f188 = f188 - dt;
        f2a0 = f2a0 + io.extra_time;
        if (!(v > 0.0f)) v = 0.0f;
        else if (!(v < 1.0f)) v = 1.0f;
        f14c = v;
    }

    // Camera speed.
    const float scale = (1.0f - fade) * k;
    float step = 0;
    auto boss_speed = [&] {
        float delta = 5.0f - speed;
        delta = delta > -0.05f ? (delta < 0.05f ? delta : 0.05f) : -0.05f;
        speed = speed + delta;
        step = -(speed * dt);
    };
    if (!game_over) {
        if (io.music_sync && !io.editor_flag) room_time = room_time * 0.9f + io.music_location * 0.1f;
        else room_time = room_time + dt * scale;
        float pace_base;
        if (boss) {
            room_time = f2a0;
            room_pace = 160.0f;
            pace_base = 159.2f;
        } else {
            pace_base = room_pace * 0.995f;
        }
        const float length = current->length;
        const float pace = io.editor_flag ? length : pace_base + length * 0.005f;
        room_pace = pace;
        float target = (room_time * 0.03125f) * pace;
        if (current->type == "credits/2" && !(target < 1.5f)) target = 1.5f;
        else if (!(target > 0.0f)) target = 0.0f;
        if (boss) {
            boss_speed();
        } else {
            const phys::RigidBody& rb = *current->body;
            const phys::Vec3 d{camera.x - rb.position.x, camera.y - rb.position.y, camera.z - rb.position.z};
            const float local_z = phys::qi_rotate_inverse(rb.rotation, d).z;
            const float err = (1.0f - io.pause_blend) * (-local_z - target);
            if (f2a4 > 0.0f) {
                const float t = f2a4 - io.dt * 0.2f;
                float a = 0, w = 1, b0 = 0;
                if (t > 0.0f) a = t, w = 1.0f - t, b0 = t * 5.0f;
                f2a4 = a;
                speed = b0 + (-err / dt) * w;
                step = -(speed * dt);
            } else {
                speed = -err / dt;
                step = err;
            }
        }
    } else if (boss) {
        boss_speed();
    }
    // The level drives the music pitch while it plays.
    if (scale > 0.01f && io.state == 3) {
        GameIO::Channel& a = io.channels[0];
        GameIO::Channel& b = io.channels[1];
        GameIO::Channel& c = io.channels[2];
        if (!a.on) a.on = true;
        if (!b.on) b.on = true;
        if (!c.on) a.on = true;  // (the original writes A's flag here)
        const float p = scale < 10.0f ? scale : 10.0f;
        a.a24 = p, a.a20 = p, b.a24 = p, b.a20 = p, c.a24 = p;
        if (scale != 1.0f) c.a20 = p;
    } else {
        for (GameIO::Channel& ch : io.channels)
            if (ch.on) ch.on = false;
    }

    // Pause toggle, then the camera moves.
    if (io.key_pause_pressed) io.paused = !io.paused;
    if (io.paused) step = 0.0f;
    if (end_transition) {
        const float t = end_t + 0.007f;
        const float pi = 3.14159274f;
        float e = (1.0f - std::cos(t * pi)) * 0.5f;
        e = e + (1.0f - e) * 0.0f;
        e = (1.0f - std::cos(e * pi)) * 0.5f;
        e = e + (1.0f - e) * 0.0f;
        e = (1.0f - std::cos(e * pi)) * 0.5f;
        e = e + (1.0f - e) * 0.0f;
        end_t = t;
        camera.z = end_from + (170.0f - end_from) * e;
        if (!(t < 1.0f)) {
            end_transition = false;
            io.spec = "0";
            log("loadCheckpoint 0");
            log("enterCheckpoint");
            room_index = 0;
            reset();
        }
    } else if (scripted_move) {
        camera.z = camera.z - scripted_speed * 0.1f;
        const float v = scripted_speed - 0.004f;
        scripted_speed = v < 0.0f ? 0.0f : v;
    } else {
        camera.z = camera.z + step;
    }

    // Distance through the level.
    const float old_distance = distance;
    const float new_distance = static_cast<float>(distance_base) +
                               static_cast<float>(rooms[static_cast<size_t>(current->def_index)].length) *
                                   current->progress(camera.z);
    distance = end_transition ? 0.0f : new_distance;
    if (invulnerable > 0.0f) invulnerable = invulnerable - io.dt;

    // Room hand-off.
    if (!end_transition) {
        if (current->z_to_room(camera.z) > current->length - 30.0f) {
            if (!next) load_next_room();
        } else if (io.music_location > 28.0f) {
            if (!next) load_next_room();
        }
        if (current->z_to_room(camera.z) > current->length && next) {
            distance_base += rooms[static_cast<size_t>(current->def_index)].length;
            if (campaign && current->level_boundary) {
                const int lvl = rooms[static_cast<size_t>(room_index)].level + 1;
                if (lvl > 0) {
                    log("evt.checkpoint" + std::to_string(lvl) + " = " + std::to_string(sc.balls));
                    log("reportCheckpoint " + std::to_string(lvl));
                    log("save 1");
                }
                sl.ui_commands.push_back("checkpoint " + std::to_string(lvl));
            }
            if (stat_frames > 0) {
                const float frames_f = static_cast<float>(stat_frames);
                log("registerRoom " + fmt(static_cast<float>(stat_streak) / frames_f) + " " +
                    fmt(static_cast<float>(stat_balls) / frames_f) + " " + fmt(current->length));
                if (PlayerStats* s = sc.stats())
                    s->register_room(static_cast<float>(stat_streak) / frames_f, static_cast<float>(stat_balls) / frames_f,
                                     current->length);
            }
            stat_streak = 0, stat_balls = 0, stat_frames = 0;
            f2a4 = boss ? 1.0f : 0.0f;
            current.reset();
            current = std::move(next);
            room_index = std::min(room_index + 1, static_cast<int>(rooms.size()) - 1);
            enter_room(*current);
            if (!io.music_swap) log("playLevelMusic " + music_path(*current) + " 0");
            else log("swapLevelMusic");
        }
    }

    // This frame's lasers, the rooms (their scripts add lasers), the lasers' effects, the floating origin.
    lasers.clear();
    sl.time = sim_time;
    sl.camera_position = camera;
    sl.camera_speed = speed, sl.time_step = io.dt, sl.current_room = current.get();
    sl.camera_shake = display ? &display->shake : nullptr;
    if (!end_transition) {
        current->update(camera.z, io.dt, io.frame);
        current->ambient(*events_, *ctx_->rng, camera, io.frame, io.time_scale_flag);  // (the end of its update)
        if (next) next->update(camera.z, io.dt, io.frame);
    }
    process_lasers();
    if (camera.z < -50.0f) center_camera();

    // Entities: their update, and free ones far ahead of the camera go.
    phys::PhysicsWorld& world = *ctx_->world;
    for (size_t i = 0; i < world.entities.size(); ++i) {
        phys::Entity* e = world.entities[i];
        if (e->kind == phys::Entity::Kind::kBody) static_cast<phys::RigidBody*>(e)->tick_age();  // Body::update
        else if (e->kind == phys::Entity::Kind::kPowerup) update_powerup(*static_cast<Powerup*>(e));
        else if (e->kind == phys::Entity::Kind::kButton) update_button(*static_cast<phys::ButtonEntity*>(e));
        if (!e->near_despawn && camera.z < e->bounds_min.z - 25.0f) {
            destroy_entity(*e);
            --i;
        }
    }
    world.camera_z = camera.z;
    world.frame = frames;  // the step reads the level's frame counter (+0xe0)
    world.camera_rotation = io.camera_rotation;  // Game +0x224
    world.gravity = phys::level_gravity(&current->gravity, camera_roll);  // Level::getGravity
    world.update(*events_, *ctx_->rng);

    // Crash test: five rays along this frame's camera motion.
    if (invulnerable <= 0.0f && timers[10] <= 0.0f && !scripted_move && !end_transition) {
        const float z1 = camera.z - speed * io.dt;
        const phys::BodyShape* shape = nullptr;
        bool all = world.raycast(camera, {camera.x + 0.0f, camera.y + 0.0f, z1}, 0xdf, nullptr, nullptr, &shape, nullptr);
        const float o = 0.01f;
        const float sx[4] = {-o, o, o, -o}, sy[4] = {-o, -o, o, o};
        for (int r = 0; r < 4; ++r) {
            const phys::Vec3 from{camera.x + sx[r], camera.y + sy[r], camera.z + 0.0f};
            const phys::Vec3 to{from.x, from.y, camera.z - speed * io.dt};
            const bool hit = world.raycast(from, to, 0xdf, nullptr, nullptr, &shape, nullptr);
            all = all && hit;
        }
        if (all && shape) {
            const auto* obstacle = static_cast<const Obstacle*>(shape->body->near_despawn);
            if (obstacle && obstacle->state.cleared == 0.0f && !shape->body->split_piece) hit_something();
        }
    }

    io.properties["lastdistance"] = fmt(distance);
    if (!game_over && fade == 1.0f) {
        log("clearQuickSave");
        log("registerScore " + std::to_string(distance_score));
        log("registerDistance " + std::to_string(static_cast<int>(distance)));
        if (PlayerStats* s = sc.stats()) {
            s->register_score(distance_score);
            s->register_distance(static_cast<int>(distance));
        }
        sl.ui_commands.push_back("gameover");
        log("game game.menu");
        game_over = true;
    }
    // The particle systems, after the physics step.
    events_->update_particles();
    properties.set("ballcount", std::to_string(sc.balls));
    properties.set("distance", fmt(distance));
    properties.set("score", std::to_string(distance_score));
    properties.set("streak", std::to_string(sc.streak));
    properties.set("pain", fmt(pain));
    properties.set("credits", credits ? "1" : "0");
    if (campaign) {
        if (endless) {
            properties.set("progress", fmt(0.0f));
        } else {
            const int lvl = rooms[static_cast<size_t>(room_index)].level;
            const int a = level_distance(lvl), b = level_distance(lvl + 1);
            properties.set("progress", fmt((distance - static_cast<float>(a)) / static_cast<float>(b - a)));
        }
    }
    sim_time = sim_time + io.dt;
    properties.set("simtime", fmt(sim_time));

    // Sparks behind armed balls every other frame.
    for (const phys::RigidBody* b : world.bodies) {
        if (b->round && (io.frame & 1) == 0 && b->type == 1) {
            const phys::Vec3 r = phys::random_unit(*ctx_->rng, 0.2f);
            events_->spawn_particle(phys::PhysicsEvents::Effect::kSpark, b->position, r, {1, 1, 1, 1}, -1);
        }
    }

    // Credits rooms.
    if (credits) {
        sc.balls = 25;
        if (io.spec.find("moregames") != std::string::npos) {
            if (distance > 89.0f) log("setState 1");
        } else if (f29c < 16) {
            if (distance > 80.0f) log("setState 1");
        } else if (current->type == "credits/2" && room_time > 31.0f) {
            log("game game.menu");
        }
    }

    // Distance achievements.
    if (!end_transition && !scripted_move) {
        const float delta = new_distance - old_distance;
        float v = delta + dist_ach;
        if (dist_ach < 2000.0f && v >= 2000.0f) sc.log.push_back("achievement 7");
        dist_ach = v;
        if (sc.streak <= 39) {
            streak_dist_ach = 0;
        } else {
            const float w = delta + streak_dist_ach;
            if (streak_dist_ach < 1000.0f && w >= 1000.0f) sc.log.push_back("achievement 8");
            streak_dist_ach = w;
        }
    }

    // Boss music and the boss loop of the camera.
    if (boss) {
        bool live_boss = false, close = false;
        for (const Room::ObstacleDef& def : current->obstacle_defs) {
            if (!def.live) continue;
            if (def.type.rfind("boss", 0) != 0 && current->test_obstacle.rfind("boss", 0) != 0) continue;
            if (def.live->state.cleared != 0.0f) continue;
            if (def.live->position.z <= camera.z - 10.0f) live_boss = true;
            else close = true, live_boss = true;
        }
        if (campaign) {
            if (live_boss) {
                if (!io.boss_loop_playing) {
                    log("playBossLoop music/boss_loop0.ogg");
                } else if (io.boss_file.find("boss_loop0") != std::string::npos) {
                    log("setNextBossLoop music/boss_loop1.ogg 0");
                } else if (io.boss_file.find("boss_loop1") != std::string::npos) {
                    log("setNextBossLoop music/boss_loop2.ogg 0");
                } else if (io.boss_file.find("boss_loop2") != std::string::npos) {
                    log("setNextBossLoop music/boss_loop3.ogg 0");
                } else if (io.boss_file.find("boss_loop3") != std::string::npos) {
                    log("setNextBossLoop music/boss_loop0.ogg 0");
                }
            } else if (io.boss_loop_playing) {
                log("setNextBossLoop music/boss_end.ogg 1");
            }
        }
        if (close) {
            loop_camera(8.0f);
            log("stopChannel");
        }
    }
    frames += 1;
}

}  // namespace sh::game
