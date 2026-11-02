#include "game/room.hpp"

#include "core/qi_string.hpp"
#include "physics/physics_world.hpp"
#include "physics/qi_rotate.hpp"

#include <pugixml.hpp>

#include <cmath>

namespace sh::game {

Room::Room(RoomContext& ctx, const std::string& room_type, const Params& params, float room_start) : ctx_(&ctx) {
    create_segment_fn = [this](const std::string& name, float z) { return create_segment(name, z); };
    add_obstacle_fn = [this](const std::string& t, phys::Vec3 p, phys::Quat q,
                             const std::vector<std::pair<std::string, std::string>>& params) {
        ObstacleDef def;
        def.type = t, def.position = p, def.rotation = q, def.params = params;
        obstacle_defs.push_back(std::move(def));
    };
    type = room_type;
    start = room_start;
    time_start = ctx.level_time;
    for (const auto& [name, value] : params)
        if (name == "difficulty") difficulty = to_float(value);

    body = &ctx.world->create_body();  // Level::create
    body->set_transform({0, 0, room_to_z(0)}, {0, 0, 0, 1});

    std::string path = "rooms/" + type + ".lua";
    std::vector<script::LevelScript::Param> script_params;
    if (type.rfind("obstacle:", 0) == 0) {
        test_obstacle = type.substr(9);
        path = "rooms/obstacletest.lua";
    } else if (type.rfind("segment:", 0) == 0) {
        test_segment = type.substr(8);
        path = "rooms/segmenttest.lua";
    } else {
        for (const auto& [name, value] : params) script_params.push_back({name, value});
    }
    // The room's obstacle sits at the room's start; it is the room's own script (isRoom).
    room_obstacle = std::make_unique<Obstacle>(ctx, *this, phys::Vec3{0, 0, room_to_z(0)}, phys::Quat{0, 0, 0, 1},
                                               std::move(script_params));
    script = room_obstacle->script.get();
    script->room_script = true;
    script->material.type = 1;          // static
    script->material.group = 0x80;      // the level geometry group
    script->material.sound_type = 4;
    script_loaded = room_obstacle->load(path);
    // Entity::setObstacle(room body, room obstacle): after the bodies init() made.
    body->near_despawn = room_obstacle.get();
    body->obstacle_list = &room_obstacle->entities;
    room_obstacle->entities.push_back(body);

    // Fog colours by player mode (after the script set them).
    if (ctx.player_mode == 2) {
        for (auto& f : fog) f[0] = f[0] * 1.1f, f[1] = f[1] * 0.8f, f[2] = f[2] * 0.6f;
    } else if (ctx.player_mode == 0) {
        for (auto& f : fog) f[0] = f[0] * 1.1f, f[1] = f[1] * 1.1f, f[2] = f[2] * 0.8f;
    }
}

Room::~Room() {
    // The room body goes first (Level::destroy), then the room obstacle with its entities, then the spawned
    // obstacles from the last to the first.
    if (body) ctx_->world->destroy_body(*body);
    body = nullptr;
    room_obstacle.reset();
    while (!obstacles.empty()) obstacles.pop_back();
}

Obstacle* Room::create_obstacle(const std::string& name, phys::Vec3 local_pos, phys::Quat local_rot,
                                const std::vector<std::pair<std::string, std::string>>& params) {
    std::string path = "obstacles/" + name + ".lua";
    std::vector<script::LevelScript::Param> script_params;
    if (name == "#OBSTACLE#") {
        path = "obstacles/" + test_obstacle + ".lua";  // with the debug object's (empty) parameters
    } else {
        for (const auto& [k, v] : params) script_params.push_back({k, v});
    }
    // Placed by the room body's pose.
    const phys::Vec3 world_pos = phys::qi_transform(body->position, body->rotation, local_pos);
    const phys::Quat world_rot = phys::qi_mul(body->rotation, local_rot);
    obstacles.push_back(std::make_unique<Obstacle>(*ctx_, *this, world_pos, world_rot, std::move(script_params)));
    Obstacle* o = obstacles.back().get();
    o->load(path);
    return o;
}

void Room::update(float camera_z, float dt, int game_frame) {
    const float d = z_to_room(camera_z);
    auto spawn = [&](ObstacleDef& def) {
        Obstacle* o = create_obstacle(def.type, def.position, def.rotation, def.params);
        def.live = o;
        def.spawned = true;
    };
    // Elevators first and earlier (40 units ahead), then everything 30 units ahead; each definition spawns once.
    for (size_t i = 0; i < obstacle_defs.size(); ++i) {
        ObstacleDef& def = obstacle_defs[i];
        if (def.type.find("elevator") != std::string::npos && -d < (def.position.z + 30.0f) + 10.0f && !def.live &&
            !def.spawned)
            spawn(def);
    }
    for (size_t i = 0; i < obstacle_defs.size(); ++i) {
        ObstacleDef& def = obstacle_defs[i];
        if (-d < def.position.z + 30.0f && !def.live && !def.spawned) spawn(def);
    }
    room_obstacle->update(dt, game_frame, camera_z);
    for (size_t i = 0; i < obstacles.size(); ++i) {
        Obstacle* o = obstacles[i].get();
        o->update(dt, game_frame, camera_z);
        if (!o->done) continue;
        if (o->state.score != 0 && ctx_->scoring) {
            ctx_->scoring->register_obstacle(o->state.cleared != 0);
            if (o->state.cleared == 0) ctx_->scoring->streak_abort(false);
        }
        for (ObstacleDef& def : obstacle_defs)
            if (def.live == o) def.live = nullptr;
        std::unique_ptr<Obstacle> dead = std::move(obstacles[i]);
        obstacles[i] = std::move(obstacles.back());  // the last takes its slot (and is updated next frame)
        obstacles.pop_back();
        dead.reset();
    }
}

void Room::ambient(phys::PhysicsEvents& events, Rand48& rng, phys::Vec3 cam, int frame, bool slow) {
    if (particles.empty() || !(progress(cam.z) < 0.9f)) return;
    if (slow && (frame & 7) != 0) return;
    using Effect = phys::PhysicsEvents::Effect;
    const phys::PhysicsEvents::Rgba white{1, 1, 1, 1};
    // One particle in a box around a point ahead of the camera: x, y, z drawn in that order.
    auto box = [&](float x0, float x1, float y0, float y1, float ahead, float vy) {
        const float x = rng.rnd(x0, x1);
        const float y = rng.rnd(y0, y1);
        const float z = rng.rnd(-5.0f, 5.0f);
        events.spawn_particle(Effect::kAmbient, {x + cam.x, y + cam.y, (z - ahead) + cam.z}, {0, vy, 0}, white, -1);
    };
    // The left or the right side, picked with a draw.
    auto side_x = [&] {
        const float c = rng.rnd(0.0f, 1.0f);
        return c > 0.5f ? rng.rnd(-4.0f, -2.0f) : rng.rnd(2.0f, 4.0f);
    };
    // Every mode is its own test, in this order ("sides" is tested twice and both run).
    const std::string& m = particles;
    if (m == "bubbles" && frame % 90 == 0) {
        const float bx = rng.rnd(-4.0f, 4.0f);
        const float bz = rng.rnd(-5.0f, 5.0f) - 20.0f;
        const int n = rng.rnd(20, 150);
        for (int i = 0; i < n; ++i) {
            const float ax = bx + cam.x, ay = cam.y + 0.0f, az = bz + cam.z;
            const float px = rng.rnd_normal(-1.0f, 1.0f) + ax;
            const float py = rng.rnd_normal(-6.0f, -4.0f) + ay;
            const float pz = rng.rnd_normal(-1.0f, 1.0f) + az;
            const float vx = rng.rnd_normal(-0.3f, 0.3f);
            const float vy = rng.rnd_normal(-1.0f, 3.0f);
            const float vz = rng.rnd_normal(-0.3f, 0.3f);
            events.spawn_particle(Effect::kBubbles, {px, py, pz}, {vx, vy, vz}, white, -1);
        }
    }
    if (m == "sides") {
        for (const float x0 : {-4.0f, 2.0f}) {
            const float x = rng.rnd(x0, x0 + 2.0f);
            const float y = rng.rnd(-5.0f, 5.0f);
            const float z = rng.rnd(-5.0f, 5.0f);
            events.spawn_particle(Effect::kAmbient, {x + cam.x, y + cam.y, (z - 15.0f) + cam.z}, {0, 0, 0}, white, -1);
        }
    }
    if (m == "sides") {
        const float x = side_x();
        const float y = rng.rnd(-5.0f, 5.0f);
        const float z = rng.rnd(-5.0f, 5.0f);
        events.spawn_particle(Effect::kAmbient, {x + cam.x, y + cam.y, (z - 15.0f) + cam.z}, {0, 0, 0}, white, -1);
    }
    if (m == "lowrising") box(-4.0f, 4.0f, -5.0f, -3.0f, 20.0f, 0.5f);
    if (m == "lowrising2") box(-2.0f, 2.0f, -5.0f, -3.0f, 20.0f, 0.0f);
    if (m == "lowrising3") box(-1.2f, 1.2f, -5.0f, -3.0f, 20.0f, 0.7f);
    if (m == "sidesrising") {
        const float x = side_x();
        const float y = rng.rnd(-5.0f, -1.0f);
        const float z = rng.rnd(-5.0f, 5.0f);
        events.spawn_particle(Effect::kAmbient, {x + cam.x, y + cam.y, (z - 15.0f) + cam.z}, {0, 1, 0}, white, -1);
    }
    if (m == "falling") box(-4.0f, 4.0f, -2.0f, 5.0f, 20.0f, -0.5f);
    if (m == "fallinglite" && rng.rnd(0.0f, 1.0f) > 0.5f) box(-6.0f, 6.0f, -2.0f, 5.0f, 20.0f, -0.5f);
    if (m == "dustyfalling" && frame % 90 == 0) {
        const float dx = rng.rnd(-4.0f, 4.0f);
        const float dy = rng.rnd(6.0f, 7.0f);
        const float dz = rng.rnd(-5.0f, 5.0f) - 25.0f;
        const int n = rng.rnd(20, 60);
        for (int i = 0; i < n; ++i) {
            const float ax = dx + cam.x, ay = dy + cam.y, az = dz + cam.z;
            const float px = rng.rnd_normal(-0.1f, 0.1f) + ax;
            const float py = rng.rnd_normal(-0.1f, 0.1f) + ay;
            const float pz = rng.rnd_normal(-0.1f, 0.1f) + az;
            const float vx = rng.rnd_normal(-0.3f, 0.3f);
            const float vy = rng.rnd_normal(0.0f, -3.0f);
            const float vz = rng.rnd_normal(-0.3f, 0.3f);
            events.spawn_particle(Effect::kAmbient, {px, py, pz}, {vx, vy, vz}, white, -1);
        }
    }
    if (m == "starfield") {
        for (int i = 0; i < 4; ++i) {
            const float a = rng.rnd(0.0f, 6.2831855f);
            const float r = rng.rnd(1.0f, 5.0f);
            const float x = r * std::cos(a), y = r * std::sin(a);
            const float z = rng.rnd_normal(-20.0f, -25.0f);
            events.spawn_particle(Effect::kAmbient, {x + cam.x, y + cam.y, z + cam.z}, {0, 0, 5}, white, 1);
        }
    }
}

float Room::create_segment(const std::string& name, float z) {
    std::string path = "segments/" + name + ".xml";
    if (name == "#SEGMENT#") path = "segments/" + test_segment + ".xml";
    float size_z = 0;
    const std::optional<std::string> text = ctx_->read_asset ? ctx_->read_asset(path) : std::nullopt;
    pugi::xml_document doc;
    if (text && doc.load_buffer(text->data(), text->size()) && std::string(doc.document_element().name()) == "segment") {
        const SegmentSchemas& schemas = SegmentSchemas::get();
        const pugi::xml_node root = doc.document_element();
        PropertyBag segment(schemas.segment);
        segment.load(root, ctx_->templates);
        size_z = segment.as_vec3("size").z;
        if (ctx_->level_spec.find("segment:") != std::string::npos) {  // testing one segment: its own fog
            segment.as_color("fogcolor", fog[0]);
            const std::string f = segment.get("fogcolor");
            if (word_count(f) < 4) {
                for (int i = 0; i < 4; ++i) fog[1][i] = fog[0][i];
            } else {
                for (int i = 0; i < 3; ++i) fog[1][i] = to_float(get_word(f, 3 + i));
            }
        }
        if (ctx_->level_spec.find("obstacle:") != std::string::npos) {
            fog[1][0] = 1, fog[1][1] = 1, fog[1][2] = 1.5f, fog[1][3] = 1;
            fog[0][0] = 0, fog[0][1] = 0, fog[0][2] = 0, fog[0][3] = 1;
        }
        // Level geometry material for the boxes.
        phys::Material material;
        material.sound_type = 4;
        for (float& c : material.color) c = 1;
        material.group = 0x80;
        material.mask = 0xff;
        material.type = 1;
        const int mode_bit = ctx_->player_mode == 3 ? 2 : 1 << ctx_->player_mode;
        PropertyBag box(schemas.box), obstacle(schemas.obstacle), power(schemas.power_up);
        for (const pugi::xml_node node : root.children()) {
            const std::string kind = node.name();
            if (kind == "box") {
                box.load(node, ctx_->templates);
                const phys::Vec3 pos = box.as_vec3("pos"), size = box.as_vec3("size");
                const float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, pos.x, pos.y, z + pos.z, 1};
                body->add_box(m, size, &material, 0.0f).in_tree_a = true;
                // Render data (not physics): reflective tops and the walls seen in floor reflections.
                const float x0 = pos.x - size.x, x1 = pos.x + size.x, y0 = pos.y - size.y, y1 = pos.y + size.y;
                const float z0 = z + pos.z - size.z, z1 = z + pos.z + size.z;
                if (box.get("reflection") != "0") {
                    const float top = pos.y + size.y + 0.02f;
                    for (const phys::Vec3& c : {phys::Vec3{x0, top, z0}, phys::Vec3{x1, top, z0}, phys::Vec3{x1, top, z1},
                                                phys::Vec3{x0, top, z1}})
                        reflection_mask.push_back(c);
                }
                if (-0.1f <= y0 && y0 < 2.0f && box.get("visible") != "0") {
                    float color[4];
                    box.as_color("color", color);
                    const ReflectionQuad faces[3] = {
                        {{{x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}}, {color[0], color[1], color[2], color[3]}},
                        {{{x0, y0, z0}, {x0, y0, z1}, {x0, y1, z1}, {x0, y1, z0}}, {color[0], color[1], color[2], color[3]}},
                        {{{x1, y0, z1}, {x1, y0, z0}, {x1, y1, z0}, {x1, y1, z1}}, {color[0], color[1], color[2], color[3]}}};
                    for (const ReflectionQuad& f : faces) reflection_walls.push_back(f);
                }
            } else if (kind == "obstacle") {
                obstacle.load(node, ctx_->templates);
                const phys::Vec3 range = obstacle.as_vec3("difficulty");
                const int modes = obstacle.as_int("mode");
                if (!(range.x <= difficulty && difficulty <= range.y && (mode_bit & modes) != 0)) continue;
                ObstacleDef def;
                const phys::Vec3 pos = obstacle.as_vec3("pos");
                def.rotation = obstacle.as_quat("rot");
                def.type = obstacle.get("type");
                def.position = {pos.x, pos.y, z + pos.z};
                for (int i = 0; i < 12; ++i) {
                    const std::string p = obstacle.get("param" + std::to_string(i));
                    const size_t eq = p.find('=');
                    if (eq != std::string::npos) def.params.emplace_back(p.substr(0, eq), p.substr(eq + 1));
                }
                obstacle_defs.push_back(std::move(def));
            } else if (kind == "powerup") {
                power.load(node, ctx_->templates);
                const phys::Vec3 range = power.as_vec3("difficulty");
                if (!(range.x <= difficulty && difficulty <= range.y)) continue;
                phys::Vec3 pos = power.as_vec3("pos");
                pos.z = pos.z + z;
                const std::string power_type = power.get("type");
                if (power_type == "multiball" || power_type == "balls") continue;
                // Into level space through the room body's pose (QiTransform3::toParentPoint).
                if (ctx_->create_powerup)
                    ctx_->create_powerup(power_type, phys::qi_transform(body->position, body->rotation, pos), *room_obstacle);
            } else if (kind == "decal") {
                // Render data only (skipped on "low" graphics, which the port does not have).
                PropertyBag decal(schemas.decal);
                decal.load(node, ctx_->templates);
                const phys::Vec3 range = decal.as_vec3("difficulty");
                if (!(range.x <= difficulty && difficulty <= range.y)) continue;
                Decal d;
                const phys::Vec3 pos = decal.as_vec3("pos");
                d.pos = {pos.x, pos.y, z + pos.z};
                d.cell = decal.as_int("tile");
                const phys::Vec3 size = decal.as_vec3("size");
                d.size[0] = size.x, d.size[1] = size.y;
                decal.as_color("color", d.color);
                d.blend = decal.as_float("blend");
                d.rot = decal.as_quat("rot");
                decals.push_back(d);
            }
            // water: not yet.
        }
        SegmentRecord record;
        record.z = z;
        record.mesh = path.substr(0, path.size() - 4) + ".mesh";
        segments.push_back(record);
        body->compute_mass_properties();
    }
    ++segment_count;
    return size_z;
}

}  // namespace sh::game

namespace sh::game {

void RoomContext::destroy_entity(phys::Entity& e) {
    if (e.kind == phys::Entity::Kind::kBody) world->destroy_body(static_cast<phys::RigidBody&>(e));
    else if (destroy_other) destroy_other(e);
    else world->remove_entity(e);  // no level to tell: at least leave the lists (the obstacle's loop needs it)
}

}  // namespace sh::game

namespace sh::game {

bool Room::has_boss() const {
    for (const ObstacleDef& def : obstacle_defs)
        if (def.live && def.type.rfind("boss", 0) == 0 && def.live->state.cleared == 0.0f) return true;
    return false;
}

const void* Room::closest_boss() const {
    const Obstacle* best = nullptr;
    for (const ObstacleDef& def : obstacle_defs) {
        if (!def.live || def.type.rfind("boss", 0) != 0 || def.live->state.cleared != 0.0f) continue;
        if (!best || def.live->position.z > best->position.z) best = def.live;
    }
    return best;
}

}  // namespace sh::game
