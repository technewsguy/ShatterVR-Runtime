#include "game/obstacle.hpp"

#include "game/room.hpp"
#include "physics/physics_world.hpp"

namespace sh::game {

Obstacle::Obstacle(RoomContext& ctx, Room& owner, phys::Vec3 pos, phys::Quat rot,
                   std::vector<script::LevelScript::Param> params)
    : room(&owner), position(pos), rotation(rot), ctx_(&ctx) {
    script = std::make_unique<script::LevelScript>(*ctx.rng, std::move(params), true, 0, ctx.level,
                                                   script::ScriptObstacle{pos, rot, this, &entities, &state, &time});
    script->room = &owner;
}

Obstacle::~Obstacle() {
    while (!entities.empty()) ctx_->destroy_entity(*entities.front());
}

bool Obstacle::load(const std::string& path) {
    script->set_index(ctx_->script_counter++);
    return script->load(path, ctx_->read_asset, ctx_->player_mode != 0);
}

void Obstacle::update(float dt, int game_frame, float camera_z) {
    time = time + dt;
    script->tick(game_frame);
    done = !entities.empty();
    for (size_t i = 0; i < entities.size(); ++i) {
        const float z = entities[i]->bounds_min.z;
        if (z - 20.0f < camera_z) done = false;
        if (!state.missed && state.score != 0 && state.cleared == 0 && camera_z < z) {
            const char* side = position.x <= 1.0f ? (-1.0f <= position.x ? "center" : "left") : "right";
            ctx_->level->ui_commands.push_back(std::string("miss ") + side);
            state.missed = true;
            if (ctx_->scoring) ctx_->scoring->streak_missed(this);
        }
    }
}

}  // namespace sh::game

namespace sh::game {

void Obstacle::translate(float dz) {
    position.z = position.z + dz;
    if (script) script->obstacle().position.z = position.z;
}

}  // namespace sh::game
