#pragma once

// Pieces of the original's RenderLevel that the PC renderer needs as data: sprite quads in
// the game's sprite layout. Drawing itself is the tool's business.

#include "core/rand48.hpp"
#include "game/level.hpp"
#include "game/particles.hpp"
#include "physics/debris.hpp"
#include "physics/physics_world.hpp"

#include <cstdint>
#include <vector>

namespace sh::game {

// RenderLevel::addSprite's quad: four corners pos - w*ax - h*ay, + w*ax - h*ay, + w*ax + h*ay, - w*ax + h*ay
// with uv[0..3], skipped within 0.5 of the camera and faded by min((d - 0.5) * 0.8, 1) beyond (Level +0xec).
void append_quad(phys::Vec3 camera, phys::Vec3 pos, phys::Vec3 ax, phys::Vec3 ay, float w, float h,
                 const float uv[4][2], const float color[4], float blend, std::vector<SpriteVertex>& out);

// The texture coordinates of cell `cell` in the sprite sheet's 8 x 8 grid (addSprite with a cell index; u inset by
// 1/512, unverified for v).
void sprite_cell(int cell, float uv[4][2]);

// A body vertex as the body shaders take it (aPosition, aNormal, aTexCoord, aColor as normalised bytes).
struct BodyVertex {
    float pos[3];
    float normal[3];
    float uv[2];
    std::uint8_t rgba[4];
};
struct BodyMesh {
    std::vector<BodyVertex> vertices;
    std::vector<std::uint32_t> indices;  // triangles
    // QiIndexBuffer's vertex limit (RenderLevel::RenderLevel): triangles that use a vertex at or past it are dropped,
    // the vertices stay.
    std::uint32_t index_limit = 0x2000;
    void triangle(std::uint32_t a, std::uint32_t b, std::uint32_t c) {
        if (a < index_limit && b < index_limit && c < index_limit) indices.insert(indices.end(), {a, b, c});
    }
    void clear() { vertices.clear(), indices.clear(); }
};
// RenderLevel::fillBuffers' outputs, by the shader that draws them.
struct BodyBuffers {
    BodyMesh basic{{}, {}, 0x400};  // +0x38c: plain and marble shapes (bodytex.glsl; basic.glsl in the refraction pass)
    BodyMesh glass;      // +0xeb4: glass (glass.glsl)
    BodyMesh glass_tex;  // +0xf1c: textured glass (glasstex.glsl)
    BodyMesh metal;      // +0x12e4: metal shapes and ropes (metal.glsl)
    std::vector<BodyVertex> glass_edges;  // +0xef4: line list along glass creases
    void clear() { basic.clear(), glass.clear(), glass_tex.clear(), metal.clear(), glass_edges.clear(); }
};

// RenderLevel::fillBuffers: the level's bodies (alive, not round) nearest first, each visible shape's faces
// that face the camera as fans in world space; ropes as flat strips. level_time: Level +0xd8 (scored glass
// flashes white and fades over a third of a second after it is cleared).
void build_body_buffers(const phys::PhysicsWorld& world, phys::Vec3 camera, float level_time, BodyBuffers& out);

// The refractive discs of balls and power-ups (ballreflection.glsl: aPosition, aTexCoord as three normalised bytes
// holding the disc normal, aColor).
struct ReflectionVertex {
    float pos[3];
    std::uint8_t normal[4];
    float color[4];
};
struct ReflectionMesh {
    std::vector<ReflectionVertex> vertices;
    std::vector<std::uint32_t> indices;
    void clear() { vertices.clear(), indices.clear(); }
};
// Soft shadows (softshadow.glsl: aPosition, aTexCoord as bytes, aAlpha as a byte).
struct ShadowVertex {
    float pos[3];
    std::uint8_t uv[2];
    std::uint8_t alpha;
    std::uint8_t pad;
};

// RenderLevel::addReflection: a ten-segment disc of the radius at pos in the frame rot (rim normals 1.05 x the rim
// direction, clamped), coloured rgb with the distance fade as alpha; nothing within 0.5 of the camera.
// Level::getPointAtDistance: where a level distance falls in the current room, (0, 0, z) with z = -start -
// ((d - distance base) / the room def's length) * the room's length; (0, 0, 0) when the def length is 0.
phys::Vec3 point_at_distance(const Level& level, float d);

// RenderLevel::draw's best-distance marker (gfx/bestdistance.png through 2dtex.glsl): in a campaign, not in zen
// mode (3), not in a room whose type contains "boss". During the game-over fade (Level +0x21c > 0) with a distance past
// the best it rides 3 ahead of the camera, faded in over the fade's last quarter; otherwise it stands at the best
// distance while that is ahead of the camera by less than 100 (best >= 1). Its alpha also falls off by 0.03 per unit
// from the camera. The quad: centre (0, 0.5, z), 2.16 wide, 0.8 high, texture upright.
struct BestDistanceMarker {
    float z = 0;
    float alpha = 0;
};
bool best_distance_marker(const Level& level, int player_mode, int best_distance, BestDistanceMarker& out);

void add_reflection(ReflectionMesh& out, phys::Vec3 camera, phys::Vec3 pos, phys::Quat rot, float radius,
                    const float rgb[3]);
// RenderLevel::addShadow: a ray one unit down onto level geometry (mask 0x80); on a hit a 1.2-wide square 0.01 above
// it, alpha (0.4 - 2 (height - 0.5)) * strength. Returns whether a shadow was added (four vertices).
bool add_shadow(const phys::PhysicsWorld& world, std::vector<ShadowVertex>& out, phys::Vec3 pos, float strength,
                float reach = 1.0f);
// QiTexture::loadBlob(1) on a size x size alpha texture: (1 - d^2)^3 * 0.7833 inside the unit circle.
std::vector<std::uint8_t> shadow_blob(int size);
// The debris shards (RenderLevel::fillBuffers2 into +0x754): each piece's four triangles facing the camera, with the
// piece's colour, as a triangle list (drawn through the glass shader).
void debris_triangles(const std::vector<phys::DebrisVertices>& pieces, phys::Vec3 camera, std::vector<BodyVertex>& out);
// fillBuffers2's balls and power-ups: balls as camera-facing sprites (cell 0) with a disc tinted by their state;
// power-ups not yet picked as bobbing sprites (cell by type) with a disc and a shadow.
void entity_sprites(const Level& level, const phys::PhysicsWorld& world, phys::Quat display_rotation,
                    std::vector<SpriteVertex>& sprites, ReflectionMesh& discs, std::vector<ShadowVertex>& shadows);

// fillBuffers2's buttons (Level +0x1fc): cell 8 (cell 9 when pressed), half size 0.3, 0.05 in front in z, in the
// button's rotation; an unpressed one also a glow (cell 14, 0.01 further, alpha 0.8 + 0.2 sin((phase + time) * 9);
// the phase is the button's address mod 100 in the original, its index here). The tutorial rooms' hint arrows
// next to a button are not drawn.
void button_sprites(const Level& level, std::vector<SpriteVertex>& sprites);
// Room::addDecals: each decal of the room as a sprite in the room body's frame, for the decal pass (doors.png).
// A cell from 0 up is on the sprite sheet and joins `sprites`; a negative one is on doors.png and goes to `doors`, tinted
// in player mode 2.
void decal_sprites(const Room& room, phys::Vec3 camera, int player_mode, std::vector<SpriteVertex>& sprites,
                   std::vector<SpriteVertex>& doors);

// RenderLevel::drawObstacleScore for each live obstacle of the room: an obstacle with a score that is not cleared
// shows a glow (cell 21, half size 0.8, alpha 0.6) at its mgGlow point moved 0.5 towards the camera; one cleared for
// points shows its score rising (y + 0.1 t) and fading (alpha 2.5 - 2t) for 1.25 s: a blue plate (cell 62) with the
// digits (cells 44-47, 52-55, 60, 61; "1" half as wide), and a shadow below it.
void obstacle_score_sprites(const Room& room, const phys::PhysicsWorld& world, phys::Vec3 camera, float level_time,
                            std::vector<SpriteVertex>& sprites, std::vector<ShadowVertex>& shadows);

// A room's reflection data in world space as triangle lists: the stencil mask (Room::drawReflectionMask) and the
// walls mirrored in the floor (Room::drawReflection).
void room_reflections(const Room& room, std::vector<BodyVertex>& mask, std::vector<BodyVertex>& walls);

// RenderLevel::draw's fog colours (lower, upper; RGBA): the current room's, blended toward the next room's while
// the camera is within 10 units of the next room's start (rgb only; alpha stays the current room's). The original
// also switches to all-lower / all-upper when the head looks within acos(0.99) of straight down / up (VR only, not
// here).
void level_fog(const Level& level, float lower[4], float upper[4]);

// RenderLevel::fillBuffers2's laser sprites, three per beam of this frame (Level +0x208): a body along the beam
// (cell 16) and caps 0.1 inside each end (cells 24 and 25), each with alpha QiRnd(0.8, 1). Those three lrand48 draws
// per beam happen every rendered frame in the original, so call this once per game frame. Appends to out.
void laser_sprites(Rand48& rng, const std::vector<Level::Laser>& lasers, phys::Vec3 camera,
                   std::vector<SpriteVertex>& out);

}  // namespace sh::game
