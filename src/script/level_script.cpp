#include "script/level_script.hpp"

#include "core/qi_string.hpp"
#include "physics/breakage.hpp"
#include "physics/collision.hpp"
#include "physics/physics_world.hpp"
#include "physics/qi_rotate.hpp"
#include "script/material_table.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <ctime>
#include <pugixml.hpp>

namespace sh::script {

const phys::ConvexHull* ScriptLevel::get_convex(const std::string& name) {
    for (const auto& [n, hull] : convexes)
        if (n == name) return hull.get();
    const std::optional<std::string> text = read_asset ? read_asset("meshes/" + name + ".xml") : std::nullopt;
    if (!text) return nullptr;
    pugi::xml_document doc;
    if (!doc.load_buffer(text->data(), text->size())) return nullptr;
    std::vector<phys::Vec3> points;
    for (const pugi::xml_node v : doc.document_element().children("v")) {
        const std::string value = v.child_value();
        points.push_back({to_float(get_word(value, 0)), to_float(get_word(value, 1)), to_float(get_word(value, 2))});
    }
    if (points.size() <= 2) return nullptr;
    convexes.emplace_back(name, std::make_unique<phys::ConvexHull>(points.data(), static_cast<int>(points.size())));
    return convexes.back().second.get();
}

const std::vector<std::string>& level_binding_names() {
    static const std::vector<std::string> names = {
        "mgTime", "mgCameraDistance", "mgGetCameraPos", "mgGetCameraSpeed", "mgGetBeat", "mgGetBeatFloat",
        "mgGetString", "mgGetFloat", "mgGetInt", "mgGetBool", "mgGetColor", "mgRandomize", "mgRndInt",
        "mgRndFloat", "mgRndBool", "mgRaycast", "mgScore", "mgGlow", "mgClear", "mgIsCleared", "mgCanPass",
        "mgSegment", "mgCameraShake", "mgCreateSound", "mgPlaySound", "mgSmoke", "mgCreateBody", "mgMove",
        "mgRotate", "mgReleaseMotion", "mgGetPos", "mgGetRot", "mgShapeReset", "mgMaterial", "mgDensity",
        "mgFriction", "mgRestitution", "mgForceLimit", "mgRoundness", "mgCategory", "mgMask", "mgStatic",
        "mgColor", "mgTexture", "mgShapeScale", "mgShapeChamfer", "mgCreateBox", "mgCreateCylinder",
        "mgCreateMesh", "mgClearShapes", "mgRemoveShape", "mgTranslateShape", "mgGetShapeBody",
        "mgBodyContainsPoint", "mgHideEdges", "mgSetShapeColor", "mgSetShapeMask", "mgSetPos", "mgSetVel",
        "mgSetAngVel", "mgClonePose", "mgBodyIsBroken", "mgShapeIsBroken", "mgCreateButton", "mgIsButtonDown",
        "mgIsButtonUp", "mgLaser", "mgCreateBallJoint", "mgCreateHingeJoint", "mgCreateSliderJoint",
        "mgCreateRope", "mgJointMotor", "mgGetJointBodies", "mgIsConnected", "mgBox", "mgLength",
        "mgSpeedMultiplier", "mgMusic", "mgReverb", "mgLowPass", "mgEcho", "mgBoxOverlap", "mgObstacle",
        "mgPowerUp", "mgFogColor", "mgGravity", "mgGetTimeStep", "mgSetDifficulty", "mgSetRotation",
        "mgParticles", "mgGetGravity", "mgGet", "mgSet", "mgObstacleTime", "mgBodyPointToWorld",
        "mgBodyVecToWorld", "mgHasBoss", "mgIsClosestBoss", "mgBodyDecal"};
    return names;
}

LevelScript::LevelScript(Rand48& rng, std::vector<Param> params, bool is_room, int index, ScriptLevel* level,
                         ScriptObstacle obstacle)
    : host_(rng), params_(std::move(params)), is_room_(is_room), index_(index), level_(level), obstacle_(obstacle) {}

namespace {

using phys::Quat;
using phys::Vec3;

using phys::qi_euler;
using phys::qi_mul;
Quat qmul(Quat a, Quat b) { return qi_mul(a, b); }
Quat euler(float rx, float ry, float rz) { return qi_euler(rx, ry, rz); }

// QiMatrix4::operator* (column-major): out[c][r] = ((A[1][r] B[c][1] + A[0][r] B[c][0]) + A[2][r] B[c][2])
// + A[3][r] B[c][3], every term included.
void mat_mul(const float a[16], const float b[16], float out[16]) {
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r)
            out[c * 4 + r] = ((a[4 + r] * b[c * 4 + 1] + a[r] * b[c * 4]) + a[8 + r] * b[c * 4 + 2]) + a[12 + r] * b[c * 4 + 3];
}

// QiQuat::getEulerAngles, returned as (x, y, z). The x term's denominator is 1 - 2 (w^2 + z^2) and the y
// term is asin of 2 (xz - wy), clamped to [-1, 1] (NaN gives -1), as the original computes them.
Vec3 euler_angles(Quat q) {
    const float zy = q.z * q.z + q.y * q.y, zw = q.z * q.w + q.y * q.x;
    const float yaw = std::atan2(zw + zw, 1.0f - (zy + zy));
    float s = q.x * q.z - q.w * q.y;
    s = s + s;
    s = !(s > -1.0f) ? -1.0f : (s >= 1.0f ? 1.0f : s);
    const float pitch = std::asin(s);
    const float ww = q.w * q.w + q.z * q.z, wx = q.z * q.y + q.w * q.x;
    const float roll = std::atan2(wx + wx, 1.0f - (ww + ww));
    return {roll, pitch, yaw};
}

// The animation target (body +0x114), created as identity with max linear speed FLT_MAX when missing.
phys::RigidBody::AnimTarget& anim_target(phys::RigidBody& b) {
    if (!b.anim) {
        b.anim.emplace();
        b.anim->position = {0, 0, 0};
        b.anim->rotation = {0, 0, 0, 1};
        b.anim->max_linear_speed = FLT_MAX;
    }
    return *b.anim;
}

// The shape matrix of mgCreateBox / mgCreateCylinder: QiMatrix4(pose) * diag(scale, 1).
void shape_matrix(Vec3 scale, Vec3 pos, Quat rot, float out[16]) {
    const phys::Transform t = phys::Transform::from(pos, rot);
    const float s[16] = {scale.x, 0, 0, 0, 0, scale.y, 0, 0, 0, 0, scale.z, 0, 0, 0, 0, 1};
    mat_mul(t.m, s, out);  // pose * scale: the scale acts in the shape's own frame, the offset is unscaled
}

}  // namespace

const std::string* LevelScript::find_param(const std::string& name) const {
    const std::string* found = nullptr;
    for (const Param& p : params_)
        if (p.name == name) found = &p.value;
    return found;
}

void LevelScript::set_parameter(const std::string& name, const std::string& value) {
    for (Param& p : params_)
        if (p.name == name) {
            p.value = value;
            return;
        }
    params_.push_back({name, value});
}

bool LevelScript::load(const std::string& path, const ReadAsset& read, bool random_seed) {
    const std::optional<std::string> text = read(path);
    if (!text) return false;
    std::string chunk = *text;
    // The include is appended to the same stream when it exists (obstacles/include.lua does not).
    if (const auto inc = read(is_room_ ? "rooms/include.lua" : "obstacles/include.lua")) chunk += *inc;
    host_.load(path, chunk);
    register_bindings();
    random.seed = random_seed ? static_cast<std::uint32_t>(host_.rng().lrand48()) : 0u;
    if (host_.has_function("init")) host_.execute("init()");
    return true;
}

void LevelScript::tick(int game_frame) {
    if (!host_.has_function("tick")) return;
    host_.execute("tick()");
    if ((index_ + game_frame) % 30 == 0) host_.collect_garbage(1);
}

void LevelScript::draw() {
    if (host_.has_function("draw")) host_.execute("draw()");
}

void LevelScript::register_bindings() {
    std::map<std::string, ScriptHost::Fn> fns;
    // Placement parameters: the default unless a parameter of that name exists (the last one wins).
    fns["mgGetFloat"] = [this](const ScriptArgs& a, ScriptRet& r) {
        float v = a.as_float(1);
        if (const std::string* p = find_param(a.as_string(0))) v = to_float(*p);
        r.add_float(v);
    };
    fns["mgGetInt"] = [this](const ScriptArgs& a, ScriptRet& r) {
        int v = a.as_int(1);
        if (const std::string* p = find_param(a.as_string(0))) v = to_int(*p);
        r.add_int(v);
    };
    fns["mgGetBool"] = [this](const ScriptArgs& a, ScriptRet& r) {
        bool v = a.as_bool(1);
        if (const std::string* p = find_param(a.as_string(0))) v = *p == "true";
        r.add_bool(v);
    };
    fns["mgGetString"] = [this](const ScriptArgs& a, ScriptRet& r) {
        std::string v = a.as_string(1);
        if (const std::string* p = find_param(a.as_string(0))) v = *p;
        r.add_string(v);
    };
    // The default colour goes through text ("r g b" with formatFloat) like a parameter value would.
    fns["mgGetColor"] = [this](const ScriptArgs& a, ScriptRet& r) {
        std::string v = format_float(a.as_float(1)) + " " + format_float(a.as_float(2)) + " " + format_float(a.as_float(3));
        if (const std::string* p = find_param(a.as_string(0))) v = *p;
        for (int i = 0; i < 3; ++i) r.add_float(to_float(get_word(v, i)));
    };
    // The script's own QiRandom.
    fns["mgRndFloat"] = [this](const ScriptArgs& a, ScriptRet& r) { r.add_float(random.get_float(a.as_float(0), a.as_float(1))); };
    fns["mgRndInt"] = [this](const ScriptArgs& a, ScriptRet& r) { r.add_int(random.get_int(a.as_int(0), a.as_int(1))); };
    fns["mgRndBool"] = [this](const ScriptArgs& a, ScriptRet& r) {
        const float p = a.size() > 0 ? a.as_float(0) : 0.5f;
        r.add_bool(random.get_float(0.0f, 1.0f) < p);
    };
    fns["mgRandomize"] = [this](const ScriptArgs& a, ScriptRet&) {
        // With a seed argument the seed is set; without, the original seeds from the clock (time()).
        random.seed = a.size() > 0 ? static_cast<std::uint32_t>(a.as_int(0)) : static_cast<std::uint32_t>(std::time(nullptr));
    };
    // The current shape state (LevelScript +0x264 .. +0x2b1).
    fns["mgShapeReset"] = [this](const ScriptArgs&, ScriptRet&) {
        material = phys::Material{};  // density 1, friction 1, strength FLT_MAX, group 1, mask 0xff, ...
        shape_scale = {1, 1, 1};      // the chamfer is kept
    };
    fns["mgMaterial"] = [this](const ScriptArgs& a, ScriptRet&) {
        if (level_ && level_->materials) level_->materials->get(a.as_string(0), material);
    };
    fns["mgDensity"] = [this](const ScriptArgs& a, ScriptRet&) { material.density = a.as_float(0); };
    fns["mgFriction"] = [this](const ScriptArgs& a, ScriptRet&) { material.friction = a.as_float(0); };
    fns["mgRestitution"] = [this](const ScriptArgs& a, ScriptRet&) { material.restitution = a.as_float(0); };
    fns["mgForceLimit"] = [this](const ScriptArgs& a, ScriptRet&) { material.strength = a.as_float(0); };
    fns["mgRoundness"] = [this](const ScriptArgs& a, ScriptRet&) { material.roundness = a.as_float(0); };
    // mgTexture(on, scale u, scale v, offset u, offset v): textured glass (its own buffer and shader).
    fns["mgTexture"] = [this](const ScriptArgs& a, ScriptRet&) {
        material.b3c = a.as_bool(0) ? 1 : 0;
        material.f34 = a.as_float(1), material.f38 = a.as_float(2);
        material.f2c = a.as_float(3), material.f30 = a.as_float(4);
    };
    // mgHideEdges(on): no crease lines on this glass.
    fns["mgHideEdges"] = [this](const ScriptArgs& a, ScriptRet&) { material.b3d = a.as_bool(0) ? 1 : 0; };
    fns["mgShapeChamfer"] = [this](const ScriptArgs& a, ScriptRet&) { chamfer = a.as_float(0); };
    fns["mgShapeScale"] = [this](const ScriptArgs& a, ScriptRet&) {
        if (a.size() == 3) shape_scale = {a.as_float(0), a.as_float(1), a.as_float(2)};
        else shape_scale = {a.as_float(0), a.as_float(0), a.as_float(0)};
    };
    fns["mgColor"] = [this](const ScriptArgs& a, ScriptRet&) {
        const float alpha = a.size() > 3 ? a.as_float(3) : 1.0f;
        material.color[0] = a.as_float(0), material.color[1] = a.as_float(1), material.color[2] = a.as_float(2);
        material.color[3] = alpha;
    };
    fns["mgStatic"] = [this](const ScriptArgs& a, ScriptRet&) { material.type = static_cast<std::uint8_t>(a.as_int(0)); };
    fns["mgMask"] = [this](const ScriptArgs& a, ScriptRet&) { material.mask = static_cast<std::uint8_t>(a.as_int(0)); };
    fns["mgCategory"] = [this](const ScriptArgs& a, ScriptRet&) { material.group = static_cast<std::uint8_t>(a.as_int(0)); };
    // Bodies and shapes (handles from the level's table).
    fns["mgCreateBody"] = [this](const ScriptArgs& a, ScriptRet& r) {
        Vec3 p{0, 0, 0};
        Quat q{0, 0, 0, 1};
        if (a.size() >= 1) {
            p = {a.as_float(0), a.as_float(1), a.as_float(2)};
            if (a.size() >= 4) q = euler(a.as_float(3), a.as_float(4), a.as_float(5));
        }
        phys::RigidBody& b = level_->world->create_body();  // Level::create
        b.set_transform(phys::qi_transform(obstacle_.position, obstacle_.rotation, p), qmul(obstacle_.rotation, q));
        b.near_despawn = obstacle_.id;  // Entity::setObstacle
        if (obstacle_.entities) b.obstacle_list = obstacle_.entities, obstacle_.entities->push_back(&b);
        r.add_handle(level_->handles.add(&b));
    };
    fns["mgCreateBox"] = [this](const ScriptArgs& a, ScriptRet& r) {
        phys::RigidBody* b = level_->handles.get<phys::RigidBody>(a.as_handle(0));
        const Vec3 half{a.as_float(1), a.as_float(2), a.as_float(3)};
        Vec3 p{0, 0, 0};
        Quat q{0, 0, 0, 1};
        if (a.size() > 4) {
            p = {a.as_float(4), a.as_float(5), a.as_float(6)};
            if (a.size() > 7) q = euler(a.as_float(7), a.as_float(8), a.as_float(9));
        }
        float m[16];
        shape_matrix(shape_scale, p, q, m);
        phys::BodyShape& s = b->add_box(m, half, &material, chamfer);
        b->compute_mass_properties();
        r.add_handle(level_->handles.add(&s));
    };
    fns["mgCreateCylinder"] = [this](const ScriptArgs& a, ScriptRet& r) {
        phys::RigidBody* b = level_->handles.get<phys::RigidBody>(a.as_handle(0));
        const float radius = a.as_float(1), half = a.as_float(2);
        Vec3 p{0, 0, 0};
        Quat q{0, 0, 0, 1};
        if (a.size() > 3) {
            p = {a.as_float(3), a.as_float(4), a.as_float(5)};
            if (a.size() > 6) q = euler(a.as_float(6), a.as_float(7), a.as_float(8));
        }
        float m[16];
        shape_matrix(shape_scale, p, q, m);
        phys::BodyShape& s = b->add_cylinder(m, radius, half, &material, chamfer);
        b->compute_mass_properties();
        r.add_handle(level_->handles.add(&s));
    };
    fns["mgSetShapeMask"] = [this](const ScriptArgs& a, ScriptRet&) {
        phys::BodyShape* s = level_->handles.get<phys::BodyShape>(a.as_handle(0));
        s->material.mask = static_cast<std::uint8_t>(a.as_int(1));
        if (a.size() >= 3) s->material.group = static_cast<std::uint8_t>(a.as_int(2));
    };
    fns["mgReleaseMotion"] = [this](const ScriptArgs& a, ScriptRet&) {
        level_->handles.get<phys::RigidBody>(a.as_handle(0))->anim.reset();
    };
    fns["mgClonePose"] = [this](const ScriptArgs& a, ScriptRet&) {
        phys::RigidBody* dst = level_->handles.get<phys::RigidBody>(a.as_handle(0));
        const phys::RigidBody* src = level_->handles.get<phys::RigidBody>(a.as_handle(1));
        dst->set_transform(src->position, src->rotation);
        dst->solver.linear_velocity = src->solver.linear_velocity;
        dst->solver.angular_velocity = src->solver.angular_velocity;
    };
    // Poses and velocities, in the obstacle's frame (LevelScript +0x260: position +4, rotation +0x10).
    fns["mgGetPos"] = [this](const ScriptArgs& a, ScriptRet& r) {
        const phys::RigidBody* b = level_->handles.get<phys::RigidBody>(a.as_handle(0));
        const Vec3 d = b->position - obstacle_.position;
        const Vec3 p = phys::qi_rotate_inverse(obstacle_.rotation, d);
        r.add_float(p.x), r.add_float(p.y), r.add_float(p.z);
    };
    fns["mgGetRot"] = [this](const ScriptArgs& a, ScriptRet& r) {
        const phys::RigidBody* b = level_->handles.get<phys::RigidBody>(a.as_handle(0));
        const Vec3 e = euler_angles(phys::qi_relative_rotation(obstacle_.rotation, b->rotation));
        r.add_float(e.x), r.add_float(e.y), r.add_float(e.z);
    };
    fns["mgSetPos"] = [this](const ScriptArgs& a, ScriptRet&) {
        // Only the entity position: the solver body stays where it was.
        phys::RigidBody* b = level_->handles.get<phys::RigidBody>(a.as_handle(0));
        b->position = phys::qi_transform(obstacle_.position, obstacle_.rotation, {a.as_float(1), a.as_float(2), a.as_float(3)});
    };
    fns["mgSetVel"] = [this](const ScriptArgs& a, ScriptRet&) {
        phys::RigidBody* b = level_->handles.get<phys::RigidBody>(a.as_handle(0));
        b->set_velocity(phys::qi_rotate(obstacle_.rotation, {a.as_float(1), a.as_float(2), a.as_float(3)}));
    };
    fns["mgSetAngVel"] = [this](const ScriptArgs& a, ScriptRet&) {
        phys::RigidBody* b = level_->handles.get<phys::RigidBody>(a.as_handle(0));
        b->set_angular_velocity(phys::qi_rotate(obstacle_.rotation, {a.as_float(1), a.as_float(2), a.as_float(3)}));
    };
    // mgMove / mgRotate set the animation target (created on first use), strength and max speed default
    // to FLT_MAX.
    fns["mgMove"] = [this](const ScriptArgs& a, ScriptRet&) {
        phys::RigidBody* b = level_->handles.get<phys::RigidBody>(a.as_handle(0));
        const Vec3 p{a.as_float(1), a.as_float(2), a.as_float(3)};
        phys::RigidBody::AnimTarget& t = anim_target(*b);
        t.position = phys::qi_transform(obstacle_.position, obstacle_.rotation, p);
        t.linear_strength = a.size() > 4 ? a.as_float(4) : FLT_MAX;
        t.max_linear_speed = a.size() > 5 ? a.as_float(5) : FLT_MAX;
    };
    fns["mgRotate"] = [this](const ScriptArgs& a, ScriptRet&) {
        phys::RigidBody* b = level_->handles.get<phys::RigidBody>(a.as_handle(0));
        const Quat q = euler(a.as_float(1), a.as_float(2), a.as_float(3));
        phys::RigidBody::AnimTarget& t = anim_target(*b);
        t.rotation = qmul(obstacle_.rotation, q);
        t.angular_strength = a.size() > 4 ? a.as_float(4) : FLT_MAX;
        t.max_angular_speed = a.size() > 5 ? a.as_float(5) : FLT_MAX;
    };
    fns["mgBodyVecToWorld"] = [this](const ScriptArgs& a, ScriptRet& r) {
        // The body's rotation only (world space; the obstacle is not involved).
        const phys::RigidBody* b = level_->handles.get<phys::RigidBody>(a.as_handle(0));
        const Vec3 v = phys::qi_rotate(b->rotation, {a.as_float(1), a.as_float(2), a.as_float(3)});
        r.add_float(v.x), r.add_float(v.y), r.add_float(v.z);
    };
    fns["mgBodyPointToWorld"] = [this](const ScriptArgs& a, ScriptRet& r) {
        // A body-local point in the obstacle's frame.
        const phys::RigidBody* b = level_->handles.get<phys::RigidBody>(a.as_handle(0));
        const Vec3 w = phys::qi_transform(b->position, b->rotation, {a.as_float(1), a.as_float(2), a.as_float(3)});
        const Vec3 v = phys::qi_rotate_inverse(obstacle_.rotation, w - obstacle_.position);
        r.add_float(v.x), r.add_float(v.y), r.add_float(v.z);
    };
    fns["mgGetShapeBody"] = [this](const ScriptArgs& a, ScriptRet& r) {
        const phys::BodyShape* s = level_->handles.get<phys::BodyShape>(a.as_handle(0));
        if (s) r.add_handle(level_->handles.add(s->body));  // no result for a null shape
    };
    fns["mgBodyIsBroken"] = [this](const ScriptArgs& a, ScriptRet& r) {
        r.add_bool(level_->handles.get<phys::RigidBody>(a.as_handle(0))->broken);
    };
    fns["mgShapeIsBroken"] = [this](const ScriptArgs& a, ScriptRet& r) {
        r.add_bool(level_->handles.get<phys::BodyShape>(a.as_handle(0))->modified);
    };
    // Shape editing.
    fns["mgCreateMesh"] = [this](const ScriptArgs& a, ScriptRet&) {
        // (body, name [, x, y, z [, rx, ry, rz]]): a convex shape from meshes/<name>.xml. Returns nothing.
        phys::RigidBody* b = level_->handles.get<phys::RigidBody>(a.as_handle(0));
        const std::string name = a.as_string(1);
        Vec3 p{0, 0, 0};
        Quat q{0, 0, 0, 1};
        if (a.size() > 2) {
            p = {a.as_float(2), a.as_float(3), a.as_float(4)};
            if (a.size() > 5) q = euler(a.as_float(5), a.as_float(6), a.as_float(7));
        }
        if (const phys::ConvexHull* hull = level_->get_convex(name)) {
            float m[16];
            shape_matrix(shape_scale, p, q, m);
            b->add_convex(m, *hull, &material);
        }
        b->compute_mass_properties();
    };
    fns["mgTranslateShape"] = [this](const ScriptArgs& a, ScriptRet&) {
        // (shape, x, y, z [, pieces]): moves the shape's collision points and polyhedron vertices as they are
        // (no frame change); with pieces, every shape of its body that came from it.
        phys::BodyShape* s = level_->handles.get<phys::BodyShape>(a.as_handle(0));
        const Vec3 d{a.as_float(1), a.as_float(2), a.as_float(3)};
        auto translate = [&d](phys::BodyShape& t) {
            for (Vec3& h : t.hull) h = {h.x + d.x, h.y + d.y, h.z + d.z};
            for (auto& v : t.poly.vertices) v.position = {v.position.x + d.x, v.position.y + d.y, v.position.z + d.z};
        };
        phys::RigidBody* b = s->body;
        if (a.size() > 4 && a.as_bool(4)) {
            for (size_t i = 0; i < b->shapes.size(); ++i)
                if (b->shapes[i]->origin == s) translate(*b->shapes[i]);
        } else {
            translate(*s);
        }
        b->compute_mass_properties();
    };
    fns["mgRemoveShape"] = [this](const ScriptArgs& a, ScriptRet&) {
        phys::BodyShape* s = level_->handles.get<phys::BodyShape>(a.as_handle(0));
        if (!s) return;
        phys::RigidBody* b = s->body;
        level_->world->destroy_shape(*b, s);
        b->compute_mass_properties();
    };
    fns["mgClearShapes"] = [this](const ScriptArgs& a, ScriptRet&) {
        // Every shape goes, first to last; the mass properties are not recomputed.
        phys::RigidBody* b = level_->handles.get<phys::RigidBody>(a.as_handle(0));
        if (!b) return;
        while (!b->shapes.empty()) level_->world->destroy_shape(*b, b->shapes.front().get());
    };
    fns["mgSetShapeColor"] = [this](const ScriptArgs& a, ScriptRet&) {
        // (shape, r, g, b): the colour of every shape in the level that came from this one (itself included).
        const void* id = level_->handles.get<void>(a.as_handle(0));
        for (phys::RigidBody* b : level_->world->bodies)
            for (auto& s : b->shapes)
                if (s->origin == id) {
                    s->material.color[0] = a.as_float(1);
                    s->material.color[1] = a.as_float(2);
                    s->material.color[2] = a.as_float(3);
                }
    };
    fns["mgBodyContainsPoint"] = [this](const ScriptArgs& a, ScriptRet& r) {
        // (body, x, y, z): whether the point (body frame) is inside one of the body's collision hulls.
        const phys::RigidBody* b = level_->handles.get<phys::RigidBody>(a.as_handle(0));
        bool inside = false;
        phys::closest_point(level_->world->context, *b, {a.as_float(1), a.as_float(2), a.as_float(3)}, nullptr,
                            nullptr, &inside);
        r.add_bool(inside);
    };
    // Joints (Level::createJoint, then Joint::init* with points and axes from the obstacle's frame). A 0 body
    // handle means the room's body.
    auto joint_body = [this](const ScriptArgs& a, int i) {
        const int h = a.as_handle(i);
        return h ? level_->handles.get<phys::RigidBody>(h) : room->body;
    };
    auto point = [this](const ScriptArgs& a, int i) {
        return phys::qi_transform(obstacle_.position, obstacle_.rotation, {a.as_float(i), a.as_float(i + 1), a.as_float(i + 2)});
    };
    auto axis = [this](const ScriptArgs& a, int i) {
        return phys::qi_rotate(obstacle_.rotation, {a.as_float(i), a.as_float(i + 1), a.as_float(i + 2)});
    };
    fns["mgCreateBallJoint"] = [=, this](const ScriptArgs& a, ScriptRet& r) {
        phys::RigidBody *ba = joint_body(a, 0), *bb = joint_body(a, 1);
        phys::BodyJoint& j = level_->world->create_joint(ba, bb);
        j.init_ball(point(a, 2));
        r.add_handle(level_->handles.add(&j));
    };
    fns["mgCreateHingeJoint"] = [=, this](const ScriptArgs& a, ScriptRet& r) {
        phys::RigidBody *ba = joint_body(a, 0), *bb = joint_body(a, 1);
        phys::BodyJoint& j = level_->world->create_joint(ba, bb);
        j.init_hinge(point(a, 2), axis(a, 5));
        r.add_handle(level_->handles.add(&j));
    };
    fns["mgCreateSliderJoint"] = [=, this](const ScriptArgs& a, ScriptRet& r) {
        phys::RigidBody *ba = joint_body(a, 0), *bb = joint_body(a, 1);
        phys::BodyJoint& j = level_->world->create_joint(ba, bb);
        j.init_slider(point(a, 2), axis(a, 5));
        r.add_handle(level_->handles.add(&j));
    };
    fns["mgCreateRope"] = [=, this](const ScriptArgs& a, ScriptRet& r) {
        // (a, b, point on a, point on b, slack)
        phys::RigidBody *ba = joint_body(a, 0), *bb = joint_body(a, 1);
        const float slack = a.as_float(8);
        phys::BodyJoint& j = level_->world->create_joint(ba, bb);
        j.init_rope(point(a, 2), point(a, 5), slack);
        r.add_handle(level_->handles.add(&j));
    };
    fns["mgJointMotor"] = [this](const ScriptArgs& a, ScriptRet&) {
        // (joint, speed, torque): hinges only.
        phys::BodyJoint* j = level_->handles.get<phys::BodyJoint>(a.as_handle(0));
        if (!j || j->type != phys::BodyJoint::kHinge) return;
        j->motor_speed = a.as_float(1);
        j->motor_torque = a.as_float(2);
    };
    fns["mgGetJointBodies"] = [this](const ScriptArgs& a, ScriptRet& r) {
        const phys::BodyJoint* j = level_->handles.get<phys::BodyJoint>(a.as_handle(0));
        if (!j) return;
        r.add_handle(level_->handles.add(j->a));
        r.add_handle(level_->handles.add(j->b));
    };
    fns["mgIsConnected"] = [=, this](const ScriptArgs& a, ScriptRet& r) {
        // Whether a chain of joints (through any bodies) links the two bodies.
        const phys::RigidBody *from = joint_body(a, 0), *to = joint_body(a, 1);
        if (from == to) return r.add_bool(true);
        std::vector<const phys::RigidBody*> seen{from}, open{from};
        while (!open.empty()) {
            const phys::RigidBody* b = open.back();
            open.pop_back();
            for (const phys::BodyJoint* j : b->joints) {
                const phys::RigidBody* other = j->a == b ? j->b : j->a;
                if (other == to) return r.add_bool(true);
                if (std::find(seen.begin(), seen.end(), other) == seen.end()) seen.push_back(other), open.push_back(other);
            }
        }
        r.add_bool(false);
    };
    // Room settings: fields of the script's room (or obstacle, for mgGlow), stored as given.
    fns["mgSegment"] = [this](const ScriptArgs& a, ScriptRet& r) {
        // (name, z): only in the room's own script (isRoom); returns the segment's length.
        if (!room_script || !room || !room->create_segment_fn) return;
        const std::string name = a.as_string(0);
        r.add_float(room->create_segment_fn(name, a.as_float(1)));
    };
    fns["mgObstacle"] = [this](const ScriptArgs& a, ScriptRet&) {
        // (type, x, y, z [, rx, ry, rz] [, "name=value" ...]): placed by this script's obstacle pose (world); the
        // room adds its body's pose again when it spawns it. Arguments from index 4 with '=' become params.
        const std::string type = a.as_string(0);
        const Vec3 p{a.as_float(1), a.as_float(2), a.as_float(3)};
        const Quat q = a.size() < 5 ? Quat{0, 0, 0, 1} : euler(a.as_float(4), a.as_float(5), a.as_float(6));
        std::vector<std::pair<std::string, std::string>> params;
        for (int i = 4; i < static_cast<int>(a.size()); ++i) {
            const std::string s = a.as_string(i);
            const size_t eq = s.find('=');
            if (eq != std::string::npos) params.emplace_back(s.substr(0, eq), s.substr(eq + 1));
        }
        if (room && room->add_obstacle_fn)
            room->add_obstacle_fn(type, phys::qi_transform(obstacle_.position, obstacle_.rotation, p),
                                  qmul(obstacle_.rotation, q), params);
    };
    fns["mgBox"] = [this](const ScriptArgs& a, ScriptRet&) {
        // Room::addBox: (hx, hy, hz, x, y, z) on the room body with the current material, bevel 0; the shape stays
        // out of tree A. Then the body's mass properties.
        const Vec3 half{a.as_float(0), a.as_float(1), a.as_float(2)};
        const float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, a.as_float(3), a.as_float(4), a.as_float(5), 1};
        room->body->add_box(m, half, &material, 0.0f);
        room->body->compute_mass_properties();
    };
    fns["mgLength"] =[this](const ScriptArgs& a, ScriptRet&) { room->length = a.as_float(0); };
    fns["mgMusic"] = [this](const ScriptArgs& a, ScriptRet&) { room->music = a.as_string(0); };
    fns["mgParticles"] = [this](const ScriptArgs& a, ScriptRet&) { room->particles = a.as_string(0); };
    fns["mgReverb"] = [this](const ScriptArgs& a, ScriptRet&) {
        room->reverb[0] = a.as_float(0), room->reverb[1] = a.as_float(1), room->reverb[2] = a.as_float(2);
    };
    fns["mgEcho"] = [this](const ScriptArgs& a, ScriptRet&) {
        for (int i = 0; i < 4; ++i) room->echo[i] = a.as_float(i);
    };
    fns["mgLowPass"] = [this](const ScriptArgs& a, ScriptRet&) { room->low_pass = a.as_float(0); };
    fns["mgSpeedMultiplier"] = [this](const ScriptArgs& a, ScriptRet&) { room->speed_multiplier = a.as_float(0); };
    fns["mgGravity"] = [this](const ScriptArgs& a, ScriptRet&) { room->gravity = a.as_float(0); };
    fns["mgSetDifficulty"] = [this](const ScriptArgs& a, ScriptRet&) { room->difficulty = a.as_float(0); };
    fns["mgSetRotation"] = [this](const ScriptArgs& a, ScriptRet&) {
        room->rotation[0] = static_cast<float>(a.as_int(0));
        room->rotation[1] = a.as_float(1);
    };
    fns["mgFogColor"] = [this](const ScriptArgs& a, ScriptRet&) {
        // (r, g, b [, r2, g2, b2]): the second colour is a copy of the first when only three are given.
        for (int i = 0; i < 3; ++i) room->fog[0][i] = a.as_float(i);
        for (int i = 0; i < 3; ++i) room->fog[1][i] = a.size() > 3 ? a.as_float(3 + i) : room->fog[0][i];
    };
    fns["mgGlow"] = [this](const ScriptArgs& a, ScriptRet&) { glow = {a.as_float(0), a.as_float(1), a.as_float(2)}; };
    // Obstacle state, camera, sounds.
    fns["mgCameraShake"] = [this](const ScriptArgs& a, ScriptRet&) {  // at most 2; raises the shake, never lowers it
        float v = a.as_float(0);
        if (v > 2.0f) v = 2.0f;
        if (level_->camera_shake && *level_->camera_shake <= v) *level_->camera_shake = v;
    };
    fns["mgTime"] = [this](const ScriptArgs&, ScriptRet& r) { r.add_float(level_->time); };
    fns["mgGetTimeStep"] = [this](const ScriptArgs&, ScriptRet& r) { r.add_float(level_->time_step); };
    fns["mgGetCameraSpeed"] = [this](const ScriptArgs&, ScriptRet& r) { r.add_float(level_->camera_speed); };
    fns["mgObstacleTime"] = [this](const ScriptArgs&, ScriptRet& r) {
        r.add_float(obstacle_.time ? *obstacle_.time : 0.0f);
    };
    fns["mgGetGravity"] = [this](const ScriptArgs&, ScriptRet& r) {  // the current room's, not this script's
        r.add_float(level_->current_room ? level_->current_room->gravity * 10.0f : 0.0f);
    };
    fns["mgHasBoss"] = [this](const ScriptArgs&, ScriptRet& r) { r.add_bool(room && room->has_boss()); };
    fns["mgIsClosestBoss"] = [this](const ScriptArgs&, ScriptRet& r) {
        r.add_bool(room && room->closest_boss() == obstacle_.id);
    };
    fns["mgGetCameraPos"] = [this](const ScriptArgs&, ScriptRet& r) {  // the camera in the obstacle's frame
        const Vec3 d = phys::qi_rotate_inverse(obstacle_.rotation, level_->camera_position - obstacle_.position);
        r.add_float(d.x), r.add_float(d.y), r.add_float(d.z);
    };
    fns["mgGetBeatFloat"] = [this](const ScriptArgs& a, ScriptRet& r) {
        r.add_float(level_->beat && room ? level_->beat(*room, a.as_float(0)) : 0.0f);
    };
    fns["mgGetBeat"] = [this](const ScriptArgs& a, ScriptRet& r) {
        const float b = level_->beat && room ? level_->beat(*room, a.as_float(0)) : 0.0f;
        r.add_int(static_cast<int>(std::floor(b)));
    };
    fns["mgGet"] = [this](const ScriptArgs& a, ScriptRet& r) {
        r.add_string(level_->get_property ? level_->get_property(a.as_string(0)) : std::string());
    };
    fns["mgSet"] = [this](const ScriptArgs& a, ScriptRet&) {
        if (level_->set_property) level_->set_property(a.as_string(0), a.as_string(1));
    };
    fns["mgCameraDistance"] = [this](const ScriptArgs&, ScriptRet& r) {
        // The camera's distance from the obstacle, computed in the obstacle's frame.
        const Vec3 d = phys::qi_rotate_inverse(obstacle_.rotation, level_->camera_position - obstacle_.position);
        r.add_float(std::sqrt((d.y * d.y + d.x * d.x) + d.z * d.z));
    };
    fns["mgIsCleared"] = [this](const ScriptArgs&, ScriptRet& r) { r.add_bool(obstacle_.state->cleared != 0); };
    fns["mgScore"] = [this](const ScriptArgs& a, ScriptRet&) {
        // (balls [, x, y, z]): the score for clearing the obstacle, and where it is shown.
        obstacle_.state->score = a.as_int(0);
        if (a.size() > 1) obstacle_.state->score_position = {a.as_float(1), a.as_float(2), a.as_float(3)};
    };
    fns["mgClear"] = [this](const ScriptArgs&, ScriptRet&) {
        obstacle_.state->cleared = level_->time;
        if (obstacle_.state->score > 0 && level_->on_clear) level_->on_clear(*obstacle_.state, obstacle_.id);
    };
    fns["mgCreateSound"] = [this](const ScriptArgs& a, ScriptRet& r) {
        // ResMan::acquireSound(name), kept in the script's list; returns its index.
        sounds.push_back(a.as_string(0));
        r.add_int(static_cast<int>(sounds.size()) - 1);
    };
    fns["mgPlaySound"] = [this](const ScriptArgs& a, ScriptRet&) {
        // (index [, volume [, pitch]]) at the obstacle's position; volume 0..10 (default 1), pitch 0.01..100
        // (default 1). Bad indices do nothing.
        const int i = a.as_int(0);
        if (i < 0 || i >= static_cast<int>(sounds.size())) return;
        float volume = 1, pitch = 1;
        if (a.size() > 1) {
            const float v = a.as_float(1);
            volume = !(v > 0) ? 0.0f : (v < 10.0f ? v : 10.0f);
        }
        if (a.size() > 2) {
            const float p = a.as_float(2);
            pitch = !(p > 0.01f) ? 0.01f : (p < 100.0f ? p : 100.0f);
        }
        level_->sound_events.push_back({sounds[static_cast<size_t>(i)], -1, obstacle_.position, true, volume, pitch});
    };
    // Physics queries.
    fns["mgCreateButton"] = [this](const ScriptArgs& a, ScriptRet& r) {  // (x, y, z) in the obstacle's frame
        const Vec3 p = phys::qi_transform(obstacle_.position, obstacle_.rotation, {a.as_float(0), a.as_float(1), a.as_float(2)});
        phys::ButtonEntity* b = level_->create_button ? level_->create_button(p, obstacle_.entities, obstacle_.id) : nullptr;
        r.add_int(b ? level_->handles.add(b) : 0);
    };
    fns["mgIsButtonDown"] = [this](const ScriptArgs& a, ScriptRet& r) {  // (button [, just pressed])
        const phys::ButtonEntity* b = level_->handles.get<phys::ButtonEntity>(a.as_handle(0));
        const bool edge = a.size() > 1 && a.as_bool(1);
        r.add_bool(b && (edge ? b->down && !b->was_down : b->down));
    };
    fns["mgLaser"] = [this](const ScriptArgs& a, ScriptRet&) {  // (x, y, z, dx, dy, dz) in the obstacle's frame
        const Vec3 from = phys::qi_transform(obstacle_.position, obstacle_.rotation, {a.as_float(0), a.as_float(1), a.as_float(2)});
        const Vec3 dir = phys::qi_rotate(obstacle_.rotation, {a.as_float(3), a.as_float(4), a.as_float(5)});
        if (level_->add_laser) level_->add_laser(from, dir);
    };
    fns["mgSmoke"] = [this](const ScriptArgs& a, ScriptRet&) {  // (x, y, z, vx, vy, vz) in the obstacle's frame
        const Vec3 pos = phys::qi_transform(obstacle_.position, obstacle_.rotation, {a.as_float(0), a.as_float(1), a.as_float(2)});
        const Vec3 vel = phys::qi_rotate(obstacle_.rotation, {a.as_float(3), a.as_float(4), a.as_float(5)});
        if (level_->smoke) level_->smoke(pos, vel);
    };
    fns["mgCanPass"] = [this](const ScriptArgs&, ScriptRet& r) {
        // Whether the line one unit above the obstacle along its own z axis ((0, 1, 10) to (0, 1, -10) in its frame)
        // misses all of its bodies.
        const Vec3 from = phys::qi_transform(obstacle_.position, obstacle_.rotation, {0.0f, 1.0f, 10.0f});
        const Vec3 to = phys::qi_transform(obstacle_.position, obstacle_.rotation, {0.0f, 1.0f, -10.0f});
        r.add_bool(!level_->world->raycast(from, to, 0xff, nullptr, nullptr, nullptr, obstacle_.id));
    };
    fns["mgRaycast"] = [this](const ScriptArgs& a, ScriptRet& r) {
        // (from xyz, to xyz [, mask = 0xff]) in the obstacle's frame: hit, then point + normal * 0.01 and the
        // normal, both back in the obstacle's frame. Without a hit the point is `to` and the normal zero.
        const Vec3 from = phys::qi_transform(obstacle_.position, obstacle_.rotation, {a.as_float(0), a.as_float(1), a.as_float(2)});
        const Vec3 to = phys::qi_transform(obstacle_.position, obstacle_.rotation, {a.as_float(3), a.as_float(4), a.as_float(5)});
        const int mask = a.size() > 6 ? a.as_int(6) : 0xff;
        Vec3 point = to, normal{0, 0, 0};
        const bool hit = level_->world->raycast(from, to, static_cast<std::uint8_t>(mask), &point, &normal, nullptr, nullptr);
        const Vec3 p = phys::qi_rotate_inverse(obstacle_.rotation, point - obstacle_.position);
        const Vec3 n = phys::qi_rotate_inverse(obstacle_.rotation, normal);
        r.add_bool(hit);
        r.add_float(p.x + n.x * 0.01f), r.add_float(p.y + n.y * 0.01f), r.add_float(p.z + n.z * 0.01f);
        r.add_float(n.x), r.add_float(n.y), r.add_float(n.z);
    };
    fns["mgBoxOverlap"] = [this](const ScriptArgs& a, ScriptRet& r) {
        // (centre xyz, half extents xyz): the centre is placed by the room body's pose (not the obstacle's), the
        // half extents shrink by 0.02, then Physics::collideBox.
        const phys::RigidBody* room_body = room->body;
        const Vec3 c = phys::qi_transform(room_body->position, room_body->rotation, {a.as_float(0), a.as_float(1), a.as_float(2)});
        const Vec3 h{a.as_float(3) - 0.02f, a.as_float(4) - 0.02f, a.as_float(5) - 0.02f};
        r.add_bool(level_->world->collide_box(c, h));
    };
    for (const std::string& name : level_binding_names()) {
        auto it = fns.find(name);
        if (it != fns.end()) {
            host_.register_function(name, std::move(it->second));
        } else {
            host_.register_function(name, [this, name](const ScriptArgs&, ScriptRet&) { ++stub_calls_[name]; });
        }
    }
}

}  // namespace sh::script