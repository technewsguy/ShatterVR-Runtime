#include "play/xr.hpp"

#include "render/gl.hpp"

#include <windows.h>
#include <unknwn.h>
#include <GL/gl.h>
#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_OPENGL
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace sh::play {

namespace {

bool ok(XrResult r) { return XR_SUCCEEDED(r); }

VrPose to_pose(const XrPosef& p, bool valid) {
    VrPose o;
    o.valid = valid;
    o.position = {p.position.x, p.position.y, p.position.z};
    o.orientation = {p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
    return o;
}

}  // namespace

struct XrRig::Impl {
    XrInstance instance = XR_NULL_HANDLE;
    XrSystemId system = XR_NULL_SYSTEM_ID;
    XrSession session = XR_NULL_HANDLE;
    XrSpace local = XR_NULL_HANDLE, view = XR_NULL_HANDLE;
    XrSessionState state = XR_SESSION_STATE_UNKNOWN;
    bool running = false, exit = false;
    XrTime display_time = 0;

    struct Eye {
        XrSwapchain swapchain = XR_NULL_HANDLE;
        int width = 0, height = 0;
        std::vector<XrSwapchainImageOpenGLKHR> images;
        gl::GLuint fbo = 0, depth = 0;
    };
    Eye eyes[2];
    XrView views[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}};

    XrActionSet actions = XR_NULL_HANDLE;
    XrAction throw_action = XR_NULL_HANDLE, back_action = XR_NULL_HANDLE, aim_action = XR_NULL_HANDLE;
    XrAction recenter_action = XR_NULL_HANDLE, haptic_action = XR_NULL_HANDLE;
    // recenter(): poses are handed out as heading_inv * (p - centre), heading_inv * q.
    phys::Quat heading_inv{0, 0, 0, 1};
    phys::Vec3 centre{0, 0, 0};
    VrPose raw_head;
    XrPath hands[2] = {XR_NULL_PATH, XR_NULL_PATH};
    XrSpace aim_space[2] = {XR_NULL_HANDLE, XR_NULL_HANDLE};
    std::string runtime;

    XrPath path(const char* s) const {
        XrPath p = XR_NULL_PATH;
        xrStringToPath(instance, s, &p);
        return p;
    }

    bool fail(std::string& error, const char* what, XrResult r) {
        char buf[64] = "";
        if (instance) xrResultToString(instance, r, buf);
        error = std::string(what) + " failed (" + (buf[0] ? buf : std::to_string(static_cast<int>(r))) + ")";
        return false;
    }

    bool init(std::string& error) {
        // The instance with the OpenGL binding extension.
        XrInstanceCreateInfo ici{XR_TYPE_INSTANCE_CREATE_INFO};
        std::strcpy(ici.applicationInfo.applicationName, "sh_play");
        ici.applicationInfo.applicationVersion = 1;
        std::strcpy(ici.applicationInfo.engineName, "shattervr");
        ici.applicationInfo.apiVersion = XR_MAKE_VERSION(1, 0, 0);
        const char* exts[] = {XR_KHR_OPENGL_ENABLE_EXTENSION_NAME};
        ici.enabledExtensionCount = 1;
        ici.enabledExtensionNames = exts;
        if (XrResult r = xrCreateInstance(&ici, &instance); !ok(r)) return fail(error, "xrCreateInstance", r);
        XrInstanceProperties ip{XR_TYPE_INSTANCE_PROPERTIES};
        if (ok(xrGetInstanceProperties(instance, &ip))) runtime = ip.runtimeName;

        XrSystemGetInfo sgi{XR_TYPE_SYSTEM_GET_INFO};
        sgi.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
        if (XrResult r = xrGetSystem(instance, &sgi, &system); !ok(r)) return fail(error, "xrGetSystem (is a headset connected?)", r);

        // The runtime insists on this call before the session.
        PFN_xrGetOpenGLGraphicsRequirementsKHR get_reqs = nullptr;
        xrGetInstanceProcAddr(instance, "xrGetOpenGLGraphicsRequirementsKHR", reinterpret_cast<PFN_xrVoidFunction*>(&get_reqs));
        XrGraphicsRequirementsOpenGLKHR reqs{XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR};
        if (!get_reqs) return fail(error, "xrGetOpenGLGraphicsRequirementsKHR lookup", XR_ERROR_FUNCTION_UNSUPPORTED);
        if (XrResult r = get_reqs(instance, system, &reqs); !ok(r)) return fail(error, "xrGetOpenGLGraphicsRequirementsKHR", r);

        XrGraphicsBindingOpenGLWin32KHR binding{XR_TYPE_GRAPHICS_BINDING_OPENGL_WIN32_KHR};
        binding.hDC = wglGetCurrentDC();
        binding.hGLRC = wglGetCurrentContext();
        if (!binding.hDC || !binding.hGLRC) {
            error = "no current OpenGL context";
            return false;
        }
        XrSessionCreateInfo sci{XR_TYPE_SESSION_CREATE_INFO};
        sci.next = &binding;
        sci.systemId = system;
        if (XrResult r = xrCreateSession(instance, &sci, &session); !ok(r)) return fail(error, "xrCreateSession", r);

        XrReferenceSpaceCreateInfo rsi{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
        rsi.poseInReferenceSpace.orientation.w = 1;
        rsi.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
        if (XrResult r = xrCreateReferenceSpace(session, &rsi, &local); !ok(r)) return fail(error, "LOCAL space", r);
        rsi.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
        if (XrResult r = xrCreateReferenceSpace(session, &rsi, &view); !ok(r)) return fail(error, "VIEW space", r);

        if (!make_swapchains(error) || !make_actions(error)) return false;
        return true;
    }

    bool make_swapchains(std::string& error) {
        uint32_t n = 0;
        xrEnumerateViewConfigurationViews(instance, system, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0, &n, nullptr);
        std::vector<XrViewConfigurationView> cfg(n, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
        if (XrResult r = xrEnumerateViewConfigurationViews(instance, system, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, n, &n,
                                                           cfg.data());
            !ok(r) || n < 2)
            return fail(error, "stereo view configuration", ok(r) ? XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED : r);
        // The game's colours are display values: write them into an 8-bit format as they are (no sRGB conversion
        // on our side). Plain RGBA8 when offered, else the sRGB format, still written without conversion.
        uint32_t fc = 0;
        xrEnumerateSwapchainFormats(session, 0, &fc, nullptr);
        std::vector<int64_t> formats(fc);
        xrEnumerateSwapchainFormats(session, fc, &fc, formats.data());
        int64_t format = 0;
        for (int64_t want : {static_cast<int64_t>(gl::RGBA8), static_cast<int64_t>(gl::SRGB8_ALPHA8)})
            if (!format && std::find(formats.begin(), formats.end(), want) != formats.end()) format = want;
        if (!format && !formats.empty()) format = formats.front();
        for (int e = 0; e < 2; ++e) {
            Eye& eye = eyes[e];
            eye.width = static_cast<int>(cfg[static_cast<size_t>(e)].recommendedImageRectWidth);
            eye.height = static_cast<int>(cfg[static_cast<size_t>(e)].recommendedImageRectHeight);
            XrSwapchainCreateInfo ci{XR_TYPE_SWAPCHAIN_CREATE_INFO};
            ci.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
            ci.format = format;
            ci.sampleCount = 1;
            ci.width = static_cast<uint32_t>(eye.width);
            ci.height = static_cast<uint32_t>(eye.height);
            ci.faceCount = 1, ci.arraySize = 1, ci.mipCount = 1;
            if (XrResult r = xrCreateSwapchain(session, &ci, &eye.swapchain); !ok(r)) return fail(error, "xrCreateSwapchain", r);
            uint32_t ic = 0;
            xrEnumerateSwapchainImages(eye.swapchain, 0, &ic, nullptr);
            eye.images.assign(ic, {XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR});
            xrEnumerateSwapchainImages(eye.swapchain, ic, &ic, reinterpret_cast<XrSwapchainImageBaseHeader*>(eye.images.data()));
            gl::GenFramebuffers(1, &eye.fbo);
            gl::GenRenderbuffers(1, &eye.depth);
            gl::BindRenderbuffer(gl::RENDERBUFFER, eye.depth);
            gl::RenderbufferStorage(gl::RENDERBUFFER, gl::DEPTH24_STENCIL8, eye.width, eye.height);
        }
        return true;
    }

    bool make_actions(std::string& error) {
        XrActionSetCreateInfo asi{XR_TYPE_ACTION_SET_CREATE_INFO};
        std::strcpy(asi.actionSetName, "gameplay");
        std::strcpy(asi.localizedActionSetName, "Gameplay");
        if (XrResult r = xrCreateActionSet(instance, &asi, &actions); !ok(r)) return fail(error, "xrCreateActionSet", r);
        hands[0] = path("/user/hand/left"), hands[1] = path("/user/hand/right");
        auto action = [&](const char* name, const char* label, XrActionType type, XrAction& out) {
            XrActionCreateInfo ai{XR_TYPE_ACTION_CREATE_INFO};
            std::strcpy(ai.actionName, name);
            std::strcpy(ai.localizedActionName, label);
            ai.actionType = type;
            ai.countSubactionPaths = 2;
            ai.subactionPaths = hands;
            return xrCreateAction(actions, &ai, &out);
        };
        if (XrResult r = action("throw", "Throw a ball", XR_ACTION_TYPE_BOOLEAN_INPUT, throw_action); !ok(r)) return fail(error, "throw action", r);
        if (XrResult r = action("back", "Back / pause", XR_ACTION_TYPE_BOOLEAN_INPUT, back_action); !ok(r)) return fail(error, "back action", r);
        if (XrResult r = action("aim", "Aim", XR_ACTION_TYPE_POSE_INPUT, aim_action); !ok(r)) return fail(error, "aim action", r);
        if (XrResult r = action("recenter", "Recentre the view", XR_ACTION_TYPE_BOOLEAN_INPUT, recenter_action); !ok(r))
            return fail(error, "recenter action", r);
        if (XrResult r = action("buzz", "Vibration", XR_ACTION_TYPE_VIBRATION_OUTPUT, haptic_action); !ok(r))
            return fail(error, "vibration action", r);

        // Suggested bindings per controller family; a runtime takes the ones it knows.
        // Every profile also gets the vibration output on both hands and, where it has one, the recentre click.
        struct Profile {
            const char* profile;
            std::vector<std::pair<XrAction*, const char*>> binds;
            const char* recenter = nullptr;  // the input under /user/hand/*/input/
        };
        const Profile profiles[] = {
            {"/interaction_profiles/khr/simple_controller",
             {{&throw_action, "/user/hand/left/input/select/click"}, {&throw_action, "/user/hand/right/input/select/click"},
              {&back_action, "/user/hand/left/input/menu/click"}, {&back_action, "/user/hand/right/input/menu/click"},
              {&aim_action, "/user/hand/left/input/aim/pose"}, {&aim_action, "/user/hand/right/input/aim/pose"}},
             nullptr},
            {"/interaction_profiles/oculus/touch_controller",
             {{&throw_action, "/user/hand/left/input/trigger/value"}, {&throw_action, "/user/hand/right/input/trigger/value"},
              {&back_action, "/user/hand/left/input/menu/click"}, {&back_action, "/user/hand/right/input/b/click"},
              {&aim_action, "/user/hand/left/input/aim/pose"}, {&aim_action, "/user/hand/right/input/aim/pose"}},
             "thumbstick/click"},
            {"/interaction_profiles/valve/index_controller",
             {{&throw_action, "/user/hand/left/input/trigger/value"}, {&throw_action, "/user/hand/right/input/trigger/value"},
              {&back_action, "/user/hand/left/input/b/click"}, {&back_action, "/user/hand/right/input/b/click"},
              {&aim_action, "/user/hand/left/input/aim/pose"}, {&aim_action, "/user/hand/right/input/aim/pose"}},
             "thumbstick/click"},
            {"/interaction_profiles/htc/vive_controller",
             {{&throw_action, "/user/hand/left/input/trigger/click"}, {&throw_action, "/user/hand/right/input/trigger/click"},
              {&back_action, "/user/hand/left/input/menu/click"}, {&back_action, "/user/hand/right/input/menu/click"},
              {&aim_action, "/user/hand/left/input/aim/pose"}, {&aim_action, "/user/hand/right/input/aim/pose"}},
             "trackpad/click"},
            {"/interaction_profiles/microsoft/motion_controller",
             {{&throw_action, "/user/hand/left/input/trigger/value"}, {&throw_action, "/user/hand/right/input/trigger/value"},
              {&back_action, "/user/hand/left/input/menu/click"}, {&back_action, "/user/hand/right/input/menu/click"},
              {&aim_action, "/user/hand/left/input/aim/pose"}, {&aim_action, "/user/hand/right/input/aim/pose"}},
             "thumbstick/click"},
        };
        for (const Profile& p : profiles) {
            std::vector<XrActionSuggestedBinding> b;
            for (const auto& [a, s] : p.binds) b.push_back({*a, path(s)});
            for (const char* hand : {"/user/hand/left", "/user/hand/right"}) {
                b.push_back({haptic_action, path((std::string(hand) + "/output/haptic").c_str())});
                if (p.recenter) b.push_back({recenter_action, path((std::string(hand) + "/input/" + p.recenter).c_str())});
            }
            XrInteractionProfileSuggestedBinding sb{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
            sb.interactionProfile = path(p.profile);
            sb.countSuggestedBindings = static_cast<uint32_t>(b.size());
            sb.suggestedBindings = b.data();
            if (XrResult r = xrSuggestInteractionProfileBindings(instance, &sb); !ok(r))
                std::fprintf(stderr, "openxr: bindings for %s not taken (%d)\n", p.profile, static_cast<int>(r));
        }
        for (int h = 0; h < 2; ++h) {
            XrActionSpaceCreateInfo si{XR_TYPE_ACTION_SPACE_CREATE_INFO};
            si.action = aim_action;
            si.subactionPath = hands[h];
            si.poseInActionSpace.orientation.w = 1;
            if (XrResult r = xrCreateActionSpace(session, &si, &aim_space[h]); !ok(r)) return fail(error, "aim space", r);
        }
        XrSessionActionSetsAttachInfo at{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
        at.countActionSets = 1;
        at.actionSets = &actions;
        if (XrResult r = xrAttachSessionActionSets(session, &at); !ok(r)) return fail(error, "xrAttachSessionActionSets", r);
        return true;
    }

    bool poll() {
        XrEventDataBuffer ev{XR_TYPE_EVENT_DATA_BUFFER};
        while (xrPollEvent(instance, &ev) == XR_SUCCESS) {
            if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
                state = reinterpret_cast<XrEventDataSessionStateChanged&>(ev).state;
                static const char* const names[] = {"unknown", "idle", "ready", "synchronized", "visible", "focused",
                                                    "stopping", "loss pending", "exiting"};
                std::printf("openxr: session %s\n", state >= 0 && state <= 8 ? names[state] : "?");
                if (state == XR_SESSION_STATE_READY) {
                    XrSessionBeginInfo bi{XR_TYPE_SESSION_BEGIN_INFO};
                    bi.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
                    running = ok(xrBeginSession(session, &bi));
                } else if (state == XR_SESSION_STATE_STOPPING) {
                    xrEndSession(session);
                    running = false;
                } else if (state == XR_SESSION_STATE_EXITING || state == XR_SESSION_STATE_LOSS_PENDING) {
                    exit = true;
                }
            } else if (ev.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) {
                exit = true;
            }
            ev = {XR_TYPE_EVENT_DATA_BUFFER};
        }
        return !exit;
    }

    bool bool_action(XrAction a, int hand, bool& changed) {
        XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO};
        gi.action = a;
        gi.subactionPath = hands[hand];
        XrActionStateBoolean s{XR_TYPE_ACTION_STATE_BOOLEAN};
        if (!ok(xrGetActionStateBoolean(session, &gi, &s)) || !s.isActive) {
            changed = false;
            return false;
        }
        changed = s.changedSinceLastSync != XR_FALSE;
        return s.currentState != XR_FALSE;
    }

    bool begin_frame(VrFrame& out) {
        out = {};
        XrFrameWaitInfo wi{XR_TYPE_FRAME_WAIT_INFO};
        XrFrameState fs{XR_TYPE_FRAME_STATE};
        if (!ok(xrWaitFrame(session, &wi, &fs))) return false;
        XrFrameBeginInfo bi{XR_TYPE_FRAME_BEGIN_INFO};
        if (!ok(xrBeginFrame(session, &bi))) return false;
        display_time = fs.predictedDisplayTime;
        out.render = fs.shouldRender != XR_FALSE;

        XrViewLocateInfo li{XR_TYPE_VIEW_LOCATE_INFO};
        li.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
        li.displayTime = display_time;
        li.space = local;
        XrViewState vs{XR_TYPE_VIEW_STATE};
        uint32_t n = 0;
        views[0] = {XR_TYPE_VIEW}, views[1] = {XR_TYPE_VIEW};
        const bool located = ok(xrLocateViews(session, &li, &vs, 2, &n, views)) && n == 2 &&
                             (vs.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT);
        if (!located) out.render = false;
        for (int e = 0; e < 2; ++e) {
            out.eyes[e].pose = rebase(to_pose(views[e].pose, located));
            out.eyes[e].angle_left = views[e].fov.angleLeft, out.eyes[e].angle_right = views[e].fov.angleRight;
            out.eyes[e].angle_up = views[e].fov.angleUp, out.eyes[e].angle_down = views[e].fov.angleDown;
        }
        XrSpaceLocation loc{XR_TYPE_SPACE_LOCATION};
        if (ok(xrLocateSpace(view, local, display_time, &loc)))
            raw_head = to_pose(loc.pose, (loc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0), out.head = rebase(raw_head);

        if (running && state == XR_SESSION_STATE_FOCUSED) {
            XrActiveActionSet active{actions, XR_NULL_PATH};
            XrActionsSyncInfo si{XR_TYPE_ACTIONS_SYNC_INFO};
            si.countActiveActionSets = 1;
            si.activeActionSets = &active;
            xrSyncActions(session, &si);
            for (int h = 0; h < 2; ++h) {
                bool changed = false;
                out.throw_down[h] = bool_action(throw_action, h, changed);
                out.throw_pressed[h] = out.throw_down[h] && changed;
                const bool back = bool_action(back_action, h, changed);
                if (back && changed) out.back_pressed = true;
                const bool centre_click = bool_action(recenter_action, h, changed);
                if (centre_click && changed) out.recenter_pressed = true;
                XrSpaceLocation al{XR_TYPE_SPACE_LOCATION};
                if (ok(xrLocateSpace(aim_space[h], local, display_time, &al))) {
                    const XrSpaceLocationFlags need = XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_POSITION_VALID_BIT;
                    out.aim[h] = rebase(to_pose(al.pose, (al.locationFlags & need) == need));
                }
            }
        }
        return true;
    }

    VrPose rebase(VrPose p) const {
        const phys::Vec3 d{p.position.x - centre.x, p.position.y - centre.y, p.position.z - centre.z};
        p.position = phys::rotate(heading_inv, d);
        p.orientation = heading_inv * p.orientation;
        return p;
    }

    void recenter() {
        if (!raw_head.valid) return;
        // The heading: the head's -z projected on the floor plane.
        const phys::Vec3 f = phys::rotate(raw_head.orientation, {0, 0, -1});
        const float yaw = std::atan2(-f.x, -f.z);  // about +y, 0 when looking along -z
        heading_inv = phys::axis_angle({0, 1, 0}, -yaw);
        centre = raw_head.position;
    }

    void vibrate(int hand, float amplitude, float seconds) {
        if (!running || hand < 0 || hand > 1) return;
        XrHapticVibration v{XR_TYPE_HAPTIC_VIBRATION};
        v.amplitude = amplitude;
        v.duration = static_cast<XrDuration>(seconds * 1e9f);
        v.frequency = XR_FREQUENCY_UNSPECIFIED;
        XrHapticActionInfo hi{XR_TYPE_HAPTIC_ACTION_INFO};
        hi.action = haptic_action;
        hi.subactionPath = hands[hand];
        xrApplyHapticFeedback(session, &hi, reinterpret_cast<const XrHapticBaseHeader*>(&v));
    }

    unsigned acquire_eye(int e, int& w, int& h) {
        Eye& eye = eyes[e];
        uint32_t index = 0;
        XrSwapchainImageAcquireInfo ai{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
        xrAcquireSwapchainImage(eye.swapchain, &ai, &index);
        XrSwapchainImageWaitInfo wi{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
        wi.timeout = XR_INFINITE_DURATION;
        xrWaitSwapchainImage(eye.swapchain, &wi);
        gl::BindFramebuffer(gl::FRAMEBUFFER, eye.fbo);
        gl::FramebufferTexture2D(gl::FRAMEBUFFER, gl::COLOR_ATTACHMENT0, gl::TEXTURE_2D, eye.images[index].image, 0);
        gl::FramebufferRenderbuffer(gl::FRAMEBUFFER, gl::DEPTH_STENCIL_ATTACHMENT, gl::RENDERBUFFER, eye.depth);
        w = eye.width, h = eye.height;
        return eye.fbo;
    }

    void release_eye(int e) {
        gl::BindFramebuffer(gl::FRAMEBUFFER, 0);
        XrSwapchainImageReleaseInfo ri{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
        xrReleaseSwapchainImage(eyes[e].swapchain, &ri);
    }

    void end_frame(const VrFrame& s) {
        XrCompositionLayerProjectionView pv[2] = {{XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW},
                                                  {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW}};
        for (int e = 0; e < 2; ++e) {
            pv[e].pose = views[e].pose;
            pv[e].fov = views[e].fov;
            pv[e].subImage.swapchain = eyes[e].swapchain;
            pv[e].subImage.imageRect = {{0, 0}, {eyes[e].width, eyes[e].height}};
        }
        XrCompositionLayerProjection layer{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
        layer.space = local;
        layer.viewCount = 2;
        layer.views = pv;
        const XrCompositionLayerBaseHeader* layers[] = {reinterpret_cast<XrCompositionLayerBaseHeader*>(&layer)};
        XrFrameEndInfo ei{XR_TYPE_FRAME_END_INFO};
        ei.displayTime = display_time;
        ei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        ei.layerCount = s.render ? 1 : 0;
        ei.layers = layers;
        xrEndFrame(session, &ei);
    }

    ~Impl() {
        for (Eye& e : eyes) {
            if (e.fbo) gl::DeleteFramebuffers(1, &e.fbo);
            if (e.depth) gl::DeleteRenderbuffers(1, &e.depth);
            if (e.swapchain) xrDestroySwapchain(e.swapchain);
        }
        for (XrSpace s : aim_space)
            if (s) xrDestroySpace(s);
        if (actions) xrDestroyActionSet(actions);
        if (view) xrDestroySpace(view);
        if (local) xrDestroySpace(local);
        if (session) xrDestroySession(session);
        if (instance) xrDestroyInstance(instance);
    }
};

XrRig::XrRig() : impl_(std::make_unique<Impl>()) {}
XrRig::~XrRig() = default;
bool XrRig::init(std::string& error) { return impl_->init(error); }
bool XrRig::poll() { return impl_->poll(); }
bool XrRig::session_running() const { return impl_->running; }
bool XrRig::begin_frame(VrFrame& s) { return impl_->begin_frame(s); }
unsigned XrRig::acquire_eye(int eye, int& w, int& h) { return impl_->acquire_eye(eye, w, h); }
void XrRig::release_eye(int eye) { impl_->release_eye(eye); }
void XrRig::end_frame(const VrFrame& s) { impl_->end_frame(s); }
std::string XrRig::runtime_name() const { return impl_->runtime; }
void XrRig::vibrate(int hand, float amplitude, float seconds) { impl_->vibrate(hand, amplitude, seconds); }
void XrRig::recenter() { impl_->recenter(); }

}  // namespace sh::play
