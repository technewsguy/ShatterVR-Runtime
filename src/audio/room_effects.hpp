#pragma once

// The room's sound effects (Audio::setReverb's SoundFilter): a low pass, an echo and a reverb,
// each run on the left and right sound-effect mix in place (floats, -1 .. 1). Read from QiMonoLowPass, QiMonoEcho,
// QiMonoReverb and SoundFilter::process; (its random draws are made by the Level).

#include <memory>
#include <vector>

namespace sh::audio {

struct RoomEffectParams {
    float low_pass = 0;                 // mgLowPass
    float echo[4] = {0, 0, 0, 0};       // mgEcho: wet, delay (s), feedback, damping
    float reverb[3] = {0, 0, 0};        // mgReverb: wet, size (pre-delay / decay), damping
    float combs[8] = {1, 1, 1, 1, 1, 1, 1, 1};  // the eight QiRnd(0.8, 1.2) comb factors (left 4, right 4)
};

// QiMonoFlanger, on the level music channel: a 4096-sample ring; off it only records. On: delay = floor(5 + 120 depth),
// out = ring * 0.9 + x * 0.5, ring gets out * 0.6 + x * 0.4.
class MonoFlanger {
public:
    MonoFlanger() : ring_(0x1000, 0.0f) {}
    void process(float* s, int n);
    bool on = true;
    float depth = 1;

private:
    std::vector<float> ring_;
    int pos_ = 0;
};

class MonoLowPass {
public:
    explicit MonoLowPass(float coef) : coef_(coef) {}
    void process(float* s, int n);  // y = x (1 - c) + c y; out = y mix + x (1 - mix); nothing when mix * c <= 0
    float mix = 0;

private:
    float y_ = 0, coef_;
};

class MonoEcho {
public:
    MonoEcho(float wet, float delay, float feedback, float damping);
    void process(float* s, int n);
    float mix = 0;

private:
    int len_, idx_ = 0;
    float f_ = 0, wet_, feedback_, damping_;
    std::vector<float> buf_;  // one second
};

class MonoReverb {
public:
    // reverb arguments and this side's four comb factors; does nothing when wet <= 0.
    MonoReverb(float wet, float size, float damping, const float combs[4]);
    void process(float* s, int n);
    float mix = 0;

private:
    float wet_;
    bool on_ = false;
    std::vector<float> early_;  // 16383-sample tap line
    int early_pos_ = 0;
    static constexpr int kTaps[4] = {366, 992, 1190, 2019};
    float tap_gain_[4] = {};
    std::vector<float> comb_[4];
    int comb_len_[4] = {}, comb_pos_[4] = {};
    float comb_fb_[4] = {}, comb_damp_[4] = {}, comb_state_[4] = {};
    std::vector<float> ap_[2];
    int ap_len_[2] = {}, ap_pos_[2] = {};
    float ap_g_[2] = {};
    std::vector<float> pre_;  // pre-delay, 8192
    int pre_len_ = 0, pre_pos_ = 0;
};

// SoundFilter: low pass, echo and reverb on both sides, all at the filter's mix (the reverb takes a new mix one block
// late, as the original sets it after processing).
class RoomEffects {
public:
    explicit RoomEffects(const RoomEffectParams& p);
    void process(float* left, float* right, int n);
    float mix = 0;

private:
    MonoLowPass lp_[2];
    MonoEcho echo_[2];
    MonoReverb reverb_[2];
};

}  // namespace sh::audio
