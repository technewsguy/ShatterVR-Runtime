#include "audio/room_effects.hpp"

#include <algorithm>
#include <cmath>

namespace sh::audio {

namespace {

constexpr float kRateF = 44100.0f;

int wrap(int i, int len) { return len > 0 ? i % len : 0; }  // (a length of 0 would divide by zero in the original)

float clamp01(float v) { return v > 0.0f ? (v < 1.0f ? v : 1.0f) : 0.0f; }

}  // namespace

void MonoFlanger::process(float* s, int n) {
    for (int i = 0; i < n; ++i) {
        pos_ = (pos_ + 1) % 0x1000;
        if (!on) {
            ring_[static_cast<size_t>(pos_)] = s[i];
            continue;
        }
        const float x = s[i];
        const int delay = static_cast<int>(std::floor(5.0f + depth * 120.0f));
        const float out = ring_[static_cast<size_t>((pos_ + 0x1000 - delay) % 0x1000)] * 0.9f + x * 0.5f;
        s[i] = out;
        ring_[static_cast<size_t>(pos_)] = out * 0.6f + x * 0.4f;
    }
}

void MonoLowPass::process(float* s, int n) {
    if (mix * coef_ <= 0.0f) return;
    for (int i = 0; i < n; ++i) {
        y_ = s[i] * (1.0f - coef_) + coef_ * y_;
        s[i] = y_ * mix + (1.0f - mix) * s[i];
    }
}

MonoEcho::MonoEcho(float wet, float delay, float feedback, float damping)
    : wet_(wet), feedback_(feedback), damping_(damping), buf_(44100, 0.0f) {
    const int len = static_cast<int>(delay * kRateF);
    len_ = len > 0 ? std::min(len, 44100) : 0;
}

void MonoEcho::process(float* s, int n) {
    if (!(mix * wet_ > 0.0f)) return;
    for (int i = 0; i < n; ++i) {
        // A damped tap from the delay line, written back with the input and the feedback one sample behind.
        f_ = (1.0f - damping_) * buf_[static_cast<size_t>(idx_)] + damping_ * f_;
        const float x = s[i];
        buf_[static_cast<size_t>(wrap(idx_ + len_ - 1, len_))] = (f_ + x) * feedback_;
        idx_ = wrap(idx_ + 1, len_);
        s[i] = s[i] + f_ * mix * wet_;
    }
}

MonoReverb::MonoReverb(float wet, float size, float damping, const float combs[4]) : wet_(wet) {
    if (!(wet > 0.0f)) return;
    on_ = true;
    const float decay = size * 4.5f, pre = size * 0.1f;
    const float comb_s[4] = {0.058f * combs[0], 0.065f * combs[1], 0.071f * combs[2], 0.077f * combs[3]};
    const float damp[4] = {damping, damping * 1.17f, damping * 1.43f, damping * 1.64f};
    for (int k = 0; k < 4; ++k) {
        comb_len_[k] = static_cast<int>(comb_s[k] * kRateF);
        comb_fb_[k] = std::pow(0.001f, comb_s[k] / decay);
        comb_damp_[k] = clamp01(damp[k]);
        comb_[k].assign(8000, 0.0f);
    }
    const float ap_s[2] = {0.0071f, 0.0053f};
    for (int k = 0; k < 2; ++k) {
        ap_len_[k] = static_cast<int>(ap_s[k] * kRateF);
        ap_g_[k] = std::pow(0.001f, ap_s[k] / 0.1f);
        ap_[k].assign(6000, 0.0f);
    }
    const float gains[4] = {0.84f, 0.49f, 0.38f, 0.29f};
    for (int k = 0; k < 4; ++k) tap_gain_[k] = 0.3f * gains[k];
    early_.assign(0x3fff, 0.0f);
    const int p = static_cast<int>(pre * kRateF);
    pre_len_ = p > 0 ? std::min(p, 0x2000) : 0;
    pre_.assign(0x2000, 0.0f);
}

void MonoReverb::process(float* s, int n) {
    if (!(mix * wet_ > 0.0f) || !on_) return;
    for (int i = 0; i < n; ++i) {
        const float x = s[i];
        // Early reflections: the input goes into four later slots of the tap line; this slot is read, the one
        // before it cleared.
        for (int k = 0; k < 4; ++k) early_[static_cast<size_t>((early_pos_ + kTaps[k]) % 0x3fff)] += x * tap_gain_[k];
        const float y = x + early_[static_cast<size_t>(early_pos_)];
        early_[static_cast<size_t>((early_pos_ + 0x3ffe) % 0x3fff)] = 0.0f;
        early_pos_ = (early_pos_ + 1) % 0x3fff;
        // Four damped feedback combs, their outputs (as read) averaged.
        float sum = 0.0f;
        for (int k = 0; k < 4; ++k) {
            float& slot = comb_[k][static_cast<size_t>(comb_pos_[k])];
            const float c = slot;
            comb_state_[k] = (1.0f - comb_damp_[k]) * (y + c * comb_fb_[k]) + comb_damp_[k] * comb_state_[k];
            slot = comb_state_[k];
            comb_pos_[k] = wrap(comb_pos_[k] + 1, comb_len_[k]);
            sum = sum + c * 0.25f;
        }
        // Two all-passes.
        float& a0 = ap_[0][static_cast<size_t>(ap_pos_[0])];
        const float t = a0 - ap_g_[0] * sum;
        a0 = sum + a0 * ap_g_[0];
        ap_pos_[0] = wrap(ap_pos_[0] + 1, ap_len_[0]);
        float& a1 = ap_[1][static_cast<size_t>(ap_pos_[1])];
        const float b = a1;
        a1 = t + b * ap_g_[1];
        const float out = b - t * ap_g_[1];
        ap_pos_[1] = wrap(ap_pos_[1] + 1, ap_len_[1]);
        // The pre-delay, then the wet signal over the dry one (with its early reflections).
        float z;
        if (pre_len_ > 0) {
            pre_[static_cast<size_t>((pre_pos_ + pre_len_ - 1) % pre_len_)] = out;
            z = pre_[static_cast<size_t>(pre_pos_)];
            pre_pos_ = (pre_pos_ + 1) % pre_len_;
        } else {
            z = out;
        }
        s[i] = (y + z * wet_) * mix + (1.0f - mix) * x;
    }
}

RoomEffects::RoomEffects(const RoomEffectParams& p)
    : lp_{MonoLowPass(p.low_pass), MonoLowPass(p.low_pass)},
      echo_{MonoEcho(p.echo[0], p.echo[1], p.echo[2], p.echo[3]), MonoEcho(p.echo[0], p.echo[1], p.echo[2], p.echo[3])},
      reverb_{MonoReverb(p.reverb[0], p.reverb[1], p.reverb[2], p.combs),
              MonoReverb(p.reverb[0], p.reverb[1], p.reverb[2], p.combs + 4)} {}

void RoomEffects::process(float* left, float* right, int n) {
    lp_[0].mix = lp_[1].mix = mix;
    lp_[0].process(left, n);
    lp_[1].process(right, n);
    echo_[0].mix = echo_[1].mix = mix;
    echo_[0].process(left, n);
    echo_[1].process(right, n);
    reverb_[0].process(left, n);
    reverb_[0].mix = mix;
    reverb_[1].process(right, n);
    reverb_[1].mix = mix;
}

}  // namespace sh::audio
