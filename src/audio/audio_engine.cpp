#include "audio/audio_engine.hpp"

#include <SDL3/SDL.h>

#include <algorithm>
#include <cmath>

namespace sh::audio {

namespace {

// QiAudioChannel::fillBuffer's mixing: each contribution x (a sample times volume and pan gain over 32767) goes in
// as out + x - x |out|, a soft limit that keeps the sum within -1 .. 1.
inline void soft_add(float& out, float x) { out = (x + out) - x * std::fabs(out); }

void SDLCALL feed(void* user, SDL_AudioStream* stream, int additional, int) {
    auto* engine = static_cast<AudioEngine*>(user);
    std::int16_t buf[0x1000];  // 0x2000 bytes: the original's music chunk
    while (additional > 0) {
        const int n = std::min(additional, static_cast<int>(sizeof buf)) & ~3;
        if (n <= 0) break;
        engine->mix(buf, n);
        SDL_PutAudioStreamData(stream, buf, n);
        additional -= n;
    }
}

}  // namespace

AudioEngine::~AudioEngine() { close(); }

bool AudioEngine::open() {
    if (stream_) return true;
    if (!SDL_InitSubSystem(SDL_INIT_AUDIO)) return false;
    const SDL_AudioSpec spec{SDL_AUDIO_S16, 2, kRate};
    stream_ = SDL_OpenAudioDeviceStream(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, &spec, feed, this);
    if (!stream_) return false;
    SDL_ResumeAudioStreamDevice(stream_);
    return true;
}

void AudioEngine::close() {
    if (!stream_) return;
    SDL_DestroyAudioStream(stream_);
    stream_ = nullptr;
}

void AudioEngine::lock() {
    if (stream_) SDL_LockAudioStream(stream_);
}

void AudioEngine::unlock() {
    if (stream_) SDL_UnlockAudioStream(stream_);
}

void AudioEngine::play(std::shared_ptr<const Pcm> pcm, float volume, float pan, float pitch, bool positional) {
    if (!pcm || pcm->frames() == 0) return;
    lock();
    std::vector<Voice>& pool = voices_[positional ? 0 : 1];
    if (pool.size() >= (positional ? 16u : 4u)) pool.erase(pool.begin());
    pool.push_back({std::move(pcm), 0.0, volume, pan, pitch});
    unlock();
}

void AudioEngine::set_room_effects(const RoomEffectParams& params) {
    auto next = std::make_unique<RoomEffects>(params);  // (about a megabyte: made outside the mixer's lock)
    lock();
    old_effects_ = std::move(effects_);
    effects_ = std::move(next);
    effects_blend_ = 0.0f;
    unlock();
}

void AudioEngine::update_room_effects(float dt) {
    lock();
    if (old_effects_) {
        old_effects_->mix = 1.0f - effects_blend_;
        effects_->mix = effects_blend_;
        effects_blend_ = effects_blend_ + dt;
        if (!(effects_blend_ < 1.0f)) effects_blend_ = 1.0f, old_effects_.reset();
    }
    unlock();
}

void AudioEngine::set_flanger(bool on, float depth) {
    lock();
    for (MonoFlanger& f : flanger_) f.on = on, f.depth = depth;
    unlock();
}

void AudioEngine::set_mixers(float music, float flat, float sounds, bool sounds_on, float sounds_pitch) {
    lock();
    music_gain_ = music, flat_gain_ = flat, sounds_gain_ = sounds, sounds_on_ = sounds_on;
    if (sounds_pitch > 0.0f) sounds_pitch_ = sounds_pitch;
    unlock();
}

void AudioEngine::set_loop(int channel, std::shared_ptr<const Pcm> pcm, float volume, float pan) {
    lock();
    auto it = std::find_if(loops_.begin(), loops_.end(), [&](const Loop& l) { return l.channel == channel; });
    if (it == loops_.end()) {
        if (pcm && pcm->frames() > 0 && volume > 0.0f) loops_.push_back({channel, std::move(pcm), 0.0, volume, pan});
    } else if (!(volume > 0.0f)) {
        loops_.erase(it);
    } else {
        it->volume = volume, it->pan = pan;
    }
    unlock();
}

void AudioEngine::stop_loop(int channel) { set_loop(channel, nullptr, 0.0f, 0.0f); }

void AudioEngine::music_play(std::shared_ptr<const Pcm> track) {
    lock();
    music_.play(std::move(track));
    music_playing_ = true;
    chan_[0].src.clear(), chan_[0].frac = 0, chan_[0].ended = false;
    unlock();
}

void AudioEngine::music_set_next(std::shared_ptr<const Pcm> track) {
    lock();
    music_.set_next(std::move(track));
    unlock();
}

void AudioEngine::music_swap() {
    lock();
    music_.swap();
    music_playing_ = true;
    chan_[0].ended = false;
    unlock();
}

void AudioEngine::music_stop() {
    lock();
    music_.stop();
    music_playing_ = false;
    chan_[0].src.clear(), chan_[0].frac = 0;
    unlock();
}

void AudioEngine::music_stop_channel() {
    lock();
    music_playing_ = false;
    unlock();
}

float AudioEngine::music_location() {
    lock();
    const float t = music_.location();
    unlock();
    return t;
}

bool AudioEngine::music_active() {
    lock();
    const bool a = music_playing_ && music_.active();
    unlock();
    return a;
}

void AudioEngine::boss_play(std::shared_ptr<const Pcm> track, const std::string& name) {
    lock();
    boss_.cur = track, boss_.next = std::move(track);
    boss_.cur_name = boss_.next_name = name;
    boss_.pos = 0;
    boss_.more = boss_.more_after = true;
    boss_.playing = static_cast<bool>(boss_.cur);
    chan_[1].src.clear(), chan_[1].frac = 0, chan_[1].ended = false;
    unlock();
}

void AudioEngine::boss_set_next(std::shared_ptr<const Pcm> track, const std::string& name, bool once) {
    lock();
    boss_.next = std::move(track);
    boss_.next_name = name;
    boss_.more_after = !once;
    unlock();
}

void AudioEngine::boss_stop() {
    lock();
    boss_ = {};
    chan_[1].src.clear(), chan_[1].frac = 0;
    unlock();
}

bool AudioEngine::boss_playing() {
    lock();
    const bool p = boss_.playing;
    unlock();
    return p;
}

std::string AudioEngine::boss_name() {
    lock();
    std::string n = boss_.playing || boss_.cur ? boss_.cur_name : std::string();
    unlock();
    return n;
}

void AudioEngine::set_music_channel(int channel, bool on, float volume, float pitch) {
    if (channel < 0 || channel > 1) return;
    lock();
    chan_[channel].on = on, chan_[channel].volume = volume, chan_[channel].pitch = pitch;
    unlock();
}

int AudioEngine::boss_get(std::int16_t* out, int frames) {
    auto copy = [&](std::int16_t* dst, int want) {
        if (!boss_.cur) return 0;
        const size_t left = boss_.cur->frames() - std::min(boss_.pos, boss_.cur->frames());
        const int n = static_cast<int>(std::min<size_t>(left, static_cast<size_t>(want)));
        std::copy_n(boss_.cur->samples.data() + 2 * boss_.pos, 2 * n, dst);
        boss_.pos += static_cast<size_t>(n);
        return n;
    };
    int got = copy(out, frames);
    if (got < frames && boss_.more) {
        // The track ran out: the queued one takes over (or nothing, which ends the loop); by default it queues
        // itself again.
        boss_.cur = boss_.next, boss_.cur_name = boss_.next_name;
        const bool after = boss_.more_after;
        boss_.more = after;
        if (!boss_.cur || boss_.cur_name.empty()) return 0;  // (the original drops the partial chunk too)
        boss_.next = boss_.cur, boss_.next_name = boss_.cur_name;
        boss_.pos = 0;
        boss_.more = boss_.more_after = after;
        got += copy(out + 2 * got, frames - got);
    }
    return got;
}

template <class Fetch>
bool AudioEngine::render_stream(StreamChannel& ch, float* acc, int frames, Fetch fetch) {
    const double step = ch.pitch;
    const size_t need = static_cast<size_t>(std::floor(ch.frac + frames * step)) + 2;
    std::int16_t chunk[0x1000];
    while (!ch.ended && ch.src.size() / 2 < need) {
        const int want = static_cast<int>(std::min<size_t>(need - ch.src.size() / 2, 0x800));  // 0x2000 bytes at most
        const int got = fetch(chunk, want);
        ch.src.insert(ch.src.end(), chunk, chunk + 2 * got);
        if (got < want) ch.ended = true;
    }
    const size_t have = ch.src.size() / 2;
    if (ch.ended && ch.frac >= static_cast<double>(have)) return false;
    const float g = ch.volume / 32767.0f;
    for (int i = 0; i < frames; ++i) {
        const double p = ch.frac + i * step;
        const size_t a = static_cast<size_t>(p);
        if (a >= have) break;
        // The nearest earlier sample, as QiAudioChannel::fillBuffer reads.
        soft_add(acc[2 * i], ch.src[2 * a] * g), soft_add(acc[2 * i + 1], ch.src[2 * a + 1] * g);
    }
    ch.frac += frames * step;
    const size_t used = std::min(static_cast<size_t>(ch.frac), have);
    ch.src.erase(ch.src.begin(), ch.src.begin() + static_cast<std::ptrdiff_t>(2 * used));
    ch.frac -= static_cast<double>(used);
    return true;
}

bool AudioEngine::render(Voice& v, float* acc, int frames, bool loop, float pitch) {
    const std::int16_t* s = v.pcm->samples.data();
    const double total = static_cast<double>(v.pcm->frames());
    // The pan law of QiAudioChannel::fillBuffer: the far side loses (1 - |pan|), over the 32767 sample scale.
    const float gl = v.volume * std::min(1.0f, 1.0f - v.pan) / 32767.0f, gr = v.volume * std::min(1.0f, 1.0f + v.pan) / 32767.0f;
    for (int i = 0; i < frames; ++i) {
        if (v.position >= total) {
            if (!loop) return false;
            v.position = std::fmod(v.position, total);
        }
        const size_t a = static_cast<size_t>(v.position);  // the nearest earlier sample
        soft_add(acc[2 * i], s[2 * a] * gl), soft_add(acc[2 * i + 1], s[2 * a + 1] * gr);
        v.position += static_cast<double>(v.pitch) * pitch;
    }
    return true;
}

void AudioEngine::mix(std::int16_t* out, int bytes) {
    const int frames = bytes / 4;
    const size_t samples = static_cast<size_t>(frames) * 2, n = static_cast<size_t>(frames);
    for (auto& m : mix_) m.assign(samples, 0.0f);
    left_.resize(n), right_.resize(n);
    auto to_planar = [&](const std::vector<float>& m) {
        for (size_t i = 0; i < n; ++i) left_[i] = m[2 * i], right_[i] = m[2 * i + 1];
    };
    auto from_planar = [&](std::vector<float>& m) {
        for (size_t i = 0; i < n; ++i) m[2 * i] = left_[i], m[2 * i + 1] = right_[i];
    };
    // The room-effects mixer (+0xcc): positional voices and the loops, then the effects, old before new.
    if (sounds_on_) {
        std::vector<float>& m = mix_[0];
        for (size_t i = 0; i < voices_[0].size();) {
            if (render(voices_[0][i], m.data(), frames, false, sounds_pitch_)) ++i;
            else voices_[0].erase(voices_[0].begin() + static_cast<std::ptrdiff_t>(i));
        }
        for (Loop& l : loops_) {
            Voice v{l.pcm, l.position, l.volume, l.pan, 1.0f};
            render(v, m.data(), frames, true, sounds_pitch_);
            l.position = v.position;
        }
        to_planar(m);
        if (old_effects_) old_effects_->process(left_.data(), right_.data(), frames);
        effects_->process(left_.data(), right_.data(), frames);
        from_planar(m);
        for (float& s : m) s *= sounds_gain_;
    }
    // The music mixer (+0xd4). The level music is read at its channel's pitch in the original's chunks (at most 0x2000
    // bytes), counted as played once handed over, and held while its channel is off or stopped. A channel's effects
    // process the mix so far: the flanger comes right after the level music.
    {
        std::vector<float>& m = mix_[1];
        if (chan_[0].on && music_playing_ && music_.active()) {
            render_stream(chan_[0], m.data(), frames, [this](std::int16_t* buf, int count) {
                music_.get_data(buf, count * 4);
                music_.report_played(count * 4);
                return count;
            });
            to_planar(m);
            flanger_[0].process(left_.data(), frames);
            flanger_[1].process(right_.data(), frames);
            from_planar(m);
        }
        // The boss loop on its own channel; it stops once its stream has ended and played out.
        if (chan_[1].on && boss_.playing) {
            if (!render_stream(chan_[1], m.data(), frames, [this](std::int16_t* buf, int count) { return boss_get(buf, count); }))
                boss_.playing = false;
        }
        for (float& s : m) s *= music_gain_;
    }
    // The dry mixer (+0xd0): unpositioned voices.
    {
        std::vector<float>& m = mix_[2];
        for (size_t i = 0; i < voices_[1].size();) {
            if (render(voices_[1][i], m.data(), frames, false, 1.0f)) ++i;
            else voices_[1].erase(voices_[1].begin() + static_cast<std::ptrdiff_t>(i));
        }
        for (float& s : m) s *= flat_gain_;
    }
    // QiAudio::fillBuffer: the mixers in creation order, soft-mixed.
    for (size_t i = 0; i < samples; ++i) {
        float a = mix_[0][i];
        soft_add(a, mix_[1][i]);
        soft_add(a, mix_[2][i]);
        out[i] = static_cast<std::int16_t>(std::clamp(a * 32767.0f, -32768.0f, 32767.0f));
    }
}

}  // namespace sh::audio
