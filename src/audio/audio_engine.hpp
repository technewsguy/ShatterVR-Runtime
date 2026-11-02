#pragma once

// The PC mixer (ours, on SDL3): one-shot voices with volume, pan and pitch, looping channels set every frame, and the
// level music stream. 16-bit stereo at 44100 Hz. How the original's QiAudioChannel turns pan into left/right gains is
// not known; this uses a linear balance.

#include "audio/level_music.hpp"
#include "audio/pcm.hpp"
#include "audio/room_effects.hpp"

#include <memory>
#include <string>
#include <vector>

struct SDL_AudioStream;

namespace sh::audio {

class AudioEngine {
public:
    AudioEngine() = default;
    ~AudioEngine();
    AudioEngine(const AudioEngine&) = delete;
    AudioEngine& operator=(const AudioEngine&) = delete;

    bool open();  // false (and silence) when there is no audio device
    void close();

    // A one-shot: volume 0..10, pan -1 (left) .. 1 (right), pitch as a playback-rate factor.
    // positional: the 16-voice pool on the room-effects mixer (Audio +0xd8 on +0xcc); else the dry 4-voice one (+0xdc
    // on +0xd0). A full pool drops its oldest voice.
    void play(std::shared_ptr<const Pcm> pcm, float volume, float pan, float pitch, bool positional);
    // A looping channel: playing while it is set with a volume above 0; set every frame (QiAudioChannel +0x20/+0x28).
    void set_loop(int channel, std::shared_ptr<const Pcm> pcm, float volume, float pan);
    void stop_loop(int channel);

    // The level music (Audio::playLevelMusic and friends). The calls lock the mixer.
    void music_play(std::shared_ptr<const Pcm> track);
    void music_set_next(std::shared_ptr<const Pcm> track);
    void music_swap();
    void music_stop();
    float music_location();
    bool music_active();  // the level music channel is playing (QiAudioChannel +0x38)
    // QiAudioChannel::stop on the level music channel (the Level, close to a boss): the stream holds where it is and
    // music_active() is false until the next play or swap.
    void music_stop_channel();

    // The boss loop (Audio::playBossLoop / setNextBossLoop / stopBossLoop / isBossLoopPlaying /
    // getCurrentBossFileName, BossStream::getData). A track runs out into the queued next one, which is the track
    // itself unless setNextBossLoop queued another; queued with `once`, that one ends the loop when it runs out.
    void boss_play(std::shared_ptr<const Pcm> track, const std::string& name);
    void boss_set_next(std::shared_ptr<const Pcm> track, const std::string& name, bool once);
    void boss_stop();
    bool boss_playing();
    std::string boss_name();

    // The music channels' QiAudioChannel state, set by the Level every frame: 0 the level music (Audio +0x15c), 1 the
    // boss loop (+0x28b8). Off: not mixed and not advanced (+0x2c); volume (+0x20); pitch (+0x24), the rate at which
    // the stream is read (slow motion slows the music, and with it the music-synced room time).
    void set_music_channel(int channel, bool on, float volume, float pitch);

    // Audio::setReverb: the new room's effects; the current ones become the old ones (older old ones go) and the blend
    // restarts. Audio::update, once per game frame: old mix = 1 - blend, new mix = blend, blend += dt; at 1 the old go.
    void set_room_effects(const RoomEffectParams& params);
    void update_room_effects(float dt);
    // The level music channel's flanger (Audio::update from the Level's power-up timers).
    void set_flanger(bool on, float depth);
    // Mixer volumes (+0x20): music (+0xd4), the dry sounds (+0xd0), and the room-effects mixer (+0xcc) with its on
    // flag and pitch (the Level drives them in slow motion and a pause).
    void set_mixers(float music, float flat, float sounds, bool sounds_on, float sounds_pitch);

    // Fills bytes of 16-bit stereo (the device callback; public for tests and offline rendering).
    void mix(std::int16_t* out, int bytes);

private:
    struct Voice {
        std::shared_ptr<const Pcm> pcm;
        double position = 0;  // in frames
        float volume = 1, pan = 0, pitch = 1;
    };
    struct Loop {
        int channel = 0;
        std::shared_ptr<const Pcm> pcm;
        double position = 0;
        float volume = 0, pan = 0;
    };
    bool render(Voice& v, float* acc, int frames, bool loop, float pitch);
    // A streamed channel: source frames read ahead (interleaved stereo) and the fractional read position.
    struct StreamChannel {
        bool on = true;
        float volume = 1, pitch = 1;
        std::vector<std::int16_t> src;
        double frac = 0;
        bool ended = false;
    };
    // Reads the channel's stream at its pitch into acc; fetch(out, frames) delivers up to that many frames and
    // fewer at the end of the stream. Returns false once the stream has ended and been played out.
    template <class Fetch>
    bool render_stream(StreamChannel& ch, float* acc, int frames, Fetch fetch);
    int boss_get(std::int16_t* out, int frames);  // BossStream::getData, in frames
    struct Boss {
        std::shared_ptr<const Pcm> cur, next;
        std::string cur_name, next_name;
        size_t pos = 0;           // frames into cur
        bool more = false;        // +0xb4: go on into the next track when this one runs out
        bool more_after = false;  // +0xb5: the value +0xb4 takes at that switch
        bool playing = false;     // the boss channel's +0x38
    };
    Boss boss_;
    StreamChannel chan_[2];
    bool music_playing_ = false;
    void lock();
    void unlock();

    SDL_AudioStream* stream_ = nullptr;
    std::vector<Voice> voices_[2];  // 0 positional, 1 dry
    std::vector<Loop> loops_;
    LevelMusic music_;
    std::vector<float> mix_[3], left_, right_;  // interleaved per mixer: room effects, music, dry; planar scratch
    std::unique_ptr<RoomEffects> effects_ = std::make_unique<RoomEffects>(RoomEffectParams{}), old_effects_;
    float effects_blend_ = 0;
    MonoFlanger flanger_[2];
    float music_gain_ = 0.7f, flat_gain_ = 1, sounds_gain_ = 1, sounds_pitch_ = 1;
    bool sounds_on_ = true;
};

}  // namespace sh::audio
