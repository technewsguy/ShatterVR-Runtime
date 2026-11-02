#pragma once

// The level music stream (the original's LevelMusicStream and Audio's level-music calls): two
// track slots; each room's music is 32 s long, and once the current track has played 32 s while the next one is
// queued, the next one starts mixing in, so rooms chain without a gap. The played position drives the level's
// room time (Audio::getLevelMusicLocation). Byte counts are of 16-bit stereo at 44100 Hz, as in the original.

#include "audio/pcm.hpp"

#include <memory>

namespace sh::audio {

class LevelMusic {
public:
    // Audio::playLevelMusic(name, skip_start = false): a fresh stream on slot 0; without skip_start the first two
    // seconds are decoded and counted as played (the Level always passes false).
    void play(std::shared_ptr<const Pcm> track, bool skip_start = false);
    void set_next(std::shared_ptr<const Pcm> track);  // Audio::setNextLevelMusic: the other slot, queued
    void swap();                                       // Audio::swapLevelMusic: the other slot becomes current
    void stop();
    // LevelMusicStream::getData: bytes of stereo 16-bit samples (the original asks at most 0x2000 at a time).
    void get_data(std::int16_t* out, int bytes);
    void report_played(int bytes);  // LevelMusicStream::reportPlayed
    float location() const;         // Audio::getLevelMusicLocation: seconds played of the current slot
    bool active() const { return started_; }

private:
    struct Slot {
        std::shared_ptr<const Pcm> track;
        int decoded = 0;      // +0xc: bytes taken from the decoder
        int played = 0;       // +0x14: bytes reported played
        bool has_data = false;  // +9 / +10: still decoding (as the other slot: mixing in under the current one)
        int decode(std::int16_t* out, int bytes);  // QiVorbisDecoder::decode from 'decoded' on
    };
    Slot slots_[2];
    int current_ = 0;        // +4
    bool next_ready_ = false;  // +8
    bool started_ = false;
};

}  // namespace sh::audio
