#pragma once

// The player's checkpoint records (Player +0x3d4 / +0x4a4, per mode): for each of the
// 12 checkpoints, the most balls held on reaching it and the best streak then. Starting a level at a checkpoint takes
// those back. Stored in our own text format (the original's encrypted save is not read or written).

#include <map>
#include <string>
#include <utility>

namespace sh::game {

// The stats bag (Player +0xc4, one per mode; the UI's "stats.*"), lifetime totals and peaks. The original keeps the
// values as strings; integers count up or keep the larger value, the three acc* sums are floats.
struct PlayerStats {
    std::map<std::string, double> values;

    void add(const std::string& key) { values[key] += 1; }
    void peak(const std::string& key, int v);
    void add_float(const std::string& key, float v);

    void register_ball(bool hit);                // balls, ballshit
    void register_obstacle(bool cleared);        // obstacles, obstaclescleared
    void register_powerup(bool taken);           // powerups, powerupstaken
    void register_pickup(int balls) { peak("peakballs", balls); }
    void register_streak(int streak) { peak("streak", streak); }
    void register_score(int score) { peak("score", score); }
    void register_distance(int distance) { peak("distance", distance); }
    void register_broken() { add("broken"); }
    void register_room(float avg_streak, float avg_balls, float length);  // accstreak, accballs, accdistance, rooms
    std::string get(const std::string& key) const;  // "0" when never set
};

struct PlayerRecords {
    static constexpr int kModes = 4, kCheckpoints = 13;
    int balls[kModes][kCheckpoints] = {};
    int streak[kModes][kCheckpoints] = {};
    int best_score[kModes] = {};  // the menu's overall high score (unverified: what mgGetHighScore() returns)
    std::string music = "1", sound = "1";  // the audio bag's settings (0, 0.3, 0.7, 1), kept by game.saveConfig
    PlayerStats stats[kModes];
    int current_mode = 1;  // Player +0x63c, kept by Player::save (unverified); "game.mode" changes it

    // Player::getHighScore / getHighScoreStreak: checkpoints 1..12, else 0.
    std::pair<int, int> high_score(int mode, int checkpoint) const;
    // Player::reportCheckpoint: keeps the larger ball count and streak.
    void report_checkpoint(int mode, int checkpoint, int ball_count, int streak_count);
    // Player::loadCheckpoint: the starting balls and streak (25 and 0 outside 1..12).
    void load_checkpoint(int mode, int checkpoint, int& ball_count, int& streak_count) const;
    void report_score(int mode, int score);

    std::string serialize() const;
    bool parse(const std::string& text);
};

}  // namespace sh::game
