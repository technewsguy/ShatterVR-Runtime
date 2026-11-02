#pragma once

// Balls and the streak: Level::addScore, streakInc, streakMissed, streakAbort, the
// clear sound, and the player counters they touch. The player's stats bag, achievements and the UI script
// are logged for now (they come with the save data and the UI).

#include "game/player_records.hpp"
#include "script/level_script.hpp"

#include <string>
#include <vector>

namespace sh::game {

struct Scoring {
    // Level
    int streak_level = 0;      // +0x220: 0..11, picks the clear sound; +1 per clear, -3 (mod 12) per miss
    int first_scores = 0;      // +0x28c: scores added while the player had no balls
    int saved_streak = 0;      // +0x2a8: the player's streak when the first pending miss happened
    std::vector<const void*> missed;  // +0x2ac: obstacles missed since the streak last held
    // Player
    int balls = 0;             // +0x594
    int streak = 0;            // +0x598
    int player_mode = 0;       // +0x63c
    PlayerRecords* records = nullptr;  // sh_play's; the tests run without
    // The current mode's stats bag (null without records).
    PlayerStats* stats() { return records ? &records->stats[player_mode] : nullptr; }

    // Calls into the stats bag, achievements and the UI, in order (for comparison until those exist).
    std::vector<std::string> log;

    // Level::addScore(n): balls += n (at most 500 in player mode 0); "pickup n" to the UI.
    void add_score(int n, script::ScriptLevel& level);
    // Level::streakInc(obstacle): a pending miss of this obstacle is forgiven; with no misses pending, the
    // streak takes back what it had before them. streak += 1, streak level += 1 (wrapping after 11).
    void streak_inc(const void* obstacle);
    // Level::streakMissed(obstacle): the obstacle joins the pending misses; streak 0, streak level -= 3.
    void streak_missed(const void* obstacle);
    // Level::streakAbort(penalize): pending misses dropped (and with penalize: streak 0, level -= 3).
    void streak_abort(bool penalize);
    // mgClear on an obstacle with a score: streakInc, the clear sound (bank "streak", the new streak level,
    // volume 0.7), addScore(score).
    void clear(script::ObstacleState& state, const void* obstacle, script::ScriptLevel& level);
    // The stats call Room::update makes for a finished scoring obstacle.
    void register_obstacle(bool cleared) {
        log.push_back(cleared ? "registerObstacle 1" : "registerObstacle 0");
        if (PlayerStats* s = stats()) s->register_obstacle(cleared);
    }
};

}  // namespace sh::game
