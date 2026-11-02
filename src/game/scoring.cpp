#include "game/scoring.hpp"

#include <algorithm>

namespace sh::game {

void Scoring::add_score(int n, script::ScriptLevel& level) {
    if (balls == 0) {
        ++first_scores;
        log.push_back("achievement 4");
        if (first_scores == 3) log.push_back("achievement 5");
    }
    const int before = balls;
    const int after = before + n;
    if (before < 100 && after > 99) log.push_back("achievement 9");
    if (before < 200 && after > 199) log.push_back("achievement 10");
    if (before < 300 && after > 299) log.push_back("achievement 11");
    balls = player_mode == 0 ? std::min(after, 500) : after;
    log.push_back("registerPickup");
    if (PlayerStats* s = stats()) s->register_pickup(balls);
    level.ui_commands.push_back("pickup " + std::to_string(n));
}

void Scoring::streak_inc(const void* obstacle) {
    const bool pending = !missed.empty();
    const bool listed = std::find(missed.begin(), missed.end(), obstacle) != missed.end();
    if (listed) {
        for (size_t i = 0; i < missed.size();) {  // every occurrence, the last entry taking the slot
            if (missed[i] != obstacle) {
                ++i;
                continue;
            }
            missed[i] = missed.back();
            missed.pop_back();
        }
    }
    if (!pending || (listed && missed.empty())) {
        streak = std::max(streak, saved_streak);
        saved_streak = 0;
    }
    streak += 1;
    if (streak == 40) log.push_back("achievement 6");
    log.push_back("registerStreak");
    if (PlayerStats* s = stats()) s->register_streak(streak);
    streak_level = streak_level + 1 > 11 ? 0 : streak_level + 1;
}

void Scoring::streak_missed(const void* obstacle) {
    if (missed.empty()) saved_streak = streak;
    missed.push_back(obstacle);
    streak = 0;
    streak_level = streak_level - 3 < 0 ? streak_level + 9 : streak_level - 3;
}

void Scoring::streak_abort(bool penalize) {
    if (penalize) {
        streak = 0;
        streak_level = streak_level - 3 < 0 ? streak_level + 9 : streak_level - 3;
    }
    missed.clear();
    saved_streak = 0;
}

void Scoring::clear(script::ObstacleState& state, const void* obstacle, script::ScriptLevel& level) {
    streak_inc(obstacle);
    level.sound_events.push_back({"streak", streak_level, {0, 0, 0}, false, 0.7f, 1.0f});
    add_score(state.score, level);
}

}  // namespace sh::game
