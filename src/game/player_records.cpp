#include "game/player_records.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace sh::game {

namespace {

bool valid_mode(int mode) { return mode >= 0 && mode < PlayerRecords::kModes; }
bool valid_checkpoint(int c) { return c >= 1 && c <= 12; }

}  // namespace

void PlayerStats::peak(const std::string& key, int v) {
    double& d = values[key];
    d = std::max(d, static_cast<double>(v));
}

void PlayerStats::add_float(const std::string& key, float v) {
    double& d = values[key];
    d = v + static_cast<float>(d);
}

void PlayerStats::register_ball(bool hit) {
    add("balls");
    if (hit) add("ballshit");
}

void PlayerStats::register_obstacle(bool cleared) {
    add("obstacles");
    if (cleared) add("obstaclescleared");
}

void PlayerStats::register_powerup(bool taken) {
    add("powerups");
    if (taken) add("powerupstaken");
}

void PlayerStats::register_room(float avg_streak, float avg_balls, float length) {
    add_float("accstreak", avg_streak);
    add_float("accballs", avg_balls);
    add_float("accdistance", length);
    add("rooms");
}

std::string PlayerStats::get(const std::string& key) const {
    const auto it = values.find(key);
    if (it == values.end()) return "0";
    char text[32];
    std::snprintf(text, sizeof text, "%.9g", it->second);
    return text;
}

std::pair<int, int> PlayerRecords::high_score(int mode, int checkpoint) const {
    if (!valid_mode(mode) || !valid_checkpoint(checkpoint)) return {0, 0};
    return {balls[mode][checkpoint], streak[mode][checkpoint]};
}

void PlayerRecords::report_checkpoint(int mode, int checkpoint, int ball_count, int streak_count) {
    if (!valid_mode(mode) || !valid_checkpoint(checkpoint)) return;
    balls[mode][checkpoint] = std::max(balls[mode][checkpoint], ball_count);
    streak[mode][checkpoint] = std::max(streak[mode][checkpoint], streak_count);
}

void PlayerRecords::load_checkpoint(int mode, int checkpoint, int& ball_count, int& streak_count) const {
    if (!valid_mode(mode) || !valid_checkpoint(checkpoint)) {
        ball_count = 25, streak_count = 0;
        return;
    }
    ball_count = balls[mode][checkpoint], streak_count = streak[mode][checkpoint];
}

void PlayerRecords::report_score(int mode, int score) {
    if (valid_mode(mode)) best_score[mode] = std::max(best_score[mode], score);
}

// "current <m>" (the selected mode), then per mode a line "mode <m> score <s>", "<checkpoint> <balls> <streak>" for
// each reached checkpoint and "stat <name> <value>" for each stat.
std::string PlayerRecords::serialize() const {
    std::ostringstream out;
    out << "shattervr-records 1\n";
    out << "current " << current_mode << "\n";
    out << "music " << music << "\nsound " << sound << "\n";
    out << "aim " << (head_aim ? "head" : "hand") << "\n";
    for (int m = 0; m < kModes; ++m) {
        out << "mode " << m << " score " << best_score[m] << "\n";
        for (int c = 1; c < kCheckpoints; ++c)
            if (balls[m][c] || streak[m][c]) out << c << " " << balls[m][c] << " " << streak[m][c] << "\n";
        for (const auto& kv : stats[m].values) out << "stat " << kv.first << " " << stats[m].get(kv.first) << "\n";
    }
    return out.str();
}

bool PlayerRecords::parse(const std::string& text) {
    std::istringstream in(text);
    std::string word;
    int version = 0;
    if (!(in >> word >> version) || word != "shattervr-records" || version != 1) return false;
    PlayerRecords r;
    int mode = -1;
    while (in >> word) {
        if (word == "music" || word == "sound") {
            if (!(in >> (word == "music" ? r.music : r.sound))) return false;
            continue;
        }
        if (word == "aim") {
            std::string v;
            if (!(in >> v)) return false;
            r.head_aim = v == "head";
            continue;
        }
        if (word == "current") {
            if (!(in >> r.current_mode) || !valid_mode(r.current_mode)) return false;
            continue;
        }
        if (word == "stat") {
            std::string key;
            double value = 0;
            if (!(in >> key >> value) || !valid_mode(mode)) return false;
            r.stats[mode].values[key] = value;
            continue;
        }
        if (word == "mode") {
            std::string score_word;
            int score = 0;
            if (!(in >> mode >> score_word >> score) || !valid_mode(mode)) return false;
            r.best_score[mode] = score;
            continue;
        }
        const int c = std::atoi(word.c_str());
        int b = 0, s = 0;
        if (!(in >> b >> s) || !valid_mode(mode) || !valid_checkpoint(c)) return false;
        r.balls[mode][c] = b, r.streak[mode][c] = s;
    }
    *this = r;
    return true;
}

}  // namespace sh::game
