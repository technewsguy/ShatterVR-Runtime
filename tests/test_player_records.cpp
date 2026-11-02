// The player records (game/player_records): checkpoint maxima, starting values, the
// selected mode, and our text format's round trip.

#include "game/player_records.hpp"

#include <cstdio>

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("%-64s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) ++g_failures;
}

}  // namespace

int main() {
    {
        sh::game::PlayerRecords a;
        sh::game::PlayerStats& s = a.stats[2];
        s.register_ball(false), s.register_ball(true);
        s.register_obstacle(true), s.register_powerup(false);
        s.register_pickup(40), s.register_pickup(30);
        s.register_streak(5), s.register_streak(9);
        s.register_broken();
        s.register_room(1.5f, 20.25f, 100), s.register_room(0.5f, 10, 100);
        check(s.get("balls") == "2" && s.get("ballshit") == "1", "registerBall counts all and hits");
        check(s.get("obstacles") == "1" && s.get("obstaclescleared") == "1", "registerObstacle");
        check(s.get("powerups") == "1" && s.get("powerupstaken") == "0", "registerPowerup, an unset stat reads 0");
        check(s.get("peakballs") == "40" && s.get("streak") == "9", "peaks keep the larger value");
        check(s.get("accstreak") == "2" && s.get("accballs") == "30.25" && s.get("rooms") == "2", "registerRoom sums");
        sh::game::PlayerRecords b;
        check(b.parse(a.serialize()) && b.stats[2].get("accballs") == "30.25" && b.stats[2].get("broken") == "1" &&
                  b.stats[0].get("balls") == "0",
              "stats survive the round trip, per mode");
    }
    sh::game::PlayerRecords r;
    int balls = 0, streak = 0;
    r.load_checkpoint(1, 0, balls, streak);
    check(balls == 25 && streak == 0, "checkpoint 0 starts with 25 balls");
    r.report_checkpoint(1, 3, 40, 2);
    r.report_checkpoint(1, 3, 30, 5);
    check(r.high_score(1, 3) == std::pair<int, int>{40, 5}, "reportCheckpoint keeps each maximum");
    r.load_checkpoint(1, 3, balls, streak);
    check(balls == 40 && streak == 5, "loadCheckpoint gives them back");
    r.report_score(3, 1234);
    r.current_mode = 3;

    sh::game::PlayerRecords back;
    check(back.parse(r.serialize()), "round trip parses");
    check(back.current_mode == 3 && back.best_score[3] == 1234 && back.balls[1][3] == 40 && back.streak[1][3] == 5,
          "round trip keeps mode, scores and checkpoints");
    check(back.parse("shattervr-records 1\nmode 1 score 7\n2 30 1\n") && back.current_mode == 1 &&
              back.balls[1][2] == 30,
          "files without a current line: mode 1");
    check(!back.parse("shattervr-records 1\ncurrent 9\n"), "bad mode rejected");

    std::printf(g_failures ? "%d FAILED\n" : "all passed\n", g_failures);
    return g_failures ? 1 : 0;
}
