"""Referee startup must not turn initial simulator pose jumps into a false fail."""
import json
import unittest
from autonav_universe.referee_startup import StartupPoseGuard, RefereeHealth


class StartupRefereeTests(unittest.TestCase):
    def test_guard_rejects_startup_jump_and_requires_stable_spawn(self):
        guard = StartupPoseGuard(6.5, 1.8, duration_s=1.0, max_gap_s=.3)
        self.assertFalse(guard.update(0, 0, 0, 0))
        self.assertFalse(guard.update(6.5, 1.8, 0, .1))
        self.assertFalse(guard.update(6.5, 1.8, .5, .2))
        for i in range(6):
            ready = guard.update(6.5, 1.8, 0, 1+i*.2)
        self.assertTrue(ready)

    def test_guard_resets_for_gaps_and_motion(self):
        guard = StartupPoseGuard(6.5, 1.8)
        for item in ((6.5,1.8,0,0), (6.5,1.8,0,.2), (7,1.8,0,.4),
                     (6.5,1.8,0,1.0), (6.5,1.8,0,1.7)):
            self.assertFalse(guard.update(*item))

    def test_health_fails_closed_when_score_missing_stale_or_failed(self):
        health = RefereeHealth()
        self.assertFalse(health.permitted(5))
        good = {'failed': False, 'finish_reached': False, 'odom_source': 'primary',
                'primary_odom_sample_count': 3, 'last_odom_age_s': .1}
        health.update(json.dumps(good), 5)
        self.assertTrue(health.permitted(6))
        self.assertFalse(health.permitted(8))
        health.update('not-json', 8)
        self.assertFalse(health.permitted(8))
        good['failed'] = True
        health.update(json.dumps(good), 9)
        good['failed'] = False
        health.update(json.dumps(good), 10)
        self.assertFalse(health.permitted(10))

    def test_health_stops_on_finish_and_stale_odom(self):
        score = {'failed': False, 'finish_reached': False, 'odom_source': 'primary',
                 'primary_odom_sample_count': 3, 'last_odom_age_s': 2}
        health = RefereeHealth()
        health.update(json.dumps(score), 1)
        self.assertFalse(health.permitted(1))
        score.update(last_odom_age_s=.1, finish_reached=True)
        health.update(json.dumps(score), 2)
        self.assertFalse(health.permitted(2))


if __name__ == '__main__':
    unittest.main()
