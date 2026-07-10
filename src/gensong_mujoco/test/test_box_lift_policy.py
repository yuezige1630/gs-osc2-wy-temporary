import tempfile
import unittest
from pathlib import Path

import numpy as np

from gensong_mujoco.box_lift_policy import (
    BoxLiftPolicy,
    LiftPhase,
    is_stable,
    load_box_center,
)


class BoxLiftPolicyTest(unittest.TestCase):
    def test_load_box_center_reads_scene_box_pos(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scene.xml"
            path.write_text('<mujoco><worldbody><body name="scene_box" pos="1.0 0.2 0.81"/></worldbody></mujoco>')
            np.testing.assert_allclose(load_box_center(path), [1.0, 0.2, 0.81])

    def test_policy_progresses_to_lift_and_hold(self):
        policy = BoxLiftPolicy(lift_height=0.15, phase_duration=0.1)
        target = np.array([1.0, 0.0, 0.81])
        phases = []
        for now in (0.0, 0.11, 0.22, 0.33, 0.44, 0.55):
            phase, target = policy.next_target(now, np.array([1.0, 0.0, 0.81]), target)
            phases.append(phase)
        self.assertIn(LiftPhase.LIFT, phases)
        self.assertEqual(phases[-1], LiftPhase.HOLD)
        self.assertGreaterEqual(target[2], 0.96)

    def test_stability_requires_height_and_low_motion(self):
        self.assertTrue(is_stable(0.96, 0.81, 0.01, 0.02))
        self.assertFalse(is_stable(0.82, 0.81, 0.01, 0.02))
        self.assertFalse(is_stable(0.96, 0.81, 0.2, 0.02))


if __name__ == "__main__":
    unittest.main()
