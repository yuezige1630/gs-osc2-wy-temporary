"""Pure helpers for staged, conservative box lifting."""

from enum import Enum
import xml.etree.ElementTree as ET

import numpy as np


class LiftPhase(Enum):
    APPROACH = "approach"
    CONTACT = "contact"
    GRASP = "grasp"
    LIFT = "lift"
    HOLD = "hold"


def load_box_center(xml_path):
    """Read the world-space center declared by the scene_box body."""
    body = ET.parse(xml_path).getroot().find('.//body[@name="scene_box"]')
    if body is None or "pos" not in body.attrib:
        raise ValueError("MuJoCo XML must contain scene_box body with pos")
    values = np.fromstring(body.attrib["pos"], sep=" ", dtype=float)
    if values.size != 3:
        raise ValueError("scene_box pos must contain exactly three values")
    return values


def is_stable(box_z, initial_z, linear_speed, attitude_error):
    """Return whether the box is lifted and sufficiently settled."""
    return (
        box_z >= initial_z + 0.10
        and abs(linear_speed) <= 0.05
        and abs(attitude_error) <= 0.10
    )


class BoxLiftPolicy:
    """Generate a monotonic vertical lift target through fixed safe phases."""

    _ORDER = (
        LiftPhase.APPROACH,
        LiftPhase.CONTACT,
        LiftPhase.GRASP,
        LiftPhase.LIFT,
        LiftPhase.HOLD,
    )

    def __init__(self, lift_height=0.15, phase_duration=0.75):
        if lift_height <= 0 or phase_duration <= 0:
            raise ValueError("lift_height and phase_duration must be positive")
        self.lift_height = float(lift_height)
        self.phase_duration = float(phase_duration)
        self.phase = LiftPhase.APPROACH
        self.started_at = None

    def next_target(self, now, box_center, current_target):
        center = np.asarray(box_center, dtype=float)
        if center.shape != (3,):
            raise ValueError("box_center must be a three-element vector")
        if self.started_at is None:
            self.started_at = float(now)
        elapsed = max(0.0, float(now) - self.started_at)
        phase_index = min(int(elapsed / self.phase_duration), len(self._ORDER) - 1)
        self.phase = self._ORDER[phase_index]
        target = np.asarray(current_target, dtype=float).copy()
        if target.shape != (3,):
            raise ValueError("current_target must be a three-element vector")
        if self.phase in (LiftPhase.CONTACT, LiftPhase.GRASP):
            target[2] = center[2]
        elif self.phase in (LiftPhase.LIFT, LiftPhase.HOLD):
            target[2] = center[2] + self.lift_height
        return self.phase, target
