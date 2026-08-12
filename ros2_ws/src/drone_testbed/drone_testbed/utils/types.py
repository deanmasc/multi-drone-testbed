"""Shared data types for the drone testbed."""

from dataclasses import dataclass, field
from typing import Optional
import numpy as np


@dataclass
class DroneState:
    drone_id: str
    position: np.ndarray = field(default_factory=lambda: np.zeros(2))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(2))

    @classmethod
    def from_flat(cls, drone_id: str, data: list) -> 'DroneState':
        """Create from flat [x, y, vx, vy] list (ROS message format)."""
        return cls(
            drone_id=drone_id,
            position=np.array(data[0:2]),
            velocity=np.array(data[2:4]),
        )

    def to_flat(self) -> list:
        """Convert to flat [x, y, vx, vy] list for ROS messages."""
        return [
            self.position[0], self.position[1],
            self.velocity[0], self.velocity[1],
        ]


@dataclass
class ControlOutput:
    """One algorithm's command for one drone.

    `acceleration` is the universal channel and every algorithm must produce
    it -- reactive algorithms (consensus, leader-follower) have nothing else
    to say, since they only know which way to push, not where they want to be.

    Trajectory-following algorithms (Square, Trochoidal) *do* know their exact
    intended position at every instant. They should also fill in `position` and
    `velocity`, because on hardware the alternative is crazyflie_node
    reconstructing that path by double-integrating the acceleration, which
    accumulates bias with nothing to correct it. Leave them None otherwise and
    the integrating path is used as before.
    """

    drone_id: str
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(2))
    position: Optional[np.ndarray] = None
    velocity: Optional[np.ndarray] = None

    def to_flat(self) -> list:
        """Convert to flat [ax, ay] list for ROS messages."""
        return [self.acceleration[0], self.acceleration[1]]

    def has_setpoint(self) -> bool:
        """True if this command carries an explicit position/velocity target."""
        return self.position is not None and self.velocity is not None

    def setpoint_to_flat(self) -> list:
        """Convert the setpoint to flat [x, y, vx, vy] for ROS messages."""
        return [
            self.position[0], self.position[1],
            self.velocity[0], self.velocity[1],
        ]
