"""Shared data types for the drone testbed."""

from dataclasses import dataclass, field
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
    drone_id: str
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(2))

    def to_flat(self) -> list:
        """Convert to flat [ax, ay] list for ROS messages."""
        return [self.acceleration[0], self.acceleration[1]]
