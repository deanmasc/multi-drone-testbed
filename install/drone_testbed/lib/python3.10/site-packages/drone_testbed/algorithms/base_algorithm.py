"""Base class for all distributed control algorithms.

To create a new algorithm:
  1. Create a new file in this directory
  2. Subclass BaseAlgorithm
  3. Decorate with @register_algorithm
  4. Import it in __init__.py
"""

from abc import ABC, abstractmethod
from typing import Dict, List

from drone_testbed.utils.types import DroneState, ControlOutput


class BaseAlgorithm(ABC):
    """Abstract base class for distributed drone control algorithms."""

    @abstractmethod
    def name(self) -> str:
        """Human-readable algorithm name."""
        ...

    @abstractmethod
    def configure(self, params: dict, drone_ids: List[str]) -> None:
        """Called once at startup (and on algorithm swap) with params from YAML.

        Args:
            params: Algorithm-specific parameters from testbed.yaml
            drone_ids: List of all drone IDs in the testbed
        """
        ...

    @abstractmethod
    def compute_controls(
        self,
        states: Dict[str, DroneState],
        dt: float,
    ) -> Dict[str, ControlOutput]:
        """Compute control output for each drone given current states.

        This is called at the control rate (e.g. 5Hz). Each algorithm
        decides how to use the full state dict -- truly distributed
        algorithms should only read neighbor states.

        Args:
            states: Current state of every drone, keyed by drone_id
            dt: Control timestep in seconds

        Returns:
            Dict mapping drone_id -> ControlOutput (acceleration command)
        """
        ...

    def reset(self) -> None:
        """Optional: reset internal state (called on algorithm swap or sim reset)."""
        pass
