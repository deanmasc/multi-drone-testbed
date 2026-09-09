"""Dummy algorithm for validating the VICON -> ROS2 pipeline.

Prints each drone's position/velocity to the terminal and always
commands zero acceleration. Useful for confirming mocap_state_node
is receiving real pose updates before wiring up flight control.
"""

from typing import Dict, List

from drone_testbed.algorithms.base_algorithm import BaseAlgorithm
from drone_testbed.algorithms.registry import register_algorithm
from drone_testbed.utils.types import DroneState, ControlOutput


@register_algorithm
class PrintState(BaseAlgorithm):

    def name(self) -> str:
        return "PrintState"

    def configure(self, params: dict, drone_ids: List[str]) -> None:
        pass

    def compute_controls(
        self,
        states: Dict[str, DroneState],
        dt: float,
    ) -> Dict[str, ControlOutput]:
        for drone_id, state in states.items():
            print(
                f'[{drone_id}] pos=({state.position[0]:+.3f}, {state.position[1]:+.3f})  '
                f'vel=({state.velocity[0]:+.3f}, {state.velocity[1]:+.3f})'
            )
        return {
            drone_id: ControlOutput(drone_id=drone_id)
            for drone_id in states
        }
