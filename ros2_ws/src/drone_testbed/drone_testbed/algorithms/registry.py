"""Algorithm registration and discovery."""

from typing import Dict, List, Type

from drone_testbed.algorithms.base_algorithm import BaseAlgorithm

ALGORITHM_REGISTRY: Dict[str, Type[BaseAlgorithm]] = {}


def register_algorithm(cls: Type[BaseAlgorithm]) -> Type[BaseAlgorithm]:
    """Decorator to register an algorithm class.

    Usage:
        @register_algorithm
        class MyAlgorithm(BaseAlgorithm):
            ...
    """
    ALGORITHM_REGISTRY[cls.__name__] = cls
    return cls


def get_algorithm(name: str) -> BaseAlgorithm:
    """Instantiate a registered algorithm by class name."""
    if name not in ALGORITHM_REGISTRY:
        available = ', '.join(ALGORITHM_REGISTRY.keys())
        raise ValueError(
            f"Unknown algorithm '{name}'. Available: {available}"
        )
    return ALGORITHM_REGISTRY[name]()


def list_algorithms() -> List[str]:
    """Return names of all registered algorithms."""
    return list(ALGORITHM_REGISTRY.keys())
