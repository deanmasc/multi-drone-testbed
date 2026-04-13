"""YAML configuration loading utilities."""

import yaml
import os


def load_config(config_path: str) -> dict:
    """Load testbed configuration from a YAML file.

    If config_path is relative, resolves against the package share directory.
    """
    # Try absolute path first
    if os.path.isabs(config_path) and os.path.exists(config_path):
        path = config_path
    else:
        # Try relative to package share
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('drone_testbed')
            path = os.path.join(pkg_share, config_path)
        except (ImportError, Exception):
            # Fallback: relative to current working directory
            path = config_path

    with open(path, 'r') as f:
        return yaml.safe_load(f)
