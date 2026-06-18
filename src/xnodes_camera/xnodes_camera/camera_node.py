"""Minimal camera node placeholder used for build verification."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from typing import Any, Dict

PACKAGE_SHARE = Path(__file__).resolve().parent.parent
CONFIG_PATH = PACKAGE_SHARE / "config" / "camera.yaml"


def _load_simple_yaml(path: Path) -> Dict[str, Any]:
    """Parse a tiny subset of YAML used in our example configuration."""
    data: Dict[str, Any] = {}
    if not path.exists():
        return data

    for line in path.read_text().splitlines():
        if "#" in line:
            line = line.split("#", 1)[0]
        if not line.strip():
            continue
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        data[key.strip()] = value.strip()
    return data


def _run_with_rclpy(config: Dict[str, Any], args: Any = None) -> int:
    """Spin up a throwaway rclpy node if the dependency is present."""
    import rclpy
    from rclpy.node import Node

    class CameraNode(Node):
        def __init__(self) -> None:
            super().__init__("xnodes_camera")
            for key, value in config.items():
                self.declare_parameter(key, value)
            self.get_logger().info(
                "Configured camera node with %d parameters.", len(config)
            )

    rclpy.init(args=args)
    node = CameraNode()
    node.destroy_node()
    rclpy.shutdown()
    return 0


def main(args: Any = None) -> int:
    """Entry point compatible with console_scripts."""
    config = _load_simple_yaml(CONFIG_PATH)

    if importlib.util.find_spec("rclpy") is None:
        print(
            "rclpy is not available in this environment."
            " Running in configuration validation mode only.",
        )
        print(f"Loaded configuration entries: {config}")
        return 0

    return _run_with_rclpy(config, args=args)


if __name__ == "__main__":
    raise SystemExit(main())
