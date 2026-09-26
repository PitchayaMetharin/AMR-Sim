#!/usr/bin/env python3
"""Run the controller manager spawner with a bounded load response timeout.

Humble's upstream spawner forwards ``--service-call-timeout`` to list,
configure, and switch calls, but omits both timeout arguments when loading a
controller.  Keep the upstream command line and lifecycle behavior while
patching only that call site to use the configured timeouts.
"""

from __future__ import annotations

import argparse
import sys
from typing import Sequence

from controller_manager import controller_manager_services as _services
from controller_manager import spawner as _spawner


def _timeout_values(argv: Sequence[str]) -> tuple[float, float]:
    """Extract the two timeout values understood by the upstream spawner."""

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--controller-manager-timeout", default=0.0, type=float)
    parser.add_argument("--service-call-timeout", default=10.0, type=float)
    parsed, _ = parser.parse_known_args(list(argv))
    return parsed.controller_manager_timeout, parsed.service_call_timeout


def _load_controller_with_timeouts(
    node,
    controller_manager_name: str,
    controller_name: str,
    *,
    controller_manager_timeout: float,
    service_call_timeout: float,
):
    """Load one controller with both configured service timeouts."""

    return _services.load_controller(
        node,
        controller_manager_name,
        controller_name,
        service_timeout=controller_manager_timeout,
        call_timeout=service_call_timeout,
    )


def main(args=None):
    """Delegate to the upstream spawner after fixing its load call."""

    controller_manager_timeout, service_call_timeout = _timeout_values(sys.argv[1:])
    original_load_controller = _spawner.load_controller

    def load_controller(node, controller_manager_name, controller_name):
        return _load_controller_with_timeouts(
            node,
            controller_manager_name,
            controller_name,
            controller_manager_timeout=controller_manager_timeout,
            service_call_timeout=service_call_timeout,
        )

    _spawner.load_controller = load_controller
    try:
        return _spawner.main(args)
    finally:
        _spawner.load_controller = original_load_controller


if __name__ == "__main__":
    sys.exit(main())
