"""
Make the sibling ``mecanumbot_joy`` source importable when not installed.

``mecanumbot_web`` shares the joystick validator with ``mecanumbot_joy``
rather than reimplementing it, so these tests need that package on the
path.  After ``colcon build`` it is installed and this is a no-op; before
one, it lets the suite run straight from a checkout -- which is the whole
point of keeping both packages' logic free of rclpy.
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PACKAGES_ROOT = os.path.dirname(os.path.dirname(_HERE))

try:
    import mecanumbot_joy  # noqa: F401
except ImportError:
    _JOY = os.path.join(_PACKAGES_ROOT, "mecanumbot_joy")
    if os.path.isdir(os.path.join(_JOY, "mecanumbot_joy")):
        sys.path.insert(0, _JOY)
