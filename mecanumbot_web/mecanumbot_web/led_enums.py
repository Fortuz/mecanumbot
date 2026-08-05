"""
LED corner, mode and colour numbering, in one place.

The joystick profiles and the behaviour constants both express LED states
as ``{'mode': N, 'color': N}`` per corner, and both are edited by this
GUI, so the numbering lives here rather than being spelled out twice.

Colours are authoritative, taken from ``mecanumbot_ledgui.COLOR_MAP``.
Modes 1-4 come from that same file's ``MODE_MAP``; 5 and 6 are used
throughout the behaviour YAMLs but are missing from it, and are named
here from the trailing comments in those files.
"""

#: The four corners, in the order ``SetLedStatus`` declares them.
LED_CORNERS = ("fl", "fr", "bl", "br")

#: Human-readable corner labels for the GUI.
CORNER_LABELS = {
    "fl": "front left",
    "fr": "front right",
    "bl": "back left",
    "br": "back right",
}

COLOR_NAMES = {
    0: "black",
    1: "white",
    2: "green",
    3: "red",
    4: "blue",
    5: "cyan",
    6: "pink",
    7: "yellow",
}

MODE_NAMES = {
    1: "wave right",
    2: "wave left",
    3: "pulse",
    4: "solid",
    5: "fast blink",
    6: "slow blink",
}

#: What each colour number looks like, for the diagnostics swatches.
#: Approximations of the WS2812 output rather than measured values --
#: they exist so the page can be read at a glance from across a room,
#: not to match the strip photometrically.
COLOR_HEX = {
    0: "#101418",
    1: "#f4f7fa",
    2: "#22d36a",
    3: "#ff3b4e",
    4: "#3b7bff",
    5: "#22d3ee",
    6: "#ff5fd2",
    7: "#ffd83b",
}


def describe_corner(values) -> dict:
    """
    Describe one corner's ``{'mode': N, 'color': N}`` pair for display.

    Unknown numbers are passed through rather than dropped: the LED node
    answers a failed serial read with -1 in every field, and a corner
    reading "mode -1" is more use to an operator than a blank cell.
    """
    try:
        mode = int(values["mode"])
        color = int(values["color"])
    except (KeyError, TypeError, ValueError):
        return {"mode": None, "color": None, "mode_name": "unknown",
                "color_name": "unknown", "hex": None, "known": False}

    known = mode in MODE_NAMES and color in COLOR_NAMES
    return {
        "mode": mode,
        "color": color,
        "mode_name": MODE_NAMES.get(mode, "mode {}".format(mode)),
        "color_name": COLOR_NAMES.get(color, "colour {}".format(color)),
        "hex": COLOR_HEX.get(color),
        "known": known,
    }


def describe_corners(corners) -> str:
    """
    Describe a four-corner LED state in words.

    Used to regenerate the inline comments in both YAML formats.  Values
    are the source of truth: several comments in the shipped behaviour
    files contradict their own numbers (``'color':6`` labelled "white"
    where 6 is pink, ``'color':2`` labelled "yellow" where 2 is green),
    so carrying those forward would preserve a known error.
    """
    try:
        modes = {int(corners[corner]["mode"]) for corner in LED_CORNERS}
        colors = {int(corners[corner]["color"]) for corner in LED_CORNERS}
    except (KeyError, TypeError, ValueError):
        return "malformed"

    if len(modes) == 1 and len(colors) == 1:
        return "{} {}".format(
            COLOR_NAMES.get(colors.pop(), "colour ?"),
            MODE_NAMES.get(modes.pop(), "mode ?"))
    return "mixed per corner"
