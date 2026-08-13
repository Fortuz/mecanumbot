"""
Which behaviour trees this GUI can start, and with what.

Pure Python -- no ``rclpy``, no Flask, no ``subprocess``.  This module only
*describes* a run; :mod:`mecanumbot_web.behaviour_runner` performs it.

Every entry names a command the workspace documents as the way to start
that tree, and every hyperparameter here is one of that command's own
arguments.  The GUI therefore cannot express a run that could not have
been typed at a terminal, which is the point: the thesis experiments are
started from this page and from a shell interchangeably, and a divergence
between the two would be a difference in what the robot did.

Two things the catalog is deliberately strict about:

**Values are validated against the spec, never interpolated into a
shell.**  A choice must be one of its choices, a namespace must be a
single ROS name token, and a constants file must be a plain filename that
exists in that behaviour's own config directory.  ``build_command``
returns an argv list; nothing is ever passed to a shell.

**The demo tree is started with ``ros2 run``, not its launch file.**
``mecanumbot_demo_behaviours``' launch file references ``*_demo_bt_node``
executables that its ``setup.py`` does not register, so launching it
fails.  ``wander_between_people_node`` is registered and takes
``--yaml_path``, so that is what this starts.

The SSID -> file mapping is here rather than in the page because it
mirrors what each behaviour's own launch file does, and the two have to
agree about which room the robot thinks it is in.
"""

import os
import re
from typing import Dict, List, Optional, Tuple

#: A hyperparameter offered as a fixed set of values (e.g. the condition).
KIND_CHOICE = "choice"

#: A free-text hyperparameter, validated by shape (e.g. the namespace).
KIND_TEXT = "text"

#: A constants file, chosen from the behaviour's own config directory.
KIND_FILE = "file"

#: A single ROS name token. Namespaces are offered as one level because
#: that is all any launch file in the workspace uses, and accepting a
#: path here would let the page build a namespace no other node expects.
NAME_TOKEN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


class CatalogError(Exception):
    """Raised when a run is asked for that the catalog cannot describe."""


class Argument:
    """One hyperparameter of a run, and how it reaches the command line."""

    def __init__(self, name, label, kind, default, emit, doc="", choices=()):
        """Describe one argument. ``emit`` are argv templates taking the value."""
        self.name = name
        self.label = label
        self.kind = kind
        self.default = default
        self.emit: Tuple[str, ...] = tuple(emit)
        self.doc = doc
        self.choices: Tuple[str, ...] = tuple(choices)

    def options(self, config_dir: str) -> List[str]:
        """
        Return the values this argument accepts, if it is a closed set.

        A file argument's options are whatever is in the behaviour's
        config directory *now*, so a constants file added since the node
        started is offered without a restart.
        """
        if self.kind == KIND_CHOICE:
            return list(self.choices)
        if self.kind == KIND_FILE:
            return list_config_files(config_dir)
        return []

    def describe(self, config_dir: str) -> dict:
        """Return this argument as JSON for the page to render."""
        return {
            "name": self.name,
            "label": self.label,
            "kind": self.kind,
            "default": self.default,
            "doc": self.doc,
            "options": self.options(config_dir),
        }

    def resolve(self, value, config_dir: str) -> str:
        """Validate one supplied value and return what goes on the command line."""
        if value is None:
            value = self.default
        text = str(value).strip()

        if self.kind == KIND_CHOICE:
            if text not in self.choices:
                raise CatalogError("{}: must be one of {}".format(
                    self.label, ", ".join(self.choices)))
            return text

        if self.kind == KIND_FILE:
            if not text:
                raise CatalogError("{}: no file chosen".format(self.label))
            # A behaviour may only be started with a constants file from
            # its own config directory: the name is matched against the
            # directory listing rather than merely sanitised, so no
            # traversal or absolute path can survive the check.
            if text not in list_config_files(config_dir):
                raise CatalogError("{}: '{}' is not a constants file in {}".format(
                    self.label, text, config_dir or "(no config directory)"))
            return os.path.join(config_dir, text)

        if not NAME_TOKEN.match(text):
            raise CatalogError(
                "{}: '{}' is not a ROS name -- letters, digits and underscores, "
                "not starting with a digit".format(self.label, text))
        return text

    def argv(self, value, config_dir: str) -> List[str]:
        """Return the argv fragments this argument contributes."""
        resolved = self.resolve(value, config_dir)
        return [template.format(resolved) for template in self.emit]


class BehaviourSpec:
    """One startable behaviour tree: its command, arguments and constants."""

    def __init__(self, key, label, summary, mode, package, target, editor,
                 arguments, requires=(), default_file="", ssid_files=None,
                 note="", node_names=()):
        """Describe one behaviour. ``target`` is a launch file or an executable."""
        self.key = key
        self.label = label
        self.summary = summary
        self.mode = mode
        self.package = package
        self.target = target
        self.editor = editor
        self.arguments: Tuple[Argument, ...] = tuple(arguments)
        self.requires: Tuple[str, ...] = tuple(requires)
        self.default_file = default_file
        self.ssid_files: Dict[str, str] = dict(ssid_files or {})
        self.note = note
        self.node_names: Tuple[str, ...] = tuple(node_names)

    # ── constants files ──────────────────────────────────────────────────

    def file_for_ssid(self, ssid: Optional[str]) -> str:
        """Return the constants file this behaviour's launch file would pick."""
        return self.ssid_files.get(ssid or "", self.default_file)

    # ── the command ──────────────────────────────────────────────────────

    def defaults(self, ssid: Optional[str] = None) -> Dict[str, str]:
        """Return the value each argument takes when the page has not set it."""
        values = {argument.name: argument.default for argument in self.arguments}
        if "params_file" in values:
            values["params_file"] = self.file_for_ssid(ssid)
        return values

    def build_command(self, values: Dict, config_dir: str) -> List[str]:
        """
        Return the argv that starts this behaviour with ``values``.

        Every argument is resolved, so an omitted one takes its default
        and a rejected one raises before anything is spawned.  Unknown
        keys are refused rather than ignored: silently dropping a
        hyperparameter would start a run that is not the one asked for,
        and these runs are experiment trials.
        """
        known = {argument.name for argument in self.arguments}
        for name in values or {}:
            if name not in known:
                raise CatalogError("{}: unknown argument '{}'".format(
                    self.label, name))

        if self.mode == "launch":
            argv = ["ros2", "launch", self.package, self.target]
        else:
            argv = ["ros2", "run", self.package, self.target]

        for argument in self.arguments:
            argv.extend(argument.argv((values or {}).get(argument.name), config_dir))
        return argv

    # ── description ──────────────────────────────────────────────────────

    def describe(self, config_dir: str, ssid: Optional[str] = None) -> dict:
        """Return this behaviour as JSON for the page to render."""
        files = list_config_files(config_dir)
        return {
            "key": self.key,
            "label": self.label,
            "summary": self.summary,
            "mode": self.mode,
            "package": self.package,
            "target": self.target,
            "editor": self.editor,
            "note": self.note,
            "requires": list(self.requires),
            "config_dir": config_dir,
            "files": files,
            "ssid_file": self.file_for_ssid(ssid),
            "arguments": [argument.describe(config_dir) for argument in self.arguments],
            "defaults": self.defaults(ssid),
            "node_names": list(self.node_names),
            # Nothing to launch with and nothing to edit: the package is
            # not in this checkout, or has not been built.
            "available": bool(files),
        }


def list_config_files(config_dir: str) -> List[str]:
    """Return the constants files in a config directory, sorted."""
    if not config_dir or not os.path.isdir(config_dir):
        return []
    return sorted(name for name in os.listdir(config_dir)
                  if name.endswith(".yaml") and not name.startswith("."))


# ── the catalog ──────────────────────────────────────────────────────────

_NAMESPACE_LAUNCH = Argument(
    name="namespace",
    label="Namespace",
    kind=KIND_TEXT,
    default="mecanumbot",
    emit=("namespace:={}",),
    doc="ROS namespace for the tree node. Everything else in the stack is "
        "under 'mecanumbot'.",
)

_PARAMS_LAUNCH = Argument(
    name="params_file",
    label="Constants file",
    kind=KIND_FILE,
    default="behaviour_setting_constants.yaml",
    # The launch files take the same path twice: 'params' as ROS
    # parameters and 'yaml_path' as the file ConstantParamsToBlackboard
    # actually reads. Setting only one would edit a file the tree never
    # opens, so the page can never set them apart.
    emit=("params:={}", "yaml_path:={}"),
    doc="The constants the tree loads at setup. This is the file edited below.",
)

LEADING = BehaviourSpec(
    key="leading",
    label="Leading",
    summary="The robot leads a person along the route in the constants file, "
            "signalling with the condition's own channel.",
    mode="launch",
    package="mecanumbot_leading_behaviour",
    target="launch_wifi_condition_sequence.launch.py",
    editor="leading",
    requires=(
        "Nav2 running and AMCL localized -- the tree navigates by the "
        "navigate_to_pose and navigate_through_poses actions",
        "The people-detection pipeline, for people_fusion and subject_pose",
    ),
    default_file="behaviour_setting_constants.yaml",
    ssid_files={
        "MecanumNet": "behaviour_setting_constants.yaml",
        "MecanumetoNet": "Eto_behaviour_setting_constants.yaml",
        "APOLLO2028": "behaviour_setting_constants.yaml",
    },
    # All four executables call setup(node_name="bottom_up_tree_node"),
    # so which of these actually appears on the graph depends on whether
    # the launch file's own __node remap wins. Both are watched for.
    node_names=("doglike_leading_bt_node", "control_leading_bt_node",
                "LED_leading_bt_node", "bottom_up_tree_node"),
    arguments=(
        Argument(
            name="condition",
            label="Condition",
            kind=KIND_CHOICE,
            default="Doglike",
            choices=("Doglike", "Control", "LED"),
            emit=("condition:={}",),
            doc="Which experimental condition to run. This selects the tree: "
                "Doglike signals with the neck and grippers, LED with the "
                "corner strips, Control with neither.",
        ),
        _PARAMS_LAUNCH,
        _NAMESPACE_LAUNCH,
    ),
)

OSTENSIVE = BehaviourSpec(
    key="ostensive",
    label="Ostensive",
    summary="The other way round: a person gets the robot's attention and "
            "points, and the robot goes where it was sent.",
    mode="launch",
    package="mecanumbot_ostensive_behaviour",
    target="launch_ostensive.launch.py",
    editor="flat",
    requires=(
        "Nav2 running and AMCL localized -- for /amcl_pose and /goal_pose",
        "The people-detection pipeline -- this tree reads cam_people_detections "
        "keypoints directly, not just people_fusion",
    ),
    default_file="ostensive_setting_constants.yaml",
    ssid_files={"MecanumetoNet": "Eto_ostensive_setting_constants.yaml"},
    node_names=("ostensive_bt_node",),
    arguments=(
        Argument(
            name="params_file",
            label="Constants file",
            kind=KIND_FILE,
            default="ostensive_setting_constants.yaml",
            emit=("params:={}", "yaml_path:={}"),
            doc="The constants the tree loads at setup. This is the file "
                "edited below.",
        ),
        _NAMESPACE_LAUNCH,
    ),
)

DEMO = BehaviourSpec(
    key="demo",
    label="Demo: wander between people",
    summary="Drives to a randomly chosen detected person and greets them. Not "
            "an experimental condition -- a demonstration.",
    mode="run",
    package="mecanumbot_demo_behaviours",
    target="wander_between_people_node",
    editor="leading",
    requires=(
        "Nav2 running and AMCL localized",
        "The people-detection pipeline, for people_fusion",
    ),
    default_file="behaviour_setting_constants.yaml",
    ssid_files={
        "MecanumNet": "behaviour_setting_constants.yaml",
        "MecanumetoNet": "Eto_behaviour_setting_constants.yaml",
    },
    note="Started with 'ros2 run': this package's launch file names "
         "*_demo_bt_node executables that its setup.py does not register, so "
         "launching it would fail immediately.",
    node_names=("wander_between_people_node",),
    arguments=(
        Argument(
            name="params_file",
            label="Constants file",
            kind=KIND_FILE,
            default="behaviour_setting_constants.yaml",
            # The node parses --yaml_path itself before rclpy sees the
            # command line, so this goes in as a plain option rather than
            # under --ros-args.
            emit=("--yaml_path", "{}"),
            doc="The constants the tree loads at setup. This is the file "
                "edited below.",
        ),
        Argument(
            name="namespace",
            label="Namespace",
            kind=KIND_TEXT,
            default="mecanumbot",
            # Last on purpose: everything after --ros-args is taken by
            # rclpy, so any plain option has to come before it.
            emit=("--ros-args", "-r", "__ns:=/{}"),
            doc="ROS namespace for the tree node.",
        ),
    ),
)

#: Every behaviour the GUI can start, in the order the page shows them.
CATALOG: Tuple[BehaviourSpec, ...] = (LEADING, OSTENSIVE, DEMO)

#: The behaviour whose files the un-scoped config routes address.
DEFAULT_BEHAVIOUR = LEADING.key


def keys() -> Tuple[str, ...]:
    """Return every behaviour key, in catalog order."""
    return tuple(spec.key for spec in CATALOG)


def get(key: str) -> BehaviourSpec:
    """Return one behaviour spec by key."""
    for spec in CATALOG:
        if spec.key == key:
            return spec
    raise CatalogError("No such behaviour '{}'".format(key))


def describe_all(config_dirs: Dict[str, str],
                 ssid: Optional[str] = None) -> List[dict]:
    """Return every behaviour as JSON, against its own config directory."""
    return [spec.describe((config_dirs or {}).get(spec.key, ""), ssid)
            for spec in CATALOG]


def all_node_names() -> Tuple[str, ...]:
    """Return every ROS node name a behaviour tree in the catalog registers."""
    names: List[str] = []
    for spec in CATALOG:
        for name in spec.node_names:
            if name not in names:
                names.append(name)
    return tuple(names)


def config_dir_for(key: str, config_dirs: Dict[str, str]) -> str:
    """Return one behaviour's config directory, checking the key exists."""
    get(key)
    return (config_dirs or {}).get(key, "")
