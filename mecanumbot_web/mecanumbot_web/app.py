"""
The robot-hosted web GUI: three pages, no database, no login.

Flask only.  Runs on the robot itself, served from a daemon thread beside
the rclpy executor (see :mod:`mecanumbot_web.web_node`).

This replaces a Flask app that ran in a Docker container on the operator
PC and kept its own SQLite database of actions and mappings, mirrored
against a second database on the robot.  Every "save to host / robot /
both" control, and the ``from_robot_db`` flag underneath them, existed
only to arbitrate between those two copies.  With the GUI on the robot
there is one copy -- the YAML files in the packages themselves -- so the
distinction has nothing left to describe.

Three pages:

``/joystick``     edit and reload the joystick profile
``/diagnostics``  measured publish rate of every important topic
``/behaviour``    edit the leading-behaviour constants

There is deliberately no user model.  The old "login" was a
process-global name rather than a session -- two browsers already shared
one identity -- so on a single-operator robot GUI it was friction that
bought nothing.
"""

import os
import subprocess
from typing import List, Optional

from flask import Flask, jsonify, redirect, render_template, request, url_for

from . import behaviour_store, joystick_store, yaml_io

#: Wi-Fi SSID to behaviour constants file.  Mirrors the selection
#: ``launch_wifi_condition_sequence.launch.py`` makes, so the page can
#: show which file is actually in play.
SSID_BEHAVIOUR_FILES = {
    "MecanumNet": "behaviour_setting_constants.yaml",
    "MecanumetoNet": "Eto_behaviour_setting_constants.yaml",
    "APOLLO2028": "behaviour_setting_constants.yaml",
}

DEFAULT_BEHAVIOUR_FILE = "behaviour_setting_constants.yaml"


def current_ssid() -> Optional[str]:
    """
    Return the connected Wi-Fi SSID, or None.

    Same probe order the launch files use: ``nmcli`` first, ``iwgetid``
    as a fallback.
    """
    try:
        output = subprocess.check_output(
            ["nmcli", "-t", "-f", "ACTIVE,SSID", "dev", "wifi"],
            stderr=subprocess.DEVNULL, timeout=3).decode()
        for line in output.splitlines():
            if line.startswith("yes:"):
                return line.split(":", 1)[1].strip() or None
    except (OSError, subprocess.SubprocessError):
        pass

    try:
        output = subprocess.check_output(
            ["iwgetid", "-r"], stderr=subprocess.DEVNULL, timeout=3).decode()
        return output.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


def behaviour_file_for_ssid(ssid: Optional[str]) -> str:
    """Return the behaviour file the current SSID selects."""
    return SSID_BEHAVIOUR_FILES.get(ssid or "", DEFAULT_BEHAVIOUR_FILE)


class WebApp:
    """The Flask application, wired to a :class:`~.ros_bridge.WebNode`."""

    def __init__(
        self,
        node=None,
        joystick_dir: str = "",
        behaviour_dir: str = "",
        backup_root: str = yaml_io.DEFAULT_BACKUP_ROOT,
    ):
        """Build the app. ``node`` may be None, in which case ROS routes 503."""
        self._node = node
        self._joystick_dir = joystick_dir
        self._behaviour_dir = behaviour_dir
        self._backup_root = backup_root

        self.app = Flask(__name__)
        self.app.config["JSON_SORT_KEYS"] = False
        self._register_routes()

    # ── helpers ──────────────────────────────────────────────────────────

    def _require_node(self):
        """Return the ROS node, or a 503 response tuple."""
        if self._node is None:
            return None, (jsonify({
                "ok": False,
                "error": "The ROS node is not running, so the robot cannot be reached.",
            }), 503)
        return self._node, None

    def _behaviour_path(self, name: str) -> str:
        """Resolve a behaviour filename inside the configured directory."""
        if not name or os.sep in name or not name.endswith(".yaml") or \
                name.startswith("."):
            raise behaviour_store.BehaviourStoreError(
                "Invalid behaviour file name '{}'".format(name))
        path = os.path.join(self._behaviour_dir, name)
        if not os.path.isfile(path):
            raise behaviour_store.BehaviourStoreError(
                "No such behaviour file '{}'".format(name))
        return path

    def _behaviour_files(self) -> List[dict]:
        """List the behaviour constants files available for editing."""
        if not self._behaviour_dir or not os.path.isdir(self._behaviour_dir):
            return []
        ssid = current_ssid()
        selected = behaviour_file_for_ssid(ssid)
        files = []
        for name in sorted(os.listdir(self._behaviour_dir)):
            if not name.endswith(".yaml"):
                continue
            path = os.path.join(self._behaviour_dir, name)
            files.append({
                "name": name,
                "path": path,
                "selected_by_ssid": name == selected,
                "symlinked": yaml_io.is_symlinked_install(path),
            })
        return files

    # ── routes ───────────────────────────────────────────────────────────

    def _register_routes(self) -> None:
        """Attach every route to the Flask app."""
        app = self.app

        # ── pages ────────────────────────────────────────────────────────

        @app.route("/")
        def index():
            """Send the operator straight to diagnostics."""
            return redirect(url_for("diagnostics_page"))

        @app.route("/joystick")
        def joystick_page():
            """Render the joystick profile editor."""
            return render_template(
                "joystick.html",
                page="joystick",
                metadata=joystick_store.editor_metadata(),
                config_dir=self._joystick_dir)

        @app.route("/diagnostics")
        def diagnostics_page():
            """Render the topic-rate diagnostics page."""
            specs = self._node.monitor_specs() if self._node else []
            return render_template(
                "diagnostics.html", page="diagnostics", specs=specs)

        @app.route("/behaviour")
        def behaviour_page():
            """Render the behaviour constants editor."""
            ssid = current_ssid()
            return render_template(
                "behaviour.html",
                page="behaviour",
                ssid=ssid,
                files=self._behaviour_files(),
                scalars=[
                    {"name": name, "unit": unit, "doc": doc}
                    for name, unit, doc in behaviour_store.SCALAR_PARAMS
                ],
                led_scripts=list(behaviour_store.LED_SCRIPTS),
                gesture_scripts=list(behaviour_store.GESTURE_SCRIPTS),
                config_dir=self._behaviour_dir)

        # ── joystick API ─────────────────────────────────────────────────

        @app.route("/api/joystick/profiles")
        def api_joystick_profiles():
            """List available profiles and which one is live."""
            state = self._node.joy_state() if self._node else {}
            return jsonify({
                "ok": True,
                "config_dir": self._joystick_dir,
                "profiles": joystick_store.list_profiles(self._joystick_dir),
                "active": state.get("active_profile"),
                "estop": state.get("estop"),
            })

        @app.route("/api/joystick/profile/<name>", methods=["GET"])
        def api_joystick_read(name):
            """Return one profile's parsed document and raw text."""
            try:
                data = joystick_store.read_profile(self._joystick_dir, name)
            except joystick_store.JoystickStoreError as exc:
                return jsonify({"ok": False, "error": str(exc)}), 404
            data["ok"] = True
            data["backups"] = yaml_io.list_backups(data["path"], self._backup_root)
            return jsonify(data)

        @app.route("/api/joystick/profile/<name>", methods=["POST"])
        def api_joystick_write(name):
            """Validate and save one profile."""
            payload = request.get_json(silent=True) or {}
            document = payload.get("document")
            if document is None:
                return jsonify({"ok": False, "errors": ["No document supplied"]}), 400

            try:
                ok, errors, backup, created = joystick_store.save_profile(
                    self._joystick_dir, name, document, backup_root=self._backup_root)
            except joystick_store.JoystickStoreError as exc:
                return jsonify({"ok": False, "errors": [str(exc)]}), 400
            except OSError as exc:
                return jsonify({
                    "ok": False,
                    "errors": ["Could not write the profile: {}".format(exc)],
                }), 500

            if not ok:
                return jsonify({"ok": False, "errors": errors}), 400

            return jsonify({
                "ok": True,
                "errors": [],
                "backup": backup,
                "created": created,
                # A profile that exists only in the source tree is invisible to
                # get_package_share_directory until the workspace is rebuilt.
                "needs_rebuild": created,
            })

        @app.route("/api/joystick/reload", methods=["POST"])
        def api_joystick_reload():
            """Ask the joy node to re-read its profile from disk."""
            node, error = self._require_node()
            if error:
                return error
            ok, message = node.reload_profile()
            return jsonify({"ok": ok, "message": message})

        @app.route("/api/joystick/select", methods=["POST"])
        def api_joystick_select():
            """Switch the joy node to a different profile."""
            node, error = self._require_node()
            if error:
                return error
            payload = request.get_json(silent=True) or {}
            name = payload.get("profile")
            if not name:
                return jsonify({"ok": False, "message": "No profile supplied"}), 400
            ok, message = node.select_profile(name)
            return jsonify({"ok": ok, "message": message})

        @app.route("/api/joystick/live")
        def api_joystick_live():
            """Return the latest controller frame, for the live readout."""
            node, error = self._require_node()
            if error:
                return error
            return jsonify({"ok": True, **node.joy_state()})

        @app.route("/api/joystick/estop/clear", methods=["POST"])
        def api_joystick_clear_estop():
            """Release the joy node's e-stop latch."""
            node, error = self._require_node()
            if error:
                return error
            ok, message = node.clear_estop()
            return jsonify({"ok": ok, "message": message})

        # ── diagnostics API ──────────────────────────────────────────────

        @app.route("/api/diagnostics")
        def api_diagnostics():
            """Return measured rates for every monitored topic."""
            node, error = self._require_node()
            if error:
                return error
            return jsonify({"ok": True, **node.diagnostics()})

        @app.route("/api/diagnostics/config")
        def api_diagnostics_config():
            """Return the configured nominal rate table."""
            node, error = self._require_node()
            if error:
                return error
            return jsonify({"ok": True, "specs": node.monitor_specs()})

        @app.route("/api/diagnostics/reset", methods=["POST"])
        def api_diagnostics_reset():
            """Clear every measurement window."""
            node, error = self._require_node()
            if error:
                return error
            node.reset_rates()
            return jsonify({"ok": True})

        # ── behaviour API ────────────────────────────────────────────────

        @app.route("/api/behaviour/files")
        def api_behaviour_files():
            """List behaviour constants files and which the SSID selects."""
            ssid = current_ssid()
            return jsonify({
                "ok": True,
                "ssid": ssid,
                "selected": behaviour_file_for_ssid(ssid),
                "config_dir": self._behaviour_dir,
                "files": self._behaviour_files(),
            })

        @app.route("/api/behaviour/<name>", methods=["GET"])
        def api_behaviour_read(name):
            """Return one behaviour file, parsed and validated."""
            try:
                path = self._behaviour_path(name)
                params = behaviour_store.load_params(path)
            except behaviour_store.BehaviourStoreError as exc:
                return jsonify({"ok": False, "error": str(exc)}), 404

            errors, warnings = behaviour_store.validate(params)
            return jsonify({
                "ok": True,
                "name": name,
                "path": path,
                "params": params,
                "structured": behaviour_store.parse_params(params),
                "errors": errors,
                "warnings": warnings,
                "backups": yaml_io.list_backups(path, self._backup_root),
            })

        @app.route("/api/behaviour/<name>", methods=["POST"])
        def api_behaviour_write(name):
            """Validate and save one behaviour file."""
            try:
                path = self._behaviour_path(name)
            except behaviour_store.BehaviourStoreError as exc:
                return jsonify({"ok": False, "errors": [str(exc)]}), 404

            payload = request.get_json(silent=True) or {}
            if "structured" in payload:
                try:
                    params = behaviour_store.params_from_structured(
                        payload["structured"])
                except (KeyError, TypeError, ValueError) as exc:
                    return jsonify({
                        "ok": False,
                        "errors": ["Malformed edit: {}".format(exc)],
                    }), 400
            elif "params" in payload:
                params = payload["params"]
            else:
                return jsonify({"ok": False, "errors": ["No parameters supplied"]}), 400

            try:
                ok, errors, warnings, backup = behaviour_store.save(
                    path, params, backup_root=self._backup_root)
            except behaviour_store.BehaviourStoreError as exc:
                return jsonify({"ok": False, "errors": [str(exc)]}), 400
            except OSError as exc:
                return jsonify({
                    "ok": False,
                    "errors": ["Could not write the file: {}".format(exc)],
                }), 500

            status = 200 if ok else 400
            return jsonify({
                "ok": ok,
                "errors": errors,
                "warnings": warnings,
                "backup": backup,
                # Only ConstantParamsToBlackboard reads these, at setup() time.
                "applies": "next behaviour tree start",
            }), status

        @app.route("/api/behaviour/<name>/backups", methods=["GET"])
        def api_behaviour_backups(name):
            """List a behaviour file's backups."""
            try:
                path = self._behaviour_path(name)
            except behaviour_store.BehaviourStoreError as exc:
                return jsonify({"ok": False, "error": str(exc)}), 404
            return jsonify({
                "ok": True,
                "backups": yaml_io.list_backups(path, self._backup_root),
            })

        @app.route("/api/behaviour/<name>/restore", methods=["POST"])
        def api_behaviour_restore(name):
            """Restore a behaviour file from one of its backups."""
            try:
                path = self._behaviour_path(name)
            except behaviour_store.BehaviourStoreError as exc:
                return jsonify({"ok": False, "error": str(exc)}), 404

            payload = request.get_json(silent=True) or {}
            wanted = payload.get("backup")
            available = {entry["name"]: entry for entry in
                         yaml_io.list_backups(path, self._backup_root)}
            if wanted not in available:
                return jsonify({"ok": False, "error": "No such backup"}), 404

            try:
                yaml_io.restore_backup(available[wanted]["path"], path,
                                       root=self._backup_root)
            except OSError as exc:
                return jsonify({
                    "ok": False,
                    "error": "Could not restore: {}".format(exc),
                }), 500
            return jsonify({"ok": True, "restored": wanted})

        @app.route("/api/joystick/profile/<name>/restore", methods=["POST"])
        def api_joystick_restore(name):
            """Restore a joystick profile from one of its backups."""
            try:
                path = joystick_store._profile_path(self._joystick_dir, name)
            except joystick_store.JoystickStoreError as exc:
                return jsonify({"ok": False, "error": str(exc)}), 400

            payload = request.get_json(silent=True) or {}
            wanted = payload.get("backup")
            available = {entry["name"]: entry for entry in
                         yaml_io.list_backups(path, self._backup_root)}
            if wanted not in available:
                return jsonify({"ok": False, "error": "No such backup"}), 404

            try:
                yaml_io.restore_backup(available[wanted]["path"], path,
                                       root=self._backup_root)
            except OSError as exc:
                return jsonify({
                    "ok": False,
                    "error": "Could not restore: {}".format(exc),
                }), 500
            return jsonify({"ok": True, "restored": wanted})

    # ── serving ──────────────────────────────────────────────────────────

    def run(self, host: str = "0.0.0.0", port: int = 8080, debug: bool = False):
        """Serve the app. Blocks; call from the Flask thread."""
        self.app.run(host=host, port=port, debug=debug, use_reloader=False,
                     threaded=True)
