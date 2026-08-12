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
``/behaviour``    choose a behaviour tree, edit its constants, run it

There is deliberately no user model.  The old "login" was a
process-global name rather than a session -- two browsers already shared
one identity -- so on a single-operator robot GUI it was friction that
bought nothing.  That does mean anything that can reach port 8080 can
start a behaviour tree, so this belongs on the robot's own network, the
same as every other control surface in the workspace.
"""

import os
import subprocess
from typing import Dict, List, Optional

from flask import Flask, jsonify, redirect, render_template, request, url_for

from . import (behaviour_catalog, behaviour_runner, behaviour_store,
               flat_store, joystick_store, yaml_io)

#: Wi-Fi SSID to behaviour constants file, for the leading behaviour.
#: Held in the catalog, which mirrors what each behaviour's own launch
#: file picks, so the page and the launch file cannot disagree about
#: which room the robot thinks it is in.
SSID_BEHAVIOUR_FILES = dict(behaviour_catalog.LEADING.ssid_files)

DEFAULT_BEHAVIOUR_FILE = behaviour_catalog.LEADING.default_file


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
    """Return the leading-behaviour file the current SSID selects."""
    return behaviour_catalog.LEADING.file_for_ssid(ssid)


class WebApp:
    """The Flask application, wired to a :class:`~.ros_bridge.WebNode`."""

    def __init__(
        self,
        node=None,
        joystick_dir: str = "",
        behaviour_dir: str = "",
        behaviour_dirs: Optional[Dict[str, str]] = None,
        runner=None,
        backup_root: str = yaml_io.DEFAULT_BACKUP_ROOT,
    ):
        """Build the app. ``node`` may be None, in which case ROS routes 503."""
        self._node = node
        self._joystick_dir = joystick_dir
        # One config directory per behaviour. ``behaviour_dir`` names the
        # leading behaviour's, which is the one the un-scoped config
        # routes address and the only one that existed before the page
        # could start trees.
        self._behaviour_dirs = dict(behaviour_dirs or {})
        if behaviour_dir:
            self._behaviour_dirs.setdefault(behaviour_catalog.DEFAULT_BEHAVIOUR,
                                            behaviour_dir)
        self._behaviour_dir = self._behaviour_dirs.get(
            behaviour_catalog.DEFAULT_BEHAVIOUR, "")
        self._backup_root = backup_root

        # The runner spawns processes but touches no ROS API, so the app
        # owns it and the node only has to stop it on shutdown.
        self.runner = runner if runner is not None else behaviour_runner.BehaviourRunner(
            config_dirs=self._behaviour_dirs,
            logger=node.get_logger() if node is not None else None)

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

    def _behaviour_spec(self, key: Optional[str]):
        """Resolve a behaviour key, defaulting to the leading behaviour."""
        try:
            return behaviour_catalog.get(key or behaviour_catalog.DEFAULT_BEHAVIOUR)
        except behaviour_catalog.CatalogError as exc:
            raise behaviour_store.BehaviourStoreError(str(exc)) from exc

    def _behaviour_dir_for(self, key: Optional[str]) -> str:
        """Return the config directory of one behaviour."""
        return self._behaviour_dirs.get(
            (key or behaviour_catalog.DEFAULT_BEHAVIOUR), "")

    def _behaviour_path(self, name: str, behaviour: Optional[str] = None) -> str:
        """Resolve a constants filename inside one behaviour's directory."""
        spec = self._behaviour_spec(behaviour)
        if not name or os.sep in name or not name.endswith(".yaml") or \
                name.startswith("."):
            raise behaviour_store.BehaviourStoreError(
                "Invalid behaviour file name '{}'".format(name))
        path = os.path.join(self._behaviour_dir_for(spec.key), name)
        if not os.path.isfile(path):
            raise behaviour_store.BehaviourStoreError(
                "No such behaviour file '{}'".format(name))
        return path

    def _behaviour_files(self, behaviour: Optional[str] = None) -> List[dict]:
        """List one behaviour's constants files, available for editing."""
        spec = self._behaviour_spec(behaviour)
        config_dir = self._behaviour_dir_for(spec.key)
        selected = spec.file_for_ssid(current_ssid())
        files = []
        for name in behaviour_catalog.list_config_files(config_dir):
            path = os.path.join(config_dir, name)
            files.append({
                "name": name,
                "path": path,
                "behaviour": spec.key,
                "selected_by_ssid": name == selected,
                "symlinked": yaml_io.is_symlinked_install(path),
            })
        return files

    def _read_constants(self, spec, name: str, path: str) -> dict:
        """Return one constants file in the shape its editor needs."""
        if spec.editor == "flat":
            described = flat_store.describe(path)
            return {
                "ok": True,
                "name": name,
                "behaviour": spec.key,
                "editor": "flat",
                "path": path,
                "flat": described,
                "errors": described["errors"],
                "warnings": described["warnings"],
                "backups": yaml_io.list_backups(path, self._backup_root),
            }

        params = behaviour_store.load_params(path)
        errors, warnings = behaviour_store.validate(params)
        return {
            "ok": True,
            "name": name,
            "behaviour": spec.key,
            "editor": "leading",
            "path": path,
            "params": params,
            "structured": behaviour_store.annotate_tunables(
                behaviour_store.parse_params(params), path),
            "errors": errors,
            "warnings": warnings,
            "backups": yaml_io.list_backups(path, self._backup_root),
        }

    def _run_status(self) -> dict:
        """Return the runner's status, plus any tree already on the graph."""
        status = self.runner.status()
        graph = []
        if self._node is not None:
            try:
                graph = self._node.behaviour_nodes(behaviour_catalog.all_node_names())
            except Exception:  # pragma: no cover - rmw-dependent
                graph = []
        # A tree started from a terminal is invisible to the runner but
        # very visible to the robot, and starting a second one would put
        # two trees on /goal_pose and /cmd_vel at once.
        status["graph_nodes"] = graph
        return status

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
            """Render the behaviour runner and constants editor."""
            ssid = current_ssid()
            return render_template(
                "behaviour.html",
                page="behaviour",
                ssid=ssid,
                behaviours=behaviour_catalog.describe_all(
                    self._behaviour_dirs, ssid),
                default_behaviour=behaviour_catalog.DEFAULT_BEHAVIOUR,
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

        @app.route("/api/behaviours")
        def api_behaviours():
            """List every startable behaviour, and what is running now."""
            ssid = current_ssid()
            return jsonify({
                "ok": True,
                "ssid": ssid,
                "behaviours": behaviour_catalog.describe_all(
                    self._behaviour_dirs, ssid),
                "status": self._run_status(),
            })

        @app.route("/api/behaviours/status")
        def api_behaviours_status():
            """Return the run status, with whatever log is new since ``after``."""
            try:
                after = int(request.args.get("after", 0))
            except (TypeError, ValueError):
                after = 0
            return jsonify({
                "ok": True,
                "status": self._run_status(),
                "log": self.runner.log(after),
            })

        @app.route("/api/behaviours/start", methods=["POST"])
        def api_behaviours_start():
            """Start one behaviour with the hyperparameters supplied."""
            payload = request.get_json(silent=True) or {}
            try:
                status = self.runner.start(
                    payload.get("behaviour"), payload.get("arguments") or {})
            except behaviour_runner.RunnerError as exc:
                return jsonify({"ok": False, "error": str(exc),
                                "status": self._run_status()}), 400
            return jsonify({"ok": True, "status": status})

        @app.route("/api/behaviours/stop", methods=["POST"])
        def api_behaviours_stop():
            """Stop the running behaviour."""
            try:
                status = self.runner.stop()
            except behaviour_runner.RunnerError as exc:
                return jsonify({"ok": False, "error": str(exc),
                                "status": self._run_status()}), 400
            return jsonify({"ok": True, "status": status})

        @app.route("/api/behaviour/files")
        def api_behaviour_files():
            """List one behaviour's constants files and which the SSID selects."""
            ssid = current_ssid()
            behaviour = request.args.get("behaviour")
            try:
                spec = self._behaviour_spec(behaviour)
            except behaviour_store.BehaviourStoreError as exc:
                return jsonify({"ok": False, "error": str(exc)}), 404
            return jsonify({
                "ok": True,
                "ssid": ssid,
                "behaviour": spec.key,
                "selected": spec.file_for_ssid(ssid),
                "config_dir": self._behaviour_dir_for(spec.key),
                "files": self._behaviour_files(spec.key),
            })

        @app.route("/api/behaviour/<name>", methods=["GET"])
        def api_behaviour_read(name):
            """Return one constants file, parsed and validated for its editor."""
            try:
                spec = self._behaviour_spec(request.args.get("behaviour"))
                path = self._behaviour_path(name, spec.key)
                return jsonify(self._read_constants(spec, name, path))
            except (behaviour_store.BehaviourStoreError,
                    flat_store.FlatStoreError) as exc:
                return jsonify({"ok": False, "error": str(exc)}), 404

        @app.route("/api/behaviour/<name>", methods=["POST"])
        def api_behaviour_write(name):
            """Validate and save one constants file."""
            try:
                spec = self._behaviour_spec(request.args.get("behaviour"))
                path = self._behaviour_path(name, spec.key)
            except behaviour_store.BehaviourStoreError as exc:
                return jsonify({"ok": False, "errors": [str(exc)]}), 404

            payload = request.get_json(silent=True) or {}

            if spec.editor == "flat":
                if "values" not in payload:
                    return jsonify({"ok": False,
                                    "errors": ["No values supplied"]}), 400
                try:
                    ok, errors, warnings, backup = flat_store.save(
                        path, payload["values"], backup_root=self._backup_root)
                except flat_store.FlatStoreError as exc:
                    return jsonify({"ok": False, "errors": [str(exc)]}), 400
                except OSError as exc:
                    return jsonify({
                        "ok": False,
                        "errors": ["Could not write the file: {}".format(exc)],
                    }), 500
                return jsonify({
                    "ok": ok, "errors": errors, "warnings": warnings,
                    "backup": backup,
                    "applies": "next behaviour tree start",
                }), (200 if ok else 400)

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
                # Only the tree's params loader reads these, at setup() time.
                "applies": "next behaviour tree start",
            }), status

        @app.route("/api/behaviour/<name>/backups", methods=["GET"])
        def api_behaviour_backups(name):
            """List a constants file's backups."""
            try:
                path = self._behaviour_path(name, request.args.get("behaviour"))
            except behaviour_store.BehaviourStoreError as exc:
                return jsonify({"ok": False, "error": str(exc)}), 404
            return jsonify({
                "ok": True,
                "backups": yaml_io.list_backups(path, self._backup_root),
            })

        @app.route("/api/behaviour/<name>/restore", methods=["POST"])
        def api_behaviour_restore(name):
            """Restore a constants file from one of its backups."""
            try:
                path = self._behaviour_path(name, request.args.get("behaviour"))
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
