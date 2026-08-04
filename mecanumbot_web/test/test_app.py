"""
Route-level tests for the web GUI, using Flask's test client.

No ROS graph: the node is a stub recording what the routes asked it to
do.  The pattern is carried over from the GUI this replaces, which tested
its Flask layer the same way.
"""

import os
import shutil
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanumbot_web import behaviour_store  # noqa: E402

# Deliberately NOT pytest.importorskip: raising Skipped at module level
# aborts collection for the whole session under pytest 6.2 (the version
# Humble ships), so a missing Flask would silently reduce `colcon test`
# from 130 tests to 1 rather than skipping this file. A module-level
# pytestmark skips only this module and leaves the rest collectable.
try:
    import flask  # noqa: F401

    from mecanumbot_web.app import WebApp, behaviour_file_for_ssid
    FLASK_AVAILABLE = True
except ImportError:  # pragma: no cover - depends on the host
    FLASK_AVAILABLE = False
    WebApp = None
    behaviour_file_for_ssid = None

pytestmark = pytest.mark.skipif(
    not FLASK_AVAILABLE,
    reason="python3-flask is not installed (sudo apt install python3-flask)")

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
SRC_DIR = os.path.dirname(PKG_ROOT)
JOYSTICK_DIR = os.path.join(
    PKG_ROOT, "mecanumbot_description", "config", "joystick")
BEHAVIOUR_DIR = os.path.join(
    SRC_DIR, "mecanumbot_behaviours", "mecanumbot_leading_behaviour", "config")


class StubNode:
    """Records calls the routes make, so no ROS graph is needed."""

    def __init__(self):
        """Start with an idle robot and no recorded calls."""
        self.calls = []
        self.reload_result = (True, "Reloaded 'xbox360'")
        self.select_result = (True, "Profile set to 'generic'")
        self.estop_result = (True, "E-stop cleared")

    def joy_state(self):
        """Return a plausible controller frame."""
        self.calls.append("joy_state")
        return {"connected": True, "axes": [0.0, 1.0], "buttons": [0, 1],
                "age_s": 0.05, "active_profile": "xbox360", "estop": False,
                "joy_topic": "/mecanumbot/joy", "joy_node": "/n"}

    def diagnostics(self):
        """Return one healthy reading."""
        self.calls.append("diagnostics")
        return {"topics": [{"topic": "/a", "status": "OK", "measured_hz": 100.0}],
                "summary": {"OK": 1}, "window_seconds": 3.0, "stale_after": 2.0}

    def monitor_specs(self):
        """Return the configured nominal table."""
        return [{"topic": "/a", "nominal_hz": 100.0, "tol": 0.3,
                 "label": "A", "note": ""}]

    def reset_rates(self):
        """Record that the windows were cleared."""
        self.calls.append("reset_rates")

    def reload_profile(self):
        """Record a reload request."""
        self.calls.append("reload_profile")
        return self.reload_result

    def clear_estop(self):
        """Record an e-stop clear request."""
        self.calls.append("clear_estop")
        return self.estop_result

    def select_profile(self, name):
        """Record a profile switch request."""
        self.calls.append(("select_profile", name))
        return self.select_result


@pytest.fixture
def context(tmp_path):
    """Build a test client over writable copies of both config directories."""
    joystick_dir = tmp_path / "joystick"
    joystick_dir.mkdir()
    for name in os.listdir(JOYSTICK_DIR):
        if name.endswith(".yaml"):
            shutil.copy(os.path.join(JOYSTICK_DIR, name), str(joystick_dir))

    behaviour_dir = tmp_path / "behaviour"
    behaviour_dir.mkdir()
    if os.path.isdir(BEHAVIOUR_DIR):
        for name in os.listdir(BEHAVIOUR_DIR):
            if name.endswith(".yaml"):
                shutil.copy(os.path.join(BEHAVIOUR_DIR, name), str(behaviour_dir))

    node = StubNode()
    web = WebApp(node=node, joystick_dir=str(joystick_dir),
                 behaviour_dir=str(behaviour_dir),
                 backup_root=str(tmp_path / "backups"))
    web.app.config["TESTING"] = True
    return {"client": web.app.test_client(), "node": node,
            "joystick_dir": str(joystick_dir), "behaviour_dir": str(behaviour_dir)}


# ── pages ────────────────────────────────────────────────────────────────

def test_root_redirects_to_diagnostics(context):
    """The operator lands somewhere useful with no login in the way."""
    response = context["client"].get("/")
    assert response.status_code == 302
    assert "/diagnostics" in response.headers["Location"]


@pytest.mark.parametrize("path", ["/joystick", "/diagnostics", "/behaviour"])
def test_pages_render(context, path):
    """All three pages return HTML."""
    response = context["client"].get(path)
    assert response.status_code == 200
    assert b"Mecanumbot" in response.data


def test_there_is_no_login_page(context):
    """The user model is gone; nothing should ask for a name."""
    response = context["client"].get("/diagnostics")
    assert b"Sign In" not in response.data
    assert b"user_name" not in response.data


def test_behaviour_page_states_when_edits_apply(context):
    """The page must not imply that edits take effect live."""
    response = context["client"].get("/behaviour")
    assert b"next time a behaviour tree starts" in response.data


# ── joystick API ─────────────────────────────────────────────────────────

def test_lists_profiles_with_the_active_one(context):
    """The page can tell which profile the node is actually running."""
    body = context["client"].get("/api/joystick/profiles").get_json()
    assert body["ok"] is True
    assert {entry["stem"] for entry in body["profiles"]} == \
        {"xbox360", "generic", "ps4"}
    assert body["active"] == "xbox360"


def test_reads_one_profile(context):
    """Reading returns the document, its controls and its backups."""
    body = context["client"].get("/api/joystick/profile/xbox360").get_json()
    assert body["ok"] is True
    assert body["document"]["profile"]["name"] == "xbox360"
    assert "left_y" in body["controls"]["axes"]
    assert body["errors"] == []


def test_reading_a_missing_profile_is_404(context):
    """A clear failure, not an empty page."""
    assert context["client"].get("/api/joystick/profile/nope").status_code == 404


def test_saving_a_valid_profile_succeeds(context):
    """A good edit lands on disk."""
    document = context["client"].get(
        "/api/joystick/profile/xbox360").get_json()["document"]
    document["profile"]["limits"]["max_lin_vel"] = 0.2

    body = context["client"].post(
        "/api/joystick/profile/xbox360", json={"document": document}).get_json()
    assert body["ok"] is True

    reread = context["client"].get("/api/joystick/profile/xbox360").get_json()
    assert reread["document"]["profile"]["limits"]["max_lin_vel"] == 0.2


def test_saving_an_invalid_profile_is_rejected_with_reasons(context):
    """The operator sees why, and the file is untouched."""
    document = context["client"].get(
        "/api/joystick/profile/xbox360").get_json()["document"]
    document["profile"]["bindings"]["drive"]["axis"] = "nonexistent"

    response = context["client"].post(
        "/api/joystick/profile/xbox360", json={"document": document})
    assert response.status_code == 400
    assert any("nonexistent" in error for error in response.get_json()["errors"])


def test_saving_without_a_document_is_a_400(context):
    """A malformed request is not a server error."""
    assert context["client"].post(
        "/api/joystick/profile/xbox360", json={}).status_code == 400


def test_reload_proxies_to_the_node(context):
    """The Reload button reaches the joy node's Trigger service."""
    body = context["client"].post("/api/joystick/reload").get_json()
    assert body["ok"] is True
    assert "reload_profile" in context["node"].calls


def test_reload_failure_is_reported_verbatim(context):
    """A rejected reload must explain itself, not just fail."""
    context["node"].reload_result = (False, "Reload rejected, keeping 'xbox360'")
    body = context["client"].post("/api/joystick/reload").get_json()
    assert body["ok"] is False
    assert "keeping" in body["message"]


def test_select_proxies_the_profile_name(context):
    """Making a profile active sets the node's parameter."""
    context["client"].post("/api/joystick/select", json={"profile": "generic"})
    assert ("select_profile", "generic") in context["node"].calls


def test_select_without_a_name_is_a_400(context):
    """No silent no-op."""
    assert context["client"].post(
        "/api/joystick/select", json={}).status_code == 400


def test_live_readout_returns_the_frame(context):
    """The live input panel has something to poll."""
    body = context["client"].get("/api/joystick/live").get_json()
    assert body["connected"] is True
    assert body["axes"] == [0.0, 1.0]


def test_clear_estop_proxies_to_the_node(context):
    """The e-stop can be released from the page."""
    context["client"].post("/api/joystick/estop/clear")
    assert "clear_estop" in context["node"].calls


def test_restoring_a_joystick_backup(context):
    """An edit can be rolled back from the GUI."""
    client = context["client"]
    document = client.get("/api/joystick/profile/xbox360").get_json()["document"]
    document["profile"]["limits"]["max_lin_vel"] = 0.111
    client.post("/api/joystick/profile/xbox360", json={"document": document})

    backups = client.get("/api/joystick/profile/xbox360").get_json()["backups"]
    assert backups
    body = client.post("/api/joystick/profile/xbox360/restore",
                       json={"backup": backups[0]["name"]}).get_json()
    assert body["ok"] is True

    reread = client.get("/api/joystick/profile/xbox360").get_json()
    assert reread["document"]["profile"]["limits"]["max_lin_vel"] == 0.234


def test_restoring_an_unknown_backup_is_404(context):
    """Backup names are not free-form paths."""
    assert context["client"].post(
        "/api/joystick/profile/xbox360/restore",
        json={"backup": "../../etc/passwd"}).status_code == 404


# ── diagnostics API ──────────────────────────────────────────────────────

def test_diagnostics_returns_readings(context):
    """The page's 1 Hz poll has something to render."""
    body = context["client"].get("/api/diagnostics").get_json()
    assert body["ok"] is True
    assert body["topics"][0]["topic"] == "/a"


def test_diagnostics_config_is_available_before_data(context):
    """The table renders before the first measurement arrives."""
    body = context["client"].get("/api/diagnostics/config").get_json()
    assert body["specs"][0]["nominal_hz"] == 100.0


def test_diagnostics_reset_reaches_the_monitor(context):
    """Reset clears the windows rather than only the display."""
    context["client"].post("/api/diagnostics/reset")
    assert "reset_rates" in context["node"].calls


# ── behaviour API ────────────────────────────────────────────────────────

def _has_behaviour(context):
    """Report whether the behaviour repo was available to copy from."""
    return bool(os.listdir(context["behaviour_dir"]))


def test_behaviour_files_are_listed(context):
    """Both experiment configurations show up, with the SSID choice marked."""
    if not _has_behaviour(context):
        pytest.skip("mecanumbot_leading_behaviour is not checked out")
    body = context["client"].get("/api/behaviour/files").get_json()
    assert body["ok"] is True
    assert any(entry["name"] == "behaviour_setting_constants.yaml"
               for entry in body["files"])
    assert body["selected"]


def test_behaviour_file_reads_with_warnings(context):
    """The known surplus-delay mismatches surface as warnings, not errors."""
    if not _has_behaviour(context):
        pytest.skip("mecanumbot_leading_behaviour is not checked out")
    body = context["client"].get(
        "/api/behaviour/behaviour_setting_constants.yaml").get_json()
    assert body["ok"] is True
    assert body["errors"] == []
    assert any("never read" in warning for warning in body["warnings"])
    assert body["structured"]["scalars"]["init_delay"] == 1.5


def test_behaviour_save_round_trips(context):
    """An edit through the structured form lands correctly."""
    if not _has_behaviour(context):
        pytest.skip("mecanumbot_leading_behaviour is not checked out")
    client = context["client"]
    name = "behaviour_setting_constants.yaml"

    structured = client.get("/api/behaviour/" + name).get_json()["structured"]
    structured["scalars"]["robot_closeness_threshold"] = 0.55

    body = client.post("/api/behaviour/" + name,
                       json={"structured": structured}).get_json()
    assert body["ok"] is True
    assert body["applies"] == "next behaviour tree start"

    reread = client.get("/api/behaviour/" + name).get_json()
    assert reread["structured"]["scalars"]["robot_closeness_threshold"] == 0.55
    assert reread["errors"] == []


def test_behaviour_save_rejects_a_short_delay_list(context):
    """The IndexError-at-runtime case must be blocked at save time."""
    if not _has_behaviour(context):
        pytest.skip("mecanumbot_leading_behaviour is not checked out")
    client = context["client"]
    name = "behaviour_setting_constants.yaml"

    structured = client.get("/api/behaviour/" + name).get_json()["structured"]
    structured["led_scripts"]["LED_thank"]["times"] = [1.0]

    response = client.post("/api/behaviour/" + name, json={"structured": structured})
    assert response.status_code == 400
    assert any("IndexError" in error for error in response.get_json()["errors"])


def test_behaviour_path_traversal_is_rejected(context):
    """File names are confined to the configured directory."""
    for bad in ("../../../etc/passwd", "..%2Fescape.yaml", "notyaml.txt"):
        assert context["client"].get("/api/behaviour/" + bad).status_code in (404, 400)


def test_behaviour_save_to_an_unknown_file_is_404(context):
    """The GUI cannot create arbitrary files."""
    assert context["client"].post(
        "/api/behaviour/invented.yaml", json={"params": {}}).status_code == 404


# ── SSID selection ───────────────────────────────────────────────────────

def test_ssid_selection_matches_the_launch_files():
    """The page must report the same choice the launch files make."""
    assert behaviour_file_for_ssid("MecanumNet") == "behaviour_setting_constants.yaml"
    assert behaviour_file_for_ssid("MecanumetoNet") == \
        "Eto_behaviour_setting_constants.yaml"
    assert behaviour_file_for_ssid("SomethingElse") == \
        "behaviour_setting_constants.yaml"
    assert behaviour_file_for_ssid(None) == "behaviour_setting_constants.yaml"


# ── degraded mode ────────────────────────────────────────────────────────

def test_ros_routes_503_without_a_node(tmp_path):
    """If rclpy failed to start, robot routes say so instead of hanging."""
    web = WebApp(node=None, joystick_dir=str(tmp_path), behaviour_dir=str(tmp_path))
    web.app.config["TESTING"] = True
    client = web.app.test_client()

    for path in ("/api/diagnostics", "/api/joystick/live"):
        assert client.get(path).status_code == 503
    assert client.post("/api/joystick/reload").status_code == 503


def test_file_routes_still_work_without_a_node(context, tmp_path):
    """Editing config does not need the robot to be running."""
    web = WebApp(node=None, joystick_dir=context["joystick_dir"],
                 behaviour_dir=context["behaviour_dir"],
                 backup_root=str(tmp_path / "b"))
    web.app.config["TESTING"] = True
    client = web.app.test_client()

    assert client.get("/api/joystick/profiles").status_code == 200
    assert client.get("/joystick").status_code == 200


def test_scalar_metadata_matches_the_store():
    """The page's threshold list stays in step with what is validated."""
    names = {name for name, _unit, _doc in behaviour_store.SCALAR_PARAMS}
    assert names == set(behaviour_store.SCALAR_NAMES)
