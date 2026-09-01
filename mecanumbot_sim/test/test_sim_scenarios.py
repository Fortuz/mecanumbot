import math
import textwrap
from pathlib import Path

import pytest

from mecanumbot_sim.sim_scenarios import (
    YOGA_MAT_LENGTH,
    YOGA_MAT_WIDTH,
    load_sim_scenario,
)

SCENARIO_ROOT = Path(__file__).resolve().parents[1] / "config" / "sim_scenarios"


def shipped_scenario(name: str) -> Path:
    return SCENARIO_ROOT / name


def write_scenario(tmp_path: Path, body: str) -> Path:
    """Write a throwaway scenario file and return its path."""
    path = tmp_path / "scenario.yaml"
    path.write_text(textwrap.dedent(body).strip() + "\n", encoding="utf-8")
    return path


def test_single_human_scenario_loads():
    scenario_path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "sim_scenarios"
        / "single_human_follow.yaml"
    )
    scenario = load_sim_scenario(str(scenario_path))

    assert scenario.name == "single_human_follow"
    assert len(scenario.actors) == 1
    actor = scenario.actors[0]
    assert actor.kind == "human"
    assert actor.body_name == "sim_human_0"
    assert actor.is_subject is True


def test_human_near_wall_scenario_loads_both_actor_types():
    scenario_path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "sim_scenarios"
        / "human_near_wall.yaml"
    )
    scenario = load_sim_scenario(str(scenario_path))

    assert scenario.name == "human_near_wall"
    assert len(scenario.actors) == 2
    kinds = {actor.kind for actor in scenario.actors}
    assert kinds == {"human", "wall"}
    assert scenario.subject_actor is not None
    assert scenario.subject_actor.kind == "human"


def test_moving_human_patrol_scenario_loads_motion_profile():
    scenario_path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "sim_scenarios"
        / "moving_human_patrol.yaml"
    )
    scenario = load_sim_scenario(str(scenario_path))

    assert scenario.name == "moving_human_patrol"
    actor = scenario.subject_actor
    assert actor is not None
    assert actor.motion_mode == "patrol"
    assert actor.motion_speed > 0.0
    assert len(actor.motion_waypoints) == 2


def test_two_humans_wall_discrimination_scenario_loads_phase3_actors():
    scenario_path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "sim_scenarios"
        / "two_humans_wall_discrimination.yaml"
    )
    scenario = load_sim_scenario(str(scenario_path))

    assert scenario.name == "two_humans_wall_discrimination"
    assert len(scenario.actors) == 4
    assert scenario.subject_actor is not None
    assert scenario.subject_actor.body_name == "sim_human_0"
    assert {actor.body_name for actor in scenario.actors} == {
        "sim_human_0",
        "sim_human_1",
        "sim_wall_panel_0",
        "sim_wall_panel_1",
    }
    assert sum(1 for actor in scenario.actors if actor.kind == "human") == 2
    assert sum(1 for actor in scenario.actors if actor.kind == "wall") == 2


def test_phase4_behavior_scenarios_load():
    scenario_root = Path(__file__).resolve().parents[1] / "config" / "sim_scenarios"
    scenario_names = {
        "behavior_wall_false_positive.yaml",
        "behavior_subject_stop.yaml",
        "behavior_wrong_human_crossing.yaml",
    }

    for scenario_name in scenario_names:
        scenario = load_sim_scenario(str(scenario_root / scenario_name))
        assert scenario.name == scenario_name.removesuffix(".yaml")
        assert scenario.subject_actor is not None
        assert any(actor.kind == "human" for actor in scenario.actors)


def test_exercise_mat_stationary_scenario_loads_gait_and_props():
    scenario = load_sim_scenario(str(shipped_scenario("exercise_mat_stationary.yaml")))

    assert scenario.name == "exercise_mat_stationary"
    actor = scenario.subject_actor
    assert actor is not None
    # An exercising person does not translate: the routine is what moves.
    assert actor.motion_mode == "static"
    assert actor.gait.mode == "routine"
    assert actor.gait.routine == ("jacks", "march", "side_bend", "step_touch")
    assert actor.gait.is_active is True
    assert math.isclose(actor.gait.leg_spread, math.radians(28.0))

    assert [prop.kind for prop in scenario.props] == ["mat", "cube", "cube", "cube"]
    mat = scenario.props[0]
    assert mat.body_name == "sim_mat_0"
    assert mat.mat_extents() == (YOGA_MAT_LENGTH, YOGA_MAT_WIDTH)


def test_fear_hri_brick_carry_scenario_loads_the_full_scene():
    scenario = load_sim_scenario(str(shipped_scenario("fear_hri_brick_carry.yaml")))

    assert scenario.name == "fear_hri_brick_carry"
    actor = scenario.subject_actor
    assert actor is not None
    assert actor.gait.mode == "routine"
    assert actor.motion_mode == "patrol"
    # Out to the carry corridor and back again, so fear has to decay on its own
    # rather than being reset by the person simply being gone.
    assert len(actor.motion_waypoints) == 9
    # The dwell on the mat waypoints is what keeps them exercising rather than
    # pacing: most of the loop is spent standing on the mat.
    assert sum(waypoint.pause for waypoint in actor.motion_waypoints) > 30.0

    mats = [prop for prop in scenario.props if prop.kind == "mat"]
    cubes = [prop for prop in scenario.props if prop.kind == "cube"]
    assert {prop.name for prop in mats} == {"exercise_mat", "goal_pad"}
    assert len(cubes) == 4
    # Sized for the grabbers, not just for pushing.
    assert all(cube.size == (0.07,) and cube.mass == 0.15 for cube in cubes)
    assert {cube.body_name for cube in cubes} == {
        "sim_cube_0",
        "sim_cube_1",
        "sim_cube_2",
        "sim_cube_3",
    }


def test_scenarios_without_a_gait_block_default_to_standing_still():
    scenario = load_sim_scenario(str(shipped_scenario("moving_human_patrol.yaml")))

    actor = scenario.subject_actor
    assert actor.gait.mode == "none"
    assert actor.gait.is_active is False
    assert scenario.props == ()
    # Waypoints with no `pause` still parse, and dwell for nothing.
    assert all(waypoint.pause == 0.0 for waypoint in actor.motion_waypoints)


def test_gait_amplitudes_are_written_in_degrees_and_stored_in_radians(tmp_path):
    path = write_scenario(
        tmp_path,
        """
        scenario:
          name: degrees
          actors:
            - body_name: sim_human_0
              gait:
                mode: jacks
                leg_spread_deg: 30
                arm_raise_deg: 90
                torso_bend_deg: 15
        """,
    )

    gait = load_sim_scenario(str(path)).actors[0].gait

    assert math.isclose(gait.leg_spread, math.radians(30.0))
    assert math.isclose(gait.arm_raise, math.radians(90.0))
    assert math.isclose(gait.torso_bend, math.radians(15.0))
    # Anything not given keeps the packaged default.
    assert gait.leg_swing == pytest.approx(math.radians(32.0))


def test_a_routine_may_only_contain_real_moves(tmp_path):
    path = write_scenario(
        tmp_path,
        """
        scenario:
          name: bad_routine
          actors:
            - body_name: sim_human_0
              gait: {mode: routine, routine: [jacks, moonwalk]}
        """,
    )

    with pytest.raises(ValueError, match="moonwalk"):
        load_sim_scenario(str(path))


def test_a_routine_without_a_list_falls_back_to_the_packaged_one(tmp_path):
    path = write_scenario(
        tmp_path,
        """
        scenario:
          name: bare_routine
          actors:
            - body_name: sim_human_0
              gait: {mode: routine}
        """,
    )

    gait = load_sim_scenario(str(path)).actors[0].gait

    assert gait.mode == "routine"
    assert gait.routine
    assert all(
        move in ("march", "jacks", "step_touch", "side_bend") for move in gait.routine
    )


def test_a_mat_size_may_be_a_square_or_a_rectangle(tmp_path):
    path = write_scenario(
        tmp_path,
        """
        scenario:
          name: mat_sizes
          props:
            - {body_name: sim_mat_0, size: [1.73, 0.61]}
            - {body_name: sim_mat_1, size: 0.70}
        """,
    )

    yoga, pad = load_sim_scenario(str(path)).props

    assert yoga.mat_extents() == (1.73, 0.61)
    assert pad.mat_extents() == (0.70, 0.70)


def test_a_mat_with_no_size_is_a_yoga_mat(tmp_path):
    path = write_scenario(
        tmp_path,
        """
        scenario:
          name: default_mat
          props:
            - {body_name: sim_mat_0}
        """,
    )

    assert load_sim_scenario(str(path)).props[0].mat_extents() == (
        YOGA_MAT_LENGTH,
        YOGA_MAT_WIDTH,
    )


def test_unsupported_gait_mode_is_rejected(tmp_path):
    path = write_scenario(
        tmp_path,
        """
        scenario:
          name: bad_gait
          actors:
            - body_name: sim_human_0
              gait: {mode: moonwalk}
        """,
    )

    with pytest.raises(ValueError, match="moonwalk"):
        load_sim_scenario(str(path))


def test_unsupported_prop_body_is_rejected(tmp_path):
    path = write_scenario(
        tmp_path,
        """
        scenario:
          name: bad_prop
          props:
            - body_name: sim_pyramid_0
        """,
    )

    with pytest.raises(ValueError, match="sim_pyramid_0"):
        load_sim_scenario(str(path))


def test_a_prop_body_cannot_be_used_twice(tmp_path):
    path = write_scenario(
        tmp_path,
        """
        scenario:
          name: reused_prop
          props:
            - body_name: sim_cube_0
            - body_name: sim_cube_0
        """,
    )

    with pytest.raises(ValueError, match="reuses a prop body"):
        load_sim_scenario(str(path))


def test_prop_kind_defaults_to_its_body_pool(tmp_path):
    path = write_scenario(
        tmp_path,
        """
        scenario:
          name: implicit_kinds
          props:
            - body_name: sim_mat_1
            - body_name: sim_cube_2
        """,
    )

    scenario = load_sim_scenario(str(path))

    assert [prop.kind for prop in scenario.props] == ["mat", "cube"]
