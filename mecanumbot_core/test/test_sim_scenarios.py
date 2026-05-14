from pathlib import Path

from mecanumbot_core.sim_scenarios import load_sim_scenario


def test_single_human_scenario_loads():
    scenario_path = Path(__file__).resolve().parents[2] / 'mecanumbot_bringup' / 'config' / 'sim_scenarios' / 'single_human_follow.yaml'
    scenario = load_sim_scenario(str(scenario_path))

    assert scenario.name == 'single_human_follow'
    assert len(scenario.actors) == 1
    actor = scenario.actors[0]
    assert actor.kind == 'human'
    assert actor.body_name == 'sim_human_0'
    assert actor.is_subject is True


def test_human_near_wall_scenario_loads_both_actor_types():
    scenario_path = Path(__file__).resolve().parents[2] / 'mecanumbot_bringup' / 'config' / 'sim_scenarios' / 'human_near_wall.yaml'
    scenario = load_sim_scenario(str(scenario_path))

    assert scenario.name == 'human_near_wall'
    assert len(scenario.actors) == 2
    kinds = {actor.kind for actor in scenario.actors}
    assert kinds == {'human', 'wall'}
    assert scenario.subject_actor is not None
    assert scenario.subject_actor.kind == 'human'


def test_moving_human_patrol_scenario_loads_motion_profile():
    scenario_path = Path(__file__).resolve().parents[2] / 'mecanumbot_bringup' / 'config' / 'sim_scenarios' / 'moving_human_patrol.yaml'
    scenario = load_sim_scenario(str(scenario_path))

    assert scenario.name == 'moving_human_patrol'
    actor = scenario.subject_actor
    assert actor is not None
    assert actor.motion_mode == 'patrol'
    assert actor.motion_speed > 0.0
    assert len(actor.motion_waypoints) == 2


def test_two_humans_wall_discrimination_scenario_loads_phase3_actors():
    scenario_path = Path(__file__).resolve().parents[2] / 'mecanumbot_bringup' / 'config' / 'sim_scenarios' / 'two_humans_wall_discrimination.yaml'
    scenario = load_sim_scenario(str(scenario_path))

    assert scenario.name == 'two_humans_wall_discrimination'
    assert len(scenario.actors) == 4
    assert scenario.subject_actor is not None
    assert scenario.subject_actor.body_name == 'sim_human_0'
    assert {actor.body_name for actor in scenario.actors} == {
        'sim_human_0',
        'sim_human_1',
        'sim_wall_panel_0',
        'sim_wall_panel_1',
    }
    assert sum(1 for actor in scenario.actors if actor.kind == 'human') == 2
    assert sum(1 for actor in scenario.actors if actor.kind == 'wall') == 2


def test_phase4_behavior_scenarios_load():
    scenario_root = Path(__file__).resolve().parents[2] / 'mecanumbot_bringup' / 'config' / 'sim_scenarios'
    scenario_names = {
        'behavior_wall_false_positive.yaml',
        'behavior_subject_stop.yaml',
        'behavior_wrong_human_crossing.yaml',
    }

    for scenario_name in scenario_names:
        scenario = load_sim_scenario(str(scenario_root / scenario_name))
        assert scenario.name == scenario_name.removesuffix('.yaml')
        assert scenario.subject_actor is not None
        assert any(actor.kind == 'human' for actor in scenario.actors)
