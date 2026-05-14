from geometry_msgs.msg import PoseStamped
from mecanumbot_core.mecanumbot_sim_detection_evaluator_node import MecanumbotSimDetectionEvaluatorNode
from mecanumbot_msgs.msg import SimActor, SimActorArray


def make_actor(actor_id, name, kind, x, y, is_subject=False):
    actor = SimActor()
    actor.id = actor_id
    actor.name = name
    actor.kind = kind
    actor.visible_to_lidar = True
    actor.is_subject = is_subject
    actor.pose.position.x = x
    actor.pose.position.y = y
    return actor


def make_pose(x, y):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    return pose


def make_evaluator(actors, pose, raw_detection_count):
    evaluator = object.__new__(MecanumbotSimDetectionEvaluatorNode)
    evaluator.latest_actors = SimActorArray()
    evaluator.latest_actors.scenario_name = 'test_scenario'
    evaluator.latest_actors.actors = actors
    evaluator.latest_subject_pose = pose
    evaluator.latest_detection_count = raw_detection_count
    evaluator.subject_match_radius = 0.55
    evaluator.wall_false_positive_radius = 0.55
    evaluator.require_raw_detection = True
    return evaluator


def test_evaluator_reports_subject_tracking_when_detection_matches_subject():
    evaluator = make_evaluator(
        [
            make_actor(1, 'subject_human', 'human', 1.0, 0.0, is_subject=True),
            make_actor(2, 'wall_panel', 'wall', 1.8, 0.0),
        ],
        make_pose(1.05, 0.05),
        raw_detection_count=1,
    )

    evaluation = evaluator.build_evaluation()

    assert evaluation.status == 'tracking_subject'
    assert evaluation.subject_tracking_ok is True
    assert evaluation.false_wall_lock is False
    assert evaluation.nearest_actor_id == 1


def test_evaluator_reports_false_wall_lock_when_detection_is_closest_to_wall():
    evaluator = make_evaluator(
        [
            make_actor(1, 'subject_human', 'human', 1.0, 0.0, is_subject=True),
            make_actor(2, 'wall_panel', 'wall', 1.8, 0.0),
        ],
        make_pose(1.78, 0.02),
        raw_detection_count=1,
    )

    evaluation = evaluator.build_evaluation()

    assert evaluation.status == 'false_wall_lock'
    assert evaluation.subject_tracking_ok is False
    assert evaluation.false_wall_lock is True
    assert evaluation.nearest_actor_id == 2


def test_evaluator_reports_pose_memory_when_subject_pose_exists_without_raw_detection():
    evaluator = make_evaluator(
        [
            make_actor(1, 'subject_human', 'human', 1.0, 0.0, is_subject=True),
        ],
        make_pose(1.02, 0.01),
        raw_detection_count=0,
    )

    evaluation = evaluator.build_evaluation()

    assert evaluation.status == 'pose_memory_only'
    assert evaluation.raw_detection_available is False
    assert evaluation.subject_tracking_ok is False


def test_evaluator_accepts_oracle_pose_when_raw_detection_is_not_required():
    evaluator = make_evaluator(
        [
            make_actor(1, 'subject_human', 'human', 1.0, 0.0, is_subject=True),
        ],
        make_pose(1.02, 0.01),
        raw_detection_count=0,
    )
    evaluator.require_raw_detection = False

    evaluation = evaluator.build_evaluation()

    assert evaluation.status == 'oracle_tracking_subject'
    assert evaluation.raw_detection_available is False
    assert evaluation.subject_tracking_ok is True
