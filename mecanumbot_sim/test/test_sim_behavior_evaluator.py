from geometry_msgs.msg import Twist
from mecanumbot_msgs.msg import SimActor, SimActorArray, SimDetectionEvaluation
from mecanumbot_sim.behavior_evaluator_node import MecanumbotSimBehaviorEvaluatorNode
from nav_msgs.msg import Odometry


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


def make_odom(x=0.0, y=0.0):
    odom = Odometry()
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    return odom


def make_cmd(x=0.0, y=0.0, yaw=0.0):
    cmd = Twist()
    cmd.linear.x = x
    cmd.linear.y = y
    cmd.angular.z = yaw
    return cmd


def make_detection(
    status="tracking_subject", subject_tracking_ok=True, raw_detection_available=True
):
    detection = SimDetectionEvaluation()
    detection.status = status
    detection.subject_tracking_ok = subject_tracking_ok
    detection.raw_detection_available = raw_detection_available
    detection.false_wall_lock = status == "false_wall_lock"
    detection.wrong_human_lock = status == "wrong_human_lock"
    return detection


def make_evaluator(actors, odom, cmd, detection):
    evaluator = object.__new__(MecanumbotSimBehaviorEvaluatorNode)
    evaluator.latest_actors = SimActorArray()
    evaluator.latest_actors.scenario_name = "test_behavior"
    evaluator.latest_actors.actors = actors
    evaluator.latest_odom = odom
    evaluator.latest_cmd_vel = cmd
    evaluator.latest_detection_evaluation = detection
    evaluator.moving_speed_threshold = 0.03
    evaluator.human_stop_distance = 0.45
    evaluator.wall_stop_distance = 0.35
    evaluator.follow_min_distance = 0.55
    evaluator.follow_max_distance = 2.00
    return evaluator


def test_behavior_evaluator_reports_safe_idle_when_target_invalid_but_robot_stopped():
    evaluator = make_evaluator(
        [make_actor(1, "subject_human", "human", 1.0, 0.0, is_subject=True)],
        make_odom(),
        make_cmd(),
        make_detection(status="subject_error_high", subject_tracking_ok=False),
    )

    evaluation = evaluator.build_evaluation()

    assert evaluation.status == "safe_idle_invalid_target"
    assert evaluation.robot_moving is False
    assert evaluation.unsafe_motion is False


def test_behavior_evaluator_flags_motion_with_invalid_target():
    evaluator = make_evaluator(
        [make_actor(1, "subject_human", "human", 1.2, 0.0, is_subject=True)],
        make_odom(),
        make_cmd(x=0.15),
        make_detection(status="subject_error_high", subject_tracking_ok=False),
    )

    evaluation = evaluator.build_evaluation()

    assert evaluation.status == "unsafe_motion_with_invalid_target"
    assert evaluation.robot_moving is True
    assert evaluation.unsafe_wrong_target_motion is True
    assert evaluation.unsafe_motion is True


def test_behavior_evaluator_flags_motion_with_stale_detection():
    evaluator = make_evaluator(
        [make_actor(1, "subject_human", "human", 1.2, 0.0, is_subject=True)],
        make_odom(),
        make_cmd(x=0.15),
        make_detection(
            status="pose_memory_only",
            subject_tracking_ok=False,
            raw_detection_available=False,
        ),
    )

    evaluation = evaluator.build_evaluation()

    assert evaluation.status == "unsafe_motion_with_stale_detection"
    assert evaluation.unsafe_stale_detection_motion is True
    assert evaluation.unsafe_motion is True


def test_behavior_evaluator_allows_safe_following_with_valid_target_and_distance():
    evaluator = make_evaluator(
        [make_actor(1, "subject_human", "human", 1.0, 0.0, is_subject=True)],
        make_odom(),
        make_cmd(x=0.10),
        make_detection(status="tracking_subject", subject_tracking_ok=True),
    )

    evaluation = evaluator.build_evaluation()

    assert evaluation.status == "safe_following_subject"
    assert evaluation.target_valid is True
    assert evaluation.unsafe_motion is False


def test_behavior_evaluator_allows_oracle_target_without_raw_detection():
    evaluator = make_evaluator(
        [make_actor(1, "subject_human", "human", 1.0, 0.0, is_subject=True)],
        make_odom(),
        make_cmd(x=0.10),
        make_detection(
            status="oracle_tracking_subject",
            subject_tracking_ok=True,
            raw_detection_available=False,
        ),
    )

    evaluation = evaluator.build_evaluation()

    assert evaluation.status == "safe_following_subject"
    assert evaluation.target_valid is True
    assert evaluation.unsafe_stale_detection_motion is False
    assert evaluation.unsafe_motion is False
