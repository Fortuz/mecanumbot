import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class MecanumbotSimOracleSubjectNode(Node):
    def __init__(self, namespace=''):
        super().__init__('mecanumbot_sim_oracle_subject_node', namespace=namespace)

        self.declare_parameter('source_topic', '/sim/subject_pose_ground_truth')
        self.declare_parameter('target_topic', 'subject_pose')

        source_topic = str(self.get_parameter('source_topic').value)
        target_topic = str(self.get_parameter('target_topic').value)

        self.publisher = self.create_publisher(PoseStamped, target_topic, 10)
        self.create_subscription(PoseStamped, source_topic, self.subject_callback, 10)

        self.get_logger().info(
            f'Oracle subject bridge active: {source_topic} -> {target_topic}'
        )

    def subject_callback(self, msg: PoseStamped) -> None:
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumbotSimOracleSubjectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
