from ainex_motion.joint_controller import JointController
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):
    """
    Goal: Retreive the joint positions of the robot and publish as JointState messages
    """
    def __init__(self):
        super().__init__('joint_state_publisher')
        # instaintiate the JointController with the current node
        self.joint_controller = JointController(self)
        
        # define a publisher for joint states on the 'joint_states' topic
        self.joint_states_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

    def publish_joint_states(self):
        # Retrieve current joint positions and publish them to the 'joint_states' topic
        joint_names = list(self.joint_controller.joint_id.keys())
        positions = self.joint_controller.getJointPositions(joint_names)
        if positions is None:
            self.get_logger().warning('Failed to retrieve joint positions')
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = positions
        msg.velocity = [0.0] * len(joint_names)
        msg.effort = [0.0] * len(joint_names)

        self.joint_states_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    joint_state_publisher = JointStatePublisher()

    while rclpy.ok():
        joint_state_publisher.publish_joint_states()

    joint_state_publisher.destroy_node()
    rclpy.shutdown()
