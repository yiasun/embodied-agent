import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')

        self.publisher = self.create_publisher(
            String, '/robot_status', 10
        )

        self.timer = self.create_timer(
            1.0, self.publish_status
        )

        self.count = 0

    def publish_status(self):
        msg = String()
        msg.data = f'robot_ready | count={self.count}'

        self.publisher.publish(msg)
        self.get_logger().info(msg.data)

        self.count += 1


def main():
    rclpy.init()
    node = StatusPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
