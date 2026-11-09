import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StatusSubscriber(Node):
    def __init__(self):
        super().__init__('status_subscriber')

        self.subscription = self.create_subscription(
            String,
            '/robot_status',
            self.status_callback,
            10
        )

        self.last_count = None
        self.get_logger().info('Status monitor started')

    def status_callback(self, msg):
        data = msg.data

        if data.startswith('robot_ready | count='):
            try:
                count = int(data.split('count=')[1])
            except ValueError:
                self.get_logger().warn('Invalid count')
                return

            if self.last_count is not None:
                if count != self.last_count + 1:
                    self.get_logger().warn(
                        'Message sequence discontinuity'
                    )

            self.last_count = count

            self.get_logger().info(
                f'Robot status: READY | count={count}'
            )
        else:
            self.get_logger().warn(
                f'Unknown robot status: {data}'
            )


def main():
    rclpy.init()
    node = StatusSubscriber()

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
