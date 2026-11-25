"""Topic 구독 노드: /chatter 토픽을 받아 출력한다."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TopicSubscriber(Node):
    def __init__(self):
        super().__init__('topic_subscriber')
        # 메시지가 올 때마다 listener_callback 이 호출된다
        self.subscription = self.create_subscription(
            String, 'chatter', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Subscribe: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = TopicSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
