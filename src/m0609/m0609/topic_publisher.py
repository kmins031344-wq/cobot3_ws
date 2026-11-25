"""Topic 발행 노드: 1초마다 /chatter 토픽에 문자열을 보낸다."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TopicPublisher(Node):
    def __init__(self):
        super().__init__('topic_publisher')
        # (메시지 타입, 토픽 이름, 큐 크기)
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROKEY: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publish: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = TopicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
