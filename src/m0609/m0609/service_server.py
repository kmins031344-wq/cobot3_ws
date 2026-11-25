"""Service 서버 노드: /add_two_ints 요청(a, b)을 받아 합(sum)을 돌려준다."""
import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.node import Node


class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        # (서비스 타입, 서비스 이름, 요청이 오면 실행할 함수)
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)
        self.get_logger().info('add_two_ints 서비스 대기 중...')

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} -> Response: {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServiceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
