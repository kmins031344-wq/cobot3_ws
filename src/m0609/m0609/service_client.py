"""Service 클라이언트 노드: 두 정수를 /add_two_ints 서버에 보내고 결과를 출력한다.

실행 예: ros2 run m0609 service_client 3 5
"""
import sys

import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.node import Node


class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        # 서버가 뜰 때까지 기다린다
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서버 기다리는 중...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)  # 응답이 올 때까지 대기
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    a = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    b = int(sys.argv[2]) if len(sys.argv) > 2 else 5

    node = ServiceClient()
    response = node.send_request(a, b)
    node.get_logger().info(f'Result: {a} + {b} = {response.sum}')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
