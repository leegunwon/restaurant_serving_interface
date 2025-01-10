import rclpy
from rclpy.node import Node
import pymysql
from database_interface import get_max_order_id, get_max_customer_id




# ROS 2 Node 클래스
class ParameterNode(Node):
    def __init__(self):
        super().__init__('db_parameter_node')

        # 초기화 로그 출력
        self.get_logger().info("Initializing DB Parameter Node...")

        # MySQL에서 데이터를 가져옴 (한 번만 수행)
        max_order_id = get_max_order_id()
        max_customer_id = get_max_customer_id()

        # 파라미터 등록
        self.declare_parameter('max_order_id', max_order_id)
        self.declare_parameter('max_customer_id', max_customer_id)

        # 현재 파라미터 값을 출력
        self.get_logger().info(f"Parameter 'max_order_id' set to: {max_order_id}")
        self.get_logger().info(f"Parameter 'max_customer_id' set to: {max_customer_id}")

        # 데이터베이스 업데이트 없이 노드를 계속 실행


def main(args=None):
    rclpy.init(args=args)

    # 노드 생성
    node = ParameterNode()

    # 사용자 종료 전까지 노드 실행 유지
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
