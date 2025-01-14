import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Pose, Quaternion

from time import sleep

class TableNavNode(Node):
    def __init__(self):
        super().__init__('table_nav_node')

        # 초기 위치
        self.initial_position = [-2.0, -0.5, 0.0, 1.0]

        # 테이블 이름 -> (x, y) 매핑 (필요에 따라 수정하세요)
        self.table_positions = {
            "tableA": (1.5, -1.0),
            "tableB": (1.5, 0.0),
            "tableC": (1.5, 1.0)
        }

        self.status_publisher = self.create_publisher(Bool, 'rob_status', 10)

        # 로봇 상태: 이동 중인지 아닌지
        self.is_moving = False
        self.returning_to_initial = False  # 원점 복귀 상태

        # 현재 테이블 이름
        self.current_table_name = None

        # 퍼블리셔 생성
        self.complete_publisher = self.create_publisher(String, 'completionstatus', 10)

        # Nav2 Action Client
        self.navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 구독자 생성
        self.subscription = self.create_subscription(
            String,
            'SendGoTable',
            self.table_callback,
            10
        )

        # Nav2 액션 서버 준비 대기
        self.wait_for_server()

    def wait_for_server(self):
        """navigate_to_pose 액션 서버가 켜질 때까지 대기"""
        self.get_logger().info('Waiting for "navigate_to_pose" action server...')
        while not self.navigate_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action server not available... waiting...')

        self.get_logger().info('Action server is ready!')


    def table_callback(self, msg: String):
        """테이블 명령 수신 및 처리"""
        table_name = msg.data.strip()
        self.get_logger().info(f'Received table command: {table_name}')

        if self.is_moving:
            self.get_logger().warn('현재 로봇이 이동 중입니다. 새 명령을 무시합니다.')
            return

        if table_name not in self.table_positions:
            self.get_logger().warn(f'Unknown table name: {table_name}')
            return
        
        self.send_status_message(self.is_moving)
        # 테이블 좌표 가져오기
        x, y = self.table_positions[table_name]

        # 테이블로 이동 시작
        self.current_table_name = table_name
        self.returning_to_initial = False
        self.send_goal_to_nav2(x, y, return_to_initial=True)
        
    # Nav2의 navigate_to_pose 액션 서버에 goal 전송
    def send_goal_to_nav2(self, x: float, y: float, return_to_initial=False):
        """Nav2 액션 서버에 이동 목표 전송"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        send_goal_future = self.navigate_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, return_to_initial)
        )

        self.is_moving = True
        log_message = f"table {self.current_table_name}" if not self.returning_to_initial else "initial position"
        self.get_logger().info(f'Sending goal to {log_message}: x={x}, y={y}')


    def goal_response_callback(self, future, return_to_initial):
        """Goal 응답 콜백 처리"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by server.')
            self.is_moving = False
            return

        self.get_logger().info('Goal accepted by server, moving now...')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.result_callback(future, return_to_initial)
        )

    def feedback_callback(self, feedback_msg):
        """이동 중 피드백 콜백"""
        feedback = feedback_msg.feedback
        distance_remaining = feedback.distance_remaining
        self.get_logger().info(f'[Feedback] distance_remaining = {distance_remaining:.2f}')

    def result_callback(self, future, return_to_initial):
        """Goal 완료 콜백"""
        try:
            action_result = future.result()
            status = action_result.status

            if status == 4:  # GoalStatus.STATUS_SUCCEEDED
                if not self.returning_to_initial:
                    self.get_logger().info('Arrived at table.')
                    self.returning_to_initial = True
                    x, y = self.initial_position[:2]
                    self.send_goal_to_nav2(x, y, return_to_initial=True)
                else:
                    self.get_logger().info('Returned to initial position.')
                    self.send_completion_message(
                        f"{self.current_table_name}"
                        # f"Completed to navigate to {self.current_table_name}"
                    )
                    self.current_table_name = None
            else:
                self.get_logger().warn(f'Goal failed with status code: {status}')

        finally:
            self.is_moving = False
            
    def send_status_message(self, message: str):
        """상태 메시지 전송"""
        status_msg = String()
        status_msg.data = message
        self.status_publisher.publish(status_msg)
        self.get_logger().info(f"Status message sent: {message}")
    
    def send_completion_message(self, message: str):
        """완료 메시지 전송"""
        completion_msg = String()
        completion_msg.data = message
        self.complete_publisher.publish(completion_msg)
        self.get_logger().info(f"Completion message sent: {message}")

def main(args=None):
    rclpy.init(args=args)
    node = TableNavNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt received.')
    finally:
        # 노드가 생성된 경우만 삭제
        if node is not None:
            node.destroy_node()
            node.get_logger().info('Node destroyed.')

        # ROS 컨텍스트가 활성 상태일 때만 shutdown 호출
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
