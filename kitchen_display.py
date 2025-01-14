import sys
import threading
import datetime
import logging
from collections import deque

from PySide2.QtWidgets import QTreeWidgetItem, QApplication, QMainWindow, QMenuBar, QMenu, QAction, QVBoxLayout, QDialog
from PySide2.QtCore import QTimer, Qt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String, Int32, Bool

from kitchen_display_layout import Ui_MainWindow
from database_interface import (
    initialize_cache,
    insert_order,
    get_max_order_id,
    fetch_menu_order_summary,
    update_order_cancel_id  # 필요 시 사용
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt

class SubWindow(QDialog):
    """서브 윈도우 클래스"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Order Analysis Chart")
        self.setGeometry(100, 100, 800, 600)

        # 레이아웃 및 Matplotlib 설정
        layout = QVBoxLayout(self)
        self.figure, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)

        # 그래프 그리기
        self.plot_menu_order_summary(fetch_menu_order_summary())

    def plot_menu_order_summary(self, order_summary):
        """
        메뉴 주문 요약 데이터를 서브 윈도우의 Matplotlib 캔버스에 출력합니다.

        :param order_summary: {menu_id: total_quantity} 형태의 딕셔너리
        """
        # 데이터 준비
        menu_ids = list(order_summary.keys())
        quantities = list(order_summary.values())

        # 기존 그래프 초기화
        self.ax.clear()

        # 바 차트 생성
        self.ax.bar(menu_ids, [float(qty) for qty in quantities], color='skyblue', edgecolor='black')

        # 그래프 꾸미기
        self.ax.set_title("Menu Order Summary", fontsize=16)
        self.ax.set_xlabel("Menu ID", fontsize=14)
        self.ax.set_ylabel("Quantity Ordered", fontsize=14)
        self.ax.set_xticks(menu_ids)
        self.ax.tick_params(axis='x', labelsize=12)
        self.ax.tick_params(axis='y', labelsize=12)

        # 수량 표시
        for i, qty in enumerate(quantities):
            qty = float(qty)  # decimal.Decimal을 float으로 변환
            self.ax.text(menu_ids[i], qty + 0.5, str(qty), ha='center', fontsize=10)

        # 캔버스 업데이트
        self.canvas.draw()


    

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setupUi(self)
        # 메뉴바 생성
        self.menubar = QMenuBar(self)
        self.setMenuBar(self.menubar)

        # 메뉴 및 액션 추가
        self.menu_kitchen_interface = QMenu("Kitchen Interface", self.menubar)
        self.menubar.addMenu(self.menu_kitchen_interface)

        self.action_order_analysis = QAction("Order Analysis", self)
        self.menu_kitchen_interface.addAction(self.action_order_analysis)

        # 액션에 서브 윈도우 연결
        self.action_order_analysis.triggered.connect(self.show_sub_window)

        # ROS 2 초기화
        rclpy.init()
        self.node = Node("kitchen_display")

        self.menu_dict, self.cancel_dict = initialize_cache()
        self.menu_name_dict = {}
        self.menu_cooking_time_dict = {}
        self.order_id = get_max_order_id() + 1
        for menu_id, menu_info in self.menu_dict.items():
            self.menu_name_dict[menu_id] = menu_info['name']
            self.menu_cooking_time_dict[menu_info['name']] = menu_info['cooking_time']

        # Logger 초기화
        self.logger = logging.getLogger("kitchen_display")
        logging.basicConfig(level=logging.INFO)
        
        # 임시 테이블별 주문 목록
        self.temp_order_table = {
            "tableA": [],
            "tableB": [],
            "tableC": []
        }
        
        # 퍼블리셔 생성
        self.publisher_send_goal = self.node.create_publisher(String, 'SendGoTable', 10)
        self.publisher_order_id = self.node.create_publisher(Int32, 'OrderId', 10)
        self.publisher_cancel_order = self.node.create_publisher(Int32, 'CancelOrder', 10)
        
        # UI 초기화
        self.initialize_tree_widgets()
        self.setup_callbacks()
        self.setup_button_controls()
        self.setup_subscribers()

        # Timer 및 작업 대기열 초기화
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.handle_timer_completion)
        self.task_queue = deque()
        self.current_task = None
        
    def show_sub_window(self):
        # 서브 윈도우 생성 및 표시
        sub_window = SubWindow(self)
        sub_window.exec_()
        
    def initialize_tree_widgets(self):
        self.treeWidget.setHeaderLabels(["Menu", "Amounts", "Status"])
        self.treeWidget_2.setHeaderLabels(["Menu", "Amounts", "Status"])
        self.treeWidget_4.setHeaderLabels(["Menu", "Amounts", "Status"])

    def setup_callbacks(self):
        # 테이블 A
        self.pushButton.clicked.connect(
            lambda: self.add_task("tableA", self.check_menu, self.treeWidget, self.pushButton, self.pushButton_2, self.pushButton_7)
        )
        self.pushButton_2.clicked.connect(
            lambda: self.serve_table(self.treeWidget, self.pushButton_2, "tableA")
        )
        self.pushButton_7.clicked.connect(
            lambda: self.cancel_order(self.treeWidget, self.pushButton, self.pushButton_7, "tableA")
        )

        # 테이블 B
        self.pushButton_3.clicked.connect(
            lambda: self.add_task("tableB", self.check_menu, self.treeWidget_2, self.pushButton_3, self.pushButton_4, self.pushButton_8)
        )
        self.pushButton_4.clicked.connect(
            lambda: self.serve_table(self.treeWidget_2, self.pushButton_4, "tableB")
        )
        self.pushButton_8.clicked.connect(
            lambda: self.cancel_order(self.treeWidget_2, self.pushButton_3, self.pushButton_8, "tableB")
        )

        # 테이블 C
        self.pushButton_5.clicked.connect(
            lambda: self.add_task("tableC", self.check_menu, self.treeWidget_4, self.pushButton_5, self.pushButton_6, self.pushButton_9)
        )
        self.pushButton_6.clicked.connect(
            lambda: self.serve_table(self.treeWidget_4, self.pushButton_6, "tableC")
        )
        self.pushButton_9.clicked.connect(
            lambda: self.cancel_order(self.treeWidget_4, self.pushButton_5, self.pushButton_9, "tableC")
        )

    def setup_button_controls(self):
        # 버튼 초깃값
        self.pushButton.setEnabled(False)
        self.pushButton_2.setEnabled(False)
        self.pushButton_3.setEnabled(False)
        self.pushButton_4.setEnabled(False)
        self.pushButton_5.setEnabled(False)
        self.pushButton_6.setEnabled(False)
        self.pushButton_7.setEnabled(False)
        self.pushButton_8.setEnabled(False)
        self.pushButton_9.setEnabled(False)

    def publish_message_send_goal(self, table_name: str):
        msg = String()
        msg.data = table_name
        self.publisher_send_goal.publish(msg)
        self.node.get_logger().info(f'Publishing send goal: "{msg.data}"')
    
    def publish_message_order_id(self, order_id: int):
        """Order ID를 다른 노드에 알릴 때 사용"""
        msg = Int32()
        msg.data = order_id
        self.publisher_order_id.publish(msg)
        self.node.get_logger().info(f'Publishing order id: "{msg.data}"')
        
    def publish_message_cancel_from_kitchen(self, order_id: int):
        """취소할 Order ID를 퍼블리시"""
        msg = Int32()
        msg.data = order_id
        self.publisher_cancel_order.publish(msg)
        self.node.get_logger().info(f'Publishing cancel: "{msg.data}"')
        
    def setup_subscribers(self):
        self.node.create_subscription(
            Int32MultiArray, "/orderA",
            lambda msg: self.process_order(msg, self.treeWidget, self.pushButton, self.pushButton_7, "tableA"),
            10
        )
        self.node.create_subscription(
            Int32MultiArray, "/orderB",
            lambda msg: self.process_order(msg, self.treeWidget_2, self.pushButton_3, self.pushButton_8, "tableB"),
            10
        )
        self.node.create_subscription(
            Int32MultiArray, "/orderC",
            lambda msg: self.process_order(msg, self.treeWidget_4, self.pushButton_5, self.pushButton_9, "tableC"),
            10
        )
        self.node.create_subscription(
            Int32, 
            "CancelOrderFromTable", 
            self.cancel_order_from_table_callback, 
            10
        )
            # CompletionStatus 토픽 구독
        self.node.create_subscription(
            String,
            "completionstatus",
            self.handle_completion_status,
            10
        )

    def process_order(self, msg, tree_widget, check_button, cancel_button, table_name):
        """
        주문 정보를 수신했을 때 호출되는 콜백
        msg.data = [customer_id, menu_id, amounts]
        여기서 주문 생성 시, order_id를 미리 부여 & TreeWidgetItem에 저장
        """
        customer_id, menu_id, amounts = msg.data

        # temp_order_table에도 order_id를 포함해 보관
        self.temp_order_table[table_name].append([self.order_id, customer_id, menu_id, amounts])

        current_time = datetime.datetime.now()
        # DB에 주문 정보 삽입``
        insert_order(self.order_id, customer_id, menu_id, amounts, current_time)
        # 트리위젯에 표시
        menu_name = self.menu_name_dict[menu_id]

        # (*) 트리 아이템 생성 시 order_id 숨겨 저장
        self.add_tree_widget_item(
            tree_widget,
            [menu_name, str(amounts), "주문 대기"], self.order_id
        )
        self.publish_message_order_id(self.order_id)
        self.order_id += 1
        check_button.setEnabled(True)
        cancel_button.setEnabled(True)

    def add_tree_widget_item(self, tree_widget, values, order_id):
        """
        TreeWidgetItem에 Order ID를 숨겨두기 위해 setData(Qt.UserRole) 활용
        """
        item = QTreeWidgetItem(values)
        # (*) 0번 컬럼에 order_id를 저장 (UserRole)
        item.setData(0, Qt.UserRole, order_id)
        tree_widget.addTopLevelItem(item)
        
    def handle_completion_status(self, msg):
        """
        CompletionStatus 토픽에서 '서빙완료' 메시지를 받으면
        '배달 중' 상태인 TreeWidgetItem을 '배달 완료'로 변경.
        """
        status_message = msg.data
        self.logger.info(f"[CompletionStatus] Received status: {status_message}")

        if status_message == "tableA":
            # 테이블 A, B, C의 treeWidget을 순회하면서 상태 업데이트
            self.update_tree_widget_status(self.treeWidget, "배달 중", "배달 완료")
        elif status_message == "tableB":
            self.update_tree_widget_status(self.treeWidget_2, "배달 중", "배달 완료")
        elif status_message == "tableC":
            self.update_tree_widget_status(self.treeWidget_4, "배달 중", "배달 완료")

    def update_tree_widget_status(self, tree_widget, old_status, new_status):
        """
        특정 상태(old_status)를 가진 아이템을 new_status로 변경하고,
        필요한 경우 (new_order_id 인자가 주어졌고 아이템에 order_id가 None일 때) 
        새로운 order_id를 아이템에 부여.
    
        :param tree_widget: 상태를 갱신할 QTreeWidget
        :param old_status: 현재 상태(예: "주문 대기")
        :param new_status: 변경할 상태(예: "주문 확인")
        :param new_order_id: 새로 부여할 order_id (기본 None)
        """
        root = tree_widget.invisibleRootItem()
        for i in range(root.childCount()):
            item = root.child(i)
    
            current_status = item.text(2)
            if current_status == old_status:
                # 1) 상태 문자열 변경
                item.setText(2, new_status)


    def add_task(self, table_name, func, *args):
        """
        '주문 확인' 단계에서 DB에 실제로 insert_order를 넣고,
        그때의 order_id를 publish_message()로 알림
        """
        for menu in self.temp_order_table[table_name]:
            order_id, customer_id, menu_id, amounts = menu

            

        # 테이블의 임시 목록 초기화
        self.temp_order_table[table_name] = []
        print(self.temp_order_table)

        check_button, cancel_button = args[1], args[3]
        check_button.setEnabled(False)
        cancel_button.setEnabled(False)
        self.logger.info("주문 확인했습니다.")
        
        self.update_tree_widget_status(args[0], "주문 대기", "주문 확인")
        self.task_queue.append((func, args))
        
        if not self.timer.isActive() and self.current_task is None:
            self.process_next_task()

    def process_next_task(self):
        if self.task_queue:
            self.current_task = self.task_queue.popleft()
            func, args = self.current_task
            func(*args)
            tree_widget = args[0]
            root = tree_widget.invisibleRootItem()
            for i in range(root.childCount()):
                item = root.child(i)
                if item.text(2) == "조리 중":
                    menu_name = item.text(0)
                    amounts = int(item.text(1))
                    cooking_time = self.menu_cooking_time_dict[menu_name] * amounts
                    self.timer.start(cooking_time * 1000)  # 타이머 시작
                    break
        else:
            self.current_task = None

    def handle_timer_completion(self):
        if self.current_task:
            func, args = self.current_task
            if func == self.check_menu:
                tree_widget, check_button, serve_button = args[0], args[1], args[2]
                self.logger.info("조리 완료.")
                self.update_tree_widget_status(tree_widget, "조리 중", "조리 완료")
                serve_button.setEnabled(True)
            self.current_task = None
        self.process_next_task()

    def check_menu(self, tree_widget, check_button, serve_button, cancel_button):
        self.logger.info("조리 중입니다.")
        self.update_tree_widget_status(tree_widget, "주문 확인", "조리 중")
        cancel_button.setEnabled(False)

    def serve_table(self, tree_widget, serve_button, table_name):
        self.logger.info("테이블에 서빙 중입니다.")
        self.publish_message_send_goal(table_name)
        serve_button.setEnabled(False)
        self.update_tree_widget_status(tree_widget, "조리 완료", "배달 중")

    def cancel_order(self, tree_widget, check_button, cancel_button, table_name):
        """
        '취소' 버튼 클릭 시,
         1) TreeWidget에 있는 '주문 대기' 상태 아이템을 찾는다
         2) 아이템에 숨겨진 order_id를 읽어, 취소 토픽 발행
         3) TreeWidget에서 제거 (또는 상태 변경)
         4) DB 업데이트가 필요하면 update_order_cancel_id 등으로 처리
        """
        self.logger.info("주문 취소 중입니다.")
        root = tree_widget.invisibleRootItem()
        
        # 해당 테이블 임시 목록도 비우기
        for menu in self.temp_order_table[table_name]:
            order_id, customer_id, menu_id, amounts = menu
            # (2) 취소 알림 퍼블리시
            self.publish_message_cancel_from_kitchen(order_id)
            update_order_cancel_id(order_id, 5)
        self.temp_order_table[table_name] = []
        print("After cancel ->", self.temp_order_table)

        check_button.setEnabled(False)
        cancel_button.setEnabled(False)

        for i in reversed(range(root.childCount())):
            item = root.child(i)
            status_text = item.text(2)
            if status_text == "주문 대기":
                # (1) order_id 읽기
                order_id = item.data(0, Qt.UserRole)
                menu_name = item.text(0)
                amounts = int(item.text(1))
                
                # (3) 트리에서 제거 (또는 "취소됨"으로 상태 변경)
                tree_widget.takeTopLevelItem(i)

                # (4) DB에 cancel_id 업데이트가 필요하면 사용
                # update_order_cancel_id(order_id, <원하는_cancel_id>)
                self.logger.info(f"Order {order_id} canceled ({menu_name} x{amounts}).")

                # 추가로 status_publisher 등을 통해 알림 가능
                # self.publish_status(order_id, menu_code, amounts, "취소됨")
    def remove_order_item_by_id(self, tree_widget, order_id):
        """
        주어진 tree_widget에서 order_id를 갖는 아이템을 찾아 제거한다.
        """
        root = tree_widget.invisibleRootItem()
        for i in reversed(range(root.childCount())):
            item = root.child(i)
            item_order_id = item.data(0, Qt.UserRole)  # 숨겨둔 order_id
            if item_order_id == order_id:
                menu_name = item.text(0)
                amounts = item.text(1)
                status = item.text(2)

                tree_widget.takeTopLevelItem(i)
                self.logger.info(
                    f"Removed item from tree_widget: order_id={order_id}, "
                    f"menu={menu_name}, amounts={amounts}, status={status}"
                )
                # 필요하다면 DB 업데이트 로직(취소 상태 반영)도 이곳에서 수행

    def cancel_order_from_table_callback(self, msg):
        """
        CancelOrderFromTable 토픽(Int32)으로부터 order_id를 수신.
        해당 order_id를 가진 TreeWidgetItem을 찾아서 삭제.
        """
        order_id_to_cancel = msg.data
        self.logger.info(f"[CancelOrderFromTable] Received order_id={order_id_to_cancel}")

        # 테이블 A, B, C의 treeWidget을 순회하면서 해당 order_id를 갖는 아이템을 삭제
        self.remove_order_item_by_id(self.treeWidget, order_id_to_cancel)
        self.remove_order_item_by_id(self.treeWidget_2, order_id_to_cancel)
        self.remove_order_item_by_id(self.treeWidget_4, order_id_to_cancel)
    
# 메인 코드
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    # ROS 2 스레드 시작
    def ros_spin():
        rclpy.spin(window.node)

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    app.exec_()

    # 종료 처리
    window.node.destroy_node()
    rclpy.shutdown()
