from PySide2.QtWidgets import QTreeWidgetItem, QApplication, QMainWindow
from PySide2.QtCore import QTimer
from kitchen_display_layout import Ui_MainWindow
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import logging
from collections import deque

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setupUi(self)

        # ROS 2 초기화
        rclpy.init()
        self.node = Node("kitchen_display")

        # Logger 초기화
        self.logger = logging.getLogger("kitchen_display")
        logging.basicConfig(level=logging.INFO)

        # 메뉴 딕셔너리 초기화
        self.menu_map = {
            1: "Pizza",
            2: "Pasta",
            3: "Coke"
        }

        # 메뉴 조리 시간 초기화
        self.menu_cooking_time_map = {
            "Pizza": 3,
            "Pasta": 5,
            "Coke": 0
        }

        # Publisher 초기화
        self.status_publisher = self.node.create_publisher(Int32MultiArray, "/order_status", 10)

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

    def initialize_tree_widgets(self):
        self.treeWidget.setHeaderLabels(["Menu", "Amounts", "Status"])
        self.treeWidget_2.setHeaderLabels(["Menu", "Amounts", "Status"])
        self.treeWidget_4.setHeaderLabels(["Menu", "Amounts", "Status"])

    def setup_callbacks(self):
        # Connect buttons to their specific callbacks
        self.pushButton.clicked.connect(lambda: self.add_task(self.check_menu, self.treeWidget, self.pushButton, self.pushButton_2, self.pushButton_7))
        self.pushButton_2.clicked.connect(lambda: self.serve_table(self.treeWidget, self.pushButton_2))
        self.pushButton_3.clicked.connect(lambda: self.add_task(self.check_menu, self.treeWidget_2, self.pushButton_3, self.pushButton_4, self.pushButton_8))
        self.pushButton_4.clicked.connect(lambda: self.serve_table(self.treeWidget_2, self.pushButton_4))
        self.pushButton_5.clicked.connect(lambda: self.add_task(self.check_menu, self.treeWidget_4, self.pushButton_5, self.pushButton_6, self.pushButton_9))
        self.pushButton_6.clicked.connect(lambda: self.serve_table(self.treeWidget_4, self.pushButton_6))
        self.pushButton_7.clicked.connect(lambda: self.cancel_order(self.treeWidget, self.pushButton, self.pushButton_7))
        self.pushButton_8.clicked.connect(lambda: self.cancel_order(self.treeWidget_2, self.pushButton_3, self.pushButton_8))
        self.pushButton_9.clicked.connect(lambda: self.cancel_order(self.treeWidget_4, self.pushButton_5, self.pushButton_9))

    def setup_button_controls(self):
        # Initialize button states
        self.pushButton.setEnabled(False)
        self.pushButton_2.setEnabled(False)
        self.pushButton_3.setEnabled(False)
        self.pushButton_4.setEnabled(False)
        self.pushButton_5.setEnabled(False)
        self.pushButton_6.setEnabled(False)
        self.pushButton_7.setEnabled(False)
        self.pushButton_8.setEnabled(False)
        self.pushButton_9.setEnabled(False)

    def setup_subscribers(self):
        self.node.create_subscription(Int32MultiArray, "/orderA", lambda msg: self.process_order(msg, self.treeWidget, self.pushButton, self.pushButton_7), 10)
        self.node.create_subscription(Int32MultiArray, "/orderB", lambda msg: self.process_order(msg, self.treeWidget_2, self.pushButton_3, self.pushButton_8), 10)
        self.node.create_subscription(Int32MultiArray, "/orderC", lambda msg: self.process_order(msg, self.treeWidget_4, self.pushButton_5, self.pushButton_9), 10)

    def process_order(self, msg, tree_widget, check_button, cancel_button):
        order_number, menu_code, amounts = msg.data
        menu_name = self.menu_map.get(menu_code, "Unknown")
        self.logger.info(f"Order {order_number}: {menu_name} x{amounts}")
        self.add_tree_widget_item(tree_widget, [menu_name, str(amounts), "주문 대기"])
        check_button.setEnabled(True)
        cancel_button.setEnabled(True)
        self.publish_status(order_number, menu_code, amounts, "주문 대기")

    def add_tree_widget_item(self, tree_widget, values):
        item = QTreeWidgetItem(values)
        tree_widget.addTopLevelItem(item)

    def update_tree_widget_status(self, tree_widget, old_status, new_status):
        root = tree_widget.invisibleRootItem()
        for i in range(root.childCount()):
            item = root.child(i)
            if item.text(2) == old_status:
                item.setText(2, new_status)
                menu_name = item.text(0)
                amounts = item.text(1)
                menu_code = next((code for code, name in self.menu_map.items() if name == menu_name), None)
                if menu_code is not None:
                    self.publish_status(-1, menu_code, int(amounts), new_status)

    def publish_status(self, order_number, menu_code, amounts, status):
        msg = Int32MultiArray()
        status_map = {"주문 대기": 0, "주문 확인": 1, "조리 중": 2, "조리 완료": 3, "배달 중": 4, "취소됨": 5}
        msg.data = [order_number, menu_code, amounts, status_map.get(status, -1)]
        self.status_publisher.publish(msg)
        self.logger.info(f"상태 발행: {msg.data}")

    def add_task(self, func, *args):
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
                    cooking_time = self.menu_cooking_time_map.get(menu_name, 1) * amounts
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

    def serve_table(self, tree_widget, serve_button):
        self.logger.info("테이블에 서빙 중입니다.")
        serve_button.setEnabled(False)
        self.update_tree_widget_status(tree_widget, "조리 완료", "배달 중")

    def cancel_order(self, tree_widget, check_button, cancel_button):
        self.logger.info("주문 취소 중입니다.")
        root = tree_widget.invisibleRootItem()
        check_button.setEnabled(False)
        cancel_button.setEnabled(False)
        for i in reversed(range(root.childCount())):
            item = root.child(i)
            if item.text(2) == "주문 대기":
                tree_widget.takeTopLevelItem(i)
                menu_name = item.text(0)
                amounts = int(item.text(1))
                menu_code = next((code for code, name in self.menu_map.items() if name == menu_name), None)
                if menu_code is not None:
                    self.publish_status(-1, menu_code, amounts, "취소됨")
        

# 메인 코드
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    # ROS 2 스레드 시작
    def ros_spin():
        rclpy.spin(window.node)

    import threading
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    app.exec_()

    # 종료 처리
    window.node.destroy_node()
    rclpy.shutdown()
