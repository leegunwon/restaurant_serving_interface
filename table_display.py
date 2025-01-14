import sys
import threading

from PySide2.QtWidgets import (
    QApplication, QTreeWidget, QTextEdit, QComboBox, QTreeWidgetItem,
    QMainWindow, QPushButton, QSpinBox, QMessageBox, QDialog, QVBoxLayout,
    QHBoxLayout, QCheckBox, QLabel
)
from PySide2.QtCore import QMetaObject, Qt
import rclpy 
from database_interface import insert_review_to_db, get_max_customer_id, insert_sales_record, update_order_cancel_id, insert_null_menu_id
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32
from std_srvs.srv import Trigger 
from table_display_layout import Ui_MainWindow  # Qt Designer로 변환된 UI 모듈
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class TableManager(Node):
    """테이블 타입에 맞춰 단일 퍼블리셔를 생성하는 Node"""
    def __init__(self, table_id: str, main_window):
    
        """
        table_id: "A", "B", "C" 중 하나
        예: table_id == "A" -> 'orderA' 토픽 퍼블리셔 생성
        """
        super().__init__('table_manager')
        self.main_window=main_window
        # 토픽 이름을 table_id에 맞춰 결정
        topic_name = f"order{table_id}"
        
        # 히나의 토픽의 서브스크라이버의 콜백 함수에서 다른 토픽의 퍼블리시를 발생시키는 구조이며 토픽이 연속되서 전달되는 구조이기 때문에 메세지를 놓치는 경우가 발생함
        # ReliabilityPolicy.RELIABLE로 설정하였으나 오히려 연속적인 토픽을 빨리 수신하는 것이 중요하여 ReliabilityPolicy.BESTEFFORT로 수신하는 경우에서 메세지를 놓치는 경우가 
        # 발생하지 않았음
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # 버퍼 크기를 10으로 설정
        )
        
        self.publisher_ = self.create_publisher(Int32MultiArray, topic_name, 10)
        self.get_logger().info(f"[{table_id}] Created publisher for topic: '{topic_name}'")

        # 테이블별 주문 식별을 위해 customer_id 관리
        self.global_customer_id = get_max_customer_id() + 1
        self.local_customer_id = None  # 이 테이블을 위한 ID (처음 한 번 할당)

         # 취소 명령 퍼블리셔 추가
        self.cancel_order_publisher = self.create_publisher(Int32, 'CancelOrderFromTable', 10)
        self.subscription_cancel_order_main = self.create_subscription(
            Int32,
            'CancelOrder',
            self.cancel_order_callback,
            10
        )
        self.subscription = self.create_subscription(
            Int32,
            'OrderId',
            self.order_id_callback,
            qos_profile
        )
        self.order_queue = []  # 주문 대기열
        self.order_ids = []    # 주방에서 수신한 order_id

    def order_id_callback(self, msg):
        """주방에서 발행한 order_id를 수신"""
        self.get_logger().info(f"Received order_id: {msg.data}")
        order_id = msg.data
        # tree_widget2에서 order_id가 비어 있는 항목 찾기
        for i in range(self.main_window.tree_widget2.topLevelItemCount()):
            item = self.main_window.tree_widget2.topLevelItem(i)
            if item.data(0, Qt.UserRole) is None:  # 아직 order_id가 설정되지 않은 경우
                item.setData(0, Qt.UserRole, order_id)  # order_id 저장
                self.get_logger().info(f"Assigned order_id {order_id} to item {i}")
                return
        self.get_logger().warning("No pending item to assign the order_id.")


    def cancel_order_callback(self, msg):
        """주방에서 발행한 order_id를 수신하여 tree_widget2에서 해당 항목 삭제"""
        self.get_logger().info(f"Received order_id to delete: {msg.data}")
        order_id_to_delete = msg.data

        for i in range(self.main_window.tree_widget2.topLevelItemCount()):
            item = self.main_window.tree_widget2.topLevelItem(i)
            stored_order_id = item.data(0, Qt.UserRole)  # 항목에 저장된 order_id 가져오기

            if stored_order_id == order_id_to_delete:  # order_id가 일치하는 경우
                self.main_window.tree_widget2.takeTopLevelItem(i)  # 항목 제거
                self.get_logger().info(f"Deleted item with order_id: {order_id_to_delete}")
                return

        # 해당 order_id를 찾지 못한 경우
        self.get_logger().warning(f"Order ID {order_id_to_delete} not found in tree_widget2.")
        QMessageBox.warning(self.main_window, "삭제 실패", f"Order ID {order_id_to_delete}를 찾을 수 없습니다.")

    def assign_customer_id(self):
        """테이블마다 첫 주문 시 고유한 customer_id를 할당"""
        if self.local_customer_id is None:
            self.local_customer_id = self.global_customer_id
            insert_null_menu_id(self.local_customer_id)
            self.global_customer_id += 1

    def publish_order(self, menu_id: int, quantity: int):
        """이 테이블의 주문 정보를 Int32MultiArray로 퍼블리시"""
        self.assign_customer_id()
        msg = Int32MultiArray()
        # [customer_id, menu_id, quantity]
        msg.data = [self.local_customer_id, menu_id, quantity]
        
        self.get_logger().info(
            f"Publishing order -> customer_id:{self.local_customer_id}, menu_id:{menu_id}, quantity:{quantity}"
        )
        self.publisher_.publish(msg)
        self.order_queue.append((menu_id, quantity, None))  # None은 아직 매칭되지 않은 order_id
        insert_sales_record(self.local_customer_id, menu_id)

    def match_orders_with_ids(self):
        """주문 항목과 주방의 order_id를 매칭"""
        for i, order in enumerate(self.order_queue):
            if order[2] is None and self.order_ids:  # 아직 매칭되지 않은 주문
                order_id = self.order_ids.pop(0)
                self.order_queue[i] = (order[0], order[1], order_id)  # order_id 매칭
                self.get_logger().info(f"Order matched with order_id: {order_id}")

class CancelReasonDialog(QDialog):
    """주문 취소 사유 선택 창"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("취소 사유 선택")
        self.setMinimumSize(300, 200)

        # 선택된 사유의 텍스트와 인덱스를 저장할 속성
        self.selected_reason = None
        self.selected_index = None

        self.checkboxes = [
            QCheckBox("단순 변심"),
            QCheckBox("조리 지연"),
            QCheckBox("주문 실수"),
            QCheckBox("기타 사유"),
        ]
        for checkbox in self.checkboxes:
            checkbox.stateChanged.connect(self.limit_checkbox_selection)

        self.cancel_button = QPushButton("취소")
        self.ok_button = QPushButton("확인")

        layout = QVBoxLayout()
        for checkbox in self.checkboxes:
            layout.addWidget(checkbox)
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.cancel_button)
        button_layout.addWidget(self.ok_button)
        layout.addLayout(button_layout)

        self.setLayout(layout)

        self.cancel_button.clicked.connect(self.reject)
        self.ok_button.clicked.connect(self.check_selection)

    def limit_checkbox_selection(self, state):
        """하나의 사유만 선택하도록 제한"""
        if state == Qt.Checked:
            for checkbox in self.checkboxes:
                if checkbox != self.sender():
                    checkbox.setChecked(False)

    def check_selection(self):
        """체크박스 선택 확인 및 다이얼로그 종료"""
        for index, checkbox in enumerate(self.checkboxes):
            if checkbox.isChecked():
                self.selected_reason = checkbox.text()  # 선택된 사유 저장
                self.selected_index = index  # 선택된 사유의 인덱스 저장

        if self.selected_reason is None:
            QMessageBox.warning(self, "경고", "취소 사유를 체크해주세요.")
        else:
            self.accept()

    def get_selected_reason_and_index(self):
        """선택된 사유와 인덱스 반환"""
        print(self.selected_reason, self.selected_index)
        return self.selected_reason, self.selected_index



class ReviewDialog(QDialog):
    """리뷰 작성 창"""
    def __init__(self, parent=None, customer_id=None):
        super().__init__(parent)
        self.setWindowTitle("리뷰 작성")
        self.setMinimumSize(400, 300)

        self.parent = parent
        self.customer_id = customer_id
        layout = QVBoxLayout()
        self.label = QLabel("리뷰를 작성해주세요.")
        self.label.setAlignment(Qt.AlignCenter)

        self.review_text = QTextEdit()
        self.review_text.setPlaceholderText("고객님의 소중한 리뷰 작성해주시면 1000원 할인해드립니다.")
        self.review_text.setReadOnly(False)

        self.star_label = QLabel("별점을 선택해주세요:")
        self.star_label.setAlignment(Qt.AlignLeft)
        self.star_rating = QComboBox()
        self.star_rating.addItems(["1", "2", "3", "4", "5"])

        star_layout = QHBoxLayout()
        star_layout.addWidget(self.star_label)
        star_layout.addWidget(self.star_rating)

        self.submit_button = QPushButton("리뷰 제출")
        self.submit_button.clicked.connect(self.submit_review)
        self.close_button = QPushButton("닫기")
        self.close_button.clicked.connect(self.close)

        layout.addWidget(self.label)
        layout.addWidget(self.review_text)
        layout.addLayout(star_layout)
        layout.addWidget(self.submit_button)
        layout.addWidget(self.close_button)
        self.setLayout(layout)

    def submit_review(self):
        """
        리뷰 제출 버튼을 눌렀을 때 호출되는 메서드
        """
        review = self.review_text.toPlainText().strip()
        stars = int(self.star_rating.currentText())

        if not review:
            QMessageBox.warning(self, "경고", "리뷰 내용을 작성해주세요.")
            return
        print(review)
        # DB에 리뷰 삽입
        try:
            insert_review_to_db(self.customer_id, stars, review)
            QMessageBox.information(self, "리뷰 제출 완료", f"리뷰: {review}\n별점: {stars}점")
            self.accept()
            self.parent.show_payment_confirmation(1000)
        except Exception as e:
            QMessageBox.critical(self, "오류", f"리뷰 저장 중 오류 발생: {e}")


class MainWindow(QMainWindow):
    """메인 윈도우: 선택된 테이블(예: A/B/C)에 대한 주문 관리"""
    def __init__(self, table_id, manager: TableManager):
        super().__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        self.table_id = table_id
        self.manager = manager  # 테이블별 로직을 수행하는 Node
        

        self.tree_widget = self.ui.treeWidget
        self.tree_widget2 = self.ui.treeWidget_2
        self.label_total_price = self.ui.label_7  # 총금액 표시 라벨

        self.order_history = {}

        self.menu_prices = {
            "Tomato Pasta": 12000,
            "Cream Pasta": 13000,
            "Vongole Pasta": 14000,
            "Combination Pizza": 15000,
            "Pepperoni Pizza": 16000,
            "Hawaiian Pizza": 15000,
            "Tenderloin Steak": 30000,
            "Sirloin Steak": 28000,
            "T-bone Steak": 35000,
            "Coke": 2000,
            "Cider": 2000,
            "Blueberry Ade": 4000,
        }

        self.menu_ids = {
            "Tomato Pasta": 1,
            "Cream Pasta": 2,
            "Vongole Pasta": 3,
            "Pepperoni Pizza": 4,
            "Combination Pizza": 5,
            "Hawaiian Pizza": 6,
            "Tenderloin Steak": 7,
            "Sirloin Steak": 8,
            "T-bone Steak": 9,
            "Coke": 10,
            "Cider": 11,
            "Blueberry Ade": 12
        }

        self.menu_names = {v: k for k, v in self.menu_ids.items()}

        self.connect_buttons()
        self.update_total_price()  # 초기 총 금액 설정

        # 윈도우 타이틀을 테이블 ID로 설정
        self.setWindowTitle(f"Table {self.table_id}")

    def clear_tree_widget_2(self):
        """트리위젯2 초기화"""
        self.ui.treeWidget_2.clear()
        self.ui.label_7.setText("총 금액: 0원")

    def process_payment(self):
        """결제 버튼을 눌렀을 때 호출되는 메서드"""
        if self.ui.treeWidget_2.topLevelItemCount() == 0:
            QMessageBox.warning(self, "결제 불가", "주문 항목이 없습니다. 결제를 진행할 수 없습니다.")
            return

        reply = QMessageBox.question(
            self,
            "리뷰 요청",
            "리뷰하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.show_review_dialog()
        else:
            self.show_payment_confirmation()

    def show_payment_confirmation(self, discount=0):
        """결제 총금액 표시 및 결제 확인"""
        total_price = self.calculate_total_price() - discount
        reply = QMessageBox.question(
            self,
            "결제 확인",
            f"총 금액: {total_price:,}원\n결제하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            QMessageBox.information(self, "결제 완료", "결제가 완료되었습니다.")
            self.clear_tree_widget_2()  # 트리위젯2 초기화
            self.local_customer_id = None 
            
        else:
            QMessageBox.information(self, "결제 취소", "결제가 취소되었습니다.")

    def calculate_total_price(self):
        """트리위젯2의 총 금액 계산"""
        total_price = 0
        for i in range(self.ui.treeWidget_2.topLevelItemCount()):
            item = self.ui.treeWidget_2.topLevelItem(i)
            total_price += int(item.text(2).replace(',', '').replace('원', ''))
        return total_price

    def show_review_dialog(self):
        """리뷰 창 표시"""
        review_dialog = ReviewDialog(self, customer_id=self.manager.local_customer_id)
        review_dialog.exec_()
        self.clear_tree_widget_2()

    def connect_buttons(self):
        self.ui.pushButton_3.clicked.connect(lambda: self.add_to_tree("Tomato Pasta"))
        self.ui.pushButton_4.clicked.connect(lambda: self.add_to_tree("Cream Pasta"))
        self.ui.pushButton_5.clicked.connect(lambda: self.add_to_tree("Vongole Pasta"))
        self.ui.pushButton_7.clicked.connect(lambda: self.add_to_tree("Combination Pizza"))
        self.ui.pushButton_8.clicked.connect(lambda: self.add_to_tree("Pepperoni Pizza"))
        self.ui.pushButton_9.clicked.connect(lambda: self.add_to_tree("Hawaiian Pizza"))
        self.ui.pushButton_10.clicked.connect(lambda: self.add_to_tree("Tenderloin Steak"))
        self.ui.pushButton_11.clicked.connect(lambda: self.add_to_tree("Sirloin Steak"))
        self.ui.pushButton_12.clicked.connect(lambda: self.add_to_tree("T-bone Steak"))
        self.ui.pushButton_13.clicked.connect(lambda: self.add_to_tree("Coke"))
        self.ui.pushButton_14.clicked.connect(lambda: self.add_to_tree("Cider"))
        self.ui.pushButton_15.clicked.connect(lambda: self.add_to_tree("Blueberry Ade"))
        self.ui.pushButton_17.clicked.connect(self.process_payment)

        self.ui.pushButton_2.clicked.connect(self.reset_tree_widget)
        self.ui.pushButton_6.clicked.connect(self.remove_selected_item)
        self.ui.pushButton.clicked.connect(self.show_order_summary)



    
    def add_to_tree(self, item_name):
        """주문 목록(왼쪽 트리)에 아이템 추가"""
        price = self.menu_prices[item_name]
        for i in range(self.tree_widget.topLevelItemCount()):
            item = self.tree_widget.topLevelItem(i)
            if item.text(0) == item_name:
                spinbox = self.tree_widget.itemWidget(item, 1)
                if spinbox:
                    spinbox.setValue(spinbox.value() + 1)
                    item.setText(2, f"{spinbox.value() * price:,}원")
                self.update_total_price()
                return

        item = QTreeWidgetItem(self.tree_widget)
        item.setText(0, item_name)
        spinbox = QSpinBox()
        spinbox.setValue(1)
        spinbox.setMinimum(1)
        spinbox.setMaximum(99)
        self.tree_widget.setItemWidget(item, 1, spinbox)
        item.setText(2, f"{price:,}원")
        self.update_total_price()

    def reset_tree_widget(self):
        self.tree_widget.clear()
        self.update_total_price()

    def remove_selected_item(self):
        """왼쪽(현재 장바구니) 또는 오른쪽(이미 주문된 내역)에서 선택 항목 제거"""
        selected_item = self.tree_widget.currentItem()
        ordered_item = self.tree_widget2.currentItem()

        # 장바구니에서 삭제
        if selected_item:
            index = self.tree_widget.indexOfTopLevelItem(selected_item)
            if index != -1:
                self.tree_widget.takeTopLevelItem(index)
                self.update_total_price()

        # 주문 내역에서 삭제
        if ordered_item:
            order_id = ordered_item.data(0, Qt.UserRole)  # 저장된 order_id 가져오기
            if order_id is not None:
                try:
                    order_id = int(order_id)  # 명시적으로 int로 변환
                except ValueError:
                    QMessageBox.warning(self, "에러", f"Order ID '{order_id}'가 유효하지 않습니다.")
                    return

                item_name = ordered_item.text(0)

                dialog = CancelReasonDialog(self)
                if dialog.exec_():
                    QMessageBox.information(self, "취소 완료", f"{item_name} 항목이 취소되었습니다.")

                    # 주방으로 취소 요청 퍼블리시
                    reason, index = dialog.get_selected_reason_and_index()
                    self.manager.cancel_order_publisher.publish(Int32(data=order_id))
                    update_order_cancel_id(order_id, index + 1)
                    index = self.tree_widget2.indexOfTopLevelItem(ordered_item)
                    self.tree_widget2.takeTopLevelItem(index)
                    self.update_total_price()
            else:
                QMessageBox.warning(self, "에러", "선택된 주문 항목에 Order ID가 없습니다.")


    def show_order_summary(self):
        """왼쪽 트리(장바구니)의 주문 내역을 확인하고, 실제 주문(ROS 토픽 발행)을 수행"""
        if self.tree_widget.topLevelItemCount() == 0:
            QMessageBox.information(self, "주문", "주문할 항목이 없습니다.")
            return

        orders_to_publish = []  # ROS로 발행할 주문 데이터
        order_summary = "주문 내역:\n"

        # 장바구니에서 주문 내역 생성
        for i in range(self.tree_widget.topLevelItemCount()):
            item = self.tree_widget.topLevelItem(i)
            menu_name = item.text(0)
            spinbox = self.tree_widget.itemWidget(item, 1)
            quantity = spinbox.value() if spinbox else 1
            menu_id = self.menu_ids[menu_name]
            total_price = quantity * self.menu_prices[menu_name]

            order_summary += f"{menu_name}: {quantity}개, 금액: {total_price:,}원\n"
            orders_to_publish.append((menu_id, quantity))

        # 주문 확인 메시지 박스
        reply = QMessageBox.question(
            self, "주문 확인", order_summary + "\n주문하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            # 주문 발행 및 UI 업데이트
            for menu_id, quantity in orders_to_publish:
                self.manager.publish_order(menu_id, quantity)  # order_id 반환
                self.add_order_to_summary(menu_id, quantity)

            QMessageBox.information(self, "주문 완료", "주문이 완료되었습니다.")
            self.reset_tree_widget()
            self.update_total_price()

    def add_order_to_summary(self, menu_id, quantity, order_id=None):
        """오른쪽 트리(주문 요약)에 주문 항목 추가"""
        menu_name = self.menu_names[menu_id]  # menu_id로 menu_name 찾기
        total_price = quantity * self.menu_prices[menu_name]
        item = QTreeWidgetItem(self.tree_widget2)
        item.setText(0, menu_name)  # 메뉴 이름
        item.setText(1, str(quantity))  # 수량
        item.setText(2, f"{total_price:,}원")  # 총 금액
        item.setData(0, Qt.UserRole, order_id)  # order_id를 저장

    def update_total_price(self):
        """오른쪽 트리(주문 확정 내역)의 총 금액 계산 및 라벨 업데이트"""
        total_price_sum = 0
        for i in range(self.tree_widget2.topLevelItemCount()):
            item = self.tree_widget2.topLevelItem(i)
            total_price_sum += int(item.text(2).replace(',', '').replace('원', ''))

        self.label_total_price.setStyleSheet("background-color: white;")
        self.label_total_price.setText(f"총 금액: {total_price_sum:,}원")


def spin_rclpy_node(node):
    """별도 스레드에서 ROS 노드 spin"""
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    # 실행 시 인자로 테이블 ID (예: "A", "B", "C")를 받음
    if len(sys.argv) < 2:
        print("Usage: python table_display.py <TableID>")
        print("Example: python table_display.py A")
        sys.exit(0)

    table_id_arg = sys.argv[1]  # "A", "B", "C" 중 하나

    # ROS 초기화
    rclpy.init(args=sys.argv)
    
    # PySide2 애플리케이션 생성
    app = QApplication(sys.argv)

    # MainWindow 생성
    window = MainWindow(table_id_arg, None)  # 초기에는 manager가 없으므로 None 전달
    
    # TableManager 생성 및 MainWindow에 연결
    manager = TableManager(table_id_arg, window)
    window.manager = manager  # MainWindow에서 manager 참조 설정

    window.show()

    # ROS 노드를 별도 스레드에서 spin
    thread_manager = threading.Thread(target=spin_rclpy_node, args=(manager,), daemon=True)
    thread_manager.start()


    # PySide2 메인 루프 실행
    sys.exit(app.exec_())
