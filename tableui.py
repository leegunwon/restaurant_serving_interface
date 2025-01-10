from PySide2.QtWidgets import (QApplication, QTreeWidget, QTreeWidgetItem, QMainWindow, QPushButton, QSpinBox, QMessageBox)
from PySide2.QtUiTools import QUiLoader
from PySide2.QtCore import QFile, QCoreApplication, Qt
import os


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # 애플리케이션 속성 설정 (QSocketNotifier 경고 방지)
        QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)

        # UI 파일 경로 확인
        ui_file_path = "table.ui"  # UI 파일 경로
        if not os.path.exists(ui_file_path):
            raise FileNotFoundError(f"UI 파일을 찾을 수 없습니다: {ui_file_path}")

        # UI 파일 로드
        ui_file = QFile(ui_file_path)
        ui_file.open(QFile.ReadOnly)
        loader = QUiLoader()
        self.ui = loader.load(ui_file, self)
        ui_file.close()

        # TreeWidget 가져오기
        self.tree_widget = self.ui.findChild(QTreeWidget, "treeWidget")  # Object Name으로 TreeWidget 가져오기

        # 버튼 이벤트 연결
        self.connect_buttons()

    def connect_buttons(self):
        """버튼 클릭 이벤트 설정"""
        self.ui.findChild(QPushButton, "pushButton_3").clicked.connect(
            lambda: self.add_to_tree("Tomato Pasta"))
        self.ui.findChild(QPushButton, "pushButton_4").clicked.connect(
            lambda: self.add_to_tree("Cream Pasta"))
        self.ui.findChild(QPushButton, "pushButton_5").clicked.connect(
            lambda: self.add_to_tree("Vongole Pasta"))

        self.ui.findChild(QPushButton, "pushButton_7").clicked.connect(
            lambda: self.add_to_tree("Combination Pizza"))
        self.ui.findChild(QPushButton, "pushButton_8").clicked.connect(
            lambda: self.add_to_tree("Pepperoni Pizza"))
        self.ui.findChild(QPushButton, "pushButton_9").clicked.connect(
            lambda: self.add_to_tree("Hawaiian Pizza"))

        self.ui.findChild(QPushButton, "pushButton_10").clicked.connect(
            lambda: self.add_to_tree("Tenderloin Steak"))
        self.ui.findChild(QPushButton, "pushButton_11").clicked.connect(
            lambda: self.add_to_tree("Sirloin Steak"))
        self.ui.findChild(QPushButton, "pushButton_12").clicked.connect(
            lambda: self.add_to_tree("T-bone Steak"))

        self.ui.findChild(QPushButton, "pushButton_13").clicked.connect(
            lambda: self.add_to_tree("Coke"))
        self.ui.findChild(QPushButton, "pushButton_14").clicked.connect(
            lambda: self.add_to_tree("Cider"))
        self.ui.findChild(QPushButton, "pushButton_15").clicked.connect(
            lambda: self.add_to_tree("Blueberry Ade"))

        # Reset 버튼 연결
        self.ui.findChild(QPushButton, "pushButton_2").clicked.connect(self.reset_tree_widget)
        self.ui.findChild(QPushButton, "pushButton_6").clicked.connect(self.remove_selected_item)

        # 주문 버튼 연결
        self.ui.findChild(QPushButton, "pushButton").clicked.connect(self.show_order_summary)

    def add_to_tree(self, item_name):
        """TreeWidget에 아이템 추가"""
        for i in range(self.tree_widget.topLevelItemCount()):
            item = self.tree_widget.topLevelItem(i)
            if item.text(0) == item_name:
                spinbox = self.tree_widget.itemWidget(item, 1)
                if spinbox:
                    spinbox.setValue(spinbox.value() + 1)
                return

        item = QTreeWidgetItem(self.tree_widget)
        item.setText(0, item_name)
        spinbox = QSpinBox()
        spinbox.setValue(1)
        spinbox.setMinimum(1)
        spinbox.setMaximum(99)
        self.tree_widget.setItemWidget(item, 1, spinbox)

    def reset_tree_widget(self):
        """TreeWidget 초기화"""
        self.tree_widget.clear()

    def remove_selected_item(self):
        """TreeWidget에서 선택된 항목 삭제"""
        selected_item = self.tree_widget.currentItem()
        if selected_item:
            index = self.tree_widget.indexOfTopLevelItem(selected_item)
            if index != -1:
                self.tree_widget.takeTopLevelItem(index)

    def show_order_summary(self):
        """주문 요약 창 표시"""
        if self.tree_widget.topLevelItemCount() == 0:
            QMessageBox.information(self, "주문", "주문할 항목이 없습니다.")
            return

        order_summary = "주문 내역:\n"
        for i in range(self.tree_widget.topLevelItemCount()):
            item = self.tree_widget.topLevelItem(i)
            menu_name = item.text(0)
            spinbox = self.tree_widget.itemWidget(item, 1)
            quantity = spinbox.value() if spinbox else 1
            order_summary += f"{menu_name}: {quantity}개\n"

        reply = QMessageBox.question(
            self, "주문 확인", order_summary + "\n주문하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            QMessageBox.information(self, "주문 완료", "주문이 완료되었습니다.")
            self.reset_tree_widget()

    def closeEvent(self, event):
        """프로그램 종료 방지"""
        event.ignore()


if __name__ == "__main__":
    app = QApplication([])
    window = MainWindow()
    window.ui.show()
    app.exec_()
