# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'kitchenNSjkFE.ui'
##
## Created by: Qt User Interface Compiler version 5.15.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1127, 641)
        self.actionOrder_Analysis = QAction(MainWindow)
        self.actionOrder_Analysis.setObjectName(u"actionOrder_Analysis")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.frame_4 = QFrame(self.centralwidget)
        self.frame_4.setObjectName(u"frame_4")
        self.frame_4.setGeometry(QRect(0, 20, 341, 561))
        self.frame_4.setFrameShape(QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QFrame.Raised)
        self.treeWidget = QTreeWidget(self.frame_4)
        self.treeWidget.setObjectName(u"treeWidget")
        self.treeWidget.setGeometry(QRect(20, 40, 301, 341))
        self.TextLabel = QLabel(self.frame_4)
        self.TextLabel.setObjectName(u"TextLabel")
        self.TextLabel.setGeometry(QRect(20, 10, 121, 20))
        self.TextLabel.setLineWidth(1)
        self.frame_5 = QFrame(self.centralwidget)
        self.frame_5.setObjectName(u"frame_5")
        self.frame_5.setGeometry(QRect(380, 20, 341, 561))
        self.frame_5.setFrameShape(QFrame.StyledPanel)
        self.frame_5.setFrameShadow(QFrame.Raised)
        self.treeWidget_2 = QTreeWidget(self.frame_5)
        self.treeWidget_2.setObjectName(u"treeWidget_2")
        self.treeWidget_2.setGeometry(QRect(20, 40, 301, 341))
        self.TextLabel_2 = QLabel(self.frame_5)
        self.TextLabel_2.setObjectName(u"TextLabel_2")
        self.TextLabel_2.setGeometry(QRect(20, 10, 121, 20))
        self.TextLabel_2.setLineWidth(1)
        self.frame_6 = QFrame(self.centralwidget)
        self.frame_6.setObjectName(u"frame_6")
        self.frame_6.setGeometry(QRect(760, 20, 341, 561))
        self.frame_6.setFrameShape(QFrame.StyledPanel)
        self.frame_6.setFrameShadow(QFrame.Raised)
        self.treeWidget_4 = QTreeWidget(self.frame_6)
        self.treeWidget_4.setObjectName(u"treeWidget_4")
        self.treeWidget_4.setGeometry(QRect(20, 40, 301, 341))
        self.TextLabel_3 = QLabel(self.frame_6)
        self.TextLabel_3.setObjectName(u"TextLabel_3")
        self.TextLabel_3.setGeometry(QRect(20, 10, 121, 20))
        self.TextLabel_3.setLineWidth(1)
        self.pushButton = QPushButton(self.centralwidget)
        self.pushButton.setObjectName(u"pushButton")
        self.pushButton.setGeometry(QRect(20, 430, 301, 41))
        self.pushButton_2 = QPushButton(self.centralwidget)
        self.pushButton_2.setObjectName(u"pushButton_2")
        self.pushButton_2.setGeometry(QRect(20, 530, 301, 41))
        self.pushButton_3 = QPushButton(self.centralwidget)
        self.pushButton_3.setObjectName(u"pushButton_3")
        self.pushButton_3.setGeometry(QRect(400, 430, 301, 41))
        self.pushButton_4 = QPushButton(self.centralwidget)
        self.pushButton_4.setObjectName(u"pushButton_4")
        self.pushButton_4.setGeometry(QRect(400, 530, 301, 41))
        self.pushButton_5 = QPushButton(self.centralwidget)
        self.pushButton_5.setObjectName(u"pushButton_5")
        self.pushButton_5.setGeometry(QRect(780, 430, 301, 41))
        self.pushButton_6 = QPushButton(self.centralwidget)
        self.pushButton_6.setObjectName(u"pushButton_6")
        self.pushButton_6.setGeometry(QRect(780, 530, 301, 41))
        self.pushButton_7 = QPushButton(self.centralwidget)
        self.pushButton_7.setObjectName(u"pushButton_7")
        self.pushButton_7.setGeometry(QRect(20, 480, 301, 41))
        self.pushButton_8 = QPushButton(self.centralwidget)
        self.pushButton_8.setObjectName(u"pushButton_8")
        self.pushButton_8.setGeometry(QRect(400, 480, 301, 41))
        self.pushButton_9 = QPushButton(self.centralwidget)
        self.pushButton_9.setObjectName(u"pushButton_9")
        self.pushButton_9.setGeometry(QRect(780, 480, 301, 41))
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1127, 28))
        self.menukitchen_interface = QMenu(self.menubar)
        self.menukitchen_interface.setObjectName(u"menukitchen_interface")
        MainWindow.setMenuBar(self.menubar)

        self.menubar.addAction(self.menukitchen_interface.menuAction())
        self.menukitchen_interface.addAction(self.actionOrder_Analysis)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"kitchen", None))
        self.actionOrder_Analysis.setText(QCoreApplication.translate("MainWindow", u"Order Analysis", None))
        ___qtreewidgetitem = self.treeWidget.headerItem()
        ___qtreewidgetitem.setText(2, QCoreApplication.translate("MainWindow", u"Status", None));
        ___qtreewidgetitem.setText(1, QCoreApplication.translate("MainWindow", u"Amounts", None));
        ___qtreewidgetitem.setText(0, QCoreApplication.translate("MainWindow", u"Menu", None));
        self.TextLabel.setText(QCoreApplication.translate("MainWindow", u"Table A order list", None))
        ___qtreewidgetitem1 = self.treeWidget_2.headerItem()
        ___qtreewidgetitem1.setText(2, QCoreApplication.translate("MainWindow", u"Status", None));
        ___qtreewidgetitem1.setText(1, QCoreApplication.translate("MainWindow", u"Amounts", None));
        ___qtreewidgetitem1.setText(0, QCoreApplication.translate("MainWindow", u"Menu", None));
        self.TextLabel_2.setText(QCoreApplication.translate("MainWindow", u"Table B order list", None))
        ___qtreewidgetitem2 = self.treeWidget_4.headerItem()
        ___qtreewidgetitem2.setText(2, QCoreApplication.translate("MainWindow", u"Status", None));
        ___qtreewidgetitem2.setText(1, QCoreApplication.translate("MainWindow", u"Amounts", None));
        ___qtreewidgetitem2.setText(0, QCoreApplication.translate("MainWindow", u"Menu", None));
        self.TextLabel_3.setText(QCoreApplication.translate("MainWindow", u"Table C order list", None))
        self.pushButton.setText(QCoreApplication.translate("MainWindow", u"Check Menu", None))
        self.pushButton_2.setText(QCoreApplication.translate("MainWindow", u"Serve to Table A", None))
        self.pushButton_3.setText(QCoreApplication.translate("MainWindow", u"Check Menu", None))
        self.pushButton_4.setText(QCoreApplication.translate("MainWindow", u"Serve to Table B", None))
        self.pushButton_5.setText(QCoreApplication.translate("MainWindow", u"Check Menu", None))
        self.pushButton_6.setText(QCoreApplication.translate("MainWindow", u"Serve to Table C", None))
        self.pushButton_7.setText(QCoreApplication.translate("MainWindow", u"Cancel Menu", None))
        self.pushButton_8.setText(QCoreApplication.translate("MainWindow", u"Cancel Menu", None))
        self.pushButton_9.setText(QCoreApplication.translate("MainWindow", u"Cancel Menu", None))
        self.menukitchen_interface.setTitle(QCoreApplication.translate("MainWindow", u"Statistics", None))
    # retranslateUi
