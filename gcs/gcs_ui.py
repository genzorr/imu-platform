# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gcs_ui.ui'
#
# Created by: PyQt5 UI code generator 5.12.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.setWindowModality(QtCore.Qt.ApplicationModal)
        MainWindow.resize(1119, 878)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.centralwidget.sizePolicy().hasHeightForWidth())
        self.centralwidget.setSizePolicy(sizePolicy)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(580, 10, 531, 211))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridTextLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridTextLayout.setSizeConstraint(QtWidgets.QLayout.SetNoConstraint)
        self.gridTextLayout.setContentsMargins(0, 0, 0, 0)
        self.gridTextLayout.setObjectName("gridTextLayout")
        self.textBrowser = QtWidgets.QTextBrowser(self.gridLayoutWidget)
        self.textBrowser.setObjectName("textBrowser")
        self.gridTextLayout.addWidget(self.textBrowser, 0, 0, 1, 1)
        self.gridLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(580, 230, 531, 601))
        self.gridLayoutWidget_2.setObjectName("gridLayoutWidget_2")
        self.grid3DGrafLayout = QtWidgets.QGridLayout(self.gridLayoutWidget_2)
        self.grid3DGrafLayout.setSizeConstraint(QtWidgets.QLayout.SetNoConstraint)
        self.grid3DGrafLayout.setContentsMargins(0, 0, 0, 0)
        self.grid3DGrafLayout.setObjectName("grid3DGrafLayout")
        self.gridLayoutWidget_3 = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget_3.setGeometry(QtCore.QRect(10, 10, 561, 821))
        self.gridLayoutWidget_3.setObjectName("gridLayoutWidget_3")
        self.gridGrafLayout = QtWidgets.QGridLayout(self.gridLayoutWidget_3)
        self.gridGrafLayout.setContentsMargins(0, 0, 0, 0)
        self.gridGrafLayout.setObjectName("gridGrafLayout")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setSizeConstraint(QtWidgets.QLayout.SetNoConstraint)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.gridGrafLayout.addLayout(self.verticalLayout_5, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1119, 21))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))


