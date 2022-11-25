'''
Copyright 2021 Twente Medical Systems international B.V., Oldenzaal The Netherlands

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

#######  #     #   #####   #  ######      #     #
   #     ##   ##  #        #  #     #     #     #
   #     # # # #  #        #  #     #     #     #
   #     #  #  #   #####   #  ######       #   #
   #     #     #        #  #  #     #      #   #
   #     #     #        #  #  #     #       # #
   #     #     #  #####    #  ######   #     #     #

TMSiSDK: GUI window used by the real-time sample data plotter


'''

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

from pyqtgraph import GraphicsLayoutWidget


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(839, 877)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.gridLayout = QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName(u"gridLayout")
        self.channel_list_groupbox = QGroupBox(self.centralwidget)
        self.channel_list_groupbox.setObjectName(u"channel_list_groupbox")
        sizePolicy = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.channel_list_groupbox.sizePolicy().hasHeightForWidth())
        self.channel_list_groupbox.setSizePolicy(sizePolicy)
        self.channel_list_groupbox.setMinimumSize(QSize(100, 0))
        self.channel_list_groupbox.setMaximumSize(QSize(400, 16777215))
        font = QFont()
        font.setBold(False)
        font.setWeight(50)
        self.channel_list_groupbox.setFont(font)

        self.gridLayout.addWidget(self.channel_list_groupbox, 0, 0, 1, 1)

        self.increase_time_button = QPushButton(self.centralwidget)
        self.increase_time_button.setObjectName(u"increase_time_button")

        self.gridLayout.addWidget(self.increase_time_button, 3, 2, 1, 1)

        self.autoscale_button = QPushButton(self.centralwidget)
        self.autoscale_button.setObjectName(u"autoscale_button")

        self.gridLayout.addWidget(self.autoscale_button, 3, 3, 1, 1)

        self.decrease_time_button = QPushButton(self.centralwidget)
        self.decrease_time_button.setObjectName(u"decrease_time_button")

        self.gridLayout.addWidget(self.decrease_time_button, 3, 1, 1, 1)

        self.RealTimePlotWidget = GraphicsLayoutWidget(self.centralwidget)
        self.RealTimePlotWidget.setObjectName(u"RealTimePlotWidget")

        self.gridLayout.addWidget(self.RealTimePlotWidget, 0, 1, 3, 3)

        self.gridLayout_2 = QGridLayout()
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.hide_UNI_button = QPushButton(self.centralwidget)
        self.hide_UNI_button.setObjectName(u"hide_UNI_button")
        self.hide_UNI_button.setMaximumSize(QSize(100, 16777215))
        self.hide_UNI_button.setLayoutDirection(Qt.RightToLeft)

        self.gridLayout_2.addWidget(self.hide_UNI_button, 0, 1, 1, 1)

        self.show_BIP_button = QPushButton(self.centralwidget)
        self.show_BIP_button.setObjectName(u"show_BIP_button")

        self.gridLayout_2.addWidget(self.show_BIP_button, 1, 0, 1, 1)

        self.show_UNI_button = QPushButton(self.centralwidget)
        self.show_UNI_button.setObjectName(u"show_UNI_button")
        self.show_UNI_button.setMaximumSize(QSize(100, 16777215))

        self.gridLayout_2.addWidget(self.show_UNI_button, 0, 0, 1, 1)

        self.hide_BIP_button = QPushButton(self.centralwidget)
        self.hide_BIP_button.setObjectName(u"hide_BIP_button")
        self.hide_BIP_button.setLayoutDirection(Qt.RightToLeft)

        self.gridLayout_2.addWidget(self.hide_BIP_button, 1, 1, 1, 1)

        self.show_AUX_button = QPushButton(self.centralwidget)
        self.show_AUX_button.setObjectName(u"show_AUX_button")

        self.gridLayout_2.addWidget(self.show_AUX_button, 2, 0, 1, 1)

        self.show_DIGI_button = QPushButton(self.centralwidget)
        self.show_DIGI_button.setObjectName(u"show_DIGI_button")

        self.gridLayout_2.addWidget(self.show_DIGI_button, 3, 0, 1, 1)

        self.hide_AUX_button = QPushButton(self.centralwidget)
        self.hide_AUX_button.setObjectName(u"hide_AUX_button")
        self.hide_AUX_button.setLayoutDirection(Qt.RightToLeft)

        self.gridLayout_2.addWidget(self.hide_AUX_button, 2, 1, 1, 1)

        self.hide_DIGI_button = QPushButton(self.centralwidget)
        self.hide_DIGI_button.setObjectName(u"hide_DIGI_button")
        self.hide_DIGI_button.setLayoutDirection(Qt.RightToLeft)

        self.gridLayout_2.addWidget(self.hide_DIGI_button, 3, 1, 1, 1)


        self.gridLayout.addLayout(self.gridLayout_2, 2, 0, 1, 1)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 839, 21))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.channel_list_groupbox.setTitle(QCoreApplication.translate("MainWindow", u"Channel list", None))
        self.increase_time_button.setText(QCoreApplication.translate("MainWindow", u"Increase time range", None))
        self.autoscale_button.setText(QCoreApplication.translate("MainWindow", u"Auto Scale", None))
        self.decrease_time_button.setText(QCoreApplication.translate("MainWindow", u"Decrease time range", None))
        self.hide_UNI_button.setText(QCoreApplication.translate("MainWindow", u"Hide UNI", None))
        self.show_BIP_button.setText(QCoreApplication.translate("MainWindow", u"Show BIP", None))
        self.show_UNI_button.setText(QCoreApplication.translate("MainWindow", u"Show UNI", None))
        self.hide_BIP_button.setText(QCoreApplication.translate("MainWindow", u"Hide BIP", None))
        self.show_AUX_button.setText(QCoreApplication.translate("MainWindow", u"Show AUX", None))
        self.show_DIGI_button.setText(QCoreApplication.translate("MainWindow", u"Show DIGI", None))
        self.hide_AUX_button.setText(QCoreApplication.translate("MainWindow", u"Hide AUX", None))
        self.hide_DIGI_button.setText(QCoreApplication.translate("MainWindow", u"Hide DIGI", None))
    # retranslateUi

