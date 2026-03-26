# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'gcs_dashboard.ui'
##
## Created by: Qt User Interface Compiler version 6.11.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QFrame, QGridLayout, QGroupBox,
    QHBoxLayout, QHeaderView, QLabel, QLineEdit,
    QMainWindow, QProgressBar, QPushButton, QSizePolicy,
    QSlider, QSpacerItem, QTabWidget, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1200, 800)
        self.centralWidget = QWidget(MainWindow)
        self.centralWidget.setObjectName(u"centralWidget")
        self.verticalLayout_main = QVBoxLayout(self.centralWidget)
        self.verticalLayout_main.setObjectName(u"verticalLayout_main")
        self.frame_topbar = QFrame(self.centralWidget)
        self.frame_topbar.setObjectName(u"frame_topbar")
        self.frame_topbar.setFrameShape(QFrame.StyledPanel)
        self.frame_topbar.setFrameShadow(QFrame.Raised)
        self.hLayout_topbar = QHBoxLayout(self.frame_topbar)
        self.hLayout_topbar.setObjectName(u"hLayout_topbar")
        self.hLayout_topbar.setContentsMargins(10, 5, 10, 5)
        self.lbl_batt_volt = QLabel(self.frame_topbar)
        self.lbl_batt_volt.setObjectName(u"lbl_batt_volt")
        font = QFont()
        font.setBold(True)
        self.lbl_batt_volt.setFont(font)

        self.hLayout_topbar.addWidget(self.lbl_batt_volt)

        self.bar_battery_volt = QProgressBar(self.frame_topbar)
        self.bar_battery_volt.setObjectName(u"bar_battery_volt")
        self.bar_battery_volt.setMinimumSize(QSize(150, 20))
        self.bar_battery_volt.setMaximumSize(QSize(200, 25))
        self.bar_battery_volt.setValue(0)
        self.bar_battery_volt.setTextVisible(False)

        self.hLayout_topbar.addWidget(self.bar_battery_volt)

        self.lbl_batt_perc = QLabel(self.frame_topbar)
        self.lbl_batt_perc.setObjectName(u"lbl_batt_perc")
        self.lbl_batt_perc.setFont(font)

        self.hLayout_topbar.addWidget(self.lbl_batt_perc)

        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.hLayout_topbar.addItem(self.horizontalSpacer)

        self.lbl_wifi_status = QLabel(self.frame_topbar)
        self.lbl_wifi_status.setObjectName(u"lbl_wifi_status")

        self.hLayout_topbar.addWidget(self.lbl_wifi_status)

        self.lbl_wifi_icon = QLabel(self.frame_topbar)
        self.lbl_wifi_icon.setObjectName(u"lbl_wifi_icon")
        self.lbl_wifi_icon.setFont(font)

        self.hLayout_topbar.addWidget(self.lbl_wifi_icon)

        self.horizontalSpacer_2 = QSpacerItem(20, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.hLayout_topbar.addItem(self.horizontalSpacer_2)

        self.btn_disconnect = QPushButton(self.frame_topbar)
        self.btn_disconnect.setObjectName(u"btn_disconnect")
        self.btn_disconnect.setMinimumSize(QSize(120, 30))

        self.hLayout_topbar.addWidget(self.btn_disconnect)


        self.verticalLayout_main.addWidget(self.frame_topbar)

        self.gridLayout_main = QGridLayout()
        self.gridLayout_main.setObjectName(u"gridLayout_main")
        self.grp_telemetry = QGroupBox(self.centralWidget)
        self.grp_telemetry.setObjectName(u"grp_telemetry")
        self.gridLayout_telemetry = QGridLayout(self.grp_telemetry)
        self.gridLayout_telemetry.setObjectName(u"gridLayout_telemetry")
        self.lbl_batt_curr = QLabel(self.grp_telemetry)
        self.lbl_batt_curr.setObjectName(u"lbl_batt_curr")

        self.gridLayout_telemetry.addWidget(self.lbl_batt_curr, 0, 0, 1, 1)

        self.val_batt_curr = QLabel(self.grp_telemetry)
        self.val_batt_curr.setObjectName(u"val_batt_curr")
        self.val_batt_curr.setFont(font)

        self.gridLayout_telemetry.addWidget(self.val_batt_curr, 0, 1, 1, 1)

        self.lbl_mode = QLabel(self.grp_telemetry)
        self.lbl_mode.setObjectName(u"lbl_mode")

        self.gridLayout_telemetry.addWidget(self.lbl_mode, 1, 0, 1, 1)

        self.val_mode = QLabel(self.grp_telemetry)
        self.val_mode.setObjectName(u"val_mode")
        self.val_mode.setFont(font)

        self.gridLayout_telemetry.addWidget(self.val_mode, 1, 1, 1, 1)

        self.lbl_armed = QLabel(self.grp_telemetry)
        self.lbl_armed.setObjectName(u"lbl_armed")

        self.gridLayout_telemetry.addWidget(self.lbl_armed, 2, 0, 1, 1)

        self.val_armed = QLabel(self.grp_telemetry)
        self.val_armed.setObjectName(u"val_armed")
        self.val_armed.setFont(font)

        self.gridLayout_telemetry.addWidget(self.val_armed, 2, 1, 1, 1)

        self.lbl_alt = QLabel(self.grp_telemetry)
        self.lbl_alt.setObjectName(u"lbl_alt")

        self.gridLayout_telemetry.addWidget(self.lbl_alt, 3, 0, 1, 1)

        self.val_alt = QLabel(self.grp_telemetry)
        self.val_alt.setObjectName(u"val_alt")
        self.val_alt.setFont(font)

        self.gridLayout_telemetry.addWidget(self.val_alt, 3, 1, 1, 1)

        self.grp_gps_coords = QGroupBox(self.grp_telemetry)
        self.grp_gps_coords.setObjectName(u"grp_gps_coords")
        self.gridLayout_gps_coords = QGridLayout(self.grp_gps_coords)
        self.gridLayout_gps_coords.setObjectName(u"gridLayout_gps_coords")
        self.lbl_lat = QLabel(self.grp_gps_coords)
        self.lbl_lat.setObjectName(u"lbl_lat")

        self.gridLayout_gps_coords.addWidget(self.lbl_lat, 0, 0, 1, 1)

        self.val_lat = QLabel(self.grp_gps_coords)
        self.val_lat.setObjectName(u"val_lat")
        self.val_lat.setFont(font)

        self.gridLayout_gps_coords.addWidget(self.val_lat, 0, 1, 1, 1)

        self.lbl_lon = QLabel(self.grp_gps_coords)
        self.lbl_lon.setObjectName(u"lbl_lon")

        self.gridLayout_gps_coords.addWidget(self.lbl_lon, 0, 2, 1, 1)

        self.val_lon = QLabel(self.grp_gps_coords)
        self.val_lon.setObjectName(u"val_lon")
        self.val_lon.setFont(font)

        self.gridLayout_gps_coords.addWidget(self.val_lon, 0, 3, 1, 1)


        self.gridLayout_telemetry.addWidget(self.grp_gps_coords, 4, 0, 1, 2)

        self.lbl_roll = QLabel(self.grp_telemetry)
        self.lbl_roll.setObjectName(u"lbl_roll")

        self.gridLayout_telemetry.addWidget(self.lbl_roll, 0, 4, 1, 1)

        self.val_roll = QLabel(self.grp_telemetry)
        self.val_roll.setObjectName(u"val_roll")
        self.val_roll.setFont(font)

        self.gridLayout_telemetry.addWidget(self.val_roll, 0, 5, 1, 1)

        self.lbl_pitch = QLabel(self.grp_telemetry)
        self.lbl_pitch.setObjectName(u"lbl_pitch")

        self.gridLayout_telemetry.addWidget(self.lbl_pitch, 1, 4, 1, 1)

        self.val_pitch = QLabel(self.grp_telemetry)
        self.val_pitch.setObjectName(u"val_pitch")
        self.val_pitch.setFont(font)

        self.gridLayout_telemetry.addWidget(self.val_pitch, 1, 5, 1, 1)

        self.lbl_yaw = QLabel(self.grp_telemetry)
        self.lbl_yaw.setObjectName(u"lbl_yaw")

        self.gridLayout_telemetry.addWidget(self.lbl_yaw, 2, 4, 1, 1)

        self.val_yaw = QLabel(self.grp_telemetry)
        self.val_yaw.setObjectName(u"val_yaw")
        self.val_yaw.setFont(font)

        self.gridLayout_telemetry.addWidget(self.val_yaw, 2, 5, 1, 1)

        self.lbl_gps_fix = QLabel(self.grp_telemetry)
        self.lbl_gps_fix.setObjectName(u"lbl_gps_fix")

        self.gridLayout_telemetry.addWidget(self.lbl_gps_fix, 3, 4, 1, 1)

        self.val_gps_fix = QLabel(self.grp_telemetry)
        self.val_gps_fix.setObjectName(u"val_gps_fix")
        self.val_gps_fix.setFont(font)

        self.gridLayout_telemetry.addWidget(self.val_gps_fix, 3, 5, 1, 1)

        self.lbl_sats = QLabel(self.grp_telemetry)
        self.lbl_sats.setObjectName(u"lbl_sats")

        self.gridLayout_telemetry.addWidget(self.lbl_sats, 4, 4, 1, 1)

        self.val_sats = QLabel(self.grp_telemetry)
        self.val_sats.setObjectName(u"val_sats")
        self.val_sats.setFont(font)

        self.gridLayout_telemetry.addWidget(self.val_sats, 4, 5, 1, 1)

        self.lbl_spd = QLabel(self.grp_telemetry)
        self.lbl_spd.setObjectName(u"lbl_spd")

        self.gridLayout_telemetry.addWidget(self.lbl_spd, 5, 4, 1, 1)

        self.val_spd = QLabel(self.grp_telemetry)
        self.val_spd.setObjectName(u"val_spd")
        self.val_spd.setFont(font)

        self.gridLayout_telemetry.addWidget(self.val_spd, 5, 5, 1, 1)


        self.gridLayout_main.addWidget(self.grp_telemetry, 0, 0, 1, 1)

        self.grp_manual_control = QGroupBox(self.centralWidget)
        self.grp_manual_control.setObjectName(u"grp_manual_control")
        self.gridLayout_manual = QGridLayout(self.grp_manual_control)
        self.gridLayout_manual.setObjectName(u"gridLayout_manual")
        self.lbl_throttle = QLabel(self.grp_manual_control)
        self.lbl_throttle.setObjectName(u"lbl_throttle")

        self.gridLayout_manual.addWidget(self.lbl_throttle, 0, 0, 1, 1)

        self.slider_throttle = QSlider(self.grp_manual_control)
        self.slider_throttle.setObjectName(u"slider_throttle")
        self.slider_throttle.setMinimum(1000)
        self.slider_throttle.setMaximum(2000)
        self.slider_throttle.setValue(1000)
        self.slider_throttle.setOrientation(Qt.Horizontal)

        self.gridLayout_manual.addWidget(self.slider_throttle, 0, 1, 1, 1)

        self.val_throttle = QLabel(self.grp_manual_control)
        self.val_throttle.setObjectName(u"val_throttle")

        self.gridLayout_manual.addWidget(self.val_throttle, 0, 2, 1, 1)

        self.lbl_roll_input = QLabel(self.grp_manual_control)
        self.lbl_roll_input.setObjectName(u"lbl_roll_input")

        self.gridLayout_manual.addWidget(self.lbl_roll_input, 1, 0, 1, 1)

        self.slider_roll = QSlider(self.grp_manual_control)
        self.slider_roll.setObjectName(u"slider_roll")
        self.slider_roll.setMinimum(1000)
        self.slider_roll.setMaximum(2000)
        self.slider_roll.setValue(1500)
        self.slider_roll.setOrientation(Qt.Horizontal)

        self.gridLayout_manual.addWidget(self.slider_roll, 1, 1, 1, 1)

        self.val_roll_input = QLabel(self.grp_manual_control)
        self.val_roll_input.setObjectName(u"val_roll_input")

        self.gridLayout_manual.addWidget(self.val_roll_input, 1, 2, 1, 1)

        self.lbl_pitch_input = QLabel(self.grp_manual_control)
        self.lbl_pitch_input.setObjectName(u"lbl_pitch_input")

        self.gridLayout_manual.addWidget(self.lbl_pitch_input, 2, 0, 1, 1)

        self.slider_pitch = QSlider(self.grp_manual_control)
        self.slider_pitch.setObjectName(u"slider_pitch")
        self.slider_pitch.setMinimum(1000)
        self.slider_pitch.setMaximum(2000)
        self.slider_pitch.setValue(1500)
        self.slider_pitch.setOrientation(Qt.Horizontal)

        self.gridLayout_manual.addWidget(self.slider_pitch, 2, 1, 1, 1)

        self.val_pitch_input = QLabel(self.grp_manual_control)
        self.val_pitch_input.setObjectName(u"val_pitch_input")

        self.gridLayout_manual.addWidget(self.val_pitch_input, 2, 2, 1, 1)

        self.lbl_yaw_input = QLabel(self.grp_manual_control)
        self.lbl_yaw_input.setObjectName(u"lbl_yaw_input")

        self.gridLayout_manual.addWidget(self.lbl_yaw_input, 3, 0, 1, 1)

        self.slider_yaw = QSlider(self.grp_manual_control)
        self.slider_yaw.setObjectName(u"slider_yaw")
        self.slider_yaw.setMinimum(1000)
        self.slider_yaw.setMaximum(2000)
        self.slider_yaw.setValue(1500)
        self.slider_yaw.setOrientation(Qt.Horizontal)

        self.gridLayout_manual.addWidget(self.slider_yaw, 3, 1, 1, 1)

        self.val_yaw_input = QLabel(self.grp_manual_control)
        self.val_yaw_input.setObjectName(u"val_yaw_input")

        self.gridLayout_manual.addWidget(self.val_yaw_input, 3, 2, 1, 1)

        self.lbl_aux1 = QLabel(self.grp_manual_control)
        self.lbl_aux1.setObjectName(u"lbl_aux1")

        self.gridLayout_manual.addWidget(self.lbl_aux1, 4, 0, 1, 1)

        self.slider_aux1 = QSlider(self.grp_manual_control)
        self.slider_aux1.setObjectName(u"slider_aux1")
        self.slider_aux1.setMinimum(1000)
        self.slider_aux1.setMaximum(2000)
        self.slider_aux1.setValue(1000)
        self.slider_aux1.setOrientation(Qt.Horizontal)

        self.gridLayout_manual.addWidget(self.slider_aux1, 4, 1, 1, 1)

        self.val_aux1 = QLabel(self.grp_manual_control)
        self.val_aux1.setObjectName(u"val_aux1")

        self.gridLayout_manual.addWidget(self.val_aux1, 4, 2, 1, 1)

        self.lbl_aux2 = QLabel(self.grp_manual_control)
        self.lbl_aux2.setObjectName(u"lbl_aux2")

        self.gridLayout_manual.addWidget(self.lbl_aux2, 5, 0, 1, 1)

        self.slider_aux2 = QSlider(self.grp_manual_control)
        self.slider_aux2.setObjectName(u"slider_aux2")
        self.slider_aux2.setMinimum(1000)
        self.slider_aux2.setMaximum(2000)
        self.slider_aux2.setValue(1000)
        self.slider_aux2.setOrientation(Qt.Horizontal)

        self.gridLayout_manual.addWidget(self.slider_aux2, 5, 1, 1, 1)

        self.val_aux2 = QLabel(self.grp_manual_control)
        self.val_aux2.setObjectName(u"val_aux2")

        self.gridLayout_manual.addWidget(self.val_aux2, 5, 2, 1, 1)

        self.hLayout_buttons = QHBoxLayout()
        self.hLayout_buttons.setObjectName(u"hLayout_buttons")
        self.btn_send_manual = QPushButton(self.grp_manual_control)
        self.btn_send_manual.setObjectName(u"btn_send_manual")

        self.hLayout_buttons.addWidget(self.btn_send_manual)

        self.btn_arm = QPushButton(self.grp_manual_control)
        self.btn_arm.setObjectName(u"btn_arm")

        self.hLayout_buttons.addWidget(self.btn_arm)

        self.btn_disarm = QPushButton(self.grp_manual_control)
        self.btn_disarm.setObjectName(u"btn_disarm")

        self.hLayout_buttons.addWidget(self.btn_disarm)

        self.btn_hold = QPushButton(self.grp_manual_control)
        self.btn_hold.setObjectName(u"btn_hold")

        self.hLayout_buttons.addWidget(self.btn_hold)

        self.btn_rth = QPushButton(self.grp_manual_control)
        self.btn_rth.setObjectName(u"btn_rth")

        self.hLayout_buttons.addWidget(self.btn_rth)

        self.btn_start_mission_main = QPushButton(self.grp_manual_control)
        self.btn_start_mission_main.setObjectName(u"btn_start_mission_main")

        self.hLayout_buttons.addWidget(self.btn_start_mission_main)


        self.gridLayout_manual.addLayout(self.hLayout_buttons, 6, 0, 1, 3)


        self.gridLayout_main.addWidget(self.grp_manual_control, 0, 1, 1, 1)

        self.grp_motors = QGroupBox(self.centralWidget)
        self.grp_motors.setObjectName(u"grp_motors")
        self.hLayout_motors = QHBoxLayout(self.grp_motors)
        self.hLayout_motors.setObjectName(u"hLayout_motors")
        self.vLayout_motor1 = QVBoxLayout()
        self.vLayout_motor1.setObjectName(u"vLayout_motor1")
        self.lbl_motor1 = QLabel(self.grp_motors)
        self.lbl_motor1.setObjectName(u"lbl_motor1")
        self.lbl_motor1.setAlignment(Qt.AlignCenter)

        self.vLayout_motor1.addWidget(self.lbl_motor1)

        self.val_motor1 = QLabel(self.grp_motors)
        self.val_motor1.setObjectName(u"val_motor1")
        self.val_motor1.setFont(font)
        self.val_motor1.setAlignment(Qt.AlignCenter)

        self.vLayout_motor1.addWidget(self.val_motor1)

        self.bar_motor1 = QProgressBar(self.grp_motors)
        self.bar_motor1.setObjectName(u"bar_motor1")
        self.bar_motor1.setMinimum(1000)
        self.bar_motor1.setMaximum(2000)
        self.bar_motor1.setValue(1000)
        self.bar_motor1.setOrientation(Qt.Vertical)
        self.bar_motor1.setTextVisible(False)

        self.vLayout_motor1.addWidget(self.bar_motor1)


        self.hLayout_motors.addLayout(self.vLayout_motor1)

        self.vLayout_motor2 = QVBoxLayout()
        self.vLayout_motor2.setObjectName(u"vLayout_motor2")
        self.lbl_motor2 = QLabel(self.grp_motors)
        self.lbl_motor2.setObjectName(u"lbl_motor2")
        self.lbl_motor2.setAlignment(Qt.AlignCenter)

        self.vLayout_motor2.addWidget(self.lbl_motor2)

        self.val_motor2 = QLabel(self.grp_motors)
        self.val_motor2.setObjectName(u"val_motor2")
        self.val_motor2.setFont(font)
        self.val_motor2.setAlignment(Qt.AlignCenter)

        self.vLayout_motor2.addWidget(self.val_motor2)

        self.bar_motor2 = QProgressBar(self.grp_motors)
        self.bar_motor2.setObjectName(u"bar_motor2")
        self.bar_motor2.setMinimum(1000)
        self.bar_motor2.setMaximum(2000)
        self.bar_motor2.setValue(1000)
        self.bar_motor2.setOrientation(Qt.Vertical)
        self.bar_motor2.setTextVisible(False)

        self.vLayout_motor2.addWidget(self.bar_motor2)


        self.hLayout_motors.addLayout(self.vLayout_motor2)

        self.vLayout_motor3 = QVBoxLayout()
        self.vLayout_motor3.setObjectName(u"vLayout_motor3")
        self.lbl_motor3 = QLabel(self.grp_motors)
        self.lbl_motor3.setObjectName(u"lbl_motor3")
        self.lbl_motor3.setAlignment(Qt.AlignCenter)

        self.vLayout_motor3.addWidget(self.lbl_motor3)

        self.val_motor3 = QLabel(self.grp_motors)
        self.val_motor3.setObjectName(u"val_motor3")
        self.val_motor3.setFont(font)
        self.val_motor3.setAlignment(Qt.AlignCenter)

        self.vLayout_motor3.addWidget(self.val_motor3)

        self.bar_motor3 = QProgressBar(self.grp_motors)
        self.bar_motor3.setObjectName(u"bar_motor3")
        self.bar_motor3.setMinimum(1000)
        self.bar_motor3.setMaximum(2000)
        self.bar_motor3.setValue(1000)
        self.bar_motor3.setOrientation(Qt.Vertical)
        self.bar_motor3.setTextVisible(False)

        self.vLayout_motor3.addWidget(self.bar_motor3)


        self.hLayout_motors.addLayout(self.vLayout_motor3)

        self.vLayout_motor4 = QVBoxLayout()
        self.vLayout_motor4.setObjectName(u"vLayout_motor4")
        self.lbl_motor4 = QLabel(self.grp_motors)
        self.lbl_motor4.setObjectName(u"lbl_motor4")
        self.lbl_motor4.setAlignment(Qt.AlignCenter)

        self.vLayout_motor4.addWidget(self.lbl_motor4)

        self.val_motor4 = QLabel(self.grp_motors)
        self.val_motor4.setObjectName(u"val_motor4")
        self.val_motor4.setFont(font)
        self.val_motor4.setAlignment(Qt.AlignCenter)

        self.vLayout_motor4.addWidget(self.val_motor4)

        self.bar_motor4 = QProgressBar(self.grp_motors)
        self.bar_motor4.setObjectName(u"bar_motor4")
        self.bar_motor4.setMinimum(1000)
        self.bar_motor4.setMaximum(2000)
        self.bar_motor4.setValue(1000)
        self.bar_motor4.setOrientation(Qt.Vertical)
        self.bar_motor4.setTextVisible(False)

        self.vLayout_motor4.addWidget(self.bar_motor4)


        self.hLayout_motors.addLayout(self.vLayout_motor4)


        self.gridLayout_main.addWidget(self.grp_motors, 1, 0, 1, 1)

        self.tabWidget = QTabWidget(self.centralWidget)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tab_mission = QWidget()
        self.tab_mission.setObjectName(u"tab_mission")
        self.vLayout_mission = QVBoxLayout(self.tab_mission)
        self.vLayout_mission.setObjectName(u"vLayout_mission")
        self.grp_waypoint_input = QGroupBox(self.tab_mission)
        self.grp_waypoint_input.setObjectName(u"grp_waypoint_input")
        self.hLayout_wp_input = QHBoxLayout(self.grp_waypoint_input)
        self.hLayout_wp_input.setObjectName(u"hLayout_wp_input")
        self.lbl_wp_lat = QLabel(self.grp_waypoint_input)
        self.lbl_wp_lat.setObjectName(u"lbl_wp_lat")

        self.hLayout_wp_input.addWidget(self.lbl_wp_lat)

        self.input_lat = QLineEdit(self.grp_waypoint_input)
        self.input_lat.setObjectName(u"input_lat")

        self.hLayout_wp_input.addWidget(self.input_lat)

        self.lbl_wp_lon = QLabel(self.grp_waypoint_input)
        self.lbl_wp_lon.setObjectName(u"lbl_wp_lon")

        self.hLayout_wp_input.addWidget(self.lbl_wp_lon)

        self.input_lon = QLineEdit(self.grp_waypoint_input)
        self.input_lon.setObjectName(u"input_lon")

        self.hLayout_wp_input.addWidget(self.input_lon)

        self.lbl_wp_alt = QLabel(self.grp_waypoint_input)
        self.lbl_wp_alt.setObjectName(u"lbl_wp_alt")

        self.hLayout_wp_input.addWidget(self.lbl_wp_alt)

        self.input_alt = QLineEdit(self.grp_waypoint_input)
        self.input_alt.setObjectName(u"input_alt")

        self.hLayout_wp_input.addWidget(self.input_alt)

        self.btn_add_waypoint = QPushButton(self.grp_waypoint_input)
        self.btn_add_waypoint.setObjectName(u"btn_add_waypoint")

        self.hLayout_wp_input.addWidget(self.btn_add_waypoint)


        self.vLayout_mission.addWidget(self.grp_waypoint_input)

        self.grp_waypoint_list = QGroupBox(self.tab_mission)
        self.grp_waypoint_list.setObjectName(u"grp_waypoint_list")
        self.vLayout_wp_list = QVBoxLayout(self.grp_waypoint_list)
        self.vLayout_wp_list.setObjectName(u"vLayout_wp_list")
        self.table_waypoints = QTableWidget(self.grp_waypoint_list)
        if (self.table_waypoints.columnCount() < 4):
            self.table_waypoints.setColumnCount(4)
        __qtablewidgetitem = QTableWidgetItem()
        self.table_waypoints.setHorizontalHeaderItem(0, __qtablewidgetitem)
        __qtablewidgetitem1 = QTableWidgetItem()
        self.table_waypoints.setHorizontalHeaderItem(1, __qtablewidgetitem1)
        __qtablewidgetitem2 = QTableWidgetItem()
        self.table_waypoints.setHorizontalHeaderItem(2, __qtablewidgetitem2)
        __qtablewidgetitem3 = QTableWidgetItem()
        self.table_waypoints.setHorizontalHeaderItem(3, __qtablewidgetitem3)
        self.table_waypoints.setObjectName(u"table_waypoints")

        self.vLayout_wp_list.addWidget(self.table_waypoints)


        self.vLayout_mission.addWidget(self.grp_waypoint_list)

        self.hLayout_tab_buttons = QHBoxLayout()
        self.hLayout_tab_buttons.setObjectName(u"hLayout_tab_buttons")
        self.btn_remove = QPushButton(self.tab_mission)
        self.btn_remove.setObjectName(u"btn_remove")

        self.hLayout_tab_buttons.addWidget(self.btn_remove)

        self.btn_clear = QPushButton(self.tab_mission)
        self.btn_clear.setObjectName(u"btn_clear")

        self.hLayout_tab_buttons.addWidget(self.btn_clear)

        self.btn_upload = QPushButton(self.tab_mission)
        self.btn_upload.setObjectName(u"btn_upload")

        self.hLayout_tab_buttons.addWidget(self.btn_upload)

        self.btn_start_mission_tab = QPushButton(self.tab_mission)
        self.btn_start_mission_tab.setObjectName(u"btn_start_mission_tab")

        self.hLayout_tab_buttons.addWidget(self.btn_start_mission_tab)

        self.btn_stop_mission = QPushButton(self.tab_mission)
        self.btn_stop_mission.setObjectName(u"btn_stop_mission")

        self.hLayout_tab_buttons.addWidget(self.btn_stop_mission)


        self.vLayout_mission.addLayout(self.hLayout_tab_buttons)

        self.tabWidget.addTab(self.tab_mission, "")
        self.tab_config = QWidget()
        self.tab_config.setObjectName(u"tab_config")
        self.tabWidget.addTab(self.tab_config, "")
        self.tab_log = QWidget()
        self.tab_log.setObjectName(u"tab_log")
        self.tabWidget.addTab(self.tab_log, "")

        self.gridLayout_main.addWidget(self.tabWidget, 1, 1, 1, 1)


        self.verticalLayout_main.addLayout(self.gridLayout_main)

        MainWindow.setCentralWidget(self.centralWidget)

        self.retranslateUi(MainWindow)

        self.tabWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Drone Ground Control Station - INAV MSP", None))
        self.lbl_batt_volt.setText(QCoreApplication.translate("MainWindow", u"-- V", None))
        self.bar_battery_volt.setStyleSheet(QCoreApplication.translate("MainWindow", u"QProgressBar { border: 2px solid #8f8f91; border-radius: 5px; background-color: #e0e0e0; text-align: center; } QProgressBar::chunk { background-color: gray; border-radius: 3px; }", None))
        self.lbl_batt_perc.setText(QCoreApplication.translate("MainWindow", u"-- %", None))
        self.lbl_wifi_status.setText(QCoreApplication.translate("MainWindow", u"WiFi Status:", None))
        self.lbl_wifi_icon.setText(QCoreApplication.translate("MainWindow", u"\U0001f4f6 \U00000110ang ch\U00001edd", None))
        self.btn_disconnect.setText(QCoreApplication.translate("MainWindow", u"Ng\u1eaft k\u1ebft n\u1ed1i", None))
        self.btn_disconnect.setStyleSheet(QCoreApplication.translate("MainWindow", u"QPushButton { background-color: #F44336; color: white; font-weight: bold; border-radius: 4px; } QPushButton:hover { background-color: #D32F2F; }", None))
        self.grp_telemetry.setTitle(QCoreApplication.translate("MainWindow", u"Telemetry Dashboard", None))
        self.lbl_batt_curr.setText(QCoreApplication.translate("MainWindow", u"Battery Current", None))
        self.val_batt_curr.setText(QCoreApplication.translate("MainWindow", u"N/A", None))
        self.lbl_mode.setText(QCoreApplication.translate("MainWindow", u"Mode", None))
        self.val_mode.setText(QCoreApplication.translate("MainWindow", u"N/A", None))
        self.lbl_armed.setText(QCoreApplication.translate("MainWindow", u"Armed", None))
        self.val_armed.setText(QCoreApplication.translate("MainWindow", u"N/A", None))
        self.lbl_alt.setText(QCoreApplication.translate("MainWindow", u"Altitude", None))
        self.val_alt.setText(QCoreApplication.translate("MainWindow", u"N/A", None))
        self.grp_gps_coords.setTitle(QCoreApplication.translate("MainWindow", u"GPS Coordinates", None))
        self.lbl_lat.setText(QCoreApplication.translate("MainWindow", u"Latitude", None))
        self.val_lat.setText(QCoreApplication.translate("MainWindow", u"N/A", None))
        self.lbl_lon.setText(QCoreApplication.translate("MainWindow", u"Longitude", None))
        self.val_lon.setText(QCoreApplication.translate("MainWindow", u"N/A", None))
        self.lbl_roll.setText(QCoreApplication.translate("MainWindow", u"Roll", None))
        self.val_roll.setText(QCoreApplication.translate("MainWindow", u"N/A", None))
        self.lbl_pitch.setText(QCoreApplication.translate("MainWindow", u"Pitch", None))
        self.val_pitch.setText(QCoreApplication.translate("MainWindow", u"N/A", None))
        self.lbl_yaw.setText(QCoreApplication.translate("MainWindow", u"Yaw", None))
        self.val_yaw.setText(QCoreApplication.translate("MainWindow", u"N/A", None))
        self.lbl_gps_fix.setText(QCoreApplication.translate("MainWindow", u"GPS Fix", None))
        self.val_gps_fix.setText(QCoreApplication.translate("MainWindow", u"N/A", None))
        self.lbl_sats.setText(QCoreApplication.translate("MainWindow", u"Satellites", None))
        self.val_sats.setText(QCoreApplication.translate("MainWindow", u"N/A", None))
        self.lbl_spd.setText(QCoreApplication.translate("MainWindow", u"Ground Speed", None))
        self.val_spd.setText(QCoreApplication.translate("MainWindow", u"N/A", None))
        self.grp_manual_control.setTitle(QCoreApplication.translate("MainWindow", u"Manual Control", None))
        self.lbl_throttle.setText(QCoreApplication.translate("MainWindow", u"Throttle", None))
        self.val_throttle.setText(QCoreApplication.translate("MainWindow", u"1000", None))
        self.lbl_roll_input.setText(QCoreApplication.translate("MainWindow", u"Roll", None))
        self.val_roll_input.setText(QCoreApplication.translate("MainWindow", u"1500", None))
        self.lbl_pitch_input.setText(QCoreApplication.translate("MainWindow", u"Pitch", None))
        self.val_pitch_input.setText(QCoreApplication.translate("MainWindow", u"1500", None))
        self.lbl_yaw_input.setText(QCoreApplication.translate("MainWindow", u"Yaw", None))
        self.val_yaw_input.setText(QCoreApplication.translate("MainWindow", u"1500", None))
        self.lbl_aux1.setText(QCoreApplication.translate("MainWindow", u"AUX1", None))
        self.val_aux1.setText(QCoreApplication.translate("MainWindow", u"1000", None))
        self.lbl_aux2.setText(QCoreApplication.translate("MainWindow", u"AUX2", None))
        self.val_aux2.setText(QCoreApplication.translate("MainWindow", u"1000", None))
        self.btn_send_manual.setText(QCoreApplication.translate("MainWindow", u"Send Manual", None))
        self.btn_arm.setText(QCoreApplication.translate("MainWindow", u"ARM", None))
        self.btn_disarm.setText(QCoreApplication.translate("MainWindow", u"DISARM", None))
        self.btn_hold.setText(QCoreApplication.translate("MainWindow", u"HOLD", None))
        self.btn_rth.setText(QCoreApplication.translate("MainWindow", u"RTH", None))
        self.btn_start_mission_main.setText(QCoreApplication.translate("MainWindow", u"Start Mission", None))
        self.grp_motors.setTitle(QCoreApplication.translate("MainWindow", u"Motors", None))
        self.lbl_motor1.setText(QCoreApplication.translate("MainWindow", u"Motor 1", None))
        self.val_motor1.setText(QCoreApplication.translate("MainWindow", u"1000", None))
        self.bar_motor1.setStyleSheet(QCoreApplication.translate("MainWindow", u"QProgressBar::chunk { background-color: #4CAF50; }", None))
        self.lbl_motor2.setText(QCoreApplication.translate("MainWindow", u"Motor 2", None))
        self.val_motor2.setText(QCoreApplication.translate("MainWindow", u"1000", None))
        self.bar_motor2.setStyleSheet(QCoreApplication.translate("MainWindow", u"QProgressBar::chunk { background-color: #4CAF50; }", None))
        self.lbl_motor3.setText(QCoreApplication.translate("MainWindow", u"Motor 3", None))
        self.val_motor3.setText(QCoreApplication.translate("MainWindow", u"1000", None))
        self.bar_motor3.setStyleSheet(QCoreApplication.translate("MainWindow", u"QProgressBar::chunk { background-color: #4CAF50; }", None))
        self.lbl_motor4.setText(QCoreApplication.translate("MainWindow", u"Motor 4", None))
        self.val_motor4.setText(QCoreApplication.translate("MainWindow", u"1000", None))
        self.bar_motor4.setStyleSheet(QCoreApplication.translate("MainWindow", u"QProgressBar::chunk { background-color: #4CAF50; }", None))
        self.grp_waypoint_input.setTitle(QCoreApplication.translate("MainWindow", u"Waypoint Input", None))
        self.lbl_wp_lat.setText(QCoreApplication.translate("MainWindow", u"Latitude", None))
        self.input_lat.setPlaceholderText(QCoreApplication.translate("MainWindow", u"21.0", None))
        self.lbl_wp_lon.setText(QCoreApplication.translate("MainWindow", u"Longitude", None))
        self.input_lon.setPlaceholderText(QCoreApplication.translate("MainWindow", u"105.0", None))
        self.lbl_wp_alt.setText(QCoreApplication.translate("MainWindow", u"Altitude (m)", None))
        self.input_alt.setText(QCoreApplication.translate("MainWindow", u"10", None))
        self.btn_add_waypoint.setText(QCoreApplication.translate("MainWindow", u"Add Waypoint", None))
        self.grp_waypoint_list.setTitle(QCoreApplication.translate("MainWindow", u"Waypoint List", None))
        ___qtablewidgetitem = self.table_waypoints.horizontalHeaderItem(0)
        ___qtablewidgetitem.setText(QCoreApplication.translate("MainWindow", u"#", None))
        ___qtablewidgetitem1 = self.table_waypoints.horizontalHeaderItem(1)
        ___qtablewidgetitem1.setText(QCoreApplication.translate("MainWindow", u"Lat", None))
        ___qtablewidgetitem2 = self.table_waypoints.horizontalHeaderItem(2)
        ___qtablewidgetitem2.setText(QCoreApplication.translate("MainWindow", u"Lon", None))
        ___qtablewidgetitem3 = self.table_waypoints.horizontalHeaderItem(3)
        ___qtablewidgetitem3.setText(QCoreApplication.translate("MainWindow", u"Alt (m)", None))
        self.btn_remove.setText(QCoreApplication.translate("MainWindow", u"Remove Selected", None))
        self.btn_clear.setText(QCoreApplication.translate("MainWindow", u"Clear All", None))
        self.btn_upload.setText(QCoreApplication.translate("MainWindow", u"Upload To FC", None))
        self.btn_start_mission_tab.setText(QCoreApplication.translate("MainWindow", u"Start Mission", None))
        self.btn_stop_mission.setText(QCoreApplication.translate("MainWindow", u"Stop Mission", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_mission), QCoreApplication.translate("MainWindow", u"Mission", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_config), QCoreApplication.translate("MainWindow", u"Config", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_log), QCoreApplication.translate("MainWindow", u"Log", None))
    # retranslateUi

