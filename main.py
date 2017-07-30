"""Pithon - a remote controlled multi-segment rescue robot

Copyright (C) 2017  Adam Vandervorst

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3 of the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

See <http://www.gnu.org/licenses/>."""

import io
import socket
import struct
import logging
import sys
from argparse import ArgumentParser
from time import time, sleep

import numpy as np
import quaternion
import urllib
from PyQt5 import QtCore
from PyQt5 import QtWidgets
from PyQt5 import QtGui
from PyQt5.QtOpenGL import QGLWidget


__author__ = "Adam Vandervorst"
__copyright__ = "Copyright 2017, The Pithon GIP Project"
__credits__ = ["Adam Vandervorst", "waveform80"]
__license__ = "GNU GPLv3"
__version__ = "0.4.3"
__email__ = "adam.vandervorst@gmail.com"
__status__ = "Internal Alpha"

parser = ArgumentParser()
parser.add_argument("--debug", help="Set debug logging level")
parser.add_argument("--nosim", help="Disable simulator")
parser.add_argument("--nostream", help="Disable livestream")
parser.add_argument("--port", help="Server port", default=8008)
args = parser.parse_args()

logging.basicConfig(level=logging.DEBUG if args.debug else logging.INFO,
                    format="%(asctime)s %(levelname)s %(message)s")


class Rpi(object):
    """Syntactic sugar"""
    def __init__(self, conn):
        self.alive = True
        self.conn = conn

    def send(self, msg):
        if self.alive:
            logging.debug("Send: " + msg)
            self.conn.send(str.encode(msg + "\n"))
        else:
            logging.warning("No live connection")

    def recv(self):
        msg = self.conn.recv(512).decode()
        logging.debug("Received: " + msg)
        return msg


class Stream(object):
    """Syntactic sugar"""
    def __init__(self, stream):
        self.stream = stream
        self.i = 0

    def __iter__(self):
        return self

    def __next__(self):
        """Read bytes and accel/gyro data from stream and write to image and tuple"""
        self.i += 1

        try:
            strct = struct.unpack('<Lffffff', self.stream.read(
                struct.calcsize('<Lffffff')))
            image_len = strct[0]
            agdata = strct[1:]
        except (struct.error, ValueError):
            raise StopIteration
        image_stream = io.BytesIO()
        image_stream.write(self.stream.read(image_len))
        image_stream.seek(0)
        im = QtGui.QImage()
        im.loadFromData(image_stream.read())
        return self.i, im, agdata


class Server(object):
    def __init__(self):
        """Accept any connection on a random port"""
        self.server_socket = socket.socket()
        self.server_socket.bind(('0.0.0.0', args.port))
        self.conn = self.streamfile = None

    def connect(self):
        """Wait for a connection and initialize stream file"""
        self.server_socket.listen(0)
        self.conn = self.server_socket.accept()[0]
        self.streamfile = self.conn.makefile('rb')
        return Rpi(self.conn)

    def disconnect(self, sd=True):
        """Safe server shutdown function for button binding"""
        if sd:
            rpi.send("sd")
            rpi.alive = False
            logging.info("Send shutdown signal to robot")
        self.streamfile.close()
        self.server_socket.close()
        logging.debug("Shut down server")


class VideoStream(QtCore.QObject):

    """Abstract class for video stream handling, emitting the image to video player"""
    VideoSignal = QtCore.pyqtSignal(QtGui.QImage, name="VideoStream")
    AGSignal = QtCore.pyqtSignal(tuple, name="AGStream")

    def __init__(self, parent=None):
        super(VideoStream, self).__init__(parent)

    @QtCore.pyqtSlot(name="Video")
    def startVideo(self):
        t = 1

        for i, image, agdata in Stream(server.streamfile):
            self.VideoSignal.emit(image)
            self.AGSignal.emit(agdata)

            if not i % 20:
                control_panel.lcdNumber.display(20 // (time() - t))
                t = time()

        control_panel.lcdNumber.display(0)
        logging.warning("Stopped Videostream")


class VideoPlayer(QtWidgets.QWidget):
    """Video player in the Qt slot, drawing the picture received from the video signal pipeline"""
    def __init__(self, parent=None):
        super(VideoPlayer, self).__init__(parent)
        self.image = QtGui.QImage()
        self.setAttribute(QtCore.Qt.WA_OpaquePaintEvent)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.drawImage(0, 0, self.image)
        self.image = QtGui.QImage()

    @QtCore.pyqtSlot(QtGui.QImage)
    def setImage(self, image):
        if image.isNull():
            logging.warning("Viewer Dropped frame!")  # Never happened in practice

        self.image = image
        if image.size() != self.size():
            self.setFixedSize(image.size())
        self.update()


class UIWidget(object):
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.resize(828, 790)
        self.controlpanel = QtWidgets.QGroupBox(Widget)
        self.controlpanel.setGeometry(QtCore.QRect(9, 9, 801, 761))
        self.controlpanel.setMinimumSize(QtCore.QSize(381, 0))
        self.controlpanel.setObjectName("controlpanel")
        self.gridLayoutWidget = QtWidgets.QWidget(self.controlpanel)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(-1, 10, 801, 751))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.gridLayoutWidget)
        self.horizontalLayout_2.setContentsMargins(11, 11, 11, 11)
        self.horizontalLayout_2.setSpacing(6)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setContentsMargins(11, 11, 11, 11)
        self.verticalLayout_2.setSpacing(6)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.startStream = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.startStream.setObjectName("startStream")
        self.verticalLayout_2.addWidget(self.startStream)
        self.lcdNumber = QtWidgets.QLCDNumber(self.gridLayoutWidget)
        self.lcdNumber.setObjectName("lcdNumber")
        self.verticalLayout_2.addWidget(self.lcdNumber)
        self.safeValue = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.safeValue.setObjectName("safeValue")
        self.verticalLayout_2.addWidget(self.safeValue)
        spacerItem = QtWidgets.QSpacerItem(20, 20, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem)
        self.resetToZero = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.resetToZero.setObjectName("resetToZero")
        self.verticalLayout_2.addWidget(self.resetToZero)
        spacerItem1 = QtWidgets.QSpacerItem(20, 300, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.verticalLayout_2.addItem(spacerItem1)
        self.horizontalLayout_2.addLayout(self.verticalLayout_2)
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setContentsMargins(11, 11, 11, 11)
        self.gridLayout.setSpacing(6)
        self.gridLayout.setObjectName("gridLayout")
        self.powerOff = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.powerOff.setObjectName("powerOff")
        self.gridLayout.addWidget(self.powerOff, 0, 1, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 2, 1, 1, 1)
        self.formLayout = QtWidgets.QFormLayout()
        self.formLayout.setContentsMargins(11, 11, 11, 11)
        self.formLayout.setSpacing(6)
        self.formLayout.setObjectName("formLayout")
        self.label_12 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_12.setObjectName("label_12")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.label_12)
        self.label_10 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_10.setObjectName("label_10")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_10)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setContentsMargins(11, 11, 11, 11)
        self.horizontalLayout_4.setSpacing(6)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.SN1H = QtWidgets.QSlider(self.gridLayoutWidget)
        self.SN1H.setMaximumSize(QtCore.QSize(100, 16777215))
        self.SN1H.setMinimum(-100)
        self.SN1H.setMaximum(100)
        self.SN1H.setOrientation(QtCore.Qt.Horizontal)
        self.SN1H.setInvertedAppearance(True)
        self.SN1H.setObjectName("SN1H")
        self.horizontalLayout_4.addWidget(self.SN1H)
        self.SN1V = QtWidgets.QSlider(self.gridLayoutWidget)
        self.SN1V.setMaximum(100)
        self.SN1V.setOrientation(QtCore.Qt.Vertical)
        self.SN1V.setObjectName("SN1V")
        self.horizontalLayout_4.addWidget(self.SN1V)
        self.formLayout.setLayout(1, QtWidgets.QFormLayout.FieldRole, self.horizontalLayout_4)
        self.label_11 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_11.setObjectName("label_11")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_11)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setContentsMargins(11, 11, 11, 11)
        self.horizontalLayout_3.setSpacing(6)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.SN2H = QtWidgets.QSlider(self.gridLayoutWidget)
        self.SN2H.setMaximumSize(QtCore.QSize(100, 16777215))
        self.SN2H.setMinimum(-100)
        self.SN2H.setMaximum(100)
        self.SN2H.setOrientation(QtCore.Qt.Horizontal)
        self.SN2H.setInvertedAppearance(True)
        self.SN2H.setObjectName("SN2H")
        self.horizontalLayout_3.addWidget(self.SN2H)
        self.SN2V = QtWidgets.QSlider(self.gridLayoutWidget)
        self.SN2V.setMaximum(100)
        self.SN2V.setOrientation(QtCore.Qt.Vertical)
        self.SN2V.setObjectName("SN2V")
        self.horizontalLayout_3.addWidget(self.SN2V)
        self.formLayout.setLayout(2, QtWidgets.QFormLayout.FieldRole, self.horizontalLayout_3)
        self.label_9 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_9.setObjectName("label_9")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.label_9)
        self.label = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label.setObjectName("label")
        self.formLayout.setWidget(4, QtWidgets.QFormLayout.LabelRole, self.label)
        self.Forwards = QtWidgets.QSlider(self.gridLayoutWidget)
        self.Forwards.setAutoFillBackground(False)
        self.Forwards.setMinimum(-100)
        self.Forwards.setMaximum(100)
        self.Forwards.setSingleStep(1)
        self.Forwards.setSliderPosition(0)
        self.Forwards.setOrientation(QtCore.Qt.Horizontal)
        self.Forwards.setInvertedAppearance(True)
        self.Forwards.setObjectName("Forwards")
        self.formLayout.setWidget(4, QtWidgets.QFormLayout.FieldRole, self.Forwards)
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_10.setContentsMargins(11, 11, 11, 11)
        self.horizontalLayout_10.setSpacing(6)
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.goLeft = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.goLeft.setObjectName("goLeft")
        self.horizontalLayout_10.addWidget(self.goLeft)
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_10.addItem(spacerItem2)
        self.goRight = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.goRight.setObjectName("goRight")
        self.horizontalLayout_10.addWidget(self.goRight)
        self.formLayout.setLayout(5, QtWidgets.QFormLayout.FieldRole, self.horizontalLayout_10)
        self.turnLabel = QtWidgets.QLabel(self.gridLayoutWidget)
        self.turnLabel.setObjectName("turnLabel")
        self.formLayout.setWidget(6, QtWidgets.QFormLayout.LabelRole, self.turnLabel)
        self.Turn = QtWidgets.QSlider(self.gridLayoutWidget)
        self.Turn.setMinimum(-100)
        self.Turn.setMaximum(100)
        self.Turn.setTracking(True)
        self.Turn.setOrientation(QtCore.Qt.Horizontal)
        self.Turn.setInvertedAppearance(True)
        self.Turn.setInvertedControls(False)
        self.Turn.setObjectName("Turn")
        self.formLayout.setWidget(6, QtWidgets.QFormLayout.FieldRole, self.Turn)
        self.label_4 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_4.setObjectName("label_4")
        self.formLayout.setWidget(7, QtWidgets.QFormLayout.FieldRole, self.label_4)
        self.label_7 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_7.setObjectName("label_7")
        self.formLayout.setWidget(9, QtWidgets.QFormLayout.LabelRole, self.label_7)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setContentsMargins(11, 11, 11, 11)
        self.horizontalLayout_5.setSpacing(6)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.SJ1H = QtWidgets.QSlider(self.gridLayoutWidget)
        self.SJ1H.setMaximumSize(QtCore.QSize(100, 16777215))
        self.SJ1H.setMinimum(-100)
        self.SJ1H.setMaximum(100)
        self.SJ1H.setOrientation(QtCore.Qt.Horizontal)
        self.SJ1H.setInvertedAppearance(True)
        self.SJ1H.setObjectName("SJ1H")
        self.horizontalLayout_5.addWidget(self.SJ1H)
        self.SJ1V = QtWidgets.QSlider(self.gridLayoutWidget)
        self.SJ1V.setMaximum(100)
        self.SJ1V.setOrientation(QtCore.Qt.Vertical)
        self.SJ1V.setObjectName("SJ1V")
        self.horizontalLayout_5.addWidget(self.SJ1V)
        self.formLayout.setLayout(9, QtWidgets.QFormLayout.FieldRole, self.horizontalLayout_5)
        self.label_8 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_8.setObjectName("label_8")
        self.formLayout.setWidget(11, QtWidgets.QFormLayout.LabelRole, self.label_8)
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setContentsMargins(11, 11, 11, 11)
        self.horizontalLayout_7.setSpacing(6)
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.SJ2H = QtWidgets.QSlider(self.gridLayoutWidget)
        self.SJ2H.setMaximumSize(QtCore.QSize(100, 16777215))
        self.SJ2H.setMinimum(-100)
        self.SJ2H.setMaximum(100)
        self.SJ2H.setOrientation(QtCore.Qt.Horizontal)
        self.SJ2H.setInvertedAppearance(True)
        self.SJ2H.setObjectName("SJ2H")
        self.horizontalLayout_7.addWidget(self.SJ2H)
        self.SJ2V = QtWidgets.QSlider(self.gridLayoutWidget)
        self.SJ2V.setMaximum(100)
        self.SJ2V.setOrientation(QtCore.Qt.Vertical)
        self.SJ2V.setObjectName("SJ2V")
        self.horizontalLayout_7.addWidget(self.SJ2V)
        self.formLayout.setLayout(11, QtWidgets.QFormLayout.FieldRole, self.horizontalLayout_7)
        self.label_13 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_13.setText("")
        self.label_13.setObjectName("label_13")
        self.formLayout.setWidget(12, QtWidgets.QFormLayout.LabelRole, self.label_13)
        self.label_14 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_14.setObjectName("label_14")
        self.formLayout.setWidget(14, QtWidgets.QFormLayout.LabelRole, self.label_14)
        self.MS1F = QtWidgets.QSlider(self.gridLayoutWidget)
        self.MS1F.setAutoFillBackground(False)
        self.MS1F.setMinimum(-100)
        self.MS1F.setMaximum(100)
        self.MS1F.setSingleStep(1)
        self.MS1F.setSliderPosition(0)
        self.MS1F.setOrientation(QtCore.Qt.Horizontal)
        self.MS1F.setInvertedAppearance(True)
        self.MS1F.setObjectName("MS1F")
        self.formLayout.setWidget(14, QtWidgets.QFormLayout.FieldRole, self.MS1F)
        self.label_5 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_5.setObjectName("label_5")
        self.formLayout.setWidget(15, QtWidgets.QFormLayout.LabelRole, self.label_5)
        self.MS1S = QtWidgets.QSlider(self.gridLayoutWidget)
        self.MS1S.setMinimum(-100)
        self.MS1S.setMaximum(100)
        self.MS1S.setTracking(True)
        self.MS1S.setOrientation(QtCore.Qt.Horizontal)
        self.MS1S.setInvertedAppearance(True)
        self.MS1S.setInvertedControls(False)
        self.MS1S.setObjectName("MS1S")
        self.formLayout.setWidget(15, QtWidgets.QFormLayout.FieldRole, self.MS1S)
        self.label_3 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.formLayout.setWidget(16, QtWidgets.QFormLayout.LabelRole, self.label_3)
        self.MS2F = QtWidgets.QSlider(self.gridLayoutWidget)
        self.MS2F.setAutoFillBackground(False)
        self.MS2F.setMinimum(-100)
        self.MS2F.setMaximum(100)
        self.MS2F.setSingleStep(1)
        self.MS2F.setSliderPosition(0)
        self.MS2F.setOrientation(QtCore.Qt.Horizontal)
        self.MS2F.setInvertedAppearance(True)
        self.MS2F.setObjectName("MS2F")
        self.formLayout.setWidget(16, QtWidgets.QFormLayout.FieldRole, self.MS2F)
        self.label_15 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_15.setObjectName("label_15")
        self.formLayout.setWidget(17, QtWidgets.QFormLayout.LabelRole, self.label_15)
        self.MS2S = QtWidgets.QSlider(self.gridLayoutWidget)
        self.MS2S.setMinimum(-100)
        self.MS2S.setMaximum(100)
        self.MS2S.setTracking(True)
        self.MS2S.setOrientation(QtCore.Qt.Horizontal)
        self.MS2S.setInvertedAppearance(True)
        self.MS2S.setInvertedControls(False)
        self.MS2S.setObjectName("MS2S")
        self.formLayout.setWidget(17, QtWidgets.QFormLayout.FieldRole, self.MS2S)
        self.label_6 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_6.setObjectName("label_6")
        self.formLayout.setWidget(18, QtWidgets.QFormLayout.LabelRole, self.label_6)
        self.MS3F = QtWidgets.QSlider(self.gridLayoutWidget)
        self.MS3F.setAutoFillBackground(False)
        self.MS3F.setMinimum(-100)
        self.MS3F.setMaximum(100)
        self.MS3F.setSingleStep(1)
        self.MS3F.setSliderPosition(0)
        self.MS3F.setOrientation(QtCore.Qt.Horizontal)
        self.MS3F.setInvertedAppearance(True)
        self.MS3F.setObjectName("MS3F")
        self.formLayout.setWidget(18, QtWidgets.QFormLayout.FieldRole, self.MS3F)
        self.label_16 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_16.setObjectName("label_16")
        self.formLayout.setWidget(19, QtWidgets.QFormLayout.LabelRole, self.label_16)
        self.MS3S = QtWidgets.QSlider(self.gridLayoutWidget)
        self.MS3S.setMinimum(-100)
        self.MS3S.setMaximum(100)
        self.MS3S.setTracking(True)
        self.MS3S.setOrientation(QtCore.Qt.Horizontal)
        self.MS3S.setInvertedAppearance(True)
        self.MS3S.setInvertedControls(False)
        self.MS3S.setObjectName("MS3S")
        self.formLayout.setWidget(19, QtWidgets.QFormLayout.FieldRole, self.MS3S)
        self.label_17 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_17.setObjectName("label_17")
        self.formLayout.setWidget(12, QtWidgets.QFormLayout.FieldRole, self.label_17)
        self.gridLayout.addLayout(self.formLayout, 4, 1, 1, 1)
        self.printStatus = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.printStatus.setObjectName("printStatus")
        self.gridLayout.addWidget(self.printStatus, 1, 1, 1, 1)
        self.horizontalLayout_2.addLayout(self.gridLayout)

        self.retranslateUi(Widget)
        QtCore.QMetaObject.connectSlotsByName(Widget)

    def retranslateUi(self, Widget):
        _translate = QtCore.QCoreApplication.translate
        Widget.setWindowTitle(_translate("Widget", "Widget"))
        self.controlpanel.setTitle(_translate("Widget", "Control panel"))
        self.startStream.setText(_translate("Widget", "Start stream"))
        self.safeValue.setText(_translate("Widget", "Safe value"))
        self.resetToZero.setText(_translate("Widget", "Reset to 0"))
        self.powerOff.setText(_translate("Widget", "Power off"))
        self.label_2.setText(_translate("Widget", "Handle with care"))
        self.label_12.setText(_translate("Widget", "Head"))
        self.label_10.setText(_translate("Widget", "Neck 1"))
        self.label_11.setText(_translate("Widget", "Neck 2"))
        self.label_9.setText(_translate("Widget", "Body"))
        self.label.setText(_translate("Widget", "Forwards"))
        self.goLeft.setText(_translate("Widget", "Left"))
        self.goRight.setText(_translate("Widget", "Right"))
        self.turnLabel.setText(_translate("Widget", "Turn"))
        self.label_4.setText(_translate("Widget", "Joints"))
        self.label_7.setText(_translate("Widget", "Joint 1"))
        self.label_8.setText(_translate("Widget", "Joint 2"))
        self.label_14.setText(_translate("Widget", "Forwards 1"))
        self.label_5.setText(_translate("Widget", "Sideways 1"))
        self.label_3.setText(_translate("Widget", "Forwards 2"))
        self.label_15.setText(_translate("Widget", "Sideways 2"))
        self.label_6.setText(_translate("Widget", "Forwards 3"))
        self.label_16.setText(_translate("Widget", "Sideways 3"))
        self.label_17.setText(_translate("Widget", "Segments"))
        self.printStatus.setText(_translate("Widget", "Status"))


class ControlPanel(QtWidgets.QGroupBox, UIWidget):
    """Connects the UI inputs to their functions"""
    def __init__(self):
        QtWidgets.QGroupBox.__init__(self)
        self.setupUi(self)

        self.sliders = {s: c for s, c in self.__dict__.items() if
                        isinstance(c, QtWidgets.QSlider)}

        for s, c in self.sliders.items():
            if s not in ["Forwards", "Turn"]:
                c.valueChanged.connect(model.slider_value(c))

        self.Forwards.valueChanged.connect(model.forwards(self.Forwards))
        self.Turn.valueChanged.connect(model.turn(self.Turn))

        self.startStream.pressed.connect(video_stream.startVideo)
        self.powerOff.pressed.connect(server.disconnect)
        # self.printStatus.pressed.connect(rpi.recv)
        # self.toggleMain.toggled.connect(model.toggle_main)
        self.goLeft.setAutoRepeatInterval(20)
        self.goLeft.setAutoRepeat(True)
        self.goLeft.pressed.connect(model.go_left)
        self.goLeft.released.connect(model.reset_to_zero)
        self.goRight.setAutoRepeatInterval(20)
        self.goRight.setAutoRepeat(True)
        self.goRight.pressed.connect(model.go_right)
        self.goRight.released.connect(model.reset_to_zero)
        self.safeValue.pressed.connect(model.safe_value)
        self.resetToZero.pressed.connect(model.reset_to_zero)


class Model(object):
    """State and control of the physical robot"""
    TOGGLE_MAIN = False
    NS2S = 1E-9
    TIMESTAMP = 0.0
    MOMENTUM = 0.0
    NORM = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    ORIENTATION_QUAT = np.quaternion(1, 0, 0, 0)

    # Abbreviation: Type|Segment|Function, value, safe-value, (lower-bound, upper-bound)
    GPIO = {"SN1V": ["Servo|Neck1|Vertical", 10, 10, (-40, 60)],
            "SN1H": ["Servo|Neck1|Horizontal", 0, 0, (-65, 65)],
            "SN2V": ["Servo|Neck2|Vertical", 10, 10, (-40, 60)],
            "SN2H": ["Servo|Neck2|Horizontal", 0, 0, (-65, 65)],
            "MS1F": ["Motor|Segment1|Forwards", 0, 0, (-80, 95)],
            "MS1S": ["Motor|Segment1|Sideways", 0, 0, (-90, 90)],
            "SJ1V": ["Servo|Joint1|Vertical", 0, 0, (-80, 80)],
            "SJ1H": ["Servo|Joint1|Horizontal", 0, 0, (-80, 80)],
            "MS2F": ["Motor|Segment2|Forwards", 0, 0, (-80, 95)],
            "MS2S": ["Motor|Segment2|Sideways", 0, 0, (-90, 90)],
            "SJ2V": ["Servo|Joint2|Vertical", 0, 0, (-80, 80)],
            "SJ2H": ["Servo|Joint2|Horizontal", 0, 0, (-80, 80)],
            "MS3F": ["Motor|Segment3|Forwards", 0, 0, (-80, 95)],
            "MS3S": ["Motor|Segment3|Sideways", 0, 0, (-90, 90)]}

    def process_ag(self, ag):
        ε = 0.01

        ax, ay, az, gx, gy, gz = ag  # - self.NORM

        if self.TIMESTAMP == 0:
            self.NORM = ag
        else:
            Δt = (time() - self.TIMESTAMP) * 1  # NS2S

            ω = np.sqrt(gx**2 + gy**2 + gz**2)

            if ω > ε:
                gx /= ω
                gy /= ω
                gz /= ω

            Θ = ω * Δt

            rotation_quat = np.quaternion(np.cos(Θ / 2),
                                          np.sin(Θ / 2) * gx,
                                          np.sin(Θ / 2) * gy,
                                          np.sin(Θ / 2) * gz)

            self.ORIENTATION_QUAT = self.ORIENTATION_QUAT + rotation_quat
        qa = quaternion.as_rotation_vector(self.ORIENTATION_QUAT)
        simulator.state = np.degrees(np.linalg.norm(qa)), *qa
        simulator.paintGL()
        self.TIMESTAMP = time()

    def process_ag2(self, ag):
        ε = 0.01
        Δt = (time() - self.TIMESTAMP)

        ax, ay, az, gx, gy, gz = ag  # - self.NORM
        qa = quaternion.from_euler_angles(gx*Δt, gy*Δt, gz*Δt)
        rv = quaternion.as_rotation_vector(qa)
        simulator.state = np.degrees(np.linalg.norm(rv)), *rv
        simulator.paintGL()
        self.TIMESTAMP = time()

    def bounds_check(self, s, v):  # could use numpy.clip, would result in extra import
        lower, upper = self.GPIO[s][3]
        # TODO check Gyro integration
        if v < lower:
            return lower
        elif v > upper:
            return upper
        else:
            return v

    def toggle_main(self):
        self.TOGGLE_MAIN = not self.TOGGLE_MAIN

    def clear_f(self, sliders):
        r = sliders.copy()
        for s in sliders:
            s = list(s)
            s[-1] = "F" if s[-1] == "S" else "S"
            s = "".join(s)
            if s not in self.GPIO.keys():
                return
            self.GPIO[s][1] = 0
            control_panel.sliders[s].setValue(0)
            rpi.send(s + ":" + str(0))
        return r

    def slider_value(self, slider):
        def value_changed():
            self.clear_f([slider.objectName()])
            value = self.bounds_check(slider.objectName(), slider.value())
            self.GPIO[slider.objectName()][1] = value
            rpi.send(slider.objectName() + ":" + str(value))
        return value_changed

    def forwards(self, slider):
        def value_changed():
            value = slider.value()
            for s in self.clear_f(["MS1F", "MS2F", "MS3F"]):
                self.GPIO[s][1] = value
                control_panel.sliders[s].setValue(value)
                rpi.send(s + ":" + str(value))
        return value_changed

    def turn(self, slider):
        def value_changed():
            value = slider.value()
            for s in ["SJ1H", "SJ2H"]:
                self.GPIO[s][1] = value
                control_panel.sliders[s].setValue(value)
                rpi.send(s + ":" + str(value))
        return value_changed

    def go_left(self):
        self.MOMENTUM += 0.01

        control_panel.sliders["Forwards"].setValue(0)
        for s in self.clear_f(["MS1S", "MS2S", "MS3S"]):
            value = int((self.MOMENTUM ** 1.6) * self.GPIO[s][3][1])
            self.GPIO[s][1] = value
            control_panel.sliders[s].setValue(value)
            rpi.send(s + ":" + str(value))

    def go_right(self):
        self.MOMENTUM += 0.01

        control_panel.sliders["Forwards"].setValue(0)
        for s in self.clear_f(["MS1S", "MS2S", "MS3S"]):
            value = int((self.MOMENTUM ** 1.6) * self.GPIO[s][3][0])
            self.GPIO[s][1] = value
            control_panel.sliders[s].setValue(value)
            rpi.send(s + ":" + str(value))

    def safe_value(self):
        control_panel.sliders["Turn"].setValue(0)
        for s in filter(lambda x: x.startswith('S'), self.GPIO.keys()):
            self.GPIO[s][1] = self.GPIO[s][2]
            control_panel.sliders[s].setValue(self.GPIO[s][2])
            rpi.send(s + ":" + str(self.GPIO[s][2]))

    def reset_to_zero(self):
        if control_panel.goLeft.isDown() or control_panel.goRight.isDown():
            return
        self.MOMENTUM = 0.0

        control_panel.sliders["Forwards"].setValue(0)
        for s in filter(lambda x: x.startswith('M'), self.GPIO.keys()):
            self.GPIO[s][1] = 0
            control_panel.sliders[s].setValue(self.GPIO[s][2])
            rpi.send(s + ":0")


class Simulator(QGLWidget):
    class Cube(object):
        def __init__(self, position, color):
            self.position = position
            self.color = color

        # Cube information
        num_faces = 6

        vertices = [(-1.0, -0.05, 0.5),
                    (1.0, -0.05, 0.5),
                    (1.0, 0.05, 0.5),
                    (-1.0, 0.05, 0.5),
                    (-1.0, -0.05, -0.5),
                    (1.0, -0.05, -0.5),
                    (1.0, 0.05, -0.5),
                    (-1.0, 0.05, -0.5)]

        normals = [(0.0, 0.0, +1.0),  # front
                   (0.0, 0.0, -1.0),  # back
                   (+1.0, 0.0, 0.0),  # right
                   (-1.0, 0.0, 0.0),  # left
                   (0.0, +1.0, 0.0),  # top
                   (0.0, -1.0, 0.0)]  # bottom

        vertex_indices = [(0, 1, 2, 3),  # front
                          (4, 5, 6, 7),  # back
                          (1, 5, 6, 2),  # right
                          (0, 4, 7, 3),  # left
                          (3, 2, 6, 7),  # top
                          (0, 1, 5, 4)]  # bottom

        def render(self):
            glColor(self.color)

            vertices = self.vertices

            # Draw all 6 faces of the cube
            glBegin(GL_QUADS)

            for face_no in range(self.num_faces):
                glNormal3dv(self.normals[face_no])
                v1, v2, v3, v4 = self.vertex_indices[face_no]
                glVertex(vertices[v1])
                glVertex(vertices[v2])
                glVertex(vertices[v3])
                glVertex(vertices[v4])
            glEnd()

    """WIP 3D representation of the motor and joint states"""
    def __init__(self, parent=None):
        super(Simulator, self).__init__(parent)
        self.setMinimumSize(400, 300)
        self.state = [1, 1, 0, 0]

    @QtCore.pyqtSlot(tuple, name="AGStream")
    def recv_data(self, ag):
        model.process_ag2(np.array(ag))

    def resize(self, width, height):
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, float(width) / height, 0.001, 10.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(0.0, 1.0, -5.0,
                  0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glColor((1., 1., 1.))
        glLineWidth(1)
        glBegin(GL_LINES)

        for x in range(-20, 22, 2):
            glVertex3f(x / 10., -1, -1)
            glVertex3f(x / 10., -1, 1)

        for x in range(-20, 22, 2):
            glVertex3f(x / 10., -1, 1)
            glVertex3f(x / 10., 1, 1)

        for z in range(-10, 12, 2):
            glVertex3f(-2, -1, z / 10.)
            glVertex3f(2, -1, z / 10.)

        for z in range(-10, 12, 2):
            glVertex3f(-2, -1, z / 10.)
            glVertex3f(-2, 1, z / 10.)

        for z in range(-10, 12, 2):
            glVertex3f(2, -1, z / 10.)
            glVertex3f(2, 1, z / 10.)

        for y in range(-10, 12, 2):
            glVertex3f(-2, y / 10., 1)
            glVertex3f(2, y / 10., 1)

        for y in range(-10, 12, 2):
            glVertex3f(-2, y / 10., 1)
            glVertex3f(-2, y / 10., -1)

        for y in range(-10, 12, 2):
            glVertex3f(2, y / 10., 1)
            glVertex3f(2, y / 10., -1)

        glEnd()
        glPushMatrix()
        print(self.state)
        glRotate(*self.state)
        self.cube.render()
        glPopMatrix()
        self.swapBuffers()

    def initializeGL(self):
        self.resize(700, 500)
        glEnable(GL_DEPTH_TEST)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_BLEND)
        glEnable(GL_POLYGON_SMOOTH)
        glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST)
        glEnable(GL_COLOR_MATERIAL)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.3, 0.3, 0.3, 1.0))

        self.cube = self.Cube((0.0, 0.0, 0.0), (.5, .5, .7))


# Initialize model
model = Model()
logging.debug("Initialized model")

# Start server
server = Server()
logging.debug("Initialized server")
logging.info("Awaiting connection")
rpi = server.connect()
logging.debug("Initialized rpi connection")

# Qt app
app = QtWidgets.QApplication(sys.argv)
logging.debug("Created QApplication")

vertical_layout = QtWidgets.QVBoxLayout()

if args.nostream is None:
    thread = QtCore.QThread()
    thread.start()
    video_stream = VideoStream()
    logging.debug("Initialized video stream")
    video_stream.moveToThread(thread)
    video_player = VideoPlayer()
    logging.debug("Initialized video player")
    video_stream.VideoSignal.connect(video_player.setImage)
    vertical_layout.addWidget(video_player)

control_panel = ControlPanel()
logging.debug("Initialized control panel")

if args.nosim is None:
    # thread2 = QtCore.QThread()
    # thread2.start()
    from OpenGL.GL import *
    from OpenGL.GLUT import *
    from OpenGL.GLU import *
    simulator = Simulator()
    # simulator.moveToThread(thread2)
    video_stream.AGSignal.connect(simulator.recv_data)
    logging.debug("Initialized simulator WIP")
    vertical_layout.addWidget(simulator)

horizontal_layout = QtWidgets.QHBoxLayout()
horizontal_layout.addLayout(vertical_layout)
horizontal_layout.addWidget(control_panel)
layout_widget = QtWidgets.QWidget()
layout_widget.setLayout(horizontal_layout)

main_window = QtWidgets.QMainWindow()
logging.debug("Initialized main window")
main_window.setCentralWidget(layout_widget)
main_window.showMaximized()
logging.debug("Showed window")

sys.exit(app.exec_())
