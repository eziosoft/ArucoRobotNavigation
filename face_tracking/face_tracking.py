
#   This file is part of ArucoRobotNavigation.
#
#      ArucoRobotNavigation is free software: you can redistribute it and/or modify
#      it under the terms of the GNU General Public License as published by
#      the Free Software Foundation, either version 3 of the License, or
#      (at your option) any later version.
#
#      Foobar is distributed in the hope that it will be useful,
#      but WITHOUT ANY WARRANTY; without even the implied warranty of
#      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#      GNU General Public License for more details.
#
#      You should have received a copy of the GNU General Public License
#      along with Foobar.  If not, see <https://www.gnu.org/licenses/>.
#  Copyright (c) 2019.

# Simple Robot face tracking
# Robot is controlled through MQTT server
# Created 15/05/2019 by Bartosz Szczygiel
import cv2
import numpy as np
import paho.mqtt.client as mqtt
from simple_pid import PID

ch1 = 100
ch2 = 100
ch3 = 100
ch4 = 100

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
font = cv2.FONT_HERSHEY_PLAIN

pidFaceLR = PID(1, 0, 0, 0)  # for face tracking
pidFaceUD = PID(1, 0, 0, 0)  # for face tracking


# --- MQTT
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))  # client.subscribe("tank/in")


def on_message(client, userdata, msg):
    print(str(msg.payload[1]) + " " + str(msg.payload[2]))


mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

try:
    # mqtt_client.connect("192.168.0.19", 1883, 60)
    # mqtt_client.connect("test.mosquitto.org", 1883, 60)
    mqtt_client.connect("localhost", 1883, 60)
except Exception as e:
    pass


# End MQTT

def MQTT_controlRobot(ch1, ch2, ch3, ch4):
    if ch1 < 0: ch1 = 0
    if ch1 > 200: ch1 = 200
    if ch2 < 0: ch2 = 0
    if ch2 > 200: ch2 = 200
    if ch3 < 0: ch3 = 0
    if ch3 > 200: ch3 = 200
    if ch4 < 0: ch4 = 0
    if ch4 > 200: ch4 = 200
    controlFrame = [0x24, 4, int(ch1), int(ch2), int(ch3), int(ch4)]
    mqtt_client.publish('tank/in', bytearray(controlFrame))


cap1 = cv2.VideoCapture(0)  # for face tracking
a, frame1 = cap1.read()
frameHeight1, frameWidth1 = frame1.shape[:2]
frame1Center = (int(frameWidth1 / 2), int(frameHeight1 / 2))
centerFace = frame1Center

# for undistort fish eye from my camera
DIM = (640, 480)
K = np.array([[209.85557633834975, 0.0, 313.04089077523355], [0.0, 212.22797927627266, 246.40203242153652], [0.0, 0.0, 1.0]])
D = np.array([[0.46754454755124986], [0.442034208498509], [-0.7896728230762422], [0.3278658795248579]])
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
kernel = np.ones((2, 2), np.float32) / 4

while True:
    faceDetected = False

    # fece detection
    a, img = cap1.read()
    h, w = img.shape[:2]
    frame1 = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    gray = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)

    dst = cv2.filter2D(gray, -1, kernel)
    faces = face_cascade.detectMultiScale(dst, 1.3, 5)

    for (x, y, w, h) in faces:
        cv2.rectangle(frame1, (x, y), (x + w, y + h), (0, 255, 0), 1)
        centerFace = (int(x + w / 2), int(y + h / 2))
        cv2.circle(frame1, centerFace, 5, (0, 255, 0), 1)
        roi_gray = gray[y:y + h, x:x + w]
        roi_color = frame1[y:y + h, x:x + w]
        faceDetected = True

    if faceDetected:
        pidFaceLR.tunings = (0.5, 0.000, 0.001)
        pidFaceLR.output_limits = (-0.5, 0.5)
        pidFaceLR.sample_time = 0.01  # update every 0.01 seconds
        pidFaceLR.setpoint = 0
        ch1 = pidFaceLR((centerFace[0] - frame1Center[0]) / 100) * 100 + 100  # left right

        pidFaceUD.tunings = (0.5, 0.000, 0.001)
        pidFaceUD.output_limits = (-1, 1)
        pidFaceUD.sample_time = 0.01  # update every 0.01 seconds
        pidFaceUD.setpoint = 0
        ch4 = pidFaceUD((centerFace[1] - frame1Center[1]) / 100) * 100 + 100  # up down

        if 110 < w < 300:
            ch2 = 80
        else:
            ch2 = 100
    else:
        ch1 = 100

    cv2.putText(frame1, "%d %d %d %d , face size %d" % (ch1, ch2, ch3, ch4, w), (0, 15), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.circle(frame1, frame1Center, 3, (255, 255, 255), 1)  # draw circle in center
    cv2.imshow('face', frame1)
    # end face detection

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        MQTT_controlRobot(100, 100, 100, 100)
        cv2.destroyAllWindows()
        break
