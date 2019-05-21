# Simple Robot navigation based on aruco markers received from Android 'vision server'
# Robot is controlled through MQTT server
# Created 15/05/2019 by Bartosz Szczygiel

import pygame
import math
import numpy as np
from math import pi
import paho.mqtt.client as mqtt
from simple_pid import PID
import json
import socket
import time
import random

from distance_from_line import pnt2line, line_intersection
from helpers import distance

aruco_vision_server = ("192.168.2.99", 5000)
vision_server_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
DARK_BLUE = (0, 0, 100)
GREEN = (0, 255, 0)
DARK_GREEN = (0, 100, 0)
RED = (255, 0, 0)
DARK_RED = (100, 0, 0)
pygame.init()
pygame.font.init()
myfont = pygame.font.SysFont('Arial', 15)

simulation = True

window_name = "Robot navigation"
frameWidth = 720
frameHeight = 480
size = [frameWidth, frameHeight]
screen = pygame.display.set_mode(size)
pygame.display.set_caption(window_name)
current_target_position = (int(frameWidth / 2), int(frameHeight / 2))

# 'channels' for controlling the robot. range 0-200
robot_aruco_id = 333
ch1 = 100
ch2 = 100
ch3 = 100
ch4 = 100
robot_center_position = [0, 0]
robot_heading = 0
robot_heading_to_target = 0
robot_distance_to_target = 0

testRobotCenterPosition = [300, 300]
testRobotHeading = 0

barriers = False
# aruco markers used as barrier corners
m10 = (50, 50)
m11 = (frameWidth - 10, 10)
m12 = (frameWidth - 10, frameHeight - 10)
m13 = (10, frameHeight - 10)

auto_control = False
navigationEnabled = False
robotDetected = False
targetDetected = True

target_aruco_id = 1
targetHeading = 0  # not used
wp_radius = 20
wps = []
addWP = False
currentWP = 0

pidLR = PID(1, 0, 0, 0)  # PID left right
pidFB = PID(1, 0, 0, 0)  # PID Forward Backward

mouse_position = (0, 0)

mqtt_server_address = "localhost"
# mqtt_server = "test.mosquitto.org"

barrier_lines = [((m10[0], m10[1], 0), (m11[0], m11[1], 0)),
                 ((m11[0], m11[1], 0), (m12[0], m12[1], 0)),
                 ((m12[0], m12[1], 0), (m13[0], m13[1], 0)),
                 ((m13[0], m13[1], 0), (m10[0], m10[1], 0))]

if not simulation:
    try:
        print("connecting to vision server " + str(aruco_vision_server))
        vision_server_client.connect(aruco_vision_server)
        vision_server_client.setblocking(0)
        vision_server_client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        vision_server_client.settimeout(1)
    except:
        print("Unable to connect to vision server" + str(aruco_vision_server))
        exit()


# MQTT
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))  # client.subscribe("tank/in")


def on_message(client, userdata, msg):
    print(str(msg.payload[1]) + " " + str(msg.payload[2]))


mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

try:
    print("connecting to mqtt server")
    mqtt_client.connect(mqtt_server_address, 1883, 60)
except Exception as e:
    print("Unable to connect to MQTT server" + str(mqtt_server_address))


# End MQTT


def mqtt_control_robot(ch1, ch2, ch3, ch4):
    global testRobotHeading

    if auto_control:
        if simulation:
            simSpeed = 5
            testRobotHeading += 10 * (ch1 - 100) / (100 * simSpeed)
            testRobotCenterPosition[0] = testRobotCenterPosition[0] + int((ch2 - 100) / 10 * simSpeed * math.sin(robot_heading))
            testRobotCenterPosition[1] = testRobotCenterPosition[1] + int((ch2 - 100) / 10 * simSpeed * math.cos(robot_heading))
            if testRobotCenterPosition[0] < 0:
                testRobotCenterPosition[0] = 0
            if testRobotCenterPosition[1] < 0:
                testRobotCenterPosition[1] = 0

        if ch1 < 0:
            ch1 = 0
        if ch1 > 200:
            ch1 = 200
        if ch2 < 0:
            ch2 = 0
        if ch2 > 200:
            ch2 = 200
        if ch3 < 0:
            ch3 = 0
        if ch3 > 200:
            ch3 = 200
        if ch4 < 0:
            ch4 = 0
        if ch4 > 200:
            ch4 = 200
        control_frame = [0x24, 4, int(ch1), int(ch2), int(ch3), int(ch4)]
        mqtt_client.publish('tank/in', bytearray(control_frame))


# def on_mouse_click(event, x, y, flags, param):
#     global mouse_position
#     global addWP
#     global current_target_position
#     mouse_position = (x, y)
#     if event == cv2.EVENT_LBUTTONDOWN:
#         if addWP:
#             wps.append(mouse_position)
#             addWP = False
#         else:
#             current_target_position = mouse_position


done = False
clock = pygame.time.Clock()

while not done:
    mouse_position = pygame.mouse.get_pos()
    screen.fill(BLACK)
    robotDetected = False

    if addWP:
        pygame.draw.circle(screen, RED, mouse_position, wp_radius, 1)

    for idx, wp in enumerate(wps):  # draw waypoints
        pygame.draw.circle(screen, GREEN, wp, wp_radius, 1)
        textsurface = myfont.render(str(idx), False, WHITE)
        screen.blit(textsurface, wp)

    if len(wps) > 1:
        pygame.draw.lines(screen, DARK_GREEN, False, wps, 1)

    # marker detection and drawing
    # try:
    s = ""
    if not simulation:
        # get new aruco positions
        try:
            vision_server_client.send("g".encode())
            clock.tick(5)
            s = vision_server_client.recv(10000)
        except:
            mqtt_control_robot(100, 100, 100, 100)  # robot stop
            continue
    else:
        clock.tick(5)
        # simulates json file received from vision server
        s = ('{"aruco":[{"ID":333,"center":{"x":%d,"y":%d},"heading":%f,"markerCorners":[{"x":0,"y":0},{"x":0,"y":0},{"x":0,"y":0},{"x":0,"y":0}],"size":50}]}' % (testRobotCenterPosition[0], testRobotCenterPosition[1], testRobotHeading)).encode()
    # print(s.decode())
    markersDict = json.loads(s.decode())
    aruco_markers = markersDict["aruco"]

    for m in aruco_markers:
        x = int(m['center']['x'])
        y = int(m['center']['y'])
        s = int(m['size'])
        h = (m['heading'])
        marker_id = int(m['ID'])
        corners = m['markerCorners']

        # draw marker
        pygame.draw.line(screen, GREEN, ([corners[0]['x'], corners[0]['y']]), ([corners[1]['x'], corners[1]['y']]), 2)
        pygame.draw.line(screen, GREEN, ([corners[1]['x'], corners[1]['y']]), ([corners[2]['x'], corners[2]['y']]), 2)
        pygame.draw.line(screen, GREEN, ([corners[2]['x'], corners[2]['y']]), ([corners[3]['x'], corners[3]['y']]), 2)
        pygame.draw.line(screen, GREEN, ([corners[3]['x'], corners[3]['y']]), ([corners[0]['x'], corners[0]['y']]), 2)
        pygame.draw.line(screen, GREEN, (x, y), (int(x + s / 2 * math.sin(h)), int(y + s / 2 * math.cos(h))), 2)

        if marker_id != robot_aruco_id: # not display marker ID for robot
            textsurface = myfont.render(str(marker_id), False, WHITE)
            screen.blit(textsurface, (x, y))

        if marker_id == robot_aruco_id:
            robotDetected = True
            robot_center_position = (x, y)
            robot_heading = h
            pygame.draw.circle(screen, RED, robot_center_position, int(s / 7), 1)

            if barriers:
                # virtual sensors
                sensor_angle = [-30, 30, 150, -150]
                for r in sensor_angle:
                    x1 = robot_center_position[0] + s * math.sin(robot_heading + math.radians(r))
                    y1 = robot_center_position[1] + s * math.cos(robot_heading + math.radians(r))
                    s1 = (int(x1), int(y1))
                    pygame.draw.line(screen, DARK_BLUE, robot_center_position, s1, 1)

                    for l in barrier_lines:
                        l1 = l[0]
                        l2 = l[1]
                        try:
                            ss1 = line_intersection((robot_center_position, s1), (l1, l2))
                            pygame.draw.circle(screen, DARK_BLUE, ss1, 3, 2)
                        except:
                            pass

            str_position = "Robot Position x=%4.0f  y=%4.0f  h=%4.0f" % (robot_center_position[0], robot_center_position[1], math.degrees(robot_heading))
            textsurface = myfont.render(str_position, False, WHITE)
            screen.blit(textsurface, (0, 20))
            mqtt_client.publish("tank/position", "x=%4.0f;y=%4.0f;h=%4.0f" % (robot_center_position[0], robot_center_position[1], robot_heading))

        if marker_id == target_aruco_id:
            targetDetected = True
            current_target_position = (x, y)
            targetHeading = h
            pygame.draw.circle(screen, WHITE, current_target_position, s / 7, 1)

        if marker_id == 10:
            m10 = (x, y)
            pygame.draw.circle(screen, WHITE, m10, s / 7, 1)

        if marker_id == 11:
            m11 = (x, y)
            pygame.draw.circle(screen, WHITE, m11, s / 7, 1)

        if marker_id == 12:
            m12 = (x, y)
            pygame.draw.circle(screen, WHITE, m12, s / 7, 1)

        if marker_id == 13:
            m13 = (x, y)
            pygame.draw.circle(screen, WHITE, m13, s / 7, 1)

    # except Exception as e:
    #     print(e)
    #     # get new aruco positions
    #     continue

    # robot control and navigation
    if robotDetected and targetDetected and navigationEnabled:
        robot_heading_to_target = math.atan2((current_target_position[0] - robot_center_position[0]), (current_target_position[1] - robot_center_position[1]))  # in radians
        if (robot_heading_to_target - robot_heading) < math.radians(-180):
            robot_heading_to_target += math.radians(360)
        else:
            if (robot_heading_to_target - robot_heading) > math.radians(180):
                robot_heading_to_target -= math.radians(360)

        robot_distance_to_target = distance(robot_center_position, current_target_position)

        pidLR.tunings = (0.3, 0.001, 0.01)  # depends on the robot configuration
        if math.fabs(robot_heading_to_target - robot_heading) > math.radians(5):  # I-term anti windup
            pidLR.Ki = 0
        pidLR.output_limits = (-0.3, 0.3)  # depends on the robot configuration
        pidLR.sample_time = 0.01  # update every 0.01 seconds
        pidLR.setpoint = robot_heading_to_target
        ch1 = pidLR(robot_heading) * 100 + 100  # steering

        pidFB.tunings = (0.01, 0.0001, 0)  # depends on the robot configuration
        if robot_distance_to_target > 5:  # I-term anti windup
            pidFB.Ki = 0
        pidFB.output_limits = (-0.3, 0.3)  # depends on the robot configuration
        pidFB.sample_time = 0.01  # update every 0.01 seconds
        pidFB.setpoint = 0

        if math.fabs(robot_heading_to_target - robot_heading) < math.radians(15):  # don't drive forward until error in heading is less than 15degrees
            ch2 = -pidFB(robot_distance_to_target) * 100 + 100
        else:
            ch2 = 100

        if robot_distance_to_target < wp_radius:  # waypoint reached then stop
            ch1 = 100
            ch2 = 100
            if (currentWP < len(wps)):
                current_target_position = wps[currentWP]
                currentWP += 1
            else:
                currentWP = 0
            # if simulation:
            #     current_target_position = (random.randint(0, frameWidth), random.randint(0, frameHeight))

        if not robotDetected:
            ch1 = 100  # stop robot
            ch2 = 100  # stop robot

        str_position = "Nav h=%4.0f  d=%4.0f" % (math.degrees(robot_heading_to_target), robot_distance_to_target)
        textsurface = myfont.render(str_position, False, WHITE)
        screen.blit(textsurface, (0, 80))
        # # draw heading line
        pygame.draw.line(screen, DARK_GREEN, robot_center_position, (int(robot_center_position[0] + 50 * math.sin(robot_heading_to_target)), int(robot_center_position[1] + 50 * math.cos(robot_heading_to_target))), 1)

    if barriers:
        barrier_lines = [((m10[0], m10[1], 0), (m11[0], m11[1], 0)),
                         ((m11[0], m11[1], 0), (m12[0], m12[1], 0)),
                         ((m12[0], m12[1], 0), (m13[0], m13[1], 0)),
                         ((m13[0], m13[1], 0), (m10[0], m10[1], 0))]

        for l in barrier_lines:
            l1 = l[0]
            l2 = l[1]
            pygame.draw.line(screen, DARK_RED, (l1[0], l1[1]), (l2[0], l2[1]), 2)
            distanceToLine, nearest = pnt2line((robot_center_position[0], robot_center_position[1], 0), l1, l2)
            pygame.draw.circle(screen, RED, (int(nearest[0]), int(nearest[1])), 5, 1)

            if distanceToLine < 50:
                pygame.draw.line(screen, RED, (int(nearest[0]), int(nearest[1])), robot_center_position, 3)
                # ch1 = 100  # stop robot
                if ch2 < 100: ch2 = 100 + 100 - ch2  # fix me
                if ch2 > 100: ch2 = 100 - ch2 - 100  # fix me

    mqtt_control_robot(ch1, ch2, ch3, ch4)

    # --- Display the frame
    if simulation:
        textsurface = myfont.render("simulation", False, WHITE)
        screen.blit(textsurface, (0, frameHeight - 50))

    textsurface = myfont.render("%d %d" % mouse_position, False, WHITE)
    screen.blit(textsurface, mouse_position)

    textsurface = myfont.render("auto %d, nav %d, barriers %d" % (auto_control, navigationEnabled, barriers), False, WHITE)
    screen.blit(textsurface, (0, 100))

    pygame.draw.circle(screen, RED, current_target_position, wp_radius, 1)

    str_position = "Target Position x=%4.0f  y=%4.0f  h=%4.0f" % (current_target_position[0], current_target_position[1], math.degrees(targetHeading))
    textsurface = myfont.render(str_position, False, WHITE)
    screen.blit(textsurface, (0, 40))

    # aaa= pygame.transform.scale(screen, (1280,720))
    # screen.blit(aaa, (0, 0))
    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            mqtt_control_robot(100, 100, 100, 100)  # stop robot
            done = True
        if event.type == pygame.MOUSEBUTTONUP:
            if addWP:
                wps.append(mouse_position)
                addWP = False
            else:
                current_target_position = mouse_position

    # pygame.key.set_repeat(1, 1000)  # does not work ?
    keys = pygame.key.get_pressed()
    if keys[pygame.K_a]:
        auto_control = not auto_control
        time.sleep(0.5)
    if keys[pygame.K_b]:
        barriers = not barriers
        time.sleep(0.5)

    if keys[pygame.K_n]:
        navigationEnabled = not navigationEnabled
        if navigationEnabled and len(wps) > 0:
            currentWP = 0
            current_target_position = wps[currentWP]
        time.sleep(0.5)

    if keys[pygame.K_w]:
        addWP = True
        time.sleep(0.5)


pygame.quit()

#
# if key == ord('a'):
#     auto_control = not auto_control
#
# if key == ord('b'):
#     barriers = not barriers
#
# if key == ord('n'):
#     navigationEnabled = not navigationEnabled
#     if navigationEnabled and len(wps) > 0:
#         currentWP = 0
#         current_target_position = wps[currentWP]
#     # if not navigationEnabled:
#     #     ch1 = 100
#     #     ch2 = 100
# if key == ord('w'):
#     addWP = True
