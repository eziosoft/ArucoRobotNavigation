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

# Simple Robot navigation based on aruco markers received from Android 'vision server'
# Robot is controlled through MQTT server
# Created 15/05/2019 by Bartosz Szczygiel


import json
import random
import socket
import time

import math
import numpy as np
import paho.mqtt.client as mqtt
import pygame
from simple_pid import PID

from distance_from_line import pnt2line, line_intersection
from helpers import distance, order_points

from marker_attitude import getMarkerAttitude

aruco_vision_server = ("192.168.0.23", 5000)

simulation = False  # if true than everything is emulated

BLACK = (0, 0, 0)
YELLOW = (255, 196, 5)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
DARK_BLUE = (0, 50, 150)
GREEN = (0, 255, 0)
DARK_GREEN = (0, 100, 0)
RED = (255, 0, 0)
ORANGE = (100, 50, 0)
pygame.init()
pygame.font.init()
myfont = pygame.font.SysFont('Arial', 15)

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
max_forward_heading = 45  # if the heading to target is grater than this value robot will rotate in place

testRobotCenterPosition = [300, 300]
testRobotHeading = 0

auto_control = False
navigationEnabled = False
robotDetected = False
targetDetected = True

target_aruco_id = 1
targetHeading = 0  # not used
wp_radius = 20
wps = []

barriers = False


def generate_simulation_wps():
    global wps
    wps = []
    max_points = 15
    for i in range(0, max_points):
        wps.append((int(i * frameWidth / max_points), random.randint(10, frameHeight - 10)))


if simulation:
    auto_control = True
    navigationEnabled = True
    # barriers = True
    generate_simulation_wps()

addWP = False
currentWP = 0

pid_left_right = PID(1, 0, 0, 0)  # PID left right
pid_speed = PID(1, 0, 0, 0)  # PID Forward Backward

mouse_position = (0, 0)

mqtt_server_address = "192.168.0.19" # my MQTT server
# mqtt_server = "test.mosquitto.org"


# aruco markers used as barrier corners
m10 = (50, 50)
m11 = (frameWidth - 10, 10)
m12 = (frameWidth - 10, frameHeight - 10)
m13 = (10, frameHeight - 10)


def prepare_barrier_lines():
    # barriers
    global barrier_lines
    barrier_points = np.array([(m10[0], m10[1]), (m12[0], m12[1]), (m11[0], m11[1]), (m13[0], m13[1])])
    barrier_points = order_points(barrier_points)  # make sure that points are in good order to make a rectangle
    barrier_lines = []
    for j in range(0, len(barrier_points) - 1, 1):
        barrier_lines.append((barrier_points[j], barrier_points[j + 1]))
    barrier_lines.append((barrier_points[len(barrier_lines)], barrier_points[0]))  # add last line


prepare_barrier_lines()

if not simulation:
    try:
        print("connecting to vision server " + str(aruco_vision_server))
        vision_server_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        vision_server_client.connect(aruco_vision_server)
        vision_server_client.setblocking(0)
        vision_server_client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        vision_server_client.settimeout(1)
    except:
        print("Unable to connect to vision server" + str(aruco_vision_server))
        exit()


# Connect to MQTT server
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
            testRobotHeading += (ch1 - 100) / 400
            testRobotCenterPosition[0] = testRobotCenterPosition[0] + ((ch2 - 100) / 10 * math.sin(robot_heading))
            testRobotCenterPosition[1] = testRobotCenterPosition[1] + ((ch2 - 100) / 10 * math.cos(robot_heading))
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
        clock.tick(60)
        # simulates json file received from vision server
        s = ('{"aruco":[{"ID":333,"center":{"x":%d,"y":%d},"heading":%f,"markerCorners":[{"x":0,"y":0},{"x":0,"y":0},{"x":0,"y":0},{"x":0,"y":0}],"size":50}]}' % (testRobotCenterPosition[0], testRobotCenterPosition[1], testRobotHeading)).encode()
    # print(s.decode())

    try:
        markersDict = json.loads(s.decode())
    except:
        continue

    aruco_markers = markersDict["aruco"]

    for m in aruco_markers:
        x = int(m['center']['x'])
        y = int(m['center']['y'])
        s = int(m['size'])
        h = (m['heading'])
        marker_id = int(m['ID'])
        corners = m['markerCorners']

        try:
            xm, ym, zm, roll_marker, pitch_marker, yaw_marker = getMarkerAttitude(corners, 10)  # for extra info, openCV needed
        except:
            pass

        # draw marker
        pygame.draw.line(screen, GREEN, ([corners[0]['x'], corners[0]['y']]), ([corners[1]['x'], corners[1]['y']]), 2)
        pygame.draw.line(screen, GREEN, ([corners[1]['x'], corners[1]['y']]), ([corners[2]['x'], corners[2]['y']]), 2)
        pygame.draw.line(screen, GREEN, ([corners[2]['x'], corners[2]['y']]), ([corners[3]['x'], corners[3]['y']]), 2)
        pygame.draw.line(screen, GREEN, ([corners[3]['x'], corners[3]['y']]), ([corners[0]['x'], corners[0]['y']]), 2)
        pygame.draw.line(screen, GREEN, (x, y), (int(x + s / 2 * math.sin(h)), int(y + s / 2 * math.cos(h))), 2)

        if marker_id != robot_aruco_id:  # not display marker ID for robot
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

            str_position = "Robot x=%4.0f y=%4.0f h=%4.0f" % (robot_center_position[0], robot_center_position[1], math.degrees(robot_heading))
            textsurface = myfont.render(str_position, False, WHITE)
            screen.blit(textsurface, (0, 20))
            mqtt_client.publish("tank/position", "x=%4.0f;y=%4.0f;h=%4.0f" % (robot_center_position[0], robot_center_position[1], robot_heading))

        if marker_id == target_aruco_id:
            targetDetected = True
            current_target_position = (x, y)
            targetHeading = h
            pygame.draw.circle(screen, WHITE, current_target_position, s / 7, 1)

        markers_barrier_found = 0
        if marker_id == 10:
            m10 = (x, y)
            pygame.draw.circle(screen, WHITE, m10, int(s / 7), 1)
            markers_barrier_found += 1

        if marker_id == 11:
            m11 = (x, y)
            pygame.draw.circle(screen, WHITE, m11, int(s / 7), 1)
            markers_barrier_found += 1

        if marker_id == 12:
            m12 = (x, y)
            pygame.draw.circle(screen, WHITE, m12, int(s / 7), 1)
            markers_barrier_found += 1

        if marker_id == 13:
            m13 = (x, y)
            pygame.draw.circle(screen, WHITE, m13, int(s / 7), 1)
            markers_barrier_found += 1

        if markers_barrier_found == 4:
            prepare_barrier_lines()

    # robot control and navigation
    if robotDetected and targetDetected and navigationEnabled:
        robot_heading_to_target = math.atan2((current_target_position[0] - robot_center_position[0]), (current_target_position[1] - robot_center_position[1]))  # in radians

        if (robot_heading_to_target - robot_heading) < math.radians(-180):  # something might be wrong here
            robot_heading_to_target += math.radians(360)  # something might be wrong here
        else:
            if (robot_heading_to_target - robot_heading) > math.radians(180):  # something might be wrong here
                robot_heading_to_target -= math.radians(360)  # something might be wrong here

        robot_distance_to_target = distance(robot_center_position, current_target_position)

        pid_left_right.tunings = (0.3, 0.001, 0.01)  # tuning depends on the robot configuration, vision delay, etc
        if math.fabs(robot_heading_to_target - robot_heading) > math.radians(5):  # I-term anti windup
            pid_left_right.Ki = 0
        pid_left_right.output_limits = (-0.3, 0.3)  # tuning depends on the robot configuration, vision delay, etc
        pid_left_right.sample_time = 0.01  # update every 0.01 seconds
        pid_left_right.setpoint = robot_heading_to_target
        ch1 = pid_left_right(robot_heading) * 100 + 100  # steering

        pid_speed.tunings = (0.01, 0.0001, 0)  # depends on the robot configuration
        if robot_distance_to_target > 5:  # I-term anti windup
            pid_speed.Ki = 0
        pid_speed.output_limits = (-0.3, 0.3)  # depends on the robot configuration
        pid_speed.sample_time = 0.01  # update every 0.01 seconds
        pid_speed.setpoint = 0

        if math.fabs(robot_heading_to_target - robot_heading) < math.radians(max_forward_heading):  # don't drive forward until error in heading is less than max_forward_heading
            ch2 = -pid_speed(robot_distance_to_target) * 100 + 100
        else:
            ch2 = 100

        if robot_distance_to_target < wp_radius:  # waypoint reached then stop
            ch1 = 100
            ch2 = 100
            if (currentWP < len(wps)):
                current_target_position = wps[currentWP]
                currentWP += 1
            else:
                if simulation:
                    generate_simulation_wps()
                currentWP = 0

        if not robotDetected:
            ch1 = 100  # stop robot
            ch2 = 100  # stop robot

        str_position = "Nav h=%4.0f  d=%4.0f" % (math.degrees(robot_heading_to_target), robot_distance_to_target)
        textsurface = myfont.render(str_position, False, WHITE)
        screen.blit(textsurface, (0, 60))
        # # draw heading line
        pygame.draw.line(screen, DARK_GREEN, robot_center_position, (int(robot_center_position[0] + 50 * math.sin(robot_heading_to_target)), int(robot_center_position[1] + 50 * math.cos(robot_heading_to_target))), 1)

    if barriers:
        for l in barrier_lines:
            l1 = (l[0][0], l[0][1], 0)
            l2 = (l[1][0], l[1][1], 0)
            pygame.draw.line(screen, ORANGE, (l1[0], l1[1]), (l2[0], l2[1]), 2)
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
    screen.blit(textsurface, (0, 80))

    pygame.draw.circle(screen, YELLOW, current_target_position, wp_radius, 1)

    str_position = "Target x=%4.0f y=%4.0f h=%4.0f" % (current_target_position[0], current_target_position[1], math.degrees(targetHeading))
    textsurface = myfont.render(str_position, False, WHITE)
    screen.blit(textsurface, (0, 40))

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
