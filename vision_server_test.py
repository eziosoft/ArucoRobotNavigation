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
# Simple Robot navigation based on aruco markers received from Android 'vision server'
# Robot is controlled through MQTT server
# Created 07/06/2019 by Bartosz Szczygiel


import json
import socket

import math
import pygame

from marker_attitude import getMarkerAttitude

aruco_vision_server = ("192.168.2.99", 5000)
vision_server_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

simulation = True  # if true than everything is emulated

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
clock = pygame.time.Clock()

pygame.font.init()
myfont = pygame.font.SysFont('Arial', 15)

window_name = "Vision server test"
frameWidth = 720  # should be the same as in vision server
frameHeight = 480  # should be the same as in vision server
size = [frameWidth, frameHeight]
screen = pygame.display.set_mode(size)
pygame.display.set_caption(window_name)
current_target_position = (int(frameWidth / 2), int(frameHeight / 2))

try:
    print("connecting to vision server " + str(aruco_vision_server))
    vision_server_client.connect(aruco_vision_server)
    vision_server_client.setblocking(0)
    vision_server_client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    vision_server_client.settimeout(1)
except:
    print("Unable to connect to vision server" + str(aruco_vision_server))
    exit()

done = False
while not done:
    screen.fill(BLACK)
    try:
        s = ""
        vision_server_client.send("g".encode())  # send 'g' to get response
        clock.tick(5)
        s = vision_server_client.recv(10000)  # there must be a better way to do this
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

    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
pygame.quit()
