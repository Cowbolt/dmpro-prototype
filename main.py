import cv2 as cv
import numpy as np
import random
import math


def find_first(arr):
    for idx, i in enumerate(arr):
        if i > 0:
            return idx
    return None

def sample_at(map, pos, angle):
    range = 300
    x = pos[1]
    y = pos[0]
    width = map.shape[1]
    height = map.shape[0]
    if angle == 0:
        return find_first(map[y][x:min(x + range, width)])
    elif angle == 1:
        return find_first(np.transpose(map)[x][y:min(y + range, height)])
    elif angle == 2:
        #return find_first(map[y][x:max(x - range, 0):-1])
        return find_first(reversed(map[y][max(x - range, 0):x]))
    elif angle == 3:
        # return find_first(np.transpose(map)[x][y:max(y - range, 0):-1])
        return find_first(reversed(np.transpose(map)[x][max(y - range, 0):y]))
    return None

def scan_at(map, result, pos, angle):
    dist = sample_at(map, pos, angle)
    if dist is not None:
        a = float(angle) * (math.pi/2.0)
        wall_pos = (
            int(pos[0] + math.sin(a) * dist),
            int(pos[1] + math.cos(a) * dist)
        )
        if wall_pos[0] >= 0 and wall_pos[0] < result.shape[0] and \
           wall_pos[1] >= 0 and wall_pos[1] < result.shape[1]:
            result[wall_pos[0]][wall_pos[1]] = 255
            return wall_pos

    return None

def scan_random(map, result):
    point_ok = False
    point = None
    angle = None
    while not point_ok:
        point = (random.randrange(0, map.shape[0]), random.randrange(0, map.shape[0]),)
        # angle = random.random() * (2*math.pi)
        angle = random.randrange(0, 4)

        if map[point[0]][point[1]] == 0:
            point_ok = True

    wall = scan_at(map, result, point, angle)

    return point, wall

def xy(yx):
    return (yx[1], yx[0])

def move(point, dir, shape, speed):
    res = point
    if dir == 0:
        res = (point[0], point[1] + speed)
    elif dir == 1:
        res = (point[0] + speed, point[1])
    elif dir == 2:
        res = (point[0], point[1] - speed)
    elif dir == 3:
        res = (point[0] - speed, point[1])

    res = (
        min(max(res[0], 0), shape[0]),
        min(max(res[1], 0), shape[1]),
    )

    return res

def should_turn(val):
    return val is not None and val < 10


map = cv.imread('map.png', cv.IMREAD_GRAYSCALE)
result = np.zeros(map.shape)


point = (153, 170)
dir = 0
speed = 10

while True:
    new_point = move(point, dir, map.shape, speed)
    while should_turn(sample_at(map, new_point, dir)) or random.randrange(0, 1000) < 10:
        dir = random.randrange(0, 4)
        new_point = move(point, dir, map.shape, speed)
    point = new_point

    walls = []
    for idx in range(0, 4):
        w = scan_at(map, result, point, idx)
        if w:
            walls.append(w)
    # point, wall = scan_random(map, result)
    res2 = np.stack((result,)*3, -1)

    # if wall is not None:
    for wall in walls:
        cv.line(res2, xy(point), xy(wall), (0, 255, 0), 2)
    cv.circle(res2, xy(point), 5, (255, 0, 0), -1)

    cv.imshow('map', map)
    cv.imshow('result', res2)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

