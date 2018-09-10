import cv2 as cv
import numpy as np
import random
import math

    

class Entity:

    def __init__(self, world, size_x, size_y, radius, speed):
        self.world = world
        self.radius = radius
        self.velocity = (0,0)
        self.speed = speed
        self.world_map = np.zeros(world.shape)
        self.pos = (153, 170)
        self.angle = 2
        self.range = 400

    # "Private" methods

    """
    Returns first point (if any) that is a wall (color != black)
    """
    def __findFirst(self, points):
        for point in points:
            if world[point] > 0:
                return point
        return None

    """
    Uses brezenham line algorithm (str8 outta wikipendium) to determine 
    which pixels to test when observing
    Takes two points in 2D space, returns array of points
    """
    def rasterizeLine(self, xy0, xy1):
        points = []
        delta_x = xy1[0] - xy0[0]
        delta_y = xy1[1] - xy0[1]
        error = 0
        slope = abs(delta_y/delta_x)
        y = xy0[1]

        # Lord help me..
        if (xy1[0] - xy0[0]) > 0:
            for x in range(xy0[0], xy1[0]):
                points.append((x, y))
                error += slope
                if error >= 0.5:
                    y+=1
                    error-=1
        else:
            for x in range(xy0[0], xy1[0], -1):
                points.append((x, y))
                error += slope
                if error >= 0.5:
                    y+=1
                    error-=1

        return points

    def checkCollision(self):
        if self.world[self.pos]:
            self.angle += random.uniform(-0.3, -0.9)

    # Returns coordinates of the next point we'll look
    def getLookaheadPoint(self):
        return (min(world.shape[0]-1, int(self.pos[0] + np.cos(self.angle)*self.range)),\
                min(world.shape[1]-1, int(self.pos[1] + np.sin(self.angle)*self.range)))

    def getLookaheadRay(self):
        return self.rasterizeLine(self.pos, self.getLookaheadPoint())

    def setVelocity(self, x, y):
        self.velocity = (x, y)
    # Interface methods

    def getPos(self):
        return self.pos

    def getWorldMap(self):
        return self.world_map

    def getLookaheadRay(self):
        return self.rasterizeLine(self.pos, self.getLookaheadPoint())

    def observe(self):
        print("Looking at: ", self.getLookaheadPoint())
        print("Map color at point is: ", self.world[self.getLookaheadPoint()])
        print("Line points are: ", self.getLookaheadRay())

        wall = self.__findFirst(self.getLookaheadRay())
        if wall:
            #  print("Found a wall yo")
            self.world_map[wall] = 255

    def move(self):
        self.setVelocity(int(np.cos(self.angle)*self.speed), int(np.sin(self.angle)*self.speed))

        res = (int(self.pos[0] + np.cos(self.angle)*self.speed),\
                int(self.pos[1] + np.sin(self.angle)*self.speed))

        res = (
            min(max(res[0], 0), self.world.shape[0]),
            min(max(res[1], 0), self.world.shape[1]),
        )
        self.pos = res
        self.checkCollision()

        print("Position:", self.pos)


world = cv.imread('map_small.png', cv.IMREAD_GRAYSCALE)
result = np.zeros(world.shape)

robot = Entity(world, None, None, 4, 3)

print(world)

def flipTuple(tup):
    return (tup[1], tup[0])

while True:
    robot.move()
    robot.observe()

    res2 = np.stack((robot.getWorldMap(), )*3, -1)
    cv.circle(res2, flipTuple(robot.getPos()), 5, (255, 0, 0), -1)
    
    cv.imshow('map', world)
    cv.imshow('result', res2)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break
