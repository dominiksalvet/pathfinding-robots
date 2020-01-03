#!/usr/bin/env python3

#-------------------------------------------------------------------------------
# Copyright 2016-2020 Dominik Salvet
# github.com/dominiksalvet/pathfinding-robots
#-------------------------------------------------------------------------------

from enum import Enum
from time import sleep
from random import randint
from collections import deque
import time

# comments are only in necessary sections, sorry for my English

class Rotation(Enum):
    up = '^'
    right = '>'
    down = 'v'
    left = '<'

    @staticmethod
    def turnLeft(rot, left):
        if (rot == Rotation.up):
            rot = Rotation.left if left == True else Rotation.right
        elif (rot == Rotation.right):
            rot = Rotation.up if left == True else Rotation.down
        elif (rot == Rotation.down):
            rot = Rotation.right if left == True else Rotation.left
        else:
            rot = Rotation.down if left == True else Rotation.up
        return rot

class Vector2(object):
    @staticmethod
    def toRot(vector2):
        if (vector2.x == 0):
            rot = Rotation.down if vector2.y > 0 else Rotation.up
        elif (vector2.y == 0):
            rot = Rotation.right if vector2.x > 0 else Rotation.left
        return rot

    @staticmethod
    def incrementByRot(pos, rot):
        if (rot == Rotation.up):
            finalPos = Vector2(pos.x, pos.y - 1)
        elif (rot == Rotation.right):
            finalPos = Vector2(pos.x + 1, pos.y)
        elif (rot == Rotation.down):
            finalPos = Vector2(pos.x, pos.y + 1)
        else:
            finalPos = Vector2(pos.x - 1, pos.y)
        return finalPos

    def __init__(this, x, y):
        this.x = x
        this.y = y

    def __eq__(this, other):
        if (type(other) != Vector2):
            return False
        else:
            return this.x == other.x and this.y == other.y

    def __str__(this):
        return ("(" + str(this.x) + ", " + str(this.y) + ")")

class World(object):
    groundPattern = '.'
    barrierPattern = '#'
    
    def __init__(this, size, name):
        this.size = size
        this.name = name
        this.running = False
        this.actualRound = 0
        this.playground = [[World.groundPattern] * size.x for _ in range(size.y)]
        this.robots = []
        this.huntedRobot = None

    def start(this, maxRound):

        this.maxRound = this.actualRound + maxRound
        
        this.show("Initial state of this game", False)
        sleep(1 / this.UPS)
        this.running = True

        while (this.running == True):

            this.actualRound += 1
            startTime = time.time()
            this.update()
            this.show("Round number " + str(this.actualRound), False)
            endTime = time.time()
            updatePeriod = (1 / this.UPS) - (endTime - startTime)

            if (updatePeriod > 0):
                sleep(updatePeriod)

            if (this.huntedRobot.caught == True):
                print("Hunted robot was caught in round number " + str(this.actualRound) + ".\n")

            this.running = not (this.actualRound == this.maxRound or this.huntedRobot.caught == True)

    def update(this):
        for robot in this.robots:
            robot.update()
        this.huntedRobot.caught = this.huntedRobot.isCaught()

    def show(this, title, highlightHuntedRobot):
        string = title + "\n"

        if (highlightHuntedRobot == True):
            string = "'x' is hunted robot. His pattern is: '" + this.huntedRobot.rot.value + "'\n"
        
        for y in range(this.size.y):
            for x in range(this.size.x):
                if (highlightHuntedRobot == True and this.huntedRobot.pos == Vector2(x, y)):
                    string = string + "x "
                else:
                    string = string + this.playground[y][x] + " "
            string = string + "\n"
        print(string)

    def showRobotTrack(this, robot):
        string = "'X' is the last position of tracked robot. His pattern is: '" + robot.rot.value + "'\n"
        for y in range(this.size.y):
            for x in range(this.size.x):
                if (Vector2(x, y) in robot.posHistory):
                    string = string + ("X " if Vector2(x, y) == robot.pos else "x ")
                else:
                    string = string + this.playground[y][x] + " "
            string = string + "\n"
        print(string)

    # method for creating robot in the world, notice that the huntedRobot
    # is created as the first, it can be used when iterating array of
    # robots to allow him to take first move
    def createRobot(this, pos, rot):
        if (this.isFreePosition(pos)):
            robot = Robot(this, pos, rot)
            this.robots.append(robot)
            this.capturePosition(robot.pos, robot.rot.value)

            if (this.huntedRobot == None):
                this.huntedRobot = robot
            return True
        else:
            return False
            
    def createBarrier(this, pos):
        if (this.isFreePosition(pos)):
            this.capturePosition(pos, World.barrierPattern)
            return True
        else:
            return False

    # this method is looking for the shortest path between position A and
    # position B using basics of Dijkstra's algorithm, if the path between
    # A and B exists, returns array of vectors to take next move from A to B,
    # if path doesn't exist, returns empty array
    # parameter ignoreRobots allows even ignore robots in the path
    def stepVectorsFromAToB(this, posA, posB, ignoreRobots):
        possToProcess = deque([posB])
        shortestFound = False
        nextStepVectors = []
        discovered = []
        
        while (possToProcess and shortestFound == False):
            
            points = deque(possToProcess)
            possToProcess.clear()
            
            while (points):
                
                pointToProcess = points.popleft()
                if (ignoreRobots == True):
                    nearestFreePositions = this.nearestMovablePositions(pointToProcess)
                else:
                    nearestFreePositions = this.nearestFreePositions(pointToProcess)
                    
                if (pointToProcess in this.nearestMovablePositions(posA)):
                    shortestFound = True
                    nextStepVectors.append(Vector2(pointToProcess.x - posA.x, pointToProcess.y - posA.y))

                for point in nearestFreePositions:
                    if (point not in discovered):
                        discovered.append(point)
                        possToProcess.append(point)

        return nextStepVectors

    # returns array of free positions around input position
    # (up, right, down, left)
    def nearestFreePositions(this, pos):
        points = []
        if (pos.x > 0):
            vector2 = Vector2.incrementByRot(pos, Rotation.left)
            if (this.isFreePosition(vector2)):
                points.append(vector2)
        if (pos.x < this.size.x - 1):
            vector2 = Vector2.incrementByRot(pos, Rotation.right)
            if (this.isFreePosition(vector2)):
                points.append(Vector2(pos.x + 1, pos.y))
        if (pos.y > 0):
            vector2 = Vector2.incrementByRot(pos, Rotation.up)
            if (this.isFreePosition(vector2)):
                points.append(Vector2(pos.x, pos.y - 1))
        if (pos.y < this.size.y - 1):
            vector2 = Vector2.incrementByRot(pos, Rotation.down)
            if (this.isFreePosition(vector2)):
                points.append(Vector2(pos.x, pos.y + 1))
        return points

    # returns array of movable positions around input position,
    # can be used when it is required to ignore robots (up, right, down, left)
    def nearestMovablePositions(this, pos):
        points = []
        if (pos.x > 0):
            if (this.playground[pos.y][pos.x - 1] != World.barrierPattern):
                points.append(Vector2(pos.x - 1, pos.y))
        if (pos.x < this.size.x - 1):
            if (this.playground[pos.y][pos.x + 1] != World.barrierPattern):
                points.append(Vector2(pos.x + 1, pos.y))
        if (pos.y > 0):
            if (this.playground[pos.y - 1][pos.x] != World.barrierPattern):
                points.append(Vector2(pos.x, pos.y - 1))
        if (pos.y < this.size.y - 1):
            if (this.playground[pos.y + 1][pos.x] != World.barrierPattern):
                points.append(Vector2(pos.x, pos.y + 1))
        return points

    # returns true if position is in playground and free, otherwise false
    def isValidFreePosition(this, pos):
        if (pos.x >= 0 and pos.y >= 0 and pos.x < this.size.x and pos.y < this.size.y):
            if (this.isFreePosition(pos) == True):
                return True
        return False

    def isFreePosition(this, pos):
        return this.playground[pos.y][pos.x] == World.groundPattern

    def capturePosition(this, pos, pattern):
        this.playground[pos.y][pos.x] = pattern
        
    def freeUpPosition(this, pos):
        this.capturePosition(pos, World.groundPattern)

class Robot(object):
    def __init__(this, world, pos, rot):
        this.world = world
        this.pos = pos
        this.rot = rot
        this.caught = False
        this.posHistory = [pos]

    def update(this):
        if (this == this.world.huntedRobot):
            this.randomMove()
        else:
            this.targetMove(this.world.huntedRobot.pos)

    def moveForward(this):
        forwardPos = Vector2.incrementByRot(this.pos, this.rot)
        if (this.world.isValidFreePosition(forwardPos) == True):
            this.world.freeUpPosition(this.pos)
            this.pos = forwardPos
            this.world.capturePosition(this.pos, this.rot.value)
            this.posHistory.append(this.pos)
            return True
        else:
            return False

    def turnLeft(this):
        this.rot = Rotation.turnLeft(this.rot, True)
        this.world.capturePosition(this.pos, this.rot.value)

    def turnRight(this):
        this.rot = Rotation.turnLeft(this.rot, False)
        this.world.capturePosition(this.pos, this.rot.value)

    def isCaught(this):
        if (
                this.world.isValidFreePosition(Vector2(this.pos.x + 1, this.pos.y)) or
                this.world.isValidFreePosition(Vector2(this.pos.x, this.pos.y + 1)) or
                this.world.isValidFreePosition(Vector2(this.pos.x - 1, this.pos.y)) or
                this.world.isValidFreePosition(Vector2(this.pos.x, this.pos.y - 1))
            ):
            return False
        else:
            return True

    def randomMove(this):
        randNum = randint(0, (this.world.size.x + this.world.size.y) // 3)
        if (randNum >= 2):
            if (this.moveForward() == False):
                randNum %= 2
        if (randNum == 1):
            this.turnLeft()
        elif (randNum == 0):
            this.turnRight()
            
    def targetMove(this, pos):
        vectorsToMove = this.world.stepVectorsFromAToB(this.pos, pos, False)

        if (not vectorsToMove):
            vectorsToMove = this.world.stepVectorsFromAToB(this.pos, pos, True)
            
        rotsToMove = []
        for vector2 in vectorsToMove:
            rotsToMove.append(Vector2.toRot(vector2))

        if (this.rot in rotsToMove):
            this.moveForward()
        elif (
                this.rot == Rotation.up and Rotation.left in rotsToMove or
                this.rot == Rotation.right and Rotation.up in rotsToMove or
                this.rot == Rotation.down and Rotation.right in rotsToMove or
                this.rot == Rotation.left and Rotation.down in rotsToMove
            ):
            this.turnLeft()
        else:
            this.turnRight()

class Game(object):
    # method for correct reading numbers from user 
    @staticmethod
    def readNumber(message, minValue, maxValue):
        isNumber = False
        number = 0
        
        while (isNumber == False or number > maxValue or number < minValue):
            print(message, end = "")
            try:
                number = int(input())
            except ValueError:
                continue
            isNumber = True
            
        return number

    def __init__(this):
        this.worlds = []
        this.running = False

    def start(this):
        print("*Recommended values are written in '(x)' format.\n")
        this.running = True
        while (this.running == True):
            this.gameMenu()

    def gameMenu(this):
        print("Game menu:\n[1] Create world\n[2] Select world\n[0] Exit\n", end = "")
        choice = Game.readNumber("> ", 0, 2)

        if (choice == 1):
            this.createWorldMenu()
        elif (choice == 2):
            this.selectWorldMenu()
        else:
            this.running = False

    def createWorldMenu(this):
        print("  Create world:")
        
        name = ""
        nameReserved = True
        while (name == "" or nameReserved == True):
            print("  World name = ", end = "")
            name = str(input())
            nameReserved = False
            for world in this.worlds:
                if (name == world.name):
                    nameReserved = True
                    print("This world name is reserved.")
                    
        width = Game.readNumber("  World width (15) = ", 5, 32)
        height = Game.readNumber("  World height (15) = ", 5, 32)
        robotNum = Game.readNumber("  Number of robots (4) = ", 2, (width * height) // (width + height))
        barrierRatio = Game.readNumber("  Barrier percentage ratio (30) = ", 5, 50)
        barrierNum = int((width * height) * (barrierRatio / 100))
        
        world = World(Vector2(width, height), name)
        this.worlds.append(world)

        print("Generating world ...")

        startTime = time.time()
        this.generateWorld(world, barrierNum, robotNum)
        print("World '" + name + "' has been generated in " + str(round(time.time() - startTime, 2)) + "s.\n")

    # generating random world as connected graph, probably exists much faster algorithm
    def generateWorld(this, world, barrierNum, robotNum):
        
        displayedProgress = 0
        remainingPoss = []
        
        for i in range(barrierNum):
            connectedGraph = False
            while (connectedGraph == False):
                connectedGraph = True

                if (len(remainingPoss) == 0):
                    for y in range(world.size.y):
                        for x in range(world.size.x):
                            pos = Vector2(x, y)
                            if (world.isFreePosition(pos)):
                                remainingPoss.append(pos)

                randNum = randint(0, len(remainingPoss) - 1)
                randPos = remainingPoss[randNum]
                remainingPoss.remove(randPos)

                nearestFreePositions = world.nearestMovablePositions(randPos)

                if (world.createBarrier(randPos) == True):
                    if (len(nearestFreePositions) != 0):
                        startPos = nearestFreePositions.pop()
                        if (len(nearestFreePositions) == 3):
                            posB = nearestFreePositions.pop()
                            if (not world.stepVectorsFromAToB(startPos, posB, True)):
                                connectedGraph = False
                            else:
                                startPos = posB
                        if (connectedGraph == True):
                            if(len(nearestFreePositions) == 2):
                                posB = nearestFreePositions.pop()
                                if (not world.stepVectorsFromAToB(startPos, posB, True)):
                                    connectedGraph = False
                                else:
                                    startPos = posB
                        if (connectedGraph == True):
                            if(len(nearestFreePositions) == 1):
                                posB = nearestFreePositions.pop()
                                if (not world.stepVectorsFromAToB(startPos, posB, True)):
                                    connectedGraph = False
                    if (connectedGraph == False):
                        world.freeUpPosition(randPos)
                else:
                    connectedGraph = False

            actualProgress = 0
            while (int((i / barrierNum) * 100) > actualProgress + 10):
                actualProgress += 10
                
            if (displayedProgress != actualProgress):
                print(str(actualProgress) + "%")
            displayedProgress = actualProgress
                    
        for _ in range(robotNum):
            success = False
            while (success == False):
                randPos = Vector2(randint(0, world.size.x - 1), randint(0, world.size.y - 1))
                
                randRot = randint(0, 3)
                if (randRot == 0):
                    randRot = Rotation.up
                elif (randRot == 1):
                    randRot = Rotation.right
                elif (randRot == 2):
                    randRot = Rotation.down
                else:
                    randRot = Rotation.left
                    
                success = world.createRobot(randPos, randRot)

    def selectWorldMenu(this):
        while (True):
            if (not this.worlds):
                print("You have no world created.")
                break
            else:
                print("  Select world:")
                for i in range(len(this.worlds)):
                    print("  [" + str(i + 1) + "] " + this.worlds[i].name)
                print("  [0] Cancel")
                choice = Game.readNumber("  > ", 0, len(this.worlds))

                if (choice == 0):
                    print()
                    break;

                this.worldMenu(this.worlds[choice - 1])

    def worldMenu(this, world):
        while (True):
            print("    Menu for '" + world.name + "' world:\n    [1] Show this world\n    [2] Start this world\n    [3] Robot path viewer\n    [0] Cancel")
            choice = Game.readNumber("    > ", 0, 3)
            if (choice == 1):
                world.show("", True)
            elif (choice == 2):
                this.startWorldMenu(world)
            elif (choice == 3):
                this.robotPathViewer(world)
            else:
                print()
                break

    def startWorldMenu(this, world):
        if (world.huntedRobot.caught == False):
            print("      Start this game with:")
            
            world.UPS = Game.readNumber("      Updates per second (1) = ", 1, 65535)
            maxRound = Game.readNumber("      Maximal round number (30) = ", 5, 65535)

            world.start(maxRound)
        else:
            print("The game has ended in round number " + str(world.actualRound) + ".\n")

    def robotPathViewer(this, world):
        choice = 1
        while (choice != 0):
            for robot in world.robots:
                print("      Robot path viewer:\n      [1] Next robot\n      [0] Cancel")
                choice = Game.readNumber("      > ", 0, 1)

                if (choice == 0):
                    break

                if (robot == world.huntedRobot):
                    print("This is hunted robot.")
                
                world.showRobotTrack(robot)
        print()

game = Game()
game.start()
