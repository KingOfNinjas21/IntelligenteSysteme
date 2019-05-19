from queue import PriorityQueue
import math

class State(object):

    '''
    Author: Kevin Pulido
    27-Sept-2017

    Steps:
    1) Generate a list of all possible next Steps toward goal from current position
    2) Store Children in PriorityQueue based on distance to goal, closest first
    3) Select closest child and Repeat until goal reached or no more Children

    Based on the youtube video:
        Let's Learn Python #20 A* Algorithm
        https://www.youtube.com/watch?v=ob4faIum4kQ

    This is an updated version of the youtube video code by Kevin Pulido
    '''

    def __init__(self, value, parent,
                 start=[0.0, 0.0],
                 goal=[0.0, 0.0], obstacles=[]):

        self.children = []
        self.parent = parent
        self.value = value
        self.dist = 0
        self.obstacles = []

        if parent:
            self.start  = parent.start
            self.goal   = parent.goal
            self.path   = parent.path[:]
            self.path.append(value)
            self.obstacles = parent.obstacles
        else:
            self.path   = [value]
            self.start  = start
            self.goal   = goal
            self.obstacles = obstacles

    def GetDistance(self):
        pass

    def CreateChildren(self):
        pass

class State_String(State):
    def __init__(self, value, parent,
                 start=[0.0, 0.0],
                 goal=[0.0, 0.0], obstacles=[]):

        super(State_String, self).__init__(value, parent, start, goal, obstacles)
        self.dist = self.GetDistance()

    def inBound(self, bound):
        xIsOk = (self.value[0] >= self.goal[0]-bound) and (self.value[0] <= self.goal[0]+bound)
        yIsOk = (self.value[1] >= self.goal[1]-bound) and (self.value[1] <= self.goal[1]+bound)
        return xIsOk and yIsOk

    def GetDistance(self):
        bound = 0.3

        # check if the goal is already reached
        if self.inBound(bound):
            return 0

        # calculate the ditance to the goal
        xDelta = abs(self.value[0]-self.goal[0])
        yDelta = abs(self.value[1]-self.goal[1])

        dist = math.sqrt(xDelta*xDelta+yDelta*yDelta)

        return dist

    def calcDistanceToTarget(self, xStart, yStart, xEnd, yEnd):
        distanceToTarget = math.sqrt((xEnd - xStart) * (xEnd - xStart) + (yEnd - yStart) * (yEnd - yStart))
        return distanceToTarget

    def isTooCloseToCube(self, coordinate):
        radiusToStayAway = 1.0
        print("Obstacles: ", self.obstacles)
        for i in range(len(self.obstacles)):
            if self.calcDistanceToTarget(coordinate[0], coordinate[1], self.obstacles[i][0], self.obstacles[i][1]) < radiusToStayAway:
                return True
            print("Calculated distance: ", self.calcDistanceToTarget(coordinate[0], coordinate[1], self.obstacles[i][0], self.obstacles[i][1]))
        return False

    def CreateChildren(self):
        step = 0.5
        # if there are no children, generate them
        if not self.children:
            # create 4 children for left, right, front and back coordinates

            #first child - front
            val = self.value[:]
            val[1] += step
            child = State_String(val, self)
            if not self.isTooCloseToCube(val):
                self.children.append(child)
                print("appended 1: ", val)

            # second child - back
            val = self.value[:]
            val[1] -= step
            child = State_String(val, self)
            if not self.isTooCloseToCube(val):
                self.children.append(child)
                print("appended 2: ", val)

            # third child - left
            val = self.value[:]
            val[0] -= step
            child = State_String(val, self)
            if not self.isTooCloseToCube(val):
                self.children.append(child)
                print("appended 3: ", val)

            # fourth child - right
            val = self.value[:]
            val[0] += step
            child = State_String(val, self)
            if not self.isTooCloseToCube(val):
                self.children.append(child)
                print("appended 4: ", val)

            '''
            for i in range(len(self.goal)-1):
                val = self.value
                val = val[:i] + val[i+1] + val[i] + val[i+2:]
                child = State_String(val, self)
                self.children.append(child)
            '''

class AStar_Solver:
    def __init__(self, start, goal, obstacles):
        self.path          = []
        self.visitedQueue  = []
        self.priorityQueue = PriorityQueue()
        self.start         = start
        self.goal          = goal
        self.obstacles = obstacles

    '''
    def __init__(self, value, parent, start=[0.0, 0.0], goal=[0.0, 0.0], obstacles=[]):
    '''

    def Solve(self):
        startState = State_String(self.start,
                                  0,
                                  self.start,
                                  self.goal,
                                  self.obstacles)

        count = 0
        self.priorityQueue.put((0,count,startState))

        while not self.path and self.priorityQueue.qsize():
            closestChild = self.priorityQueue.get()[2]
            closestChild.CreateChildren()
            self.visitedQueue.append(closestChild.value)

            for child in closestChild.children:
                if child.value not in self.visitedQueue:
                    count +=1
                    if not child.dist:
                        self.path = child.path
                        break
                    self.priorityQueue.put((child.dist,count,child))

        if not self.path:
            print("Goal of %s is not possible!" % self.goal)

        return self.path


if __name__ == "__main__":
    start1 = [0.0, 0.0]
    goal1  = [0.0, 5.5]
    obstacles = [[0.0, 3.0]]
    print("Starting...")

    a = AStar_Solver(start1, goal1, obstacles)
    a.Solve()

    for i in range(len(a.path)):
        print("{}) {}".format(i, a.path[i]))
