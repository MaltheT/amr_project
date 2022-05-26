"""
Path Planning Sample Code with Randamized Rapidly-Exploring Random Trees (RRT)

@author: AtsushiSakai(@Atsushi_twi)

#Tilføj link fra Ubuntu:
"""

import matplotlib.pyplot as plt
import random
import math
import copy
import rospy


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea, expandDis=1.0, goalSampleRate=5, maxIter=500):
        """
        Setting Parameter

        start: Start Position [x,y]
        goal: Goal Position [x,y]
        obstacleList: Obstacle Positions [[x,y,size],...]
        randArea: Random Samping Area [min,max]
        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList


    def planning(self):
        """
        Implementation of RRT algorithm: Determining the path from start to goal 
        Step 1: Create a random sample 
        Step 2: Find the node in the current path with lowest distance to the random sample  
        Step 3: Create a node for the random sample 
        Step 4: Check for collision
        Step 5: Expanding the tree in case of no collision, otherwise repeat step 1-4
        Step 6: Check if goal has been hit, otherwise repeat step 1-5
        """

        self.nodeList = [self.start] #list of nodes (the path itself)

        while True:
            # random sampling of a x-coordinate and y-coordinate within the specified configuration area
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(
                    self.minrand, self.maxrand)] 
            else:
                rnd = [self.end.x, self.end.y]

            # finding the index of the nearest node in the current path from the random sample point 
            nind = self.get_nearest_list_index(self.nodeList, rnd)
            nearestNode = self.nodeList[nind] 

            #calculating the angle between the nearest node and the random sample point  
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x) 

            #creating node for the random sample 
            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta) 
            newNode.y += self.expandDis * math.sin(theta)
            newNode.parent = nind

            #check for colission - in case of a colission, the sample node is not added to the current path 
            if not self.collision_check(newNode, self.obstacleList):
                continue
            
            #expanding the list/tree
            self.nodeList.append(newNode)

            # checking if the target goal has been hit 
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!")
                break
        

        #creating the path: going through the parent of every connected node 
        #and adding the x- and y-coordinates to the path 
        path = [[self.end.x, self.end.y]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent

        path.append([self.start.x, self.start.y])

        #smoothing out the path 
        smooth_path = path_smoothing(path, self.maxIter, self.obstacleList)

        return smooth_path



    def get_nearest_list_index(self, nodeList, rnd):
        """
        Getting the index of the nearest node in the current path from the random sample
        """

        #calculating the distance for each node in the path 
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]

        #determining the index of the node with lowest distance 
        minind = dlist.index(min(dlist))

        return minind


    def collision_check(self, node, obstacleList):
        """
        Checking for collision between node and specified obstacles 
        """

        #calculating the euclidean distance between node and every obstacle 
        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = math.sqrt(dx * dx + dy * dy)

            if d <= size:
                return False  # collision

        return True  # safe




class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def get_path_length(path):
    """
    Calculating length of the entire path 
    """
    le = 0

    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d

    return le


def get_target_point(path, targetL):
    """
    Determining new path point for a target length (determined randomly)
    """

    le = 0
    ti = 0
    lastPairLen = 0


    for i in range(len(path) - 1):
        #going through every point in that path and adding the length
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d

        #stopping when the accumulated length of the path has reached the target length
        if le >= targetL:
            ti = i - 1          #going back one index/point 
            lastPairLen = d    
            break

    #computing ratio between the difference of the target length and the accumulated length and
    partRatio = (le - targetL) / lastPairLen

    #computing new x- and y-coordinates 
    x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
    y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio

    return [x, y, ti]


def line_collision_check(first, second, obstacleList):
    """
    Checking if a line between two points yields a collision with obstacle 
    """
    x1 = first[0]
    y1 = first[1]
    x2 = second[0]
    y2 = second[1]

    try:
        a = y2 - y1
        b = -(x2 - x1)
        c = y2 * (x2 - x1) - x2 * (y2 - y1)
    except ZeroDivisionError:
        return False

    for (ox, oy, size) in obstacleList:
        d = abs(a * ox + b * oy + c) / (math.sqrt(a * a + b * b))
        if d <= (size):
            return False #collision

    return True  # no collision 


def path_smoothing(path, maxIter, obstacleList):
    """
    Smoothing out afther RRT has computed a path 
    """

    le = get_path_length(path)

    for i in range(maxIter):
        # sample two random lengths within the current path 
        pickPoints = [random.uniform(0, le), random.uniform(0, le)]
        pickPoints.sort()

        #getting the target point from both samples 
        first = get_target_point(path, pickPoints[0])
        second = get_target_point(path, pickPoints[1])

        #conditions in which the calculated target points not used:
        #if index of the targets are equal to the first point (no new targets have been calculated)
        if first[2] <= 0 or second[2] <= 0:
            continue
        
        #if the index of the greater sample exceed the the path list 
        if (second[2] + 1) > len(path):
            continue
        
        #if the index of the samples are equal 
        if second[2] == first[2]:
            continue

        # collision check
        if not line_collision_check(first, second, obstacleList):
            continue

        # creating a new path 
        newPath = []
        newPath.extend(path[:first[2] + 1])     #creating room for the first sample in the list 
        newPath.append([first[0], first[1]])    #adding the first sample 
        newPath.append([second[0], second[1]])  #adding the second sample 
        newPath.extend(path[second[2] + 1:])    #creating room for second sample in the list 
        path = newPath
        le = get_path_length(path)

    return path


if __name__ == '__main__':
    try:
        RRT() 
    except rospy.ROSInterruptException:
        print ("error!")
