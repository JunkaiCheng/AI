import sys
from MST import Kruskal
import time


# class for each node
class Node:
    def __init__(self, char, x, y):
        self.x = x
        self.y = y
        self.char = char
        self.prev = None
        self.g = 0
        self.h = 0
        self.setValid()

    def gcost(self, other):
        # calculate the new g_cost
        if other.valid:
            return 1
        else:
            return 10000

    def setValid(self):
        # if it is a wall, then invalid
        if self.char in ['%']:
            self.valid = False
        else:
            self.valid = True


class AStar:
    def __init__(self, maze):
        self.maze = maze

    def createPath(self, curr, path):
        # create the final path by tracking back
        while curr.prev:
            path.append(curr)
            tmp = curr
            curr = curr.prev
            tmp.prev = None
        path.append(curr)
        return path[::-1]

    def getNear(self, node, near):
        # get the four near nodes
        if node.x > 0:
            near.append(self.maze[node.x - 1][node.y])
        if node.y > 0:
            near.append(self.maze[node.x][node.y - 1])
        if node.y < len(self.maze[0]) - 1:
            near.append(self.maze[node.x][node.y + 1])
        if node.x < len(self.maze) - 1:
            near.append(self.maze[node.x + 1][node.y])
        return near

    def aStarSearch(self, curr, end):
        # a star function to get the shortest path between two dots
        self.end = end
        # Currently discovered that is not yet evaluated
        openSet = set()
        openSet.add(curr)
        # Already evaluated
        closedSet = set()

        while len(openSet):
            curr = min(openSet, key=lambda i: i.g + i.h)
            openSet.remove(curr)
            closedSet.add(curr)
            if curr == end:
                path = []
                return self.createPath(curr, path)
            near = []
            for node in self.getNear(curr, near):
                if node in openSet:
                    newdis = curr.g + curr.gcost(node)
                    if node.g > newdis:
                        # update the node in openset
                        node.g = newdis
                        node.h = abs(self.end.x - node.x) + abs(self.end.y - node.y)
                        node.prev = curr
                elif node in closedSet:
                    continue
                else:
                    node.prev = curr
                    node.g = curr.g + curr.gcost(node)
                    node.h = abs(self.end.x - node.x) + abs(self.end.y - node.y)
                    openSet.add(node)


def findCost(start, goal, costList):
    # find the smallest cost between two given dots
    for i in range(len(costList)):
        if ((costList[i][0] == start and costList[i][1] == goal) or (
                costList[i][1] == start and costList[i][0] == goal)):
            return costList[i][2]


def shortestPath(vertex, remaining, dotsDis):
    # choose one dot in a list of dots, so that the path between this dot and another given dot is shortest
    # and return the shortest path
    min_cost = 1000000
    for i in range(len(remaining)):
        cost = findCost(vertex, remaining[i], dotsDis)
        if (cost < min_cost):
            min_cost = cost
    return min_cost


def mstDis(waitlist, dotsDis):
    # return the total weights of all the edges in the MST
    graph = []
    for i in range(len(dotsDis)):
        if (dotsDis[i][0] in waitlist and dotsDis[i][1] in waitlist):
            graph.append(dotsDis[i])
    k = Kruskal()
    mst = k.kruskal(waitlist, graph)
    cost = 0
    for i in range(len(mst)):
        cost += mst[i][2]
    return cost


def MSTAStarSearch(start, foodDots, dotsDis, dotToCoor):
    # a star search to find the shortest path through all the dots
    path = [start]
    expanded_nodes = 0
    totalDis = 0
    numOfDots = len(foodDots)
    unvisited = list(foodDots)

    while (len(path) < len(foodDots) - 1):
        expanded_nodes = expanded_nodes + 1
        partial_min_cost = 100000
        partial_min_node = 0
        print("The unvisited is: ", unvisited)
        for vertex in unvisited:
            print("The vertex is: ", vertex)
            remaining = list(unvisited)
            remaining.remove(vertex)
            print("mstDis is : ", mstDis(remaining, dotsDis))
            cost = findCost(path[-1], vertex, dotsDis) + mstDis(remaining, dotsDis) + shortestPath(vertex, remaining,
                                                                                                   dotsDis)
            if (cost < partial_min_cost):
                # update the min_cost
                partial_min_cost = cost
                partial_min_node = vertex
        print("The minimum vertex is: ", partial_min_node)
        unvisited.remove(partial_min_node)
        totalDis += findCost(path[-1], partial_min_node, dotsDis)
        path.append(partial_min_node)
        print("The temporary path is: ", path)
        print("The remaining vertices are: ", unvisited)

    print("The unvisited list is: ", unvisited)
    totalDis += shortestPath(path[-1], unvisited, dotsDis) + findCost(unvisited[0], unvisited[1], dotsDis)
    if findCost(path[-1], unvisited[0], dotsDis) == shortestPath(path[-1], unvisited, dotsDis):
        path.append(unvisited[0])
        path.append(unvisited[1])
    else:
        path.append(unvisited[1])
        path.append(unvisited[0])
    print("The path is: ", path)
    print("The cost is: ", totalDis)
    print("The expanded nodes is: ", totalDis + expanded_nodes)
    return path


def multiDots(maze):
    # use dict to save coord of start and goals
    dotToCoor = {}
    numOfDots = 1

    for i in range(0, len(maze)):
        for j in range(0, len(maze[0])):
            if maze[i][j].char == 'P':
                dotToCoor[0] = (i, j)
            # print(dotToCoor[0])
            # print("satrtdot", numOfDots)
            elif maze[i][j].char == '.':
                dotToCoor[numOfDots] = (i, j)
                # print("dot", numOfDots)
                # print(dotToCoor[numOfDots])
                numOfDots = numOfDots + 1
    # precompute the pairwise distance
    # and save it as (dotA, dotB, distance)
    dotsDis = []
    for a in range(0, len(dotToCoor) - 1):
        for b in range(a + 1, len(dotToCoor)):
            # print("a", a)
            # print("b", b)
            origin = dotToCoor[a]
            dest = dotToCoor[b]
            # print("origin", origin)
            # print("dest", dest)
            temp = AStar(maze)
            dis = 0
            temp.aStarSearch(maze[origin[0]][origin[1]], maze[dest[0]][dest[1]])
            for node in temp.aStarSearch(maze[origin[0]][origin[1]], maze[dest[0]][dest[1]]):
                dis = dis + 1
            # print(node.y, node.x)
            dotsDis.append([a, b, dis - 1])

    print("unsorted", dotsDis)

    dotsDis.sort(key=lambda tup: tup[2])
    # print("sorted", dotsDis)
    foodPoints = [i + 1 for i in range(len(dotToCoor) - 1)]

    path = MSTAStarSearch(0, foodPoints, dotsDis, dotToCoor)
    return path


# Read maze from txt
def readMazeFromFile():
    maze = []
    for i, l in enumerate(sys.stdin):
        row = []
        l = list(l.rstrip("\r\n"))
        for j in range(len(l)):
            row.append(Node(l[j], i, j))
        maze.append(row)
    return maze


# Write maze to file
def writeMazeToFile(maze):
    for line in maze:
        for node in line:
            sys.stdout.write(node.char)
        print


# Show path
def show(maze, path):
    print(path)
    dotToCoor = {}
    numOfDots = 1
    for i in range(0, len(maze)):
        for j in range(0, len(maze[0])):
            if maze[i][j].char == 'P':
                dotToCoor[0] = (i, j)
            elif maze[i][j].char == '.':
                dotToCoor[numOfDots] = (i, j)
                numOfDots = numOfDots + 1
    print(dotToCoor)
    for i in range(len(path)):
        k = path[i]
        coo = dotToCoor[k]
        x = coo[0]
        y = coo[1]
        if (i >= 10):
            p = i - 10
            maze[x][y].char = chr(ord('a') + p)
        else:
            maze[x][y].char = str(i)
        if (i == 0):
            maze[x][y].char = 'P'
    return maze


if __name__ == "__main__":
    start = time.time()
    maze = readMazeFromFile()
    path = multiDots(maze)
    new_maze = show(maze, path)
    writeMazeToFile(new_maze)
    end = time.time()
    print(end - start)
