from collections import Counter, OrderedDict
import copy
import heapq

origin = ['A', 'B', 'C', 'D', 'E']

stop_list = [['A', 'E', 'D', 'C', 'A'],
             ['B', 'E', 'A', 'C', 'D'],
             ['B', 'A', 'B', 'C', 'E'],
             ['D', 'A', 'D', 'B', 'D'],
             ['B', 'E', 'C', 'B', 'D']]

cost_list = [['A', 'B', 1064], ['A', 'C', 673], ['A', 'D', 1401], ['A', 'E', 277], ['B', 'C', 958],
             ['B', 'D', 1934], ['B', 'E', 337], ['C', 'D', 1001], ['C', 'E', 399], ['D', 'E', 387],
             ['A', 'A', 0], ['B', 'B', 0], ['C', 'C', 0], ['D', 'D', 0], ['E', 'E', 0]]

def findDistance(point1, point2):
    for cost in cost_list:
        if (cost[0] == point1 and cost[1] == point2) or (cost[1] == point1 and cost[0] == point2):
            return cost[2]

def shortest_distance(inputList, cost_list):
    result_list = []
    for start in inputList:
        for stop in inputList:
            for medium in inputList:
                temp = min(findDistance(start, stop), findDistance(start, medium) + findDistance(medium, stop))
            result_list.append([start, stop, temp])
    return result_list

minimum_cost_list = shortest_distance(origin, cost_list)

class Node:
    def __init__(self, current_path, remaining_stops):
        self.c_path = current_path
        self.r_stops = remaining_stops
        self.g = totalDistance(current_path)
        self.h = heuristic(current_path, remaining_stops)
    def __lt__(self, other):
        return (len(self.c_path) < len(other.c_path))

def findMinimumDistance(point1, point2):
    for cost in minimum_cost_list:
        if (cost[0] == point1 and cost[1] == point2) or (cost[1] == point1 and cost[0] == point2):
            return cost[2]

def totalDistance(inputlist):
    distance = 0
    if (inputlist == ''):
        return 0
    for i in range(len(inputlist) - 1):
        distance += findDistance(inputlist[i], inputlist[i + 1])
    return distance

def heuristic(current_path, remaining_stops):
    result = 0
    if current_path == '':
        for sublist in remaining_stops:
            if sublist:
                current_distance = 0
                for i in range(len(sublist) - 1):
                    current_distance += findMinimumDistance(sublist[i], sublist[i + 1])
                result = max(result, current_distance)
    else:
        for sublist in remaining_stops:
            if sublist:
                current_distance = findMinimumDistance(current_path[-1], sublist[0])
                for i in range(len(sublist) - 1):
                    current_distance += findMinimumDistance(sublist[i], sublist[i + 1])
                result = max(result, current_distance)
    # Original heuristic:
    # for sublist in remaining_stops:
    #     if sublist:
    #         current_distance = 0
    #         for i in range(len(sublist) - 1):
    #             if (findDistance(sublist[i], sublist[i + 1]) == findDistance('A', 'B')):
    #                 current_distance += (277+377)
    #             else:
    #                 current_distance += findDistance(sublist[i], sublist[i + 1])
    #         result = max(result, current_distance)
    return result

def reachFinalState(inputlist):
    total_length = 0
    for sublist in inputlist:
        total_length += len(sublist)
    if (total_length == 0):
        return True
    else:
        return False

def findNeighbor(current_path, remaining_stops):
    result = list()
    if not reachFinalState(remaining_stops):
        if (current_path == ''):
            return origin
        for stop in origin:
            if (stop != current_path[-1]):
                result.append(stop)
    return result

def removeElement(stop, remaining_list):
    new_remaining_list = copy.deepcopy(remaining_list)
    for sublist in new_remaining_list:
        if sublist:
            if (stop == sublist[0]):
                sublist.remove(stop)
    return new_remaining_list

def AStarMinDistance():
    frontier = list()
    start = Node('', stop_list)
    heapq.heappush(frontier, (start.g, start))
    count = 0

    while frontier:
        current = heapq.heappop(frontier)[1]
        print ("The", count, "th:")
        print ("The current cost is: ", current.g)
        print ("The current path is: ", current.c_path)

        if (reachFinalState(current.r_stops)):
            print ("The final solution is: ", current.c_path)
            print ("The total length is: ", totalDistance(current.c_path))
            print ("The number of expanded nodes is: ", count)
            print ("The number of stops is: ", len(current.c_path))
            break

        for neighbor in findNeighbor(current.c_path, current.r_stops):
            count += 1
            # print ("The neighbor is: ", neighbor)
            # print ("The original remaining stops are: ", current.r_stops)
            new_remaining_stops = removeElement(neighbor, current.r_stops)
            new_current_path = current.c_path + neighbor
            next_node = Node(new_current_path, new_remaining_stops)
            if (next_node not in frontier):
                if (next_node.g <= 5474):
                    heapq.heappush(frontier, (next_node.g, next_node))
                # print ("The heuristic is: ", next_node.h)
                #print ("The new path cost is: ", next_node.g)
                #print ("The new path is: ", next_node.c_path)
                # print ("The remaining stops are: ", new_remaining_stops)

AStarMinDistance()
