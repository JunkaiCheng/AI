from collections import Counter, OrderedDict
import copy
import random
import numpy as np
import matplotlib.pyplot as plt

origin = ['A', 'B', 'C', 'D', 'E']

stop_dict = {0: 'A', 1: 'B', 2: 'C', 3: 'D', 4: 'E'}

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

class min_stop_Node:
    def __init__(self, current_path, remaining_stops):
        self.c_path = current_path
        self.r_stops = remaining_stops
        self.g = len(current_path)
        self.h = min_stop_heuristic(remaining_stops)

class min_distance_Node:
    def __init__(self, current_path, remaining_stops):
        self.c_path = current_path
        self.r_stops = remaining_stops
        self.g = totalDistance(current_path)
        self.h = min_distance_heuristic(current_path, remaining_stops)

def generate_random_widget(N):
    widget = []
    for i in range(N):
        temp = random.randint(0, 4)
        while (i >= 1 and stop_dict[temp] == widget[-1]):
            temp = random.randint(0, 4)
        widget.append(stop_dict[temp])
    return widget

def generate_stop_list(N):
    stop_list = []
    for i in range(5):
        temp_list = generate_random_widget(N)
        while (temp_list in stop_list):
            temp_list = generate_random_widget(N)
        stop_list.append(temp_list)
    return stop_list

def min_stop_heuristic(inputlist):
    result = 0
    for sublist in inputlist:
        result = max(result, len(sublist))
    return result

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

def min_distance_heuristic(current_path, remaining_stops):
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
    return result

def reachFinalState(inputlist):
    total_length = 0
    for sublist in inputlist:
        total_length += len(sublist)
    if (total_length == 0):
        return True
    else:
        return False

def findNeighbor(inputList):
    result = list()
    if not reachFinalState(inputList):
        for sublist in inputList:
            if sublist:
                result.append(sublist[0])
            else:
                continue
        result.sort(key=Counter(result).get, reverse=True)
        result = list(OrderedDict.fromkeys(result))
    return result

def removeElement(stop, remaining_list):
    new_remaining_list = copy.deepcopy(remaining_list)
    for sublist in new_remaining_list:
        if sublist:
            if (stop == sublist[0]):
                sublist.remove(stop)
    return new_remaining_list

def AStarMinStop():
    frontier = set()
    start = min_stop_Node('', stop_list)
    frontier.add(start)
    count = 0

    while frontier:
        current = min(frontier, key = lambda x: x.g + x.h)
        frontier.remove(current)

        if (reachFinalState(current.r_stops)):
            print ("The final solution is: ", current.c_path)
            print ("The number of expanded nodes is: ", count)
            print ("The number of minimum stops is: ", len(current.c_path))
            return count

        for neighbor in findNeighbor(current.r_stops):
            count += 1
            # print ("The", count, "th:")
            new_remaining_stops = removeElement(neighbor, current.r_stops)
            new_current_path = current.c_path + neighbor
            next_node = min_stop_Node(new_current_path, new_remaining_stops)
            frontier.add(next_node)
            # print ("The current path is: ", new_current_path)
            # print ("The remaining stops are: ", new_remaining_stops)

def AStarMinDistance():
    frontier = set()
    start = min_distance_Node('', stop_list)
    frontier.add(start)
    count = 0

    while frontier:
        current = min(frontier, key = lambda x: x.g + x.h)
        frontier.remove(current)

        if (reachFinalState(current.r_stops)):
            print ("The final solution is: ", current.c_path)
            print ("The total length is: ", totalDistance(current.c_path))
            print ("The number of expanded nodes is: ", count)
            return count

        for neighbor in findNeighbor(current.r_stops):
            count += 1
            # print ("The", count, "th:")
            new_remaining_stops = removeElement(neighbor, current.r_stops)
            new_current_path = current.c_path + neighbor
            next_node = min_distance_Node(new_current_path, new_remaining_stops)
            frontier.add(next_node)

min_stop_expanded_nodes = []
min_distance_expanded_nodes = []

for i in range(3, 9, 1):
    print ("======================================================================")
    print ("When N is:", i)
    stop_list = generate_stop_list(i)
    print ("The stop list is:", stop_list)
    print ("=================The minimum stop result is as follows=================")
    min_stop = AStarMinStop()
    min_stop_expanded_nodes.append(min_stop)
    print ("===============The minimum distance result is as follows===============")
    min_distance = AStarMinDistance()
    min_distance_expanded_nodes.append(min_distance)

x_axis = np.arange(3, 9)

bar1 = plt.bar(x_axis - 0.2, min_stop_expanded_nodes, width = 0.4, color='r', label='min_stop')

bar2 = plt.bar(x_axis + 0.2, min_distance_expanded_nodes, width = 0.4, color='b', label='min_distance')

plt.xlabel('Number of stops')
plt.ylabel('Number of expanded nodes')
plt.title('The plot of number of expanded nodes')
plt.xticks(x_axis)
plt.legend()
# plt.tight_layout()
plt.show()