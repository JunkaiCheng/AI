from collections import Counter, OrderedDict
import copy

stop_list = [['A', 'E', 'D', 'C', 'A'],
             ['B', 'E', 'A', 'C', 'D'],
             ['B', 'A', 'B', 'C', 'E'],
             ['D', 'A', 'D', 'B', 'D'],
             ['B', 'E', 'C', 'B', 'D']]

class Node:
    def __init__(self, current_path, remaining_stops):
        self.c_path = current_path
        self.r_stops = remaining_stops
        self.g = len(current_path)
        self.h = heuristic(remaining_stops)

def heuristic(inputlist):
    result = 0
    for sublist in inputlist:
        result = max(result, len(sublist))
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
    start = Node('', stop_list)
    frontier.add(start)
    count = 0

    while frontier:
        current = min(frontier, key = lambda x: x.g + x.h)
        frontier.remove(current)

        if (reachFinalState(current.r_stops)):
            print ("The final solution is: ", current.c_path)
            print ("The number of expanded nodes is: ", count)
            print ("The number of minimum stops is: ", len(current.c_path))
            break

        for neighbor in findNeighbor(current.r_stops):
            count += 1
            # print ("The", count, "th:")
            new_remaining_stops = removeElement(neighbor, current.r_stops)
            new_current_path = current.c_path + neighbor
            next_node = Node(new_current_path, new_remaining_stops)
            frontier.add(next_node)
            # print ("The current path is: ", new_current_path)
            # print ("The remaining stops are: ", new_remaining_stops)

AStarMinStop()