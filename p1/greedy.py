import heapq
import Queue

path = Queue.Queue()


def greedyS(path_m, maze, sx, sy, ex, ey):
    step = 0
    current_x = sx
    current_y = sy
    heap = []
    heapq.heappush(heap, (abs(current_x - ex) + abs(current_y - ey), current_x, current_y))#(heuristic, x coordinate, y coordinate)
    while heap:
        step = step + 1
        currentTup = heapq.heappop(heap)
        current_x = currentTup[1]
        current_y = currentTup[2]
        if (current_x == ex and current_y == ey):
            #end point
            return (step, path_m)
        #check the four near nodes
        if (maze[current_x + 1][current_y] != '%' and maze[current_x + 1][current_y] != '1'):
            maze[current_x + 1][current_y] = '1'
            heapq.heappush(heap, (abs(current_x + 1 - ex) + abs(current_y - ey), current_x + 1, current_y))
            path_m[current_x + 1][current_y] = (current_x, current_y)
        if (maze[current_x - 1][current_y] != '%' and maze[current_x - 1][current_y] != '1'):
            maze[current_x - 1][current_y] = '1'
            heapq.heappush(heap, (abs(current_x - 1 - ex) + abs(current_y - ey), current_x - 1, current_y))
            path_m[current_x - 1][current_y] = (current_x, current_y)
        if (maze[current_x][current_y + 1] != '%' and maze[current_x][current_y + 1] != '1'):
            maze[current_x][current_y + 1] = '1'
            heapq.heappush(heap, (abs(current_x - ex) + abs(current_y + 1 - ey), current_x, current_y + 1))
            path_m[current_x][current_y + 1] = (current_x, current_y)
        if (maze[current_x][current_y - 1] != '%' and maze[current_x][current_y - 1] != '1'):
            maze[current_x][current_y - 1] = '1'
            heapq.heappush(heap, (abs(current_x - ex) + abs(current_y - 1 - ey), current_x, current_y - 1))
            path_m[current_x][current_y - 1] = (current_x, current_y)
    return (step, path_m)

