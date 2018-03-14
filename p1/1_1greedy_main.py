import helper
import greedy
file_operation = helper.FileOperation()
input_filepath = "openMaze.txt"
maze_array = file_operation.read_file(input_filepath)
row= len(maze_array) #number of rows
col= len(maze_array[1])-1 #number of columns
matrix = [[(0,0) for i in range(col)] for j in range(row)]
for i in range(0, row-1):
    for j in range(0, col-1):
        if maze_array[i][j]=='P':
            sx=i
            sy=j #start point
        elif maze_array[i][j]=='.':
            ex=i
            ey=j #end point
path=greedy.greedyS(matrix, maze_array, sx, sy, ex, ey)
array_maze = file_operation.read_file(input_filepath)
matrix=path[1]
x=ex
y=ey
while not(x==sx and y==sy):
    tx=x
    ty=y
    x=matrix[tx][ty][0]
    y=matrix[tx][ty][1]
    array_maze[tx][ty]='.'
output_filepath = "output_openMaze.txt"
file_operation.write_file(array_maze, output_filepath)
print path[0]
