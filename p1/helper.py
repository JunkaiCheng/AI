class FileOperation:
    #Return the 2D array generated from the given input file
    def read_file(self, _input_filepath):
        input_file = open(_input_filepath, "r")
        lines = input_file.readlines()
        input_file.close()
        input_file = open(_input_filepath, "r")
        maze_array = []
        for line in lines:
            maze_array.append(list(input_file.readline()))
        input_file.close()
        return maze_array

    #Write the updated 2D array into the output file"
    def write_file(self, _maze_array, _output_filepath):
        output_file = open(_output_filepath,"w+")
        i = 0
        while(i < len(_maze_array)):
            output_file.write(''.join(_maze_array[i]))
            i += 1
        output_file.close()
