import board
import string


class openChain3:
    def __init__(self, board):
        self.this_board = board
        self.place = None


    def check_chain(self, color, num, oneHead, curr_x, curr_y, dir_x, dir_y):
        # check different directions
        flag = 1  # whether such a chain exits
        for i in range(num):
            curr_x += dir_x
            curr_y += dir_y
            if curr_x >= 7 or curr_x < 0 or curr_y >= 7 or curr_y < 0:
                flag = 0
                break
            if not (self.this_board.board[curr_x][curr_y] and self.this_board.board[curr_x][curr_y] == color):
                flag = 0
                break
        if oneHead == 0 and flag == 1:
            curr_x += dir_x
            curr_y += dir_y
            if curr_x >= 7 or curr_x < 0 or curr_y >= 7 or curr_y < 0:
                flag = 0
            elif self.this_board.board[curr_x][curr_y] != ".":
                flag = 0
        return flag

    def find_chain(self, color, num, oneHead):
        # find one adjacent place for the chain with particular color(color) and number of stones(num)
        # oneHead: 1 to find empty space on either end of the chain, 0 to find empty spaces on both ends of the chain
        # return the coordinate
        dir = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        for j in range(7):
            for i in range(7):
                if self.this_board.board[j][i] != ".":
                    continue
                for k in dir:
                    result = self.check_chain(color, num, oneHead, j, i, k[0], k[1])
                    if result == 1:
                        self.place = j, i
                        return 1


    def avoid_openChain3(self):
        return self.place



