import copy


class Stone:
    def __init__(self):
        self.color = None

    def place(self, color):
        # to place a stone
        self.color = color
        # color = 0: red
        # color = 1: blue


class ChessBoard:
    def __init__(self):
        self.length = 7
        self.width = 7
        self.board = [[Stone() for i in range(self.length)] for i in range(self.width)]
        self.print = [['.' for i in range(self.length)] for i in range(self.width)]  # for printing
        self.stone = [0, 0]  # record the order

        # initialize


    def first_move(self, color):
        self.board[0][0].place(0)
        if color == 0:
            self.print[0][0] = 'a'
            self.stone = [1, 0]
        else:
            self.print[0][0] = 'A'
            self.stone = [0, 1]

    def move(self, spot, color):
        x = spot[0]
        y = spot[1]
        self.board[x][y].place(color)
        if color == 0:
            self.print[x][y] = chr(ord('a') + self.stone[color])
        else:
            self.print[x][y] = chr(ord('A') + self.stone[color])
        self.stone[color] += 1

    def check_chain(self, color, num, oneHead, curr_x, curr_y, dir_x, dir_y):
        # check different directions
        flag = 1  # whether such a chain exits
        for i in range(num):
            curr_x += dir_x
            curr_y += dir_y
            if curr_x >= 7 or curr_x < 0 or curr_y >= 7 or curr_y < 0:
                flag = 0
                break
            if not (self.board[curr_x][curr_y] and self.board[curr_x][curr_y].color == color):
                flag = 0
                break
        if oneHead == 0 and flag == 1:
            curr_x += dir_x
            curr_y += dir_y
            if curr_x >= 7 or curr_x < 0 or curr_y >= 7 or curr_y < 0:
                flag = 0
            elif self.board[curr_x][curr_y].color is not None:
                flag = 0
        return flag

    def find_chain(self, color, num, oneHead):
        # find one adjacent place for the chain with particular color(color) and number of stones(num)
        # oneHead: 1 to find empty space on either end of the chain, 0 to find empty spaces on both ends of the chain
        # return the coordinate
        dir = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        for j in range(self.width):
            for i in range(self.length):
                if self.board[j][i].color is not None:
                    continue
                for k in dir:
                    result = self.check_chain(color, num, oneHead, j, i, k[0], k[1])
                    if result == 1:
                        return j, i

    def four_block(self, color):
        # find blocks with four stones
        # return the empty place in such a block
        available = []
        if color == 0:
            opp = 1
        else:
            opp = 0
        for i in range(self.width):
            for j in range(self.length):
                if self.board[i][j].color == opp:
                    continue
                result = self.check_block(color, i, j)
                if result[0] == 1 and result[1] == 4:
                    available.append((i, j))
        result_list = []
        for place in available:
            result = self.place_stone(place[0], place[1], 4, color)
            result_list.append(result)
        result_list.sort(key=lambda x: (x[0], x[1]))
        if len(result_list) > 0:
            return result_list[0]

    def check_block(self, color, x, y):
        # to check whether is a wining block and the number of stones in it
        flag = 0  # whether a winning block exits
        max_num = 0
        if color == 0:
            opp = 1
        else:
            opp = 0
        dir = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        for k in dir:
            curr_x = x
            curr_y = y
            num = 0
            break_flag = 0
            for i in range(5):
                if curr_x >= 7 or curr_x < 0 or curr_y >= 7 or curr_y < 0:
                    break_flag = 1
                    break
                if self.board[curr_x][curr_y].color == opp:
                    break_flag = 1
                    break
                if self.board[curr_x][curr_y].color == color:
                    num += 1
                curr_x += k[0]
                curr_y += k[1]
            if break_flag == 0:
                flag = 1
                if max_num < num:
                    max_num = num
        return flag, max_num

    def check_next(self, x, y, dir, color):
        # check whether there is an adjacent stone
        if x >= 7 or x < 0 or y >= 7 or y < 0:
            return 0
        if self.board[x][y].color is not None:
            return 0
        if x + dir[0] < 7 and x + dir[0] >= 0 and y + dir[1] < 7 and y + dir[1] >= 0:
            if self.board[x + dir[0]][y + dir[1]].color == color:
                return 1
        if x - dir[0] < 7 and x - dir[0] >= 0 and y - dir[1] < 7 and y - dir[1] >= 0:
            if self.board[x - dir[0]][y - dir[1]].color == color:
                return 1
        return 0

    def place_stone(self, x, y, max_num, color):
        #  place the stone next to a stone already in the winning block on board
        dir = [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        available = []
        for k in dir:
            curr_x = x
            curr_y = y
            num = 0
            for i in range(5):
                if curr_x >= 7 or curr_x < 0 or curr_y >= 7 or curr_y < 0:
                    break
                if self.board[curr_x][curr_y].color == color:
                    num += 1
                curr_x += k[0]
                curr_y += k[1]
            if max_num == num:
                for j in range(5):
                    curr_x -= k[0]
                    curr_y -= k[1]
                    if curr_x >= 7 or curr_x < 0 or curr_y >= 7 or curr_y < 0:
                        break
                    if max_num != 0:
                        if self.check_next(curr_x, curr_y, k, color) == 1:
                            available.append((curr_x, curr_y))
                    else:
                        available.append((curr_x, curr_y))
        available.sort(key=lambda x: (x[0], x[1]))
        return available[0]

    def find_block(self, color):
        # to find winning block
        available = []
        if color == 0:
            opp = 1
        else:
            opp = 0
        max_num = 0
        for i in range(self.width):
            for j in range(self.length):
                if self.board[i][j].color == opp:
                    continue
                result = self.check_block(color, i, j)
                if result[0] == 1 and result[1] > max_num:
                    max_num = result[1]
                    available.clear()
                    available.append((i, j))
                elif result[0] == 1 and result[1] == max_num:
                    available.append((i, j))
        result_list = []
        for place in available:
            result = self.place_stone(place[0], place[1], max_num, color)
            result_list.append(result)
        result_list.sort(key=lambda x: (x[0], x[1]))
        if len(result_list) != 0:
            return result_list[0]

    def check_win(self, turn):
        # the first step of checking
        # turn: whose turn to place a stone (0: red, 1: blue)
        # return whether find a chain of 4
        result = self.four_block(turn)
        if result is None:
            # No block of 4
            return
        x = result[0]
        y = result[1]
        self.board[x][y].place(turn)
        if turn == 0:
            self.print[x][y] = chr(ord('a') + self.stone[turn])
        else:
            self.print[x][y] = chr(ord('A') + self.stone[turn])
        self.stone[turn] += 1
        return x, y

    def check_opp4(self, turn):
        # the second step of checking
        # turn: whose turn to place a stone (0: red, 1: blue)
        # return whether find a chain of 4
        if turn == 0:
            result = self.find_chain(1, 4, 1)
        else:
            result = self.find_chain(0, 4, 1)
        if not result:
            # No chain of 4
            return
        else:
            x = result[0]
            y = result[1]
            self.board[x][y].place(turn)
            if turn == 0:
                self.print[x][y] = chr(ord('a') + self.stone[turn])
            else:
                self.print[x][y] = chr(ord('A') + self.stone[turn])
            self.stone[turn] += 1
            return x, y

    def check_opp3(self, turn):
        # the third step of checking
        # turn: whose turn to place a stone (0: red, 1: blue)
        # return whether find a chain of 3
        if turn == 0:
            result = self.find_chain(1, 3, 0)
        else:
            result = self.find_chain(0, 3, 0)
        if not result:
            # No chain of 3
            return
        else:
            x = result[0]
            y = result[1]
            self.board[x][y].place(turn)
            if turn == 0:
                self.print[x][y] = chr(ord('a') + self.stone[turn])
            else:
                self.print[x][y] = chr(ord('A') + self.stone[turn])
            self.stone[turn] += 1
            return x, y

    def last_check(self, turn, opp):
        # the last step of checking
        # turn: whose turn to place a stone (0: red, 1: blue)
        # if to check opp, opp = True
        if not opp:
            result = self.find_block(turn)
            if result:
                (x, y) = result
                self.board[x][y].place(turn)
                if turn == 0:
                    self.print[x][y] = chr(ord('a') + self.stone[turn])
                else:
                    self.print[x][y] = chr(ord('A') + self.stone[turn])
                self.stone[turn] += 1
                return x, y
        else:
            temp = copy.deepcopy(turn)
            temp = change_turn(temp)
            result = self.find_block(temp)
            if result:
                (x, y) = result
                self.board[x][y].place(turn)
                if turn == 0:
                    self.print[x][y] = chr(ord('a') + self.stone[turn])
                else:
                    self.print[x][y] = chr(ord('A') + self.stone[turn])
                self.stone[turn] += 1
                return x, y

    def print_board(self):
        # print the chess_board
        for j in range(self.width):
            line = ""
            for i in range(self.length):
                line = line + self.print[i][6 - j]
            print(line)


def change_turn(turn):
    if turn == 0:
        return 1
    else:
        return 0


class reflexAgent:
    def __init__(self, color):
        self.color = color

    def reflex_move(self, board):
        result1 = board.check_win(self.color)
        if result1 is not None:
            print("Reflex wins")
            return 1, result1
        result2 = board.check_opp4(self.color)
        if result2 is not None:
            return 0, result2
        result3 = board.check_opp3(self.color)
        if result3 is not None:
            return 0, result3
        result4 = board.last_check(self.color, False)
        if result4 is not None:
            return 0, result4
        result5 = board.last_check(self.color, True)
        if result5 is not None:
            return 0, result5
        else:
            return 2, (0, 0)
