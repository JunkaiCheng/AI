"""
All board manipulation and look up function.
First player uses lowercase letter from "a" to "z"
Second player uses uppercase letter from "A" to "Z"
Empty intersection "."

Board Size : 7 * 7

		---------------
		| | | | | | | |
		---------------
		| | | | | | | |
		---------------
		| | | | | | | |
		---------------
		| | | | | | | |
		---------------
		| | | | | | | |
		---------------
		| | | | | | | |
		---------------
		| | | | | | | |
		---------------

In order to count winning blocks, we first initialize a 
list that contain all the possible winning blocks in the 
7 * 7 board. Then modify the value of the elem(block) in
the list accordingly when a move is place
"""

# TODO: initialize the list of blocks and the hash table(index in list -> value of block)

import string


class Board:
    def __init__(self):
        # fill board with "."
        # to start a new board, pass in [] for board/blocks, 0 for copyxcount/copyocount
        # initialize the list of blocks & the hash table (key = index in list, value the # of agent stone thus far）
        # initialize all starting block
        self.xset = set(string.ascii_lowercase)
        self.oset = set(string.ascii_uppercase)
        self.prevPlayer = ""
        self.nextPlayer = ""
        self.xcount = 0
        self.ocount = 0
        self.blockValue = []
        self.blocks = []
        for i in range(7):
            for j in range(3):
                temp = []
                for k in range(5):
                    temp.append((i, j + k))
                self.blocks.append(temp)
                rev = []
                for codn in temp:
                    r_codn = codn[::-1]
                    rev.append(r_codn)
                self.blocks.append(rev)
        for i in range(3):
            temp = []
            for j in range(5):
                temp.append((i + j, j))
            self.blocks.append(temp)
            if i != 2:
                temp2 = []
                for j in range(5):
                    temp2.append((i + j + 1, j + 1))
                self.blocks.append(temp2)
            if i == 0:
                temp3 = []
                for j in range(5):
                    temp3.append((i + j + 2, j + 2))
                self.blocks.append(temp3)
        for i in range(2):
            temp = []
            for j in range(5):
                temp.append((i + 4 - j, 6 - j))
            self.blocks.append(temp)
            if i == 1:
                temp2 = []
                for j in range(5):
                    temp2.append((i + 3 - j, 5 - j))
                self.blocks.append(temp2)
        for i in range(3):
            temp = []
            for j in range(5):
                temp.append((i + j, 6 - j))
            self.blocks.append(temp)
            if i != 2:
                temp2 = []
                for j in range(5):
                    temp2.append((i + j + 1, 6 - j - 1))
                self.blocks.append(temp2)
            if i == 0:
                temp3 = []
                for j in range(5):
                    temp3.append((i + j + 2, 6 - j - 2))
                self.blocks.append(temp3)
        for i in range(2):
            temp = []
            for j in range(5):
                temp.append((4 + i - j, j))
            self.blocks.append(temp)
            if i == 1:
                temp2 = []
                for j in range(5):
                    temp2.append((3 + i - j, j + 1))
                self.blocks.append(temp2)
        self.board = []
        for i in range(len(self.blocks)):
            self.blockValue.append(0)
        for i in range(7):
            self.board.append([])
            for j in range(7):
                self.board[i].append(".")

    # get a list of all empty spot(tuples) in the board
    def getEmpty(self):
        emptySpot = []
        for i in range(7):
            for j in range(7):
                if self.board[i][j] == ".":
                    emptySpot.append((i, j))
        return emptySpot

    # place a move in the board
    # coor - a tuple containing the coordinate (x, y)
    # player - the player who are placing the stone
    def placeMove(self, coor, player):
        if player == "x":
            self.board[coor[0]][coor[1]] = chr(ord("a") + self.xcount)
            self.xcount += 1
            for i in range(len(self.blocks)):
                if (coor in self.blocks[i]) and self.blockValue[i] != 100000:
                    if self.blockValue[i] < 0:
                        # block became useless
                        self.blockValue[i] = 100000
                    else:
                        self.blockValue[i] += 1
        else:
            self.board[coor[0]][coor[1]] = chr(ord("A") + self.ocount)
            self.ocount += 1
            for i in range(len(self.blocks)):
                if (coor in self.blocks[i]) and self.blockValue[i] != 100000:
                    if self.blockValue[i] > 0:
                        # block become useless
                        self.blockValue[i] = 100000
                    else:
                        self.blockValue[i] -= 1

        self.prevPlayer = player
        if player == "x":
            self.nextPlayer = "o"
        else:
            self.nextPlayer = "x"

    # calculate the number of winning blocks of 5-x consecutive
    # stones for the given player
    def xFromWin(self, x):
        numOfblocks = 0
        i = len(self.blockValue) - 1
        while i >= 0:
            if self.blockValue[i] == 5 - x:
                numOfblocks += 1
            i -= 1
        return numOfblocks

    def oFromWin(self, x):
        numOfblocks = 0
        i = len(self.blockValue) - 1
        while i >= 0:
            if self.blockValue[i] == x - 5:
                numOfblocks += 1
            i -= 1
        return numOfblocks

    def xWin(self):
        numOfblocks = 0
        i = len(self.blockValue) - 1
        while i >= 0:
            if self.blockValue[i] == 5:
                numOfblocks += 1
            i -= 1
        return numOfblocks

    def oWin(self):
        numOfblocks = 0
        i = len(self.blockValue) - 1
        while i >= 0:
            if self.blockValue[i] == -5:
                numOfblocks += 1
            i -= 1
        return numOfblocks

    # check if the board is a terminal board
    # block of consecutive 5 exist
    def endGame(self):
        i = len(self.blockValue) - 1
        while i >= 0:
            if self.blockValue[i] == 5:
                return True
            elif self.blockValue[i] == -5:
                return True
            i -= 1
        if len(self.getEmpty()) == 0:
            return True
        return False

    def first_move(self, player):
        if self.board[3][3] == ".":
            self.placeMove((3, 3), player)
            return 3, 3
        else:
            self.placeMove((2, 2), player)
            return 2, 2

    def printBoard(self):
        # print the chess_board
        for j in range(7):
            line = ""
            for i in range(7):
                line = line + self.board[i][6 - j]
            print(line)

    def numOfPossibleBlocks(self, player):
        numOfblocks = 0
        if player == "x":
            i = len(self.blockValue) - 1
            while i >= 0:
                if self.blockValue[i] > 0 and self.blockValue[i] != 100000:
                    numOfblocks += 1
                i -= 1
        elif player == "o":
            i = len(self.blockValue) - 1
            while i >= 0:
                if self.blockValue[i] < 0:
                    numOfblocks += 1
                i -= 1
        return numOfblocks
