import board
import copy
import eval

"""
Alpha-Beta Pruning
"""


class AlphaBeta:
    def __init__(self, color):
        self.color = color
        self.expand = 0

    '''    
    def evalHelper(self, board, attack):
        value = 0
        player = board.prevPlayer if attack else board.nextPlayer
        # print ("player ", player)
        pathNum = board.numOfPossibleBlocks(board.prevPlayer)
        # print ("pathNum ", pathNum)
        if pathNum > 0:
            oneAway = board.xFromWin(1)
            twoAway = board.xFromWin(2)
            threeAway = board.xFromWin(3)
            fourAway = board.xFromWin(4)
            # print ("oneAway", oneAway)
            # print ("twoAway", twoAway)
            # print ("threeAway", threeAway)
            # print ("fourAway", fourAway)
            value = fourAway ** 5 + threeAway ** 5 + twoAway ** 5 + oneAway ** (9 if attack else 8)
        return value

    def evaluation(self, board):
        return self.evalHelper(board, True) + self.evalHelper(board, False)
    '''

    def evaluation(self, board):
        # return self.evalHelper(board, True) + self.evalHelper(board, False)
        xone = board.xFromWin(4)
        xtwo = board.xFromWin(3)
        xthree = board.xFromWin(2)
        xfour = board.xFromWin(1)
        xfive = board.xWin()
        xvalue = xone + 10 * xtwo + 100 * xthree + 1000 * xfour + 10000 * xfive

        oone = board.oFromWin(4)
        otwo = board.oFromWin(3)
        othree = board.oFromWin(2)
        ofour = board.oFromWin(1)
        ofive = board.xWin()
        ovalue = oone + 10 * otwo + 100 * othree + 1000 * ofour + 10000 * ofive

        if self.color == "x":
            return xvalue - 0.8 * ovalue
        else:
            return -ovalue + 0.8 * xvalue

    def check_first(self, board):
        # return 0: continue minimax
        # return 1: place to win
        # return 2: place to avoid opponent winning
        # return 3: place to avoid open chain with three stones
        if self.color == "x":
            if board.xFromWin(1) > 0:
                return 1
            elif board.oFromWin(1) > 0:
                return 2
            else:
                op3 = eval.openChain3(board)
                if op3.find_chain("o", 3, 0):
                    return 3
        if self.color == "o":
            if board.oFromWin(1) > 0:
                return 1
            elif board.xFromWin(1) > 0:
                return 2
            else:
                op3 = eval.openChain3(board)
                if op3.find_chain("x", 3, 0):
                    return 3
        return 0

    def direct_place(self, board, temp):
        if temp == 3:
            op3 = eval.openChain3(board)
            op3.find_chain("x", 3, 0)
            return op3.avoid_openChain3()
        if self.color == "x":
            if temp == 1:
                to_place = "x"
            else:
                to_place = "o"
        else:
            if temp == 1:
                to_place = "o"
            else:
                to_place = "x"
        for i in range(7):
            for j in range(7):
                if board.board[i][j] != ".":
                    continue
                new_board = copy.deepcopy(board)
                new_board.placeMove((i, j), to_place)
                if new_board.endGame():
                    return i, j

    def takeTurn(self, maxPlayer):
        if maxPlayer:
            return "x"
        else:
            return "o"

    def alphabeta(self, curr_board, depth, a, b, maxPlayer):
        self.expand = 0
        if depth == 3:
            temp = self.check_first(curr_board)
            if temp != 0:
                return self.direct_place(curr_board, temp)
            else:
                if not maxPlayer:
                    emptySpots = curr_board.getEmpty()
                    value = 100000
                    place = (0, 0)
                    for spot in emptySpots:
                        self.expand += 1
                        new_board = copy.deepcopy(curr_board)
                        new_board.placeMove(spot, self.takeTurn(maxPlayer))
                        temp_value = min(value, self.alphabeta(new_board, depth - 1, a, b, not maxPlayer))
                        # print(temp_value)
                        if temp_value < value:
                            value = temp_value
                            place = spot
                        a = min(a, value)
                        if b <= a:
                            break
                    return place
                else:
                    emptySpots = curr_board.getEmpty()
                    value = -100000
                    place = (0, 0)
                    for spot in emptySpots:
                        self.expand += 1
                        new_board = copy.deepcopy(curr_board)
                        new_board.placeMove(spot, self.takeTurn(maxPlayer))
                        temp_value = max(value, self.alphabeta(new_board, depth - 1, a, b, not maxPlayer))
                        # print(temp_value)
                        if temp_value > value:
                            value = temp_value
                            place = spot
                        a = max(a, value)
                        if b <= a:
                            break
                    return place
        else:
            # if depth == 0 or curr_board.endGame():
            if depth == 0:
                result = self.evaluation(curr_board)
                return result
            emptySpots = curr_board.getEmpty()
            if maxPlayer:
                value = -100000
                for spot in emptySpots:
                    self.expand += 1
                    new_board = copy.deepcopy(curr_board)
                    new_board.placeMove(spot, self.takeTurn(maxPlayer))
                    value = max(value, self.alphabeta(new_board, depth - 1, a, b, not maxPlayer))
                    a = max(a, value)
                    if b <= a:
                        break
                return value
            else:
                value = 100000
                for spot in emptySpots:
                    self.expand += 1
                    new_board = copy.deepcopy(curr_board)
                    new_board.placeMove(spot, self.takeTurn(maxPlayer))
                    value = min(value, self.alphabeta(new_board, depth - 1, a, b, not maxPlayer))
                    b = min(b, value)
                    if b <= a:
                        break
                return value
