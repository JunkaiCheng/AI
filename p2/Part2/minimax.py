"""
Minimax Search
"""
import copy
import board
import eva


class Minimax:
    # optionList = {}

    def __init__(self, board):
        self.expand = 0
        self.board = board

    def evaluation(self, board):
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

        if board.prevPlayer == "x":
            return xvalue - 0.8 * ovalue
        else:
            return -ovalue + 0.8 * xvalue

    def check_first(self, board):
        if board.prevPlayer == "x":
            if board.xFromWin(1) > 0:
                return 1
            elif board.oFromWin(1) > 0:
                return 2
            else:
                op3 = eva.openChain3(board)
                if op3.find_chain("o", 3, 0):
                    return 3
        if board.prevPlayer == "o":
            if board.oFromWin(1) > 0:
                return 1
            elif board.xFromWin(1) > 0:
                return 2
            else:
                op3 = eva.openChain3(board)
                if op3.find_chain("x", 3, 0):
                    return 3
        return 0

    def direct_place(self, board, temp):
        if temp == 3:
            op3 = eva.openChain3(board)
            if board.prevPlayer == "x":
                op3.find_chain("o", 3, 0)
            else:
                op3.find_chain("x", 3, 0)
            return op3.avoid_openChain3()
        if board.prevPlayer == "x":
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

    def playMinimax(self, board, depth, maximizingPlayer):
        if depth == 3:
            temp = self.check_first(board)
            if temp != 0:
                print("expanded node", self.expand)
                return self.direct_place(board, temp)
            else:
                if maximizingPlayer:
                    emptySpots = board.getEmpty()
                    bestValue = -100000
                    position = (3, 3)
                    for spot in emptySpots:
                        self.expand += 1
                        newBoard = copy.deepcopy(board)
                        newBoard.placeMove(spot, board.nextPlayer)
                        value = max(bestValue, self.playMinimax(newBoard, depth - 1, False))
                        if value > bestValue:
                            bestValue = value
                            position = spot
                    print("expanded node", self.expand)
                    return position
                elif not maximizingPlayer:
                    emptySpots = board.getEmpty()
                    bestValue = 100000
                    poaition = (3, 3)
                    for spot in emptySpots:
                        self.expand += 1
                        newBoard = copy.deepcopy(board)
                        newBoard.placeMove(spot, board.nextPlayer)
                        value = min(bestValue, self.playMinimax(newBoard, depth - 1, True))
                        if value < bestValue:
                            bestValue = value
                            position = spot
                    print("expanded node", self.expand)
                    return position
        else:
            if depth == 0 or board.endGame():
                value = self.evaluation(board)
                # board.printBoard()
                # print ("value", value)
                # print "-----------------------"
                return value
            emptySpots = board.getEmpty()
            if maximizingPlayer:
                bestValue = -100000
                for spot in emptySpots:
                    self.expand += 1
                    newBoard = copy.deepcopy(board)
                    newBoard.placeMove(spot, newBoard.nextPlayer)
                    bestValue = max(bestValue, self.playMinimax(newBoard, depth - 1, False))
                return bestValue
            else:
                bestValue = 100000
                for spot in emptySpots:
                    self.expand += 1
                    newBoard = copy.deepcopy(board)
                    newBoard.placeMove(spot, newBoard.nextPlayer)
                    bestValue = min(bestValue, self.playMinimax(newBoard, depth - 1, True))
                return bestValue
