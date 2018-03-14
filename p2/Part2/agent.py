import board
from minimax import Minimax
from alphabeta import AlphaBeta


class MinimaxAgent:
    def __init__(self, firstPlayer):
        self.board = board.Board()
        if firstPlayer:
            self.me = "x"
        else:
            self.me = "o"

        if firstPlayer:
            self.oppo = "o"
        else:
            self.oppo = "x"
        # self.me = firstPlayer ? "x" : "o"
        # self.oppo = firstPlayer ? "o" : "x"
        self.firstPlayer = firstPlayer

    # get opponent's turn and place move
    def receiveTurn(self, coor):
        self.board.placeMove(coor, self.oppo)
        return coor

    def firstMove(self):
        # use default first move - the center of the board
        self.board.placeMove((3, 3), self.me)
        return (3, 3)

    def pickMove(self):
        minimax = Minimax(self.board)
        if self.me == "x":
            return minimax.playMinimax(self.board, 3, True)
        elif self.me == "o":
            return minimax.playMinimax(self.board, 3, False)

    def takeTurn(self):
        move = self.pickMove()
        print("Picked move - ", move)
        self.board.placeMove(move, self.me)
        return move

    # winner 1 -> me win
    # winner 2 -> oppo win
    # 0 -> tie
    def getWinner(self):
        return self.board.winner()


class AlphaBetaAgent:
    def __init__(self, firstPlayer):
        self.board = board.Board()
        if firstPlayer:
            self.me = "x"
        else:
            self.me = "o"

        if firstPlayer:
            self.oppo = "o"
        else:
            self.oppo = "x"
        # self.me = firstPlayer ? "x" : "o"
        # self.oppo = firstPlayer ? "o" : "x"
        self.firstPlayer = firstPlayer

    # get opponent's turn and place move
    def receiveTurn(self, coor):
        self.board.placeMove(coor, self.oppo)
        return coor

    def firstMove(self):
        # use default first move - the center of the board
        self.board.placeMove((3, 3), self.me)
        return (3, 3)

    def pickMove(self):
        alphabeta = AlphaBeta(self.board)
        if self.me == "x":
            return alphabeta.playAlphabeta(self.board, 3, -100000, 100000, True)
        elif self.me == "o":
            return alphabeta.playAlphabeta(self.board, 3, -100000, 100000, False)

    def takeTurn(self):
        move = self.pickMove()
        print("Picked move - ", move)
        self.board.placeMove(move, self.me)
        return move

    # winner 1 -> me win
    # winner 2 -> oppo win
    # 0 -> tie
    def getWinner(self):
        return self.board.winner()
