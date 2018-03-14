"""
Play the game between Agent:
"""

import agent
import time
import minimax
import board
import alphabeta

if __name__ == "__main__":
    start = time.time()

    # test = board.Board()
    # test.placeMove((3,3), "x")
    # test.placeMove((2,2), "o")
    # test.placeMove((3,4), "x")
    # test.placeMove((4,3), "o")
    # mini = minimax.Minimax(test)
    # spot = mini.playMinimax(test, 3, True)
    # print ("Final spot - ", spot)

    # test = board.Board()
    # test.placeMove((0,0), "x")
    # test.placeMove((3,3), "o")
    # test.placeMove((0,1), "x")
    # test.placeMove((1,2), "o")
    # test.placeMove((0,2), "x")
    # test.placeMove((4,1), "o")
    # test.placeMove((0,3), "x")
    # test.printBoard()
    # ab = alphabeta.AlphaBeta(test)
    # spot = ab.playAlphabeta(test, 3, -100000, 100000, True)
    # test.placeMove(spot, "o")
    # print ("Final spot - ", spot)
    # test.printBoard()

    # x = agent.AlphaBetaAgent(True)
    o = agent.MinimaxAgent(False)
    x = agent.AlphaBetaAgent(True)
    # o = agent.MinimaxAgent(False)
    xMove = x.firstMove()
    print("Starting board: First Move of x")
    x.board.printBoard()
    print("-----------------------")

    oMove = None
    while (True):
        o.receiveTurn(xMove)
        oMove = o.takeTurn()
        # MARK
        print("o Move")
        o.board.printBoard()
        print("-----------------------")
        if (o.getWinner() == 1):
            print("winner : x")
            break
        if (o.getWinner() == 2):
            print("winner : o")
            break
        if (o.getWinner() == 0):
            print("tie")
            break
        x.receiveTurn(oMove)
        xMove = x.takeTurn()
        # MARK
        print("x Move")
        x.board.printBoard()
        print("-----------------------")
        if (x.getWinner() == 1):
            print("winner : x")
            break
        if (x.getWinner() == 2):
            print("winner : o")
            break
        if (x.getWinner() == 0):
            print("tie")
            break

    end = time.time()
    print(end - start)
