import abp
import board
import reflex


def change_turn(turn, maximizing):
    if turn == 0:
        return 1, not maximizing
    else:
        return 0, not maximizing


if __name__ == "__main__":
    win = 0
    reflex_board = reflex.ChessBoard()
    reflex_agent = reflex.reflexAgent(0)
    abp_board = board.Board()
    abp_agent = abp.AlphaBeta("o")
    maximizing = False
    turn = 1
    reflex_board.first_move(0)
    abp_board.placeMove((0, 0), "x")
    print("Reflex Moves:")
    reflex_board.print_board()
    """"
    first_spot = abp_board.first_move("o")
    reflex_board.move(first_spot, 1)
    print("Alpha-Beta Moves:")
    reflex_board.print_board()
    """
    while win == 0:
        if turn == 0:
            print("Reflex Moves:")
            (win, spot) = reflex_agent.reflex_move(reflex_board)
            if win != 2:
                abp_board.placeMove(spot, "x")
        else:
            print("Alpha-Beta Moves:")
            spot = abp_agent.alphabeta(abp_board, 3, -100000, 100000, maximizing)
            abp_board.placeMove(spot, "o")
            reflex_board.move(spot, 1)
            if abp_board.endGame():
                win = 1
                print("Alpha-Beta wins")
        (turn, maximizing) = change_turn(turn, maximizing)
        reflex_board.print_board()
    if win == 2:
        print("Tie")
