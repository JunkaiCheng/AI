from tkinter import *
import tkinter.messagebox
import reflex
import board
from time import sleep


class Game:
    def __init__(self):
        self.reflex_board = reflex.ChessBoard()
        self.player_board = board.Board()
        self.reflex_agent = reflex.reflexAgent(1)
        self.root = Tk()
        self.root.title('Gomoku')
        self.chess_board = Canvas(self.root, width=450, height=450, bg='white')
        self.chess_board.create_rectangle(50, 50, 400, 400)
        for i in range(6):
            self.chess_board.create_line(50, 100 + 50 * i, 400, 100 + 50 * i)
            self.chess_board.create_line(100 + 50 * i, 50, 100 + 50 * i, 400)

    def place(self, x, y, color):
        if color == 0:
            self.chess_board.create_oval(75 + 50 * x - 20, 375 - 50 * y - 20, 75 + 50 * x + 20, 375 - 50 * y + 20,
                                         fill="red")
        else:
            self.chess_board.create_oval(75 + 50 * x - 20, 375 - 50 * y - 20, 75 + 50 * x + 20, 375 - 50 * y + 20,
                                         fill="blue")

    def win0(self):
        tkinter.messagebox.askokcancel("Game over", "Red wins!")

    def win1(self):
        tkinter.messagebox.askokcancel("Game over", "Blue wins!")

    def click(self, event):
        u = (event.x - 50) / 50
        v = 7 - (event.y - 50) / 50
        x = int(u)
        y = int(v)
        while self.player_board.board[x][y] == ".":
            self.place(x, y, 0)
            self.player_board.placeMove((x, y), "x")
            self.reflex_board.move((x, y), 0)
            if self.player_board.endGame():
                self.win0()


    def next(self):
        (win, spot2) = self.reflex_agent.reflex_move(self.reflex_board)
        print(win)
        print(spot2)
        if win != 2:
            self.player_board.placeMove(spot2, "o")
            self.place(spot2[0], spot2[1], 1)
        else:
            tkinter.messagebox.askokcancel("Game over", "Tie!")
        if win == 1:
            self.win1()

    def start(self):

        self.chess_board.pack()
        self.chess_board.bind("<Button-1>", self.click)
        Button(self.root, text="next", command=self.next).place(x=200, y=410)
        self.root.mainloop()


if __name__ == '__main__':
    game = Game()
    game.start()
