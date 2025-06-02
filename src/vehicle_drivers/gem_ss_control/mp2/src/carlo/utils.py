import numpy as np
import time
import pickle
from tkinter import *
import tkinter as tk


# User number list
USERS = [1,2,3,4,5,7,8,9,10,11,12,13,14,15]


# GUI design for the flag window and the score
class Flag(object):
    # def __init__(self, parent):
        # super().__init__(parent)
    def __init__(self):
        self.root = Tk() # the flag window
        self.root.geometry("300x200+920+150")
        self.root.title('Flag')

        # Flag window parameters
        self.flag = ["Ready", "Set", "Go"]
        self.flag_text = Label (self.root, text = "", font=("Georgia", 80), bd = 100)
        self.flag_text.place(relx = 0.5, rely = 0.5, anchor = 'center')

# GUI design for the timer window between scenes
class Timer(object):
    # def __init__(self, parent):
        # super().__init__(parent)
    def __init__(self):
        self.root = Tk() # the flag window
        self.root.geometry("+920+150")
        self.root.title('Timer')

        # Flag window parameters
        self.timer = ["3", "2", "1"]
        self.timer_text = Label (self.root, text = "", font=("Georgia", 100), bd = 100)
        self.timer_text.place(relx = 0.5, rely = 0.5, anchor = 'center')

class H_score(object):
    def __init__(self):
        self.root = Tk() # the score window
        self.root.geometry("+700+100")
        self.root.title("Highway Score")
        self.update_time = 0
        font = "Georgia"

        # Running Highway Score
        myLabel1 = Label(self.root, text = "Highway Running Score", font=(font, 40))
        myLabel1.grid(row = 0, column = 0, pady = 50, padx = 50)
        self.textbox1 = Entry(self.root, width = 7, bg = "white", fg = "#676767", borderwidth = 3, font=(font, 40))
        self.textbox1.grid(row = 0, column = 1,  pady = 10, padx = 20)
        self.textbox1.insert(0,0)

        # Cumulative Highway Score
        myLabel2 = Label(self.root, text = "Total Score", font=(font, 40))
        myLabel2.grid(row = 1, column = 0, pady = 50, padx = 50)
        self.textbox2 = Entry(self.root, width = 7, bg = "white", fg = "#676767", borderwidth = 3, font=(font, 40))
        self.textbox2.grid(row = 1, column = 1,  pady = 10, padx = 20)
        self.textbox2.insert(0,0)

class I_score(object):
    def __init__(self):
        self.root = Tk() # the score window
        self.root.geometry("+700+100")
        self.root.title("Intersection Score")
        self.update_time = 0
        font = "Georgia"
        
        # Intersection Highway Score
        myLabel3 = Label(self.root, text = "Intersection Running Score", font=(font, 40))
        myLabel3.grid(row = 0, column = 0, pady = 50, padx = 50)
        self.textbox3 = Entry(self.root, width = 7, bg = "white", fg = "#676767", borderwidth = 3, font=(font, 40))
        self.textbox3.grid(row = 0, column = 1,  pady = 10, padx = 20)
        self.textbox3.insert(0,0)

        # Cumulative Intersection Score
        myLabel4 = Label(self.root, text = "Total Score", font=(font, 40))
        myLabel4.grid(row = 1, column = 0, pady = 50, padx = 50)
        self.textbox4 = Entry(self.root, width = 7, bg = "white", fg = "#676767", borderwidth = 3, font=(font, 40))
        self.textbox4.grid(row = 1, column = 1,  pady = 10, padx = 20)
        self.textbox4.insert(0,0)

class R_score(object):
    def __init__(self):
        self.root = Tk() # the score window
        self.root.geometry("+700+100")
        self.root.title("Roundabout Score")
        self.update_time = 0
        font = "Georgia"

        # Roundabout Highway Score
        myLabel5 = Label(self.root, text = "Roundabout Running Score", font=(font, 40))
        myLabel5.grid(row = 0, column = 0, pady = 50, padx = 50)
        self.textbox5 = Entry(self.root, width = 7, bg = "white", fg = "#676767", borderwidth = 3, font=(font, 40))
        self.textbox5.grid(row = 0, column = 1,  pady = 10, padx = 20)
        self.textbox5.insert(0,0)

        # Cumulative Roundabout Score
        myLabel6 = Label(self.root, text = "Total Score", font=(font, 40))
        myLabel6.grid(row = 1, column = 0, pady = 50, padx = 50)
        self.textbox6 = Entry(self.root, width = 7, bg = "white", fg = "#676767", borderwidth = 3, font=(font, 40))
        self.textbox6.grid(row = 1, column = 1,  pady = 10, padx = 20)
        self.textbox6.insert(0,0)

