#!/usr/bin/env python
# coding: utf-8

# In[1]:


import abc
import copy
import time
import heapq
import threading
import pickle
from PIL import Image, ImageTk
from tkinter import Tk, W, E
from tkinter.ttk import Frame, Button, Entry, Label, Style
#from sklearn.neural_network import MLPRegressor
#from sklearn.preprocessing import StandardScaler



#def loadModel():
#    return pickle.load(open("pickle_model.pkl", 'rb'))

#model = loadModel()

class Sokobon:
    """
    Know its observers. Any number of Observer objects may observe a
    subject.
    Send a notification to its observers when its state changes.
    """

    def __init__(self, initialBoard):
        self._observers = set()
        self.initialBoard = initialBoard
        self.goalBoard = None
        self._sokobon_state = None
        self.initialPlayerX = None
        self.initialPlayerY = None
        self.initialState = None
        self.currentState = None
        self.initGame()

    
    def initGame(self):

        goalsPoses = []
        boxsPoses = []

        ##
        self.goalBoard = [' ']*len(self.initialBoard)
        for i in range(0, len(self.initialBoard)):
            self.goalBoard[i] = [' ']*len(self.initialBoard[0])
        ##
        for i in range(0,len(self.initialBoard)):
            for j in range(0, len(self.initialBoard[0])):
                ch = self.initialBoard[i][j]
                if ch == '.':
                    self.goalBoard[i][j] = '&'
                    goalsPoses.append(Point(j,i))
                elif ch == '$':
                    self.goalBoard[i][j] = ' '
                    boxsPoses.append(Point(j,i))
                elif ch == '@':
                    self.goalBoard[i][j] = ' '
                    self.initialPlayerX = j
                    self.initialPlayerY = i
                else:
                    self.goalBoard[i][j] = ch

        self.initialState = State(self.initialBoard, Point(self.initialPlayerX, self.initialPlayerY), boxsPoses, goalsPoses)
        self.currentState = self.initialState
        self.initialState.cost = 0
    

    def isSolved(self, trialBoard):
        for i in range(0, len(trialBoard)):
            for j in range(0, len(trialBoard[0])):
                if trialBoard[i][j]== '@':
                    continue
                if trialBoard[i][j]!= self.goalBoard[i][j]:
                    return False;
        return True

    def move(self,x,y,dx,dy,state):
        board = state.board

        ## Initializind 2D new Board
        newBoard= [' ']*len(board)
        for i in range(0,len(board)):
            newBoard[i] = [' ']*len(board[0])
        newBoard = copy.deepcopy(board)
        ##
        if (board[y+dy][x+dx] != ' ') & (board[y+dy][x+dx] != '.'):
            return None
        if board[y][x] == '+':
            newBoard[y][x] = '.'
        else:
            newBoard[y][x] = ' '

        if board[y+dy][x+dx] == ' ':
            newBoard[y+dy][x+dx] = '@'
        elif board[y+dy][x+dx] == '.':
            newBoard[y+dy][x+dx] = '+'

        newState = State(newBoard, Point(x+dx, y+dy), state.goalsPos, state.boxsPos)
        self.currentState = newState
        return newState
            

    def push(self,x, y, dx, dy, state):
        board = state.board
        newBoard = copy.deepcopy(board)
        newx = x + dx
        newy = y + dy
        if (newBoard[newy][newx] != '$') & (newBoard[newy][newx] != '&'):
            return None
        if (newBoard[newy + dy][newx + dx] != ' ') & (newBoard[newy + dy][newx + dx]!= '.'):
            return None
        if board[y][x]=='+':
            newBoard[y][x] = '.'
        else:
            newBoard[y][x] = ' '
            

        if board[y + dy][x + dx]=='$':
            newBoard[y + dy][x + dx] = '@'
        elif board[y+dy][x+dx]=='&':
            newBoard[y+dy][x+dx] = '+'

        if board[newy + dy][newx + dx]==' ':
            newBoard[newy + dy][newx + dx] = '$'
        elif board[newy + dy][newx + dx]=='.':
            newBoard[newy + dy][newx + dx] = '&'

        boxsPos = state.boxsPos
        for pos in boxsPos:
            if pos.x == x+dx & pos.y == y+dy:
                pos.x = newx+dx
                pos.y = newy+dy

        newState = State(newBoard, Point(x+dx, y+dy), state.goalsPos, boxsPos)
        self.currentState = newState
        return newState

    def solve(self, sokobon):
        initialState = sokobon.initialState
        if sokobon.isSolved(initialState.board):
            solution = []
            solution.append(Result(initialState.board,0,0))
            return solution

        frontier = []
        explored = []
        heapq.heapify(frontier)

        heapq.heappush(frontier, initialState)
        explored.append(initialState)
        extendedNum = 0
        while(len(frontier)!=0):
            extendedNum += 1
            node = heapq.heappop(frontier)
            if sokobon.isSolved(node.board):
                states = []
                results = []

                p = node
                i = 0
                while p!=None:
                    states.append(p)
                    r = Result(p.board, p.cost, extendedNum)
                    results.append(r)
                    p = p.parent
                    i += 1

                return results
            
            explored.append(node)
            childs = []

            firstState = self.move(node.playerPos.x, node.playerPos.y, 0, -1, node)
            secondState = self.move(node.playerPos.x, node.playerPos.y, 0, 1, node)
            thirdState = self.move(node.playerPos.x, node.playerPos.y, -1, 0, node)
            fourthState = self.move(node.playerPos.x, node.playerPos.y, 1, 0, node)

            fifthState = self.push(node.playerPos.x, node.playerPos.y, 0, -1, node)
            sixthState = self.push(node.playerPos.x, node.playerPos.y, 0, 1, node)
            seventhState = self.push(node.playerPos.x, node.playerPos.y, -1, 0, node)
            eightthState = self.push(node.playerPos.x, node.playerPos.y, 1, 0, node)

            if firstState != None:
                childs.append(firstState)
            if secondState != None:
                childs.append(secondState)
            if thirdState != None:
                childs.append(thirdState)
            if fourthState != None:
                childs.append(fourthState)
            if fifthState != None:
                childs.append(fifthState)
            if sixthState != None:
                childs.append(sixthState)
            if seventhState != None:
                childs.append(seventhState)
            if eightthState != None:
                childs.append(eightthState)

            for s in childs:
                if (not(s in explored)) & (not(s in frontier)):
                    s.cost = node.cost + 1
                    heapq.heappush(frontier, s)
                    s.parent = node
                elif s in frontier:
                    for it in frontier:
                        board = it.board
                        board1 = s.board
                        finalCheck = True
                        for i in range(0, len(board)):
                            for j in range(0, len(board[0])):
                                if board[i][j] != board1[i][j]:
                                    finalCheck = False
                    
        results= []
        results.append(Result(initialState.board, -1, extendedNum))
        return results


    def showSolution(self, sokobon):
        t1 = time.time()
        results = self.solve(sokobon)
        t2 = time.time()
        print ("Time taken to search in seconds")
        print (t2-t1)
        results.reverse()
        print ("Finished Searching.")
        print ("Extended Number of nodes = ")
        print (results[len(results)-1].extendedNumber)
        print ("Path length: ")
        print (results[len(results)-1].cost)
        #self.popupmsg("Finished Searching. Time taken to search" + str(t2-t1) + " seconds.")
        for r in results:
            self._sokobon_state = r.board
            self._notify()
            time.sleep(0.3)
        print ("Finished the show")
        
    def popupmsg(self, msg):
        popup = Tk()
        popup.wm_title("Sokobon")
        label = Label(popup, text=msg)
        label.pack(side="top", fill="x", pady=10)
        B1 = Button(popup, text="Start displaying", command = popup.destroy)
        B1.pack()
        popup.mainloop()
    
    def attach(self, observer):
        observer._subject = self
        self._observers.add(observer)

    def detach(self, observer):
        observer._subject = None
        self._observers.discard(observer)

    def _notify(self):
        for observer in self._observers:
            observer.update(self._sokobon_state)

    @property
    def sokobon_state(self):
        return self._sokobon_state

    @sokobon_state.setter
    def sokobon_state(self, arg):
        self._sokobon_state = arg
        self._notify()
class State:

    def __init__(self, board, playerPos, boxsPos, goalsPos):
        self.board = board
        self.playerPos = playerPos
        self.boxsPos = boxsPos
        self.goalsPos = goalsPos
        self.parent = None
        self.cost = None
        #self.heuristic = 0
        self.heuristic = (abs(playerPos.x - boxsPos[0].x) + abs(playerPos.y - boxsPos[0].y))+ (abs(goalsPos[0].x - boxsPos[0].x) + abs(goalsPos[0].y - boxsPos[0].y))
    def __lt__(self, other):
        return self.cost + self.heuristic < other.cost+other.heuristic
    def areTheSame(self, goalBoard, board):
        for i in range(0,len(goalBoard)):
            for j in range(0, len(goalBoard[0])):
                if goalBoard[i][j] != board[i][j]:
                    return False
        return True
    def __eq__(self,other):
        if other == None:
            return False
        return self.areTheSame(self.board, other.board)

class Point:

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return "X: "+ str(x)+ " Y: "+str(y)

class Result:

    def __init__(self, board, cost, extendedNumber):
        self.board = board
        self.cost = cost
        self.extendedNumber = extendedNumber
    def __str__(self):
        return "Number extended = "+  self.extendedNumber + " Cost = "+ self.cost;
        

        




class Observer(metaclass=abc.ABCMeta):
    """
    Define an updating interface for objects that should be notified of
    changes in a subject.
    """

    def __init__(self):
        self._subject = None
        self._observer_state = None
        self.labelsTable = None
        self.cellsTable = None

    @abc.abstractmethod
    def update(self, arg):
        pass

    
class ConcreteObserver(Frame, Observer, threading.Thread):
    """
    Implement the Observer updating interface to keep its state
    consistent with the subject's.
    Store state that should stay consistent with the subject's.
    """
    def __init__(self):
        Frame.__init__(self)
        Observer.__init__(self)
        self.initUI()

    def initUI(self):
      
        self.master.title("Sokobon")        
        #Style().configure("Sokobon")
        
        self.columnconfigure(0)
        self.columnconfigure(1)
        self.columnconfigure(2, pad=0)
        self.columnconfigure(3, pad=0)
        self.columnconfigure(4, pad=0)
        self.columnconfigure(5, pad=0)
        self.columnconfigure(6, pad=0)
        self.columnconfigure(7, pad=0)
        self.columnconfigure(8, pad=0)
        
        self.rowconfigure(0, pad=0)
        self.rowconfigure(1, pad=0)
        self.rowconfigure(2, pad=0)
        self.rowconfigure(3, pad=0)
        self.rowconfigure(4, pad=0)
        self.rowconfigure(5, pad=0)
        self.rowconfigure(6, pad=0)
        self.rowconfigure(7, pad=0)
        self.rowconfigure(8, pad=0)
        self.rowconfigure(9, pad=0)
        
        #entry = Entry(self)

        #entry.grid(row=0, columnspan=4, sticky=W+E)
        
        image10 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label10 = Label(self, image=image10)
        label10.image = image10
        label10.grid(row=1, column=0)
        
        image11 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label11 = Label(self, image=image11)
        label11.image = image11
        label11.grid(row=1, column=1)

        
        image12 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label12 = Label(self, image=image12)
        label12.image = image12
        label12.grid(row=1, column=2)
        
        image13 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label13 = Label(self, image=image13)
        label13.image = image13
        label13.grid(row=1, column=3)
        
        image14 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label14 = Label(self, image=image14)
        label14.image = image14
        label14.grid(row=1, column=4)

        image15 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label15 = Label(self, image=image15)
        label15.image = image15
        label15.grid(row=1, column=5)

        image16 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label16 = Label(self, image=image16)
        label16.image = image16
        label16.grid(row=1, column=6)

        image17 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label17 = Label(self, image=image17)
        label17.image = image17
        label17.grid(row=1, column=7)

        image18 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label18 = Label(self, image=image18)
        label18.image = image18
        label18.grid(row=1, column=8)

        ## Second Row
        image20 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label20 = Label(self, image=image20)
        label20.image = image20
        label20.grid(row=2, column=0)
        
        image21 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label21 = Label(self, image=image21)
        label21.image = image21
        label21.grid(row=2, column=1)

        
        image22 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label22 = Label(self, image=image22)
        label22.image = image22
        label22.grid(row=2, column=2)
        
        image23 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label23 = Label(self, image=image23)
        label23.image = image23
        label23.grid(row=2, column=3)
        
        image24 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label24 = Label(self, image=image24)
        label24.image = image24
        label24.grid(row=2, column=4)

        image25 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label25 = Label(self, image=image25)
        label25.image = image25
        label25.grid(row=2, column=5)

        image26 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label26 = Label(self, image=image26)
        label26.image = image26
        label26.grid(row=2, column=6)

        image27 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label27 = Label(self, image=image27)
        label27.image = image27
        label27.grid(row=2, column=7)

        image28 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label28 = Label(self, image=image28)
        label28.image = image28
        label28.grid(row=2, column=8)

        ####Third Row
        image30 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label30 = Label(self, image=image30)
        label30.image = image30
        label30.grid(row=3, column=0)
        
        image31 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label31 = Label(self, image=image31)
        label31.image = image31
        label31.grid(row=3, column=1)

        
        image32 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label32 = Label(self, image=image32)
        label32.image = image32
        label32.grid(row=3, column=2)
        
        image33 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label33 = Label(self, image=image33)
        label33.image = image33
        label33.grid(row=3, column=3)
        
        image34 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label34 = Label(self, image=image34)
        label34.image = image34
        label34.grid(row=3, column=4)

        image35 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label35 = Label(self, image=image35)
        label35.image = image35
        label35.grid(row=3, column=5)

        image36 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label36 = Label(self, image=image36)
        label36.image = image36
        label36.grid(row=3, column=6)

        image37 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label37 = Label(self, image=image37)
        label37.image = image37
        label37.grid(row=3, column=7)

        image38 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label38 = Label(self, image=image38)
        label38.image = image38
        label38.grid(row=3, column=8)

        ####Fourth Row
        image40 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label40 = Label(self, image=image40)
        label40.image = image40
        label40.grid(row=4, column=0)
        
        image41 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label41 = Label(self, image=image41)
        label41.image = image41
        label41.grid(row=4, column=1)

        
        image42 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label42 = Label(self, image=image42)
        label42.image = image42
        label42.grid(row=4, column=2)
        
        image43 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label43 = Label(self, image=image43)
        label43.image = image43
        label43.grid(row=4, column=3)
        
        image44 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label44 = Label(self, image=image44)
        label44.image = image44
        label44.grid(row=4, column=4)

        image45 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label45 = Label(self, image=image45)
        label45.image = image45
        label45.grid(row=4, column=5)

        image46 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label46 = Label(self, image=image46)
        label46.image = image46
        label46.grid(row=4, column=6)

        image47 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label47 = Label(self, image=image47)
        label47.image = image47
        label47.grid(row=4, column=7)

        image48 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label48 = Label(self, image=image48)
        label48.image = image48
        label48.grid(row=4, column=8)

        ####Fifth Row
        image50 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label50 = Label(self, image=image50)
        label50.image = image50
        label50.grid(row=5, column=0)
        
        image51 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label51 = Label(self, image=image51)
        label51.image = image51
        label51.grid(row=5, column=1)

        
        image52 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label52 = Label(self, image=image52)
        label52.image = image52
        label52.grid(row=5, column=2)
        
        image53 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label53 = Label(self, image=image53)
        label53.image = image53
        label53.grid(row=5, column=3)
        
        image54 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label54 = Label(self, image=image54)
        label54.image = image54
        label54.grid(row=5, column=4)

        image55 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label55 = Label(self, image=image55)
        label55.image = image55
        label55.grid(row=5, column=5)

        image56 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label56 = Label(self, image=image56)
        label56.image = image56
        label56.grid(row=5, column=6)

        image57 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label57 = Label(self, image=image57)
        label57.image = image57
        label57.grid(row=5, column=7)

        image58 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label58 = Label(self, image=image58)
        label58.image = image58
        label58.grid(row=5, column=8)

        ####Sixth Row
        image60 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label60 = Label(self, image=image60)
        label60.image = image60
        label60.grid(row=6, column=0)
        
        image61 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label61 = Label(self, image=image61)
        label61.image = image61
        label61.grid(row=6, column=1)

        
        image62 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label62 = Label(self, image=image62)
        label62.image = image62
        label62.grid(row=6, column=2)
        
        image63 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label63 = Label(self, image=image63)
        label63.image = image63
        label63.grid(row=6, column=3)
        
        image64 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label64 = Label(self, image=image64)
        label64.image = image64
        label64.grid(row=6, column=4)

        image65 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label65 = Label(self, image=image65)
        label65.image = image65
        label65.grid(row=6, column=5)

        image66 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label66 = Label(self, image=image66)
        label66.image = image66
        label66.grid(row=6, column=6)

        image67 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label67 = Label(self, image=image67)
        label67.image = image67
        label67.grid(row=6, column=7)

        image68 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label68 = Label(self, image=image68)
        label68.image = image68
        label68.grid(row=6, column=8)

        ####Seventh Row
        image70 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label70 = Label(self, image=image70)
        label70.image = image70
        label70.grid(row=7, column=0)
        
        image71 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label71 = Label(self, image=image71)
        label71.image = image71
        label71.grid(row=7, column=1)

        
        image72 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label72 = Label(self, image=image72)
        label72.image = image72
        label72.grid(row=7, column=2)
        
        image73 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label73 = Label(self, image=image73)
        label73.image = image73
        label73.grid(row=7, column=3)
        
        image74 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label74 = Label(self, image=image74)
        label74.image = image74
        label74.grid(row=7, column=4)

        image75 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label75 = Label(self, image=image75)
        label75.image = image75
        label75.grid(row=7, column=5)

        image76 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label76 = Label(self, image=image76)
        label76.image = image76
        label76.grid(row=7, column=6)

        image77 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label77 = Label(self, image=image77)
        label77.image = image77
        label77.grid(row=7, column=7)

        image78 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label78 = Label(self, image=image78)
        label78.image = image78
        label78.grid(row=7, column=8)

        ####Eightth Row
        image80 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label80 = Label(self, image=image80)
        label80.image = image80
        label80.grid(row=8, column=0)
        
        image81 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label81 = Label(self, image=image81)
        label81.image = image81
        label81.grid(row=8, column=1)

        
        image82 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label82 = Label(self, image=image82)
        label82.image = image82
        label82.grid(row=8, column=2)
        
        image83 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label83 = Label(self, image=image83)
        label83.image = image83
        label83.grid(row=8, column=3)
        
        image84 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label84 = Label(self, image=image84)
        label84.image = image84
        label84.grid(row=8, column=4)

        image85 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label85 = Label(self, image=image85)
        label85.image = image85
        label85.grid(row=8, column=5)

        image86 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label86 = Label(self, image=image86)
        label86.image = image86
        label86.grid(row=8, column=6)

        image87 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label87 = Label(self, image=image87)
        label87.image = image87
        label87.grid(row=8, column=7)

        image88 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label88 = Label(self, image=image88)
        label88.image = image88
        label88.grid(row=8, column=8)

        ####Nineth Row
        image90 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label90 = Label(self, image=image90)
        label90.image = image90
        label90.grid(row=9, column=0)
        
        image91 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label91 = Label(self, image=image91)
        label91.image = image91
        label91.grid(row=9, column=1)

        
        image92 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label92 = Label(self, image=image92)
        label92.image = image92
        label92.grid(row=9, column=2)
        
        image93 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label93 = Label(self, image=image93)
        label93.image = image93
        label93.grid(row=9, column=3)
        
        image94 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label94 = Label(self, image=image94)
        label94.image = image94
        label94.grid(row=9, column=4)

        image95 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label95 = Label(self, image=image95)
        label95.image = image95
        label95.grid(row=9, column=5)

        image96 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label96 = Label(self, image=image96)
        label96.image = image96
        label96.grid(row=9, column=6)

        image97 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label97 = Label(self, image=image97)
        label97.image = image97
        label97.grid(row=9, column=7)

        image98 = ImageTk.PhotoImage(Image.open("Sokobon/Sokobon_Block.png"))
        label98 = Label(self, image=image98)
        label98.image = image98
        label98.grid(row=9, column=8)
        
        ####### Initializing Table
        table = {}
        table["label10"] = label10
        table["label11"] = label11
        table["label12"] = label12
        table["label13"] = label13
        table["label14"] = label14
        table["label15"] = label15
        table["label16"] = label16
        table["label17"] = label17
        table["label18"] = label18

        table["label20"] = label20
        table["label21"] = label21
        table["label22"] = label22
        table["label23"] = label23
        table["label24"] = label24
        table["label25"] = label25
        table["label26"] = label26
        table["label27"] = label27
        table["label28"] = label28

        table["label30"] = label30
        table["label31"] = label31
        table["label32"] = label32
        table["label33"] = label33
        table["label34"] = label34
        table["label35"] = label35
        table["label36"] = label36
        table["label37"] = label37
        table["label38"] = label38

        table["label40"] = label40
        table["label41"] = label41
        table["label42"] = label42
        table["label43"] = label43
        table["label44"] = label44
        table["label45"] = label45
        table["label46"] = label46
        table["label47"] = label47
        table["label48"] = label48

        table["label50"] = label50
        table["label51"] = label51
        table["label52"] = label52
        table["label53"] = label53
        table["label54"] = label54
        table["label55"] = label55
        table["label56"] = label56
        table["label57"] = label57
        table["label58"] = label58

        table["label60"] = label60
        table["label61"] = label61
        table["label62"] = label62
        table["label63"] = label63
        table["label64"] = label64
        table["label65"] = label65
        table["label66"] = label66
        table["label67"] = label67
        table["label68"] = label68

        table["label70"] = label70
        table["label71"] = label71
        table["label72"] = label72
        table["label73"] = label73
        table["label74"] = label74
        table["label75"] = label75
        table["label76"] = label76
        table["label77"] = label77
        table["label78"] = label78

        table["label80"] = label80
        table["label81"] = label81
        table["label82"] = label82
        table["label83"] = label83
        table["label84"] = label84
        table["label85"] = label85
        table["label86"] = label86
        table["label87"] = label87
        table["label88"] = label88

        table["label90"] = label90
        table["label91"] = label91
        table["label92"] = label92
        table["label93"] = label93
        table["label94"] = label94
        table["label95"] = label95
        table["label96"] = label96
        table["label97"] = label97
        table["label98"] = label98

        self.labelsTable = table

        ## Cells Table

        table_1 = {}
        table_1['@'] = "Sokobon/Sokobon_Player.png"
        table_1[' '] = "Sokobon/Sokobon_Empty.png"
        table_1['#'] = "Sokobon/Sokobon_Block.png"
        table_1['$'] = "Sokobon/Sokobon_Box.png"
        table_1['.'] = "Sokobon/Sokobon_Goal.png"
        table_1['&'] = "Sokobon/Sokobon_BoxOnGoal.png"
        table_1['+'] = "Sokobon/Sokobon_PlayerOnGoal.png"
        
        self.cellsTable = table_1
        
        

        ###### Packing
        self.pack()
        #######

           
    def changeImage(self, label, path):
        img2 = ImageTk.PhotoImage(Image.open(path))
        label.configure(image=img2)
        label.image = img2
        label.update_idletasks()

    """def drawGame(self, maze):
        for i in range(0,len(maze)):
            for j in range(0,len(maze[0])):
                label = self.getLabelsTable()["label"+str(i+1)+str(j)]
                self.changeImage(label, self.getCellsTable()[maze[i][j]])"""
    def drawGame(self, maze):
        for i in range(1,len(maze)-1):
            for j in range(1,len(maze[0])-1):
                label = self.getLabelsTable()["label"+str(i)+str(j-1)]
                self.changeImage(label, self.getCellsTable()[maze[i][j]])
                
    def getCellsTable(self):
        return self.cellsTable
    def getLabelsTable(self):
        return self.labelsTable
    def update(self, arg):
        self._observer_state = arg
        self.drawGame(arg)
        # ...


def tryMaze(sokobon):
    sokobon.showSolution(sokobon)

        

    
def main():

    mazeStrings = [
            "###########",
            "##        #",
            "#  $ #    #",
            "###  #    #",
            "##   # #  #",
            "##      # #",
            "##     ####",
            "#####  #.@#",
            "#         #",
            "# ####    #",
            "###########"]
    mazeChars = []
    for s in mazeStrings:
        mazeChars.append(list(s))
    sokobon = Sokobon(mazeChars)
    sokobon.sokobon_state = mazeChars
    concrete_observer = ConcreteObserver()
    concrete_observer.drawGame(mazeChars)
    sokobon.attach(concrete_observer)
    root = Tk()
    B = Button(root, text ="Solve", command = lambda: tryMaze(sokobon))
    B.pack()
    
    ######
    ######
    root.mainloop()


if __name__ == "__main__":
    main()


# In[ ]:




