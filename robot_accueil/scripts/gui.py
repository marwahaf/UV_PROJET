#!/usr/bin/python3
import tkinter as tk
from tkinter import *
import tkinter.font as font
from turtle import down, heading
from typing_extensions import Self
from xml.etree.ElementInclude import include
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped


class MyNode:

    def __init__(self, master):
    #Creation of the Ros Subscribers and publishers for the good topics
        self.sub = rospy.Subscriber("person", PointStamped, callback=self.people)
        self.arrived = rospy.Subscriber("/goal/arrived", PoseStamped, callback=self.page2)
        self.homed = rospy.Subscriber("/goal/home_returned", PoseStamped, callback=self.welcomePage)
        self.pub = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1)
        self.pub2 = rospy.Publisher("/goal/returnhome",PoseStamped,queue_size=1)

    #getting the frame of the GUI
        self.root = master
        self.root.attributes('-fullscreen', False)     #Set True for fullscreen
        self.root.configure(background='gray')
        self.fullScreenState = False   
        #creating frames for objects placement after
        self.leftFrame = Frame(self.root,background = 'blue')
        self.leftFrame.pack(side=LEFT,fill=Y,ipadx = 5)
        self.rightFrame = Frame(self.root,background= 'red')    
        self.rightFrame.pack(side = RIGHT,fill = Y,ipadx = 5)
        self.upFrame = Frame(self.root, background = 'yellow')
        self.upFrame.pack(side = TOP, fill = X,ipady = 5)
        self.downFrame = Frame(self.root,background='green')
        self.downFrame.pack(side = BOTTOM,fill= X,ipady = 5)
        self.master = Frame(self.root, background='white')
        self.master.pack()

        #setting the font in the buttons
        self.buttonFont= font.Font(family='Times',size=20)
        self.root.bind("<F11>", self.toggleFullScreen)  #Binds F11 for full screen mode
        self.root.bind("<Escape>", self.quitFullScreen) #Binds ESCAPE For quitting fullscreen mode
        
        #Define important buttons
        self.quitButton = Button(self.rightFrame,text="dbg : quit",font = self.buttonFont,command = self.root.destroy)
        self.quitButton.pack(side =BOTTOM, pady=5,padx=5)
        self.stopButton = Button(self.leftFrame, text = "Leave Robot",font=self.buttonFont, command= lambda:self.sendGoal(0,0))
        self.stopButton.pack(side=BOTTOM,pady=5,padx=5)
        self.welcomePage() #Go to welcome page
        
    #Sends a goal with a b coords
    def sendGoal(self, a,b):
        tmp_goal = PoseStamped()
        tmp_goal.header.frame_id = 'map'
        tmp_goal.pose.position.x = a
        tmp_goal.pose.position.y = b
        print(tmp_goal)
        self.pub.publish(tmp_goal)

    #sends the returnhome goal topic to makes the robot returning home
    def returnhome(self):
        tmp_goal = PoseStamped()
        self.pub2.publish(tmp_goal)

    #Defines the welcome page, the first page printed
    def welcomePage(self):
    #Sets the first page of the GUI that the user see when entering.
        for w in self.master.winfo_children():
            w.destroy()
        self.label = tk.Label(self.master, text="Where to go ? ",font=('Times',30),bg = 'white')
        self.label.pack(anchor='center',pady=10)
        self.button = Button(self.master, text="Goal 1", font=self.buttonFont, command= lambda:self.sendGoal(6, 0))
        self.button.pack()
        self.button2 = Button(self.master, text="Goal 2", font=self.buttonFont, command= lambda:self.sendGoal(7,2))
        self.button2.pack()


    def people(self,point):
        #When a "/people" topic is received, put the "starting" gui
        self.welcomePage()

    #when the robot is arrived at goal sets a new page
    def page2(self, data):
        for w in self.master.winfo_children():
            w.destroy()
        self.label = tk.Label(self.master, text="C'est la page 2 ",font=('Times',30),bg = 'white')
        self.label.pack(anchor='center',pady=10)
        self.room2 = Button(self.master,text="Return Home", font= self.buttonFont, command = lambda:self.sendGoal(0,0))
        self.room2.pack()
        self.room2 = Button(self.master,text="New goal", font= self.buttonFont, command = self.welcomePage)
        self.room2.pack()
        
    #if F11 is used
    def toggleFullScreen(self, event):
        self.fullScreenState = not self.fullScreenState
        self.root.attributes("-fullscreen", self.fullScreenState)

    #if ESCAPE is used
    def quitFullScreen(self, event):
        self.fullScreenState = False
        self.root.attributes("-fullscreen", self.fullScreenState)

rospy.init_node('my_test',anonymous=True)
root = tk.Tk()
mynode = MyNode(root)
root.mainloop()
