import pygame_widgets
import pygame
from pygame_widgets.textbox import TextBox
import time
import numpy as np
import math
import sys, threading, queue, serial
import serial.tools.list_ports
import struct
from pathlib import Path

# Arduino Class (Used for interfacing)
class Arduino:
    # Defines class attributes
    def __init__(self):
        self.connectedPort = ""
        self.portOptions = []
        self.connectedDevice = None
        self.isConnected = False
    
    # Connects to selected port (0: Unable to connect 1: Connected successfully)
    def connect(self, baudRate = 1000000, timeout = 10):
        self.isConnected = False

        # Finds the correct port
        port = serial.tools.list_ports.comports()
        self.connectedPort = port[0]
        for i in list(port):
            if "Arduino Mega 2560" in i.description:
                self.connectedPort = i
        self.connectedDevice = self.connectedPort.device
        self.connectedDevice = serial.Serial(self.connectedDevice, baudrate=baudRate, timeout=.1)

        # Checks to see if the device is correct
        message = b''
        refTime = time.time()
        while time.time()-refTime < timeout:
            incoming = self.connectedDevice.read()
            if incoming == b'\n':
                try:
                    message = message.decode('utf-8').strip()
                    if message == "CNC Ready":
                        self.isConnected = True
                        return 1
                    else:
                        return 0
                except:
                    return 0
            else:
                if incoming not in (b'', b'\r'):
                    message += incoming
        return 0

    # Writes message to arduino
    def write(self, message):
        if self.isConnected:
            self.connectedDevice.write(struct.pack('f', message))

    # Read message from arduino
    def read(self, timeout = 1):
        if self.isConnected:
            message = b''
            refTime = time.time()
            while time.time()-refTime < timeout:
                incoming = self.connectedDevice.read()
                if len(incoming) >= 1:
                    message += incoming
                if len(message) == 4:
                    try:
                        return struct.unpack('f', message)[0]
                    except:
                        raise Exception("Invalid message")
            raise Exception("Timeout")
        raise Exception("Not Connected")

# CNC Class (All units in mm)
class CNC:
    def __init__(self, minSpeed, maxSpeed, startingFeedRate, maxAcceleration, spindleMaxSpeed, pathAttraction, tolerance, divisions):
        # Variables to store speed
        self.velocity = np.array([0.0, 0.0, 0.0])

        # Variables to store position
        self.position = np.array([0.0, 0.0, 0.0])

        # Variables to store current target
        self.target = np.array([0.0, 0.0, 0.0])

        # Variables to store previous target point
        self.prevTarget = np.array([0.0, 0.0, 0.0])
        self.prevTarget2 = np.array([0.0, 0.0, 0.0])

        # Variable to store the path as a list of strings
        self.path = [""]

        # Variable to store which step the cnc is on
        self.step = 0

        # Variable to store the last update time
        self.lastUpdateTime = -1

        # Variable to store speed and accelerations
        self.minSpeed = minSpeed
        self.maxSpeed = maxSpeed
        self.startingFeedRate = startingFeedRate
        self.feedRate = self.startingFeedRate
        self.maxAcceleration = maxAcceleration

        # Variables to store the spindle speeds
        self.spindleMaxSpeed = spindleMaxSpeed

        # Variable to store tuning parameter
        self.pathAttraction = pathAttraction

        # Variable to store tolerance
        self.tolerance = tolerance

        # Variable to store number of divisions of a path
        self.divisions = divisions

        # Variable to store the feed rate of the last step
        self.prevFeedRate = self.feedRate

        self.arduino = Arduino()

        self.simulate = True

    # Connects to the arduino
    def connect(self):
        return self.arduino.connect()

    # Checks if arduino is connected
    def isConnected(self):
        return self.arduino.isConnected
    
    # Sets whether or not the machine will simulate or not
    def setSimulation(self, state):
        self.simulate = state

    # Sets the velocity of the toolhead
    def setVelocity(self, newVelocity):
        self.velocity = np.copy(newVelocity)
        if (not self.simulate):
            self.arduino.write(6)
            self.arduino.write(newVelocity[0])
            self.arduino.write(newVelocity[1])
            self.arduino.write(newVelocity[2])
            self.arduino.read()

    # Zeroes the position of the toolhead
    def resetPosition(self):
        self.position = np.array([0.0, 0.0, 0.0])
        if (not self.simulate):
            self.arduino.write(0)
            self.arduino.read()
    
    # Gets the velocity of the toolhead
    def getVelocity(self):
        return np.copy(self.velocity)

    # Gets the position of the toolhead
    def getPosition(self):
        return np.copy(self.position)
    
    # Writes a message to the cnc
    def writeCNC(self, command):
        if (not self.simulate):
            self.arduino.write(command)
    
    def readCNC(self, t=5):
        if (not self.simulate):
            self.arduino.read(timeout=t)

    # Updates the position of the toolhead
    def updatePosition(self):
        if (not self.simulate):
            self.arduino.write(1)
            self.position = np.array([self.arduino.read(), self.arduino.read(), self.arduino.read()])
        else:
            timeElapsed = ((time.time_ns()-self.lastUpdateTime)/(10**9))/60
            self.lastUpdateTime = time.time_ns()
            self.position += timeElapsed*self.velocity

    # Sets the speed of the spindle
    def setSpindleSpeed(self, speed):
        if (not self.simulate):
            self.arduino.write(2)
            self.arduino.write(speed)
            self.arduino.read(timeout=2)
    
    # Sets the path of the toolhead
    def setPath(self, p):
        self.path = p.split("\n")

        # Variables to store speed
        self.velocity = np.array([0.0, 0.0, 0.0])

        # Variables to store position
        self.position = np.array([0.0, 0.0, 0.0])

        # Variables to store current target
        self.target = np.array([0.0, 0.0, 0.0])

        # Variables to store previous target point
        self.prevTarget = np.array([0.0, 0.0, 0.0])
        self.prevTarget2 = np.array([0.0, 0.0, 0.0])

        # Variable to store which step the cnc is on
        self.step = 0

        # Variable to store the last update time
        self.lastUpdateTime = -1

        # Variable to store the feed rate of the last step
        self.feedRate = self.startingFeedRate
        self.prevFeedRate = self.feedRate
        
    # Gets a point on the path
    def getPointOnPath(self, p, t):
        # For straight line
        if (p.split()[0] in ["G1", "G01"]):
            # Gets target
            targetTemp = np.copy(self.prevTarget)
            for i in p.split():
                if i[0] == "X":
                    targetTemp[0] = float(i[1:])
                elif i[0] == "Y":
                    targetTemp[1] = float(i[1:])
                elif i[0] == "Z":
                    targetTemp[2] = float(i[1:])
            
            # Finds point on line
            return self.prevTarget+t*(targetTemp-self.prevTarget)
        
        # For clockwise arc
        elif (p.split()[0] in ["G2", "G02"]):
            # Gets center point and target
            center = np.copy(self.prevTarget)
            targetTemp = np.copy(self.prevTarget)
            for i in p.split():
                if i[0] == "X":
                    targetTemp[0] = float(i[1:])
                elif i[0] == "Y":
                    targetTemp[1] = float(i[1:])
                elif i[0] == "Z":
                    targetTemp[2] = float(i[1:])
                elif i[0] == "I":
                    center[0] = float(i[1:])
                elif i[0] == "J":
                    center[1] = float(i[1:])
                elif i[0] == "K":
                    center[2] = float(i[1:])
            
            # Angle between start and end
            angle = math.acos(np.dot(self.prevTarget-center, targetTemp-center)/(np.linalg.norm(self.prevTarget-center)*np.linalg.norm(targetTemp-center)))
            
            # Axis of rotation
            axis = np.cross(targetTemp-center, self.prevTarget-center)
            axis = axis/np.linalg.norm(axis)
            
            # Switches if going opposite directions
            if (np.dot(axis, np.array([0.0, 0.0, 1.0])) < 0):
                angle = -2*math.pi+angle
            angle *= (1-t)
            
            # Caculates rotation with Rodrigues' Rotation Formula
            result = (targetTemp-center)*math.cos(angle)+(np.cross(axis, targetTemp-center))*math.sin(angle)+axis*(np.dot(axis, targetTemp-center))*(1-math.cos(angle))
            return result+center
        
        # For counterclockwise arc
        elif (p.split()[0] in ["G3", "G03"]):
            # Gets center and target
            center = np.copy(self.prevTarget)
            targetTemp = np.copy(self.prevTarget)
            for i in p.split():
                if i[0] == "X":
                    targetTemp[0] = float(i[1:])
                elif i[0] == "Y":
                    targetTemp[1] = float(i[1:])
                elif i[0] == "Z":
                    targetTemp[2] = float(i[1:])
                elif i[0] == "I":
                    center[0] = float(i[1:])
                elif i[0] == "J":
                    center[1] = float(i[1:])
                elif i[0] == "K":
                    center[2] = float(i[1:])
            
            # Caculates angle between starting and ending
            angle = math.acos(np.dot(self.prevTarget-center, targetTemp-center)/(np.linalg.norm(self.prevTarget-center)*np.linalg.norm(targetTemp-center)))
            
            # Finds axis of rotation
            axis = np.cross(targetTemp-center, self.prevTarget-center)
            axis = axis/np.linalg.norm(axis)
            
            # Switches if going opposite direction
            if (np.dot(axis, np.array([0.0, 0.0, -1.0])) < 0):
                angle = -2*math.pi+angle
            angle *= (1-t)
            
            # Caculates rotation with Rodrigues' Rotation Formula
            result = (targetTemp-center)*math.cos(angle)+(np.cross(axis, targetTemp-center))*math.sin(angle)+axis*(np.dot(axis, targetTemp-center))*(1-math.cos(angle))
            return result+center
        
        # For cubic bezier splines
        elif (p.split()[0] in ["G5", "G05"]):
            # Gets points and target for spline
            point1 = np.copy(self.prevTarget)
            point2 = np.copy(self.prevTarget)
            targetTemp = np.copy(self.prevTarget)
            for i in p.split():
                if i[0] == "X":
                    targetTemp[0] = float(i[1:])
                elif i[0] == "Y":
                    targetTemp[1] = float(i[1:])
                elif i[0] == "Z":
                    targetTemp[2] = float(i[1:])
                elif i[0] == "I":
                    point1[0] = float(i[1:])
                    point2[0] = float(i[1:])
                    targetTemp[0] = float(i[1:])
                elif i[0] == "J":
                    point1[1] = float(i[1:])
                    point2[1] = float(i[1:])
                    targetTemp[1] = float(i[1:])
                elif i[0] == "K":
                    point1[2] = float(i[1:])
                    point2[2] = float(i[1:])
                    targetTemp[2] = float(i[1:])
                elif i[0] == "P":
                    point2[0] = float(i[1:])
                    targetTemp[0] = float(i[1:])
                elif i[0] == "Q":
                    point2[1] = float(i[1:])
                    targetTemp[1] = float(i[1:])
                elif i[0] == "R":
                    point2[2] = float(i[1:])
                    targetTemp[2] = float(i[1:])

            # Calculates point on spline
            return (1-t)**3*self.prevTarget+3*t*(1-t)**2*point1+3*t**2*(1-t)*point2+t**3*targetTemp

        else:
            return np.array([0.0, 0.0, 0.0])
    
    # Gets the tangent at time t
    def getTangent(self, stepIndex, t):
        # Tangents of straight line
        if (self.path[stepIndex].split()[0] in ["G1", "G01"]):
            # Gets starting and ending point
            startPos = np.copy(self.prevTarget)
            endPos = np.copy(self.target)
            if (stepIndex < self.step): # Gets previous starting and ending point
                startPos = np.copy(self.prevTarget2)
                endPos = np.copy(self.prevTarget)
            elif (stepIndex > self.step): # Gets next starting and ending points
                startPos = np.copy(self.target)
                for i in self.path[stepIndex].split():
                    if i[0] == "X":
                        endPos[0] = float(i[1:])
                    if i[0] == "Y":
                        endPos[1] = float(i[1:])
                    if i[0] == "Z":
                        endPos[2] = float(i[1:])

            # Calculates unit vector in direction of line
            return (endPos-startPos)/np.linalg.norm(endPos-startPos)
        
        # Tangents of clockwise arc
        elif (self.path[stepIndex].split()[0] in ["G2", "G02"]):
            # Gets starting and center point
            center = np.copy(self.prevTarget)
            targetTemp = np.copy(self.prevTarget)
            initial = np.copy(self.prevTarget)

            # Changes based on posiiton
            if (stepIndex < self.step):
                center = np.copy(self.prevTarget2)
                targetTemp = np.copy(self.prevTarget2)
                initial = np.copy(self.prevTarget2)
            elif (stepIndex > self.step):
                center = np.copy(self.target)
                targetTemp = np.copy(self.target)
                initial = np.copy(self.target)
            
            # Updates points
            for i in self.path[stepIndex].split():
                if i[0] == "X":
                    targetTemp[0] = float(i[1:])
                elif i[0] == "Y":
                    targetTemp[1] = float(i[1:])
                elif i[0] == "Z":
                    targetTemp[2] = float(i[1:])
                elif i[0] == "I":
                    center[0] = float(i[1:])
                elif i[0] == "J":
                    center[1] = float(i[1:])
                elif i[0] == "K":
                    center[2] = float(i[1:])
            
            # Angle between start and end
            angle = math.acos(np.dot(initial-center, targetTemp-center)/(np.linalg.norm(initial-center)*np.linalg.norm(targetTemp-center)))
            
            # Axis of rotation
            axis = np.cross(targetTemp-center, initial-center)
            axis = axis/np.linalg.norm(axis)
            
            # Switches if going opposite directions
            mult = -1
            if (np.dot(axis, np.array([0.0, 0.0, 1.0])) < 0):
                angle = -2*math.pi+angle
                mult = 1
            angle *= (1-t)

            # Take derivative with respect to angle
            result = -(targetTemp-center)*math.sin(angle)+(np.cross(axis, targetTemp-center))*math.cos(angle)+axis*(np.dot(axis, targetTemp-center))*(math.sin(angle))
            return mult*result/np.linalg.norm(result)

        # Tangents of counterclockwise arc
        elif (self.path[stepIndex].split()[0] in ["G3", "G03"]):
            # Gets starting and center point
            center = np.copy(self.prevTarget)
            targetTemp = np.copy(self.prevTarget)
            initial = np.copy(self.prevTarget)

            # Changes based on posiiton
            if (stepIndex < self.step):
                center = np.copy(self.prevTarget2)
                targetTemp = np.copy(self.prevTarget2)
                initial = np.copy(self.prevTarget2)
            elif (stepIndex > self.step):
                center = np.copy(self.target)
                targetTemp = np.copy(self.target)
                initial = np.copy(self.target)
            
            # Updates points
            for i in self.path[stepIndex].split():
                if i[0] == "X":
                    targetTemp[0] = float(i[1:])
                elif i[0] == "Y":
                    targetTemp[1] = float(i[1:])
                elif i[0] == "Z":
                    targetTemp[2] = float(i[1:])
                elif i[0] == "I":
                    center[0] = float(i[1:])
                elif i[0] == "J":
                    center[1] = float(i[1:])
                elif i[0] == "K":
                    center[2] = float(i[1:])

            # Caculates angle between starting and ending
            angle = math.acos(np.dot(initial-center, targetTemp-center)/(np.linalg.norm(initial-center)*np.linalg.norm(targetTemp-center)))
            
            # Finds axis of rotation
            axis = np.cross(targetTemp-center, initial-center)
            axis = axis/np.linalg.norm(axis)
            
            # Switches if going opposite direction
            mult = -1
            if (np.dot(axis, np.array([0.0, 0.0, -1.0])) < 0):
                angle = -2*math.pi+angle
                mult = 1
            angle *= (1-t)

            # Takes derivative with respect to angle
            result = -(targetTemp-center)*math.sin(angle)+(np.cross(axis, targetTemp-center))*math.cos(angle)+axis*(np.dot(axis, targetTemp-center))*(math.sin(angle))
            return mult*result/np.linalg.norm(result)

        # Tangent of cubic bezier curve
        elif (self.path[stepIndex].split()[0] in ["G5", "G05"]):
            # Gets points and target for spline
            initial = np.copy(self.prevTarget)
            point1 = np.copy(self.prevTarget)
            point2 = np.copy(self.prevTarget)
            targetTemp = np.copy(self.prevTarget)

            # Changes based on which step it is
            if (stepIndex < self.step):
                initial = np.copy(self.prevTarget2)
                point1 = np.copy(self.prevTarget2)
                point2 = np.copy(self.prevTarget2)
                targetTemp = np.copy(self.prevTarget2)
            elif (stepIndex > self.step):
                initial = np.copy(self.target)
                point1 = np.copy(self.target)
                point2 = np.copy(self.target)
                targetTemp = np.copy(self.target)

            # Updates points
            for i in self.path[stepIndex].split():
                if i[0] == "X":
                    targetTemp[0] = float(i[1:])
                elif i[0] == "Y":
                    targetTemp[1] = float(i[1:])
                elif i[0] == "Z":
                    targetTemp[2] = float(i[1:])
                elif i[0] == "I":
                    point1[0] = float(i[1:])
                    point2[0] = float(i[1:])
                    targetTemp[0] = float(i[1:])
                elif i[0] == "J":
                    point1[1] = float(i[1:])
                    point2[1] = float(i[1:])
                    targetTemp[1] = float(i[1:])
                elif i[0] == "K":
                    point1[2] = float(i[1:])
                    point2[2] = float(i[1:])
                    targetTemp[2] = float(i[1:])
                elif i[0] == "P":
                    point2[0] = float(i[1:])
                    targetTemp[0] = float(i[1:])
                elif i[0] == "Q":
                    point2[1] = float(i[1:])
                    targetTemp[1] = float(i[1:])
                elif i[0] == "R":
                    point2[2] = float(i[1:])
                    targetTemp[2] = float(i[1:])

            # Returns unit vector in direction of tangent
            derivative = 3*(1-t)**2*(point1-initial)+6*(1-t)*t*(point2-point1)+3*t**2*(targetTemp-point2)
            return derivative/np.linalg.norm(derivative)
        
        return np.array([0.0, 0.0, 0.0]) # Returns zero vector if none of previous were movements
    
    # Gets the max allowed speed at time t
    def getSpeed(self, stepIndex, t):
        # For straight line
        if (self.path[stepIndex].split()[0] in ["G1", "G01"]):
            if (stepIndex < self.step): # Returns last feedrate if previous step
                return self.prevFeedRate
            elif (stepIndex > self.step): # Returns new feedrate if present in the step
                for i in self.path[stepIndex].split():
                    if i[0] == "F":
                        return float(i[1:])
                return self.feedRate # Returns current feedrate if not present in the step
            else:
                return self.feedRate # Returns current feedrate if current step
        
        # For arcs
        elif (self.path[stepIndex].split()[0] in ["G2", "G02", "G3", "G03"]):
            # Gets the center of the arc and the feedrate
            center = np.copy(self.prevTarget)
            targetTemp = np.copy(self.prevTarget)
            tempFeedRate = self.feedRate
            if (stepIndex < self.step): # Sets feedrate to previous step's feedrate
                tempFeedRate = self.prevFeedRate
            elif (stepIndex > self.step): # Finds feedrate of next step
                for i in self.path[stepIndex].split():
                    if i[0] == "F":
                        tempFeedRate = float(i[1:])
            # Gets the target and center
            for i in self.path[stepIndex].split():
                if i[0] == "X":
                    targetTemp[0] = float(i[1:])
                elif i[0] == "Y":
                    targetTemp[1] = float(i[1:])
                elif i[0] == "Z":
                    targetTemp[2] = float(i[1:])
                elif i[0] == "I":
                    center[0] = float(i[1:])
                elif i[0] == "J":
                    center[1] = float(i[1:])
                elif i[0] == "K":
                    center[2] = float(i[1:])
            
            # Calculates the max speed
            return min(tempFeedRate, (self.maxAcceleration*np.linalg.norm(targetTemp-center))**0.5)
    
        # For cubic bezier splines
        elif (self.path[stepIndex].split()[0] in ["G5", "G05"]):
            # Gets points and target for spline
            initial = np.copy(self.prevTarget)
            point1 = np.copy(self.prevTarget)
            point2 = np.copy(self.prevTarget)
            targetTemp = np.copy(self.prevTarget)
            tempFeedRate = self.feedRate

            if (stepIndex < self.step): # Sets feedrate to previous step's feedrate
                tempFeedRate = self.prevFeedRate
            elif (stepIndex > self.step): # Finds feedrate of next step
                for i in self.path[stepIndex].split():
                    if i[0] == "F":
                        tempFeedRate = float(i[1:])

            # Changes based on which step it is
            if (stepIndex < self.step):
                initial = np.copy(self.prevTarget2)
                point1 = np.copy(self.prevTarget2)
                point2 = np.copy(self.prevTarget2)
                targetTemp = np.copy(self.prevTarget2)
            elif (stepIndex > self.step):
                initial = np.copy(self.target)
                point1 = np.copy(self.target)
                point2 = np.copy(self.target)
                targetTemp = np.copy(self.target)

            # Updates points
            for i in self.path[stepIndex].split():
                if i[0] == "X":
                    targetTemp[0] = float(i[1:])
                elif i[0] == "Y":
                    targetTemp[1] = float(i[1:])
                elif i[0] == "Z":
                    targetTemp[2] = float(i[1:])
                elif i[0] == "I":
                    point1[0] = float(i[1:])
                    point2[0] = float(i[1:])
                    targetTemp[0] = float(i[1:])
                elif i[0] == "J":
                    point1[1] = float(i[1:])
                    point2[1] = float(i[1:])
                    targetTemp[1] = float(i[1:])
                elif i[0] == "K":
                    point1[2] = float(i[1:])
                    point2[2] = float(i[1:])
                    targetTemp[2] = float(i[1:])
                elif i[0] == "P":
                    point2[0] = float(i[1:])
                    targetTemp[0] = float(i[1:])
                elif i[0] == "Q":
                    point2[1] = float(i[1:])
                    targetTemp[1] = float(i[1:])
                elif i[0] == "R":
                    point2[2] = float(i[1:])
                    targetTemp[2] = float(i[1:])

            # Only limits radial acceleration, not total but idk how to do that
            derivative = 3*(1-t)**2*(point1-initial)+6*(1-t)*t*(point2-point1)+3*t**2*(targetTemp-point2)
            secondDerivative = 6*(1-t)*(point2-2*point1+initial)+6*t*(targetTemp-2*point2+point1)
            speed = 0
            if (np.linalg.norm(np.cross(derivative, secondDerivative)) == 0):
                speed = tempFeedRate
            else:
                speed = (self.maxAcceleration*np.linalg.norm(derivative)**3/np.linalg.norm(np.cross(derivative, secondDerivative)))**0.5
            return max(self.minSpeed, min(tempFeedRate, speed))

        # Returns minspeed if step is neither of the above
        return self.minSpeed

    # Performs an action based on the curent state and commands
    def followPath(self):
        # Checks whether the path is complete
        if (self.step >= len(self.path) or len(self.path) == 0):
            self.setVelocity(np.array([0.0, 0.0, 0.0]))
            return

        # Path following for straight line
        if (self.path[self.step].split()[0] in ["G1", "G01"]):
            # Sets the end point
            if (np.allclose(self.target, self.prevTarget)):
                # Sets the target positions
                self.target = np.copy(self.prevTarget)
                for i in self.path[self.step].split():
                    if i[0] == "X":
                        self.target[0] = float(i[1:])
                    elif i[0] == "Y":
                        self.target[1] = float(i[1:])
                    elif i[0] == "Z":
                        self.target[2] = float(i[1:])
                    elif i[0] == "F":
                        self.feedRate = min(float(i[1:]), self.maxSpeed)

            # Finds the closest point
            closestPoint = np.copy(self.getPointOnPath(self.path[self.step], 0))
            closestLength = np.linalg.norm(closestPoint-self.position)
            closestTime = 0
            for i in range(0, self.divisions+1):
                point = self.getPointOnPath(self.path[self.step], i/self.divisions)
                if (np.linalg.norm(point-self.position) <= closestLength):
                    closestLength = np.linalg.norm(point-self.position)
                    closestPoint = np.copy(point)
                    closestTime = i/self.divisions
            
            # Gets the vectors for the movement direction
            direction = self.getTangent(self.step, closestTime)
            if (closestTime == 1):
                direction = np.array([0.0, 0.0, 0.0])
            dirToLine = (closestPoint-self.position)*self.pathAttraction

            # If first and last step
            if (self.step == 0 and self.step == len(self.path)-1):
                if (np.linalg.norm(self.target-self.position) <= self.tolerance):
                    self.setVelocity(np.array([0.0, 0.0, 0.0]))
                    self.prevTarget2 = np.copy(self.prevTarget)
                    self.prevTarget = np.copy(self.target)
                    self.prevFeedRate = self.feedRate
                    self.step += 1
                    return
                distTravelled = np.linalg.norm(self.position-self.prevTarget)
                distToTarget = np.linalg.norm(self.target-self.position)
                dirToTarget = direction+dirToLine
                mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (self.minSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
            # If first step
            elif (self.step == 0 and self.step < len(self.path)-1):
                # If tangents of this and next are equal
                if (np.allclose(self.getTangent(self.step, 1), self.getTangent(self.step+1, 0))):
                    # Stopping condition
                    if (np.linalg.norm(self.target-closestPoint) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    endingSpeed = self.getSpeed(self.step+1, 0)
                    distTravelled = np.linalg.norm(self.position-self.prevTarget)
                    distToTarget = np.linalg.norm(self.target-self.position)
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (endingSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
                else:
                    # Stopping condition
                    if (np.linalg.norm(self.target-self.position) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    distTravelled = np.linalg.norm(self.position-self.prevTarget)
                    distToTarget = np.linalg.norm(self.target-self.position)
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (self.minSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
            # If last step
            elif (self.step > 0 and self.step == len(self.path)-1):
                # If tangents of this and previous are equal
                if (np.allclose(self.getTangent(self.step, 0), self.getTangent(self.step-1, 1))):
                    # Stopping condition
                    if (np.linalg.norm(self.target-self.position) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    startingSpeed = self.getSpeed(self.step-1, 1)
                    distTravelled = np.linalg.norm(self.position-self.prevTarget)
                    distToTarget = np.linalg.norm(self.target-self.position)
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (self.minSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (startingSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
                else:
                    # Stopping condition
                    if (np.linalg.norm(self.target-self.position) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    distTravelled = np.linalg.norm(self.position-self.prevTarget)
                    distToTarget = np.linalg.norm(self.target-self.position)
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (self.minSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
            # If middle step
            else:
                # If this and previous and next tangents are equal
                if (np.allclose(self.getTangent(self.step, 0), self.getTangent(self.step-1, 1)) and np.allclose(self.getTangent(self.step, 1), self.getTangent(self.step+1, 0))):
                    # Stopping condition
                    if (np.linalg.norm(self.target-closestPoint) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    endingSpeed = self.getSpeed(self.step+1, 0)
                    startingSpeed = self.getSpeed(self.step-1, 1)
                    distTravelled = np.linalg.norm(self.position-self.prevTarget)
                    distToTarget = np.linalg.norm(self.target-self.position)
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (endingSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (startingSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
                # If this and previous tangents are equal
                elif (np.allclose(self.getTangent(self.step, 0), self.getTangent(self.step-1, 1))):
                    # Stopping condition
                    if (np.linalg.norm(self.target-self.position) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    startingSpeed = self.getSpeed(self.step-1, 1)
                    distTravelled = np.linalg.norm(self.position-self.prevTarget)
                    distToTarget = np.linalg.norm(self.target-self.position)
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (self.minSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (startingSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
                # If this and next tangents are equal
                elif (np.allclose(self.getTangent(self.step, 1), self.getTangent(self.step+1, 0))):
                    # Stopping condition
                    if (np.linalg.norm(self.target-closestPoint) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    endingSpeed = self.getSpeed(self.step+1, 0)
                    distTravelled = np.linalg.norm(self.position-self.prevTarget)
                    distToTarget = np.linalg.norm(self.target-self.position)
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (endingSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
                # If no tangents are equal
                else:
                    if (np.linalg.norm(self.target-self.position) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    distTravelled = np.linalg.norm(self.position-self.prevTarget)
                    distToTarget = np.linalg.norm(self.target-self.position)
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (self.minSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
                
        # Path following for arc
        elif (self.path[self.step].split()[0] in ["G2", "G02", "G3", "G03"]):
            flipRotation = 1 if self.path[self.step].split()[0] in ["G2", "G02"] else -1
            # Sets the center
            center = np.copy(self.prevTarget)
            for i in self.path[self.step].split():
                if i[0] == "I":
                    center[0] = float(i[1:])
                elif i[0] == "J":
                    center[1] = float(i[1:])
                elif i[0] == "K":
                    center[2] = float(i[1:])

            # Sets the end point
            if (np.allclose(self.target, self.prevTarget)):
                # Sets the target positions
                self.target = np.copy(self.prevTarget)
                for i in self.path[self.step].split():
                    if i[0] == "X":
                        self.target[0] = float(i[1:])
                    elif i[0] == "Y":
                        self.target[1] = float(i[1:])
                    elif i[0] == "Z":
                        self.target[2] = float(i[1:])
                    elif i[0] == "F":
                        self.feedRate = min(float(i[1:]), self.maxSpeed)
            # Finds the closest point
            closestPoint = np.copy(self.getPointOnPath(self.path[self.step], 0))
            closestLength = np.linalg.norm(closestPoint-self.position)
            closestTime = 0
            for i in range(0, self.divisions+1):
                point = self.getPointOnPath(self.path[self.step], i/self.divisions)
                if (np.linalg.norm(point-self.position) <= closestLength):
                    closestLength = np.linalg.norm(point-self.position)
                    closestPoint = np.copy(point)
                    closestTime = i/self.divisions
            
            # Gets the vectors for the movement direction
            direction = self.getTangent(self.step, closestTime)
            
            if (closestTime == 1):
                direction = np.array([0.0, 0.0, 0.0])
            dirToLine = (closestPoint-self.position)*self.pathAttraction
            # If first and last step
            if (self.step == 0 and self.step == len(self.path)-1):
                if (np.linalg.norm(self.target-self.position) <= self.tolerance):
                    self.setVelocity(np.array([0.0, 0.0, 0.0]))
                    self.prevTarget2 = np.copy(self.prevTarget)
                    self.prevTarget = np.copy(self.target)
                    self.prevFeedRate = self.feedRate
                    self.step += 1
                    return
                # Caculates angle between starting and ending
                angle = math.acos(np.dot(self.prevTarget-center, self.target-center)/(np.linalg.norm(self.prevTarget-center)*np.linalg.norm(self.target-center)))
                
                # Finds axis of rotation
                axis = np.cross(self.target-center, self.prevTarget-center)
                axis = axis/np.linalg.norm(axis)
                
                # Switches if going opposite direction
                if (np.dot(axis, np.array([0.0, 0.0, flipRotation*1.0])) < 0):
                    angle = -2*math.pi+angle
                distTravelled = (np.linalg.norm(self.position-closestPoint)**2+(angle*closestTime*np.linalg.norm(self.target-center))**2)**0.5
                angle *= (1-closestTime)
                distToTarget = (np.linalg.norm(self.position-closestPoint)**2+(angle*np.linalg.norm(self.target-center))**2)**0.5
                dirToTarget = direction+dirToLine
                mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (self.minSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
            # If first step
            elif (self.step == 0 and self.step < len(self.path)-1):
                # If tangents of this and next are equal
                if (np.allclose(self.getTangent(self.step, 1), self.getTangent(self.step+1, 0))):
                    # Stopping condition
                    if (np.linalg.norm(self.target-closestPoint) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    endingSpeed = self.getSpeed(self.step+1, 0)
                    # Caculates angle between starting and ending
                    angle = math.acos(np.dot(self.prevTarget-center, self.target-center)/(np.linalg.norm(self.prevTarget-center)*np.linalg.norm(self.target-center)))
                    
                    # Finds axis of rotation
                    axis = np.cross(self.target-center, self.prevTarget-center)
                    axis = axis/np.linalg.norm(axis)
                    
                    # Switches if going opposite direction
                    if (np.dot(axis, np.array([0.0, 0.0, flipRotation*1.0])) < 0):
                        angle = -2*math.pi+angle
                    distTravelled = (np.linalg.norm(self.position-closestPoint)**2+(angle*closestTime*np.linalg.norm(self.target-center))**2)**0.5
                    angle *= (1-closestTime)
                    distToTarget = (np.linalg.norm(self.position-closestPoint)**2+(angle*np.linalg.norm(self.target-center))**2)**0.5
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (endingSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
                else:
                    # Stopping condition
                    if (np.linalg.norm(self.target-self.position) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    # Caculates angle between starting and ending
                    angle = math.acos(np.dot(self.prevTarget-center, self.target-center)/(np.linalg.norm(self.prevTarget-center)*np.linalg.norm(self.target-center)))
                    
                    # Finds axis of rotation
                    axis = np.cross(self.target-center, self.prevTarget-center)
                    axis = axis/np.linalg.norm(axis)
                    
                    # Switches if going opposite direction
                    if (np.dot(axis, np.array([0.0, 0.0, flipRotation*1.0])) < 0):
                        angle = -2*math.pi+angle
                    distTravelled = (np.linalg.norm(self.position-closestPoint)**2+(angle*closestTime*np.linalg.norm(self.target-center))**2)**0.5
                    angle *= (1-closestTime)
                    distToTarget = (np.linalg.norm(self.position-closestPoint)**2+(angle*np.linalg.norm(self.target-center))**2)**0.5
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (self.minSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
            # If last step
            elif (self.step > 0 and self.step == len(self.path)-1):
                # If tangents of this and previous are equal
                if (np.allclose(self.getTangent(self.step, 0), self.getTangent(self.step-1, 1))):
                    # Stopping condition
                    if (np.linalg.norm(self.target-self.position) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    startingSpeed = self.getSpeed(self.step-1, 1)
                    # Caculates angle between starting and ending
                    angle = math.acos(np.dot(self.prevTarget-center, self.target-center)/(np.linalg.norm(self.prevTarget-center)*np.linalg.norm(self.target-center)))
                    
                    # Finds axis of rotation
                    axis = np.cross(self.target-center, self.prevTarget-center)
                    axis = axis/np.linalg.norm(axis)
                    
                    # Switches if going opposite direction
                    if (np.dot(axis, np.array([0.0, 0.0, flipRotation*1.0])) < 0):
                        angle = -2*math.pi+angle
                    distTravelled = (np.linalg.norm(self.position-closestPoint)**2+(angle*closestTime*np.linalg.norm(self.target-center))**2)**0.5
                    angle *= (1-closestTime)
                    distToTarget = (np.linalg.norm(self.position-closestPoint)**2+(angle*np.linalg.norm(self.target-center))**2)**0.5
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (self.minSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (startingSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
                else:
                    # Stopping condition
                    if (np.linalg.norm(self.target-self.position) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    # Caculates angle between starting and ending
                    angle = math.acos(np.dot(self.prevTarget-center, self.target-center)/(np.linalg.norm(self.prevTarget-center)*np.linalg.norm(self.target-center)))
                    
                    # Finds axis of rotation
                    axis = np.cross(self.target-center, self.prevTarget-center)
                    axis = axis/np.linalg.norm(axis)
                    
                    # Switches if going opposite direction
                    if (np.dot(axis, np.array([0.0, 0.0, flipRotation*1.0])) < 0):
                        angle = -2*math.pi+angle
                    distTravelled = (np.linalg.norm(self.position-closestPoint)**2+(angle*closestTime*np.linalg.norm(self.target-center))**2)**0.5
                    angle *= (1-closestTime)
                    distToTarget = (np.linalg.norm(self.position-closestPoint)**2+(angle*np.linalg.norm(self.target-center))**2)**0.5
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (self.minSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
            # If middle step
            else:
                # If this and previous and next tangents are equal
                if (np.allclose(self.getTangent(self.step, 0), self.getTangent(self.step-1, 1)) and np.allclose(self.getTangent(self.step, 1), self.getTangent(self.step+1, 0))):
                    # Stopping condition
                    if (np.linalg.norm(self.target-closestPoint) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    endingSpeed = self.getSpeed(self.step+1, 0)
                    startingSpeed = self.getSpeed(self.step-1, 1)
                    # Caculates angle between starting and ending
                    angle = math.acos(np.dot(self.prevTarget-center, self.target-center)/(np.linalg.norm(self.prevTarget-center)*np.linalg.norm(self.target-center)))
                    
                    # Finds axis of rotation
                    axis = np.cross(self.target-center, self.prevTarget-center)
                    axis = axis/np.linalg.norm(axis)
                    
                    # Switches if going opposite direction
                    if (np.dot(axis, np.array([0.0, 0.0, flipRotation*1.0])) < 0):
                        angle = -2*math.pi+angle
                    distTravelled = (np.linalg.norm(self.position-closestPoint)**2+(angle*closestTime*np.linalg.norm(self.target-center))**2)**0.5
                    angle *= (1-closestTime)
                    distToTarget = (np.linalg.norm(self.position-closestPoint)**2+(angle*np.linalg.norm(self.target-center))**2)**0.5
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (endingSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (startingSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
                # If this and previous tangents are equal
                elif (np.allclose(self.getTangent(self.step, 0), self.getTangent(self.step-1, 1))):
                    # Stopping condition
                    if (np.linalg.norm(self.target-self.position) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    startingSpeed = self.getSpeed(self.step-1, 1)
                    # Caculates angle between starting and ending
                    angle = math.acos(np.dot(self.prevTarget-center, self.target-center)/(np.linalg.norm(self.prevTarget-center)*np.linalg.norm(self.target-center)))
                    
                    # Finds axis of rotation
                    axis = np.cross(self.target-center, self.prevTarget-center)
                    axis = axis/np.linalg.norm(axis)
                    
                    # Switches if going opposite direction
                    if (np.dot(axis, np.array([0.0, 0.0, flipRotation*1.0])) < 0):
                        angle = -2*math.pi+angle
                    distTravelled = (np.linalg.norm(self.position-closestPoint)**2+(angle*closestTime*np.linalg.norm(self.target-center))**2)**0.5
                    angle *= (1-closestTime)
                    distToTarget = (np.linalg.norm(self.position-closestPoint)**2+(angle*np.linalg.norm(self.target-center))**2)**0.5
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (self.minSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (startingSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
                # If this and next tangents are equal
                elif (np.allclose(self.getTangent(self.step, 1), self.getTangent(self.step+1, 0))):
                    # Stopping condition
                    if (np.linalg.norm(self.target-closestPoint) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    endingSpeed = self.getSpeed(self.step+1, 0)
                    # Caculates angle between starting and ending
                    angle = math.acos(np.dot(self.prevTarget-center, self.target-center)/(np.linalg.norm(self.prevTarget-center)*np.linalg.norm(self.target-center)))
                    
                    # Finds axis of rotation
                    axis = np.cross(self.target-center, self.prevTarget-center)
                    axis = axis/np.linalg.norm(axis)
                    
                    # Switches if going opposite direction
                    if (np.dot(axis, np.array([0.0, 0.0, flipRotation*1.0])) < 0):
                        angle = -2*math.pi+angle
                    distTravelled = (np.linalg.norm(self.position-closestPoint)**2+(angle*closestTime*np.linalg.norm(self.target-center))**2)**0.5
                    angle *= (1-closestTime)
                    distToTarget = (np.linalg.norm(self.position-closestPoint)**2+(angle*np.linalg.norm(self.target-center))**2)**0.5
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (endingSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
                # If no tangents are equal
                else:
                    if (np.linalg.norm(self.target-self.position) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Caculates angle between starting and ending
                    angle = math.acos(np.dot(self.prevTarget-center, self.target-center)/(np.linalg.norm(self.prevTarget-center)*np.linalg.norm(self.target-center)))
                    
                    # Finds axis of rotation
                    axis = np.cross(self.target-center, self.prevTarget-center)
                    axis = axis/np.linalg.norm(axis)
                    
                    # Switches if going opposite direction
                    if (np.dot(axis, np.array([0.0, 0.0, flipRotation*1.0])) < 0):
                        angle = -2*math.pi+angle
                    distTravelled = (np.linalg.norm(self.position-closestPoint)**2+(angle*closestTime*np.linalg.norm(self.target-center))**2)**0.5
                    angle *= (1-closestTime)
                    distToTarget = (np.linalg.norm(self.position-closestPoint)**2+(angle*np.linalg.norm(self.target-center))**2)**0.5
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (self.minSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))

        # Path following for cubic bezier
        elif (self.path[self.step].split()[0] in ["G5", "G05"]):
            # Sets the reference points
            point1 = np.copy(self.prevTarget)
            point2 = np.copy(self.prevTarget)
            for i in self.path[self.step].split():
                if i[0] == "I":
                    point1[0] = float(i[1:])
                elif i[0] == "J":
                    point1[1] = float(i[1:])
                elif i[0] == "K":
                    point1[2] = float(i[1:])
                elif i[0] == "P":
                    point2[0] = float(i[1:])
                elif i[0] == "Q":
                    point2[1] = float(i[1:])
                elif i[0] == "R":
                    point2[2] = float(i[1:])

            # Sets the end point
            if (np.allclose(self.target, self.prevTarget)):
                # Sets the target positions
                self.target = np.copy(self.prevTarget)
                for i in self.path[self.step].split():
                    if i[0] == "X":
                        self.target[0] = float(i[1:])
                    elif i[0] == "Y":
                        self.target[1] = float(i[1:])
                    elif i[0] == "Z":
                        self.target[2] = float(i[1:])
                    elif i[0] == "F":
                        self.feedRate = min(float(i[1:]), self.maxSpeed)
            # Finds the closest point
            closestPoint = np.copy(self.getPointOnPath(self.path[self.step], 0))
            closestLength = np.linalg.norm(closestPoint-self.position)
            closestTime = 0
            for i in range(0, self.divisions+1):
                point = self.getPointOnPath(self.path[self.step], i/self.divisions)
                if (np.linalg.norm(point-self.position) <= closestLength):
                    closestLength = np.linalg.norm(point-self.position)
                    closestPoint = np.copy(point)
                    closestTime = i/self.divisions
            
            # Gets the vectors for the movement direction
            direction = self.getTangent(self.step, closestTime)
            
            if (closestTime == 1):
                direction = np.array([0.0, 0.0, 0.0])
            dirToLine = (closestPoint-self.position)*self.pathAttraction

            # If first and last step
            if (self.step == 0 and self.step == len(self.path)-1):
                if (np.linalg.norm(self.target-self.position) <= self.tolerance):
                    self.setVelocity(np.array([0.0, 0.0, 0.0]))
                    self.prevTarget2 = np.copy(self.prevTarget)
                    self.prevTarget = np.copy(self.target)
                    self.prevFeedRate = self.feedRate
                    self.step += 1
                    return
                
                distTravelled = 0
                distToTarget = 0
                point = self.getPointOnPath(self.path[self.step], 0)
                for i in range(1, self.divisions+1):
                    nextPoint = self.getPointOnPath(self.path[self.step], i/self.divisions)
                    if (i/self.divisions <= closestTime):
                        distTravelled += np.linalg.norm(nextPoint-point)
                    else:
                        distToTarget += np.linalg.norm(nextPoint-point)
                    point = np.copy(nextPoint)

                dirToTarget = direction+dirToLine
                mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (self.minSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
            # If first step
            elif (self.step == 0 and self.step < len(self.path)-1):
                # If tangents of this and next are equal
                if (np.allclose(self.getTangent(self.step, 1), self.getTangent(self.step+1, 0))):
                    # Stopping condition
                    if (np.linalg.norm(self.target-closestPoint) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    endingSpeed = self.getSpeed(self.step+1, 0)
                    distTravelled = 0
                    distToTarget = 0
                    point = self.getPointOnPath(self.path[self.step], 0)
                    for i in range(1, self.divisions+1):
                        nextPoint = self.getPointOnPath(self.path[self.step], i/self.divisions)
                        if (i/self.divisions <= closestTime):
                            distTravelled += np.linalg.norm(nextPoint-point)
                        else:
                            distToTarget += np.linalg.norm(nextPoint-point)
                        point = np.copy(nextPoint)
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (endingSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
                else:
                    # Stopping condition
                    if (np.linalg.norm(self.target-self.position) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    distTravelled = 0
                    distToTarget = 0
                    point = self.getPointOnPath(self.path[self.step], 0)
                    for i in range(1, self.divisions+1):
                        nextPoint = self.getPointOnPath(self.path[self.step], i/self.divisions)
                        if (i/self.divisions <= closestTime):
                            distTravelled += np.linalg.norm(nextPoint-point)
                        else:
                            distToTarget += np.linalg.norm(nextPoint-point)
                        point = np.copy(nextPoint)
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (self.minSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
            # If last step
            elif (self.step > 0 and self.step == len(self.path)-1):
                # If tangents of this and previous are equal
                if (np.allclose(self.getTangent(self.step, 0), self.getTangent(self.step-1, 1))):
                    # Stopping condition
                    if (np.linalg.norm(self.target-self.position) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    startingSpeed = self.getSpeed(self.step-1, 1)
                    distTravelled = 0
                    distToTarget = 0
                    point = self.getPointOnPath(self.path[self.step], 0)
                    for i in range(1, self.divisions+1):
                        nextPoint = self.getPointOnPath(self.path[self.step], i/self.divisions)
                        if (i/self.divisions <= closestTime):
                            distTravelled += np.linalg.norm(nextPoint-point)
                        else:
                            distToTarget += np.linalg.norm(nextPoint-point)
                        point = np.copy(nextPoint)
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (self.minSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (startingSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
                else:
                    # Stopping condition
                    if (np.linalg.norm(self.target-self.position) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    distTravelled = 0
                    distToTarget = 0
                    point = self.getPointOnPath(self.path[self.step], 0)
                    for i in range(1, self.divisions+1):
                        nextPoint = self.getPointOnPath(self.path[self.step], i/self.divisions)
                        if (i/self.divisions <= closestTime):
                            distTravelled += np.linalg.norm(nextPoint-point)
                        else:
                            distToTarget += np.linalg.norm(nextPoint-point)
                        point = np.copy(nextPoint)
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (self.minSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
            # If middle step
            else:
                # If this and previous and next tangents are equal
                if (np.allclose(self.getTangent(self.step, 0), self.getTangent(self.step-1, 1)) and np.allclose(self.getTangent(self.step, 1), self.getTangent(self.step+1, 0))):
                    # Stopping condition
                    if (np.linalg.norm(self.target-closestPoint) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    endingSpeed = self.getSpeed(self.step+1, 0)
                    startingSpeed = self.getSpeed(self.step-1, 1)
                    distTravelled = 0
                    distToTarget = 0
                    point = self.getPointOnPath(self.path[self.step], 0)
                    for i in range(1, self.divisions+1):
                        nextPoint = self.getPointOnPath(self.path[self.step], i/self.divisions)
                        if (i/self.divisions <= closestTime):
                            distTravelled += np.linalg.norm(nextPoint-point)
                        else:
                            distToTarget += np.linalg.norm(nextPoint-point)
                        point = np.copy(nextPoint)
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (endingSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (startingSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
                # If this and previous tangents are equal
                elif (np.allclose(self.getTangent(self.step, 0), self.getTangent(self.step-1, 1))):
                    # Stopping condition
                    if (np.linalg.norm(self.target-self.position) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    startingSpeed = self.getSpeed(self.step-1, 1)
                    distTravelled = 0
                    distToTarget = 0
                    point = self.getPointOnPath(self.path[self.step], 0)
                    for i in range(1, self.divisions+1):
                        nextPoint = self.getPointOnPath(self.path[self.step], i/self.divisions)
                        if (i/self.divisions <= closestTime):
                            distTravelled += np.linalg.norm(nextPoint-point)
                        else:
                            distToTarget += np.linalg.norm(nextPoint-point)
                        point = np.copy(nextPoint)
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (self.minSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (startingSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
                # If this and next tangents are equal
                elif (np.allclose(self.getTangent(self.step, 1), self.getTangent(self.step+1, 0))):
                    # Stopping condition
                    if (np.linalg.norm(self.target-closestPoint) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    # Sets speed to maintain acceleration
                    endingSpeed = self.getSpeed(self.step+1, 0)
                    distTravelled = 0
                    distToTarget = 0
                    point = self.getPointOnPath(self.path[self.step], 0)
                    for i in range(1, self.divisions+1):
                        nextPoint = self.getPointOnPath(self.path[self.step], i/self.divisions)
                        if (i/self.divisions <= closestTime):
                            distTravelled += np.linalg.norm(nextPoint-point)
                        else:
                            distToTarget += np.linalg.norm(nextPoint-point)
                        point = np.copy(nextPoint)
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (endingSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))
                # If no tangents are equal
                else:
                    if (np.linalg.norm(self.target-self.position) <= self.tolerance):
                        self.setVelocity(np.array([0.0, 0.0, 0.0]))
                        self.prevTarget2 = np.copy(self.prevTarget)
                        self.prevTarget = np.copy(self.target)
                        self.prevFeedRate = self.feedRate
                        self.step += 1
                        return
                    distTravelled = 0
                    distToTarget = 0
                    point = self.getPointOnPath(self.path[self.step], 0)
                    for i in range(1, self.divisions+1):
                        nextPoint = self.getPointOnPath(self.path[self.step], i/self.divisions)
                        if (i/self.divisions <= closestTime):
                            distTravelled += np.linalg.norm(nextPoint-point)
                        else:
                            distToTarget += np.linalg.norm(nextPoint-point)
                        point = np.copy(nextPoint)
                    dirToTarget = direction+dirToLine
                    mult = min(self.getSpeed(self.step, closestTime), self.feedRate, (self.minSpeed**2+2*self.maxAcceleration*distToTarget)**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)
                    self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))

        # Moving to point at max speed
        elif (self.path[self.step].split()[0] in ["G00", "G0"]):
            if (np.allclose(self.target, self.prevTarget)):
                # Sets the target positions
                self.target = np.copy(self.prevTarget)
                for i in self.path[self.step].split():
                    if i[0] == "X":
                        self.target[0] = float(i[1:])
                    elif i[0] == "Y":
                        self.target[1] = float(i[1:])
                    elif i[0] == "Z":
                        self.target[2] = float(i[1:])
            
            # Gets the distance to the target
            dirToTarget = self.target-self.position

            # Moves to next command if within tolerances
            if (np.linalg.norm(dirToTarget) <= self.tolerance):
                self.setVelocity(np.array([0.0, 0.0, 0.0]))
                self.prevTarget2 = np.copy(self.prevTarget)
                self.prevTarget = np.copy(self.target)
                self.prevFeedRate = self.feedRate
                self.step += 1
                return

            # Calculates distance travelled from start
            distTravelled = np.linalg.norm(self.position-self.prevTarget)

            # Finds multiplier to maintain acceleration
            mult = min(self.maxSpeed, (self.minSpeed**2+2*self.maxAcceleration*np.linalg.norm(dirToTarget))**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)

            # Sets velocity
            self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))

        # Stops for a specific time
        elif (self.path[self.step].split()[0] in ["G04", "G4"]):
            self.setVelocity([0.0, 0.0, 0.0])
            time.sleep(int(self.path[self.step].split()[1][1:])/1000)
            self.prevTarget2 = np.copy(self.prevTarget)
            self.prevTarget = np.copy(self.target)
            self.prevFeedRate = self.feedRate
            self.step += 1

        # Stops at the end point of the previous command
        elif (self.path[self.step].split()[0] in ["G09", "G9"]):
            self.setVelocity([0.0, 0.0, 0.0])
            self.prevTarget2 = np.copy(self.prevTarget)
            self.prevTarget = np.copy(self.target)
            self.prevFeedRate = self.feedRate
            self.step += 1

        # Moves to zero position with intermediate point
        elif (self.path[self.step].split()[0] in ["G28"]):
            if (np.allclose(self.target, self.prevTarget)):
                # Sets the target positions
                self.target = np.copy(self.position)
                for i in self.path[self.step].split():
                    if i[0] == "X":
                        self.target[0] = float(i[1:])
                    elif i[0] == "Y":
                        self.target[1] = float(i[1:])
                    elif i[0] == "Z":
                        self.target[2] = float(i[1:])
                
            # Gets the distance to the target
            dirToTarget = self.target-self.position

            # Moves to next command if within tolerances
            if (np.linalg.norm(dirToTarget) <= self.tolerance):
                self.setVelocity([0.0, 0.0, 0.0])
                self.prevTarget2 = np.copy(self.prevTarget)
                self.prevTarget = np.copy(self.target)
                self.prevFeedRate = self.feedRate

                if (np.allclose(self.target, [0.0, 0.0, self.target[2]])):
                    self.step += 1
                else:
                    self.target = np.array([0.0, 0.0, self.target[2]])
                return

            # Calculates distance travelled from start
            distTravelled = np.linalg.norm(self.position-self.prevTarget)

            # Finds multiplier to maintain acceleration
            mult = min(self.maxSpeed, (self.minSpeed**2+2*self.maxAcceleration*np.linalg.norm(dirToTarget))**0.5, (self.minSpeed**2+2*self.maxAcceleration*distTravelled)**0.5)

            # Sets velocity
            self.setVelocity(mult*dirToTarget/np.linalg.norm(dirToTarget))

        # Stops the machine
        elif (self.path[self.step].split()[0] in ["M00", "M01", "M02", "M30", "M0", "M1", "M2"]):
            self.setVelocity(np.array([0.0, 0.0, 0.0]))
            self.setSpindleSpeed(0)
            self.prevTarget2 = np.copy(self.prevTarget)
            self.prevTarget = np.copy(self.target)
            self.prevFeedRate = self.feedRate
            self.step += 1

        # Spindle on clockwise
        elif (self.path[self.step].split()[0] in ["M03", "M3"]):
            self.setSpindleSpeed(self.spindleMaxSpeed)
            self.prevTarget2 = np.copy(self.prevTarget)
            self.prevTarget = np.copy(self.target)
            self.prevFeedRate = self.feedRate
            self.step += 1

        # Spindle on counterclockwise
        elif (self.path[self.step].split()[0] in ["M04", "M4"]):
            self.setSpindleSpeed(-self.spindleMaxSpeed)
            self.prevTarget2 = np.copy(self.prevTarget)
            self.prevTarget = np.copy(self.target)
            self.prevFeedRate = self.feedRate
            self.step += 1

        # Spindle stop
        elif (self.path[self.step].split()[0] in ["M05", "M5"]):
            self.setSpindleSpeed(0)
            self.prevTarget2 = np.copy(self.prevTarget)
            self.prevTarget = np.copy(self.target)
            self.prevFeedRate = self.feedRate
            self.step += 1

        # If syntax error, freeze
        else:
            self.setVelocity(np.array([0.0, 0.0, 0.0]))

# Sets up the cnc object
cnc = CNC(50, 300, 300, 3600000, 100, 0.2, 0.02, 100)
cncRunning = False

pygame.init()
screen = pygame.display.set_mode((1200, 600))
# Sets up all GUI elements
textbox = TextBox(screen, 775, 63, 280, 35, fontSize=25, textColour=(0, 0, 0), radius=10, borderThickness=0, colour = (97, 114, 151))
backgroundImg = pygame.image.load(str(Path(__file__).resolve().parents[0] / "assets" / "CNC Software.png"))
startHover = pygame.transform.scale(pygame.image.load(str(Path(__file__).resolve().parents[0] / "assets" / "startHover.png")), (263, 105))
simulateHover = pygame.transform.scale(pygame.image.load(str(Path(__file__).resolve().parents[0] / "assets" / "simulateHover.png")), (287, 105))
openHover = pygame.transform.scale(pygame.image.load(str(Path(__file__).resolve().parents[0] / "assets" / "openHover.png")), (80, 50))
controlHover = pygame.transform.scale(pygame.image.load(str(Path(__file__).resolve().parents[0] / "assets" / "controlHover.png")), (50, 50))
stepHover = pygame.transform.scale(pygame.image.load(str(Path(__file__).resolve().parents[0] / "assets" / "stepHover.png")), (80, 38))
stopHover = pygame.transform.scale(pygame.image.load(str(Path(__file__).resolve().parents[0] / "assets" / "stopHover.png")), (115, 115))
connectHover = pygame.transform.scale(pygame.image.load(str(Path(__file__).resolve().parents[0] / "assets" / "connectHover.png")), (225, 105))
font = pygame.font.Font(pygame.font.get_default_font(), 15)

# Display variables
selectedStepSize = 1
run = True
displayPoints = set()
code = ""
isHoming = False
while run:
    # Handles events
    screen.blit(backgroundImg, (0, 0))
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.QUIT:
            pygame.quit()
            run = False
            sys.exit()
        # If click
        if event.type == pygame.MOUSEBUTTONUP:
            # Remove message
            if "ERROR" in textbox.getText() or "SUCCESS" in textbox.getText():
                textbox.setText("")
            
            # Start button
            if pygame.Rect(337, 25, 263, 105).collidepoint(pygame.mouse.get_pos()) and cnc.isConnected() and not cncRunning:
                cnc.setVelocity(np.array([0.0, 0.0, 0.0]))
                cnc.setSpindleSpeed(0)
                cnc.updatePosition()
                cnc.setSimulation(False)
                isHoming = False
                cnc.setPath(code)
                cncRunning = True
                displayPoints = set()
            
            # Simulate button
            if pygame.Rect(25, 25, 287, 105).collidepoint(pygame.mouse.get_pos()) and not cncRunning and len(code) > 0:
                cnc.setVelocity(np.array([0.0, 0.0, 0.0]))
                cnc.setSpindleSpeed(0)
                cnc.setSimulation(True)
                cnc.resetPosition()
                isHoming = False
                cnc.setPath(code)
                cncRunning = True
                displayPoints = set()

            # Open file button
            if pygame.Rect(1070, 53, 80, 50).collidepoint(pygame.mouse.get_pos()) and not cncRunning:
                cnc.setVelocity(np.array([0.0, 0.0, 0.0]))
                cnc.setSpindleSpeed(0)
                cnc.updatePosition()
                cnc.setSimulation(True)
                isHoming = False
                cncRunning = False
                displayPoints = set()
                try:
                    with open(Path(__file__).resolve().parents[1] / "input" / textbox.getText()) as f:
                        code = f.read().split("G21\n")[1]
                        cnc.setPath(code)
                        textbox.setText("SUCCESS")
                except:
                    textbox.setText("FILE ERROR")
            
            # Stop button
            if pygame.Rect(1005, 408, 115, 115).collidepoint(pygame.mouse.get_pos()) and cncRunning:
                cnc.setVelocity(np.array([0.0, 0.0, 0.0]))
                cnc.setSpindleSpeed(0)
                cnc.updatePosition()
                cnc.setPath("\n".join(code))
                cnc.setSimulation(True)
                isHoming = False
                cncRunning = False

            # Connect button
            if pygame.Rect(950, 156, 225, 105).collidepoint(pygame.mouse.get_pos()) and not cncRunning:
                cnc.setVelocity(np.array([0.0, 0.0, 0.0]))
                cnc.setSpindleSpeed(0)
                cnc.updatePosition()
                cnc.setSimulation(True)
                cnc.setPath("\n".join(code))
                isHoming = False
                cncRunning = False
                if not cnc.connect():
                    textbox.setText("CONNECTION ERROR")
                else:
                    textbox.setText("SUCCESS")
            
            # Step size buttons
            if pygame.Rect(640, 208, 80, 38).collidepoint(pygame.mouse.get_pos()):
                selectedStepSize = 0.1
            if pygame.Rect(735, 208, 80, 38).collidepoint(pygame.mouse.get_pos()):
                selectedStepSize = 1
            if pygame.Rect(830, 208, 80, 38).collidepoint(pygame.mouse.get_pos()):
                selectedStepSize = 10
            
            # Home button
            if pygame.Rect(1005, 343, 50, 50).collidepoint(pygame.mouse.get_pos()) and cnc.isConnected() and not cncRunning:
                cnc.setVelocity(np.array([0.0, 0.0, 0.0]))
                cnc.setSpindleSpeed(0)
                cnc.updatePosition()
                cnc.setSimulation(False)
                code = "\n".join(cnc.path)
                cnc.setPath("G28 Z50.0")
                isHoming = True
                cncRunning = True
            
            # Zero position button
            if pygame.Rect(1070, 343, 50, 50).collidepoint(pygame.mouse.get_pos()) and cnc.isConnected() and not cncRunning:
                cnc.setSimulation(False)
                cnc.setVelocity(np.array([0.0, 0.0, 0.0]))
                cnc.setSpindleSpeed(0)
                cnc.resetPosition()
                cnc.setSimulation(True)
                cnc.setPath("\n".join(code))
                isHoming = False
                cncRunning = False
            
            # Control buttons
            if pygame.Rect(745, 343, 50, 50).collidepoint(pygame.mouse.get_pos()) and cnc.isConnected() and not cncRunning:
                cnc.setSimulation(False)
                cnc.writeCNC(3)
                cnc.writeCNC(selectedStepSize)
                cnc.readCNC(t=20)
                cnc.setSimulation(True)
            if pygame.Rect(875, 343, 50, 50).collidepoint(pygame.mouse.get_pos()) and cnc.isConnected() and not cncRunning:
                cnc.setSimulation(False)
                cnc.writeCNC(4)
                cnc.writeCNC(selectedStepSize)
                cnc.readCNC(t=20)
                cnc.setSimulation(True)
            if pygame.Rect(745, 473, 50, 50).collidepoint(pygame.mouse.get_pos()) and cnc.isConnected() and not cncRunning:
                cnc.setSimulation(False)
                cnc.writeCNC(3)
                cnc.writeCNC(-selectedStepSize)
                cnc.readCNC(t=20)
                cnc.setSimulation(True)
            if pygame.Rect(875, 473, 50, 50).collidepoint(pygame.mouse.get_pos()) and cnc.isConnected() and not cncRunning:
                cnc.setSimulation(False)
                cnc.writeCNC(4)
                cnc.writeCNC(-selectedStepSize)
                cnc.readCNC(t=20)
                cnc.setSimulation(True)
            if pygame.Rect(680, 408, 50, 50).collidepoint(pygame.mouse.get_pos()) and cnc.isConnected() and not cncRunning:
                cnc.setSimulation(False)
                cnc.writeCNC(5)
                cnc.writeCNC(-selectedStepSize)
                cnc.readCNC(t=20)
                cnc.setSimulation(True)
            if pygame.Rect(810, 408, 50, 50).collidepoint(pygame.mouse.get_pos()) and cnc.isConnected() and not cncRunning:
                cnc.setSimulation(False)
                cnc.writeCNC(5)
                cnc.writeCNC(selectedStepSize)
                cnc.readCNC(t=20)
                cnc.setSimulation(True)

    # Displays all hovers
    if pygame.Rect(337, 25, 263, 105).collidepoint(pygame.mouse.get_pos()) and cnc.isConnected() and not cncRunning:
        screen.blit(startHover, (337, 25))
    if pygame.Rect(25, 25, 287, 105).collidepoint(pygame.mouse.get_pos()) and not cncRunning and len(code) > 0:
        screen.blit(simulateHover, (25, 25))
    if pygame.Rect(1070, 53, 80, 50).collidepoint(pygame.mouse.get_pos()) and not cncRunning:
        screen.blit(openHover, (1070, 53))
    if pygame.Rect(1005, 408, 115, 115).collidepoint(pygame.mouse.get_pos()) and cncRunning:
        screen.blit(stopHover, (1005, 408))
    if pygame.Rect(950, 156, 225, 105).collidepoint(pygame.mouse.get_pos()) and not cncRunning:
        screen.blit(connectHover, (950, 156))
    if pygame.Rect(640, 208, 80, 38).collidepoint(pygame.mouse.get_pos()) or selectedStepSize == 0.1:
        screen.blit(stepHover, (640, 208))
    if pygame.Rect(735, 208, 80, 38).collidepoint(pygame.mouse.get_pos()) or selectedStepSize == 1:
        screen.blit(stepHover, (735, 208))
    if pygame.Rect(830, 208, 80, 38).collidepoint(pygame.mouse.get_pos()) or selectedStepSize == 10:
        screen.blit(stepHover, (830, 208))
    if pygame.Rect(745, 343, 50, 50).collidepoint(pygame.mouse.get_pos()) and cnc.isConnected() and not cncRunning:
        screen.blit(controlHover, (745, 343))
    if pygame.Rect(875, 343, 50, 50).collidepoint(pygame.mouse.get_pos()) and cnc.isConnected() and not cncRunning:
        screen.blit(controlHover, (875, 343))
    if pygame.Rect(1005, 343, 50, 50).collidepoint(pygame.mouse.get_pos()) and cnc.isConnected() and not cncRunning:
        screen.blit(controlHover, (1005, 343))
    if pygame.Rect(1070, 343, 50, 50).collidepoint(pygame.mouse.get_pos()) and cnc.isConnected() and not cncRunning:
        screen.blit(controlHover, (1070, 343))
    if pygame.Rect(745, 473, 50, 50).collidepoint(pygame.mouse.get_pos()) and cnc.isConnected() and not cncRunning:
        screen.blit(controlHover, (745, 473))
    if pygame.Rect(875, 473, 50, 50).collidepoint(pygame.mouse.get_pos()) and cnc.isConnected() and not cncRunning:
        screen.blit(controlHover, (875, 473))
    if pygame.Rect(680, 408, 50, 50).collidepoint(pygame.mouse.get_pos()) and cnc.isConnected() and not cncRunning:
        screen.blit(controlHover, (680, 408))
    if pygame.Rect(810, 408, 50, 50).collidepoint(pygame.mouse.get_pos()) and cnc.isConnected() and not cncRunning:
        screen.blit(controlHover, (810, 408))

    # Gets position to display
    if (cnc.isConnected() and not cncRunning):
        cnc.setSimulation(False)
        cnc.updatePosition()
        cnc.setSimulation(True)
    else:
        cnc.updatePosition()

    # Displays position
    position = cnc.getPosition()
    text_surface = font.render(f"X: {position[0]:.2f}    Y: {position[1]:.2f}    Z: {position[2]:.2f}", True, (255, 255, 255))
    screen.blit(text_surface, dest=(35,550))
    pygame.draw.circle(screen, (155, 155, 155), (int(position[0]*2+312.5), int(365.5-position[1]*2)), 5)
    
    if (cncRunning):
        # Adds point to display
        if (position[2] <= 0.05):
            displayPoints.add((int(position[0]*2+312.5), int(365.5-position[1]*2)))

        # Stops moving/simulating if complete
        if cnc.step >= len(cnc.path):
            cnc.setVelocity(np.array([0.0, 0.0, 0.0]))
            cnc.setSpindleSpeed(0)
            cnc.updatePosition()
            cnc.setSimulation(True)
            cnc.setPath("\n".join(code))
            isHoming = False
            cncRunning = False
        else:
            # Tries to follow the path
            try:
                cnc.followPath()
            except:
                cnc.setVelocity(np.array([0.0, 0.0, 0.0]))
                cnc.setSpindleSpeed(0)
                cnc.updatePosition()
                cnc.setSimulation(True)
                cnc.setPath("\n".join(code))
                isHoming = False
                cncRunning = False
                textbox.setText("RUNNING ERROR")
    
    # Displays path travelled
    for i in displayPoints:
        screen.set_at(i, (255, 255, 255))

    pygame_widgets.update(events)
    pygame.display.update()