#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Nov 21 13:54:54 2018

@author: Paulo Camasmie and Ed Terry
CMU - Learning from Manipulation
"""

import numpy as np
import math

class choreBot(object):

    def __init__(self, n):
        self.n = n    # number of parts
        self.p = 0.5 # prior probability of two parts being connected = link is active
        self.L = []
        self.genLink() # lower triang matrix of links probabilities
        self.E = np.tril(np.full((self.n,self.n),1.0),-1) # initiate it with highest entropy
        self.action = ['N','E','S','W']
        self.direction = ''
        self.distance = 10 # distance in milimeters to move part
        self.count = 0
        self.sureLink = []
        # variables simulation part No: positions that it occupies
        self.partSimula = {0:[(2,2)], 1:[(2,3)],2:[(3,2),(3,3)],3:[]}
        self.gridSize = 6 # square grid for simulation
        self.occupGrid = np.zeros([self.gridSize, self.gridSize]) # occupancy grid simulation
        self.pos = {} # positions from camera part number: tuple position(x,y) {0:(2,3),...,part_n:(25,30)}
        self.posOld = {}
        self.moved=[]
        self.genGrid()
        self.getPos()
                
    # Generates matrix of prior probabilities that each link combination is active
    def genLink(self): # generates ma
        L = np.full((self.n,self.n),self.p)
        self.L = np.tril(L,-1)
    
    # Generates simulation grid using partSimula positions
    def genGrid(self):
        self.occupGrid = np.zeros([self.gridSize, self.gridSize])
        for part in self.partSimula:
            for pos in self.partSimula[part]:
                self.occupGrid[pos[0],pos[1]] = 1
        print('Updated Grid position: Parts overlap/ no push')
        print(self.occupGrid)
        
    def getPos(self): # here we get position from simulate grid 
        #TO DO replace below initialization with actual position numbers and location from image
        # scan grid to find occupied cells
        part = 0 # initialize first part with number 0
        for i in range(self.n):
            for j in range(self.n):
                if self.occupGrid[i][j] == 1:
                    self.pos[part] = (i, j)
                    part += 1
             
    def bestAction(self): # TO DO this is just a baseline: Develop actual action policy
        #publish to robot next part to push
        maxEnt = np.argmax(self.E) # link with highest entropy
        highLink = np.unravel_index(maxEnt, self.E.shape) # unravel it into a position tuple
        targetPart = np.random.choice(highLink) # pick a random part of the high link
        self.target = self.pos[targetPart]
        self.direction = np.random.choice(self.action) # pick a random direction
        
    def robotAction(self): # publish to robot next action
        nextAction = (self.target, self.direction, self.distance)
        self.count += 1 # adds to number of actions taken to compare methods

    #method used for simulation only to move part. Happens automatically if robot pushes
    def partMove(self): 
        # map target grid position to actual part to move all related parts
        for part, pos in self.partSimula.items():
            if self.target in pos:
                key = part
        # update all grid positions of part
        positions = self.partSimula[key]
        positionNew = []
        for each in positions:
            eachNew = list(each)
            if (self.direction == 'N'):
                eachNew[0] -= 1*(eachNew[0]>0)
            elif (self.direction == 'E'):
                eachNew[1] += 1*(eachNew[1]<self.n)
            elif (self.direction == 'S'):
                eachNew[0] += 1*(eachNew[0]<self.n)
            else :
                eachNew[1] -= 1*(eachNew[1]>0)
            positionNew.append(tuple(eachNew))
        # update part with new positions = part moved
        self.partSimula[key] = positionNew
        
    def getMoved(self):
        self.moved = [self.posOld[part] != self.pos[part] for part in range(0, self.n)]
        
    def updateBelief(self):
        # iterate through lower triangular portion of matrix L and update probabilitiies links
        for i in range(0,self.n):
            for j in range(0,i): # iterate through row of part just moved
                # check if link is not already determined
                if (i,j) not in self.sureLink:
                    probPrior = self.L[i][j]
                    # these are Bayesian updated similar to HW1. TODO: update these ratios
                    if (self.moved[i] == True and self.moved[j] == True):
                        probPos = 3 * probPrior / (2 + probPrior)
                    elif (self.moved[i] == False and self.moved[j] == False):
                        probPos = probPrior # nothing was learned here
                    else:
                        probPos = 2 * probPrior / (3 -  probPrior)
                    # update L matrix
                    self.L[i][j] = probPos
                    # adds to set link with high certainty to capture that relation
                    # to avoid updating it and moving towards convergence
                    if probPos > .9 or probPos < .1:
                        self.sureLink.append((i,j))
                        print('Found a link between parts: ',i,' and ',j)
        print('Updated Links Probabilities:')
        print(self.L)
        
    # update entropy matrix    
    def updateEntropy(self):
        for i in range(0,self.n):
            for j in range(0,i):
                p = self.L[i][j]
                entropy = -p * math.log(p, 2) - (1-p) * math.log(1-p, 2)
                self.E[i][j] = entropy
                self.E = np.tril(self.E,-1) # extracts lower triangular
        print('Entropy Matrix (lower triangular part # vs part #)')
        print self.E
                                    
                                    
    # Main Class script for interactive perception
    def main(self):
        while True:
            self.genGrid() #simulation only
            self.posOld = self.pos.copy() # captures current position before move
            self.getPos() # TODO: Ed this function receives part numbers and positions from camera
            self.bestAction()
            self.robotAction() # TODO: Ed, this function sends command to robot
            self.partMove() # simulation only
            self.getMoved()
            self.updateBelief()
            self.updateEntropy()
            if np.max(self.E) < 0.1:
                print('ChoreBot has completed decluttering job!')
                print('with ', self.n,' parts in ', self.count,' iterations')
                break
            answer = raw_input('click to continue simulation')
            print('#########################################')
        
# Main        
bot = choreBot(4) # n=4 here is the number of parts. in our case is the number of tags (not of actual parts)
bot.main()