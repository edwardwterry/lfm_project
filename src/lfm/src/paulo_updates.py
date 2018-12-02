  def updateBelief(self):
        # iterate through lower triangular portion of matrix L and update probabilitiies links
        for i in range(0,self.n):
            for j in range(0,i): # iterate through row of part just moved
                # check if link is not already determined
                if (i,j) not in self.sureLink:
                    probPrior = self.L[i][j]
                    probPos = self.BayesUpdate(probPrior, self.moved[i], self.moved[j])

                    # update L matrix
                    self.L[i][j] = probPos
                    # adds to set link with high certainty to capture that relation
                    # to avoid updating it and moving towards convergence
                    if probPos > .9 or probPos < .1:
                        self.sureLink.append((i,j))
                        print('Found a link between parts: ',i,' and ',j)
        print('Updated Links Probabilities:')
        print(self.L)
        
    # Bayesian update input 1 if moved, 0 if not
    def BayesUpdate(self, probPrior, iMove, jMove):
        # probability description format pImJsL1 = probability
        # that I moved and J remained stationary given that link between
        # IJ is active (values below are arbitrary and can be adjusted)
        pIsJsL1 = 0.5
        pIsJsL0 = 0.5
        pImJsL1 = 0.4
        pIsJmL1 = 0.4
        pImJsL0 = 0.6
        pIsJmL0 = 0.6
        pImJmL1 = 0.6
        pImJmL0 = 0.4
        
        # these are Bayesian updated similar to HW1. TODO: update these ratios
        if (iMove == True and jMove == True):
            probPos = pImJmL1*probPrior/(pImJmL1*probPrior + pImJmL0*(1-probPrior))
        elif (iMove == False and jMove == False):
            probPos = probPrior # nothing was learned here
        else:
            # Case when one of the blocks moved. Probability of I moved and J static
            # given that the link is positive is the same as the probability that I is static
            # and J moved given that the link is positive
            probPos = pImJsL1*probPrior/(pImJsL1*probPrior + pImJsL0*probPrior)        
        return probPos

#####
# added!!
    def bestAction(self):
        #publish to robot next part to push
        maxEnt = np.argmax(self.E) # link with highest entropy
        print('Max Entropy:', np.max(self.E))
        print('Max Entropy link:', maxEnt)
        highLink = np.unravel_index(maxEnt, self.E.shape) # unravel it into a position tuple
        targetPart = np.random.choice(highLink) # pick a random part of the high link
        self.target = self.pos[targetPart]
        x = self.target[0]
        y = self.target[1]
        if  x < self.cx and y < self.cy:
            self.direction = 'N'
        elif x < self.cx and y > self.cy:
            self.direction = 'E'
        elif x > self.cx and y > self.cy:
            self.direction = 'S'
        else:
            self.direction = 'W'
#        self.direction = np.random.choice(self.action) # pick a random direction

#####
#added!!!!
   # update entropy matrix    
    def updateEntropy(self):
        for i in range(0,self.n):
            for j in range(0,i):
                p = self.L[i][j]
                entropy = -p * math.log(p, 2) - (1-p) * math.log(1-p, 2)
                self.E[i][j] = entropy
                self.E = np.tril(self.E,-1) # extracts lower triangular
        print('Entropy Matrix (lower triangular part # vs part #)')
        print('without distance factor')
        print self.E
        # update it considering distance of each part to to center 
        # pre-multiplying Entropy matrix by weight vector
        weight = self.distWeight()
        for i in range(0,len(self.E)):
            for j in range(0,len(self.E[0])):
                self.E[i][j] = self.E[i][j]*weight[i]
        print('Entropy updated with weighted distance')
        print(self.E)
        
    # generates a vector of normalized Euclidean distances between each apparent part
    # and the centroid of the image which can be used to weight on the Entropy matrix
    # to favor that parts farther away from centroid are picked up first for similar
    # entropies. The goal is to induce a more efficient exploratory policy where parts
    # are pulled apart from each other, rather than pushed into each other, for a quicker
    # learning of segmentation
    def distWeight(self):
        pos = self.pos
        weightDist = [math.sqrt((self.cx-pos[_][0])**2 + (self.cy-pos[_][1])**2) for _ in self.pos]
        # normalize it
        normWeight = [_/sum(weightDist) for _ in weightDist]
        print('WEIGHT:',weightDist)
        print('Normalized Weight:', normWeight)
        return normWeight