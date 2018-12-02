from matplotlib import pyplot as plt
import numpy as np
from pandas import *

# TODO
# read in "prob.csv"
# work out the matrix size (n) based on the number of commas in the first row
# push n rows into an array, then plot it using the 
# loop through til end of file

# sample 4x4 array!
results = np.array([[0.000e+00,0.000e+00,0.000e+00,0.000e+00],[8.071e-02,0.000e+00,0.000e+00,0.000e+00],[8.071e-02,9.193e-01,0.000e+00,0.000e+00],[8.071e-02,8.071e-02,8.071e-02,0.000e+00]])

df = DataFrame(results) # put into pandas dataframe
fig, ax = plt.subplots() # preparing figure objects
ax.matshow(results, cmap=plt.cm.hot) # choose which one you like! https://matplotlib.org/examples/color/colormaps_reference.html 
plt.show() # display it