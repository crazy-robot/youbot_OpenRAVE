#!/usr/bin/env python
# a bar plot with errorbars
import numpy as np
import matplotlib.pyplot as plt

N = 5
menMeans = (15, 15,10,5,2)

ind = np.arange(N)  # the x locations for the groups
width = 0.35       # the width of the bars

fig = plt.figure()
ax = fig.add_subplot(111)
rects1 = ax.bar(ind, menMeans, width, color='b')

womenMeans = (10,5.3,3.6,1.2,0.2)
rects2 = ax.bar(ind+width, womenMeans, width, color='g')

# add some
ax.set_ylabel('Duration')
ax.set_xlabel('Different Scenarios')
ax.set_title('Comparison of Planning Duration')
ax.set_xticks(ind+width)
ax.set_xticklabels( ('S1', 'S2', 'S3', 'S4', 'S5') )

ax.legend( (rects1[0], rects2[0]), ('initial wieghts', 'adjusted wieghts') )


plt.show()
