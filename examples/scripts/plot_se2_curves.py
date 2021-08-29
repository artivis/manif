#!/usr/bin/python3

import csv
import fileinput
import numpy as np
import matplotlib.pyplot as plt

with fileinput.input() as f_input:
    data_iter = csv.reader(f_input)
    header_data = next(data_iter)
    data = [data for data in data_iter]

header_array = np.asarray(header_data, dtype=np.int32)
data_array = np.asarray(data, dtype=np.float32)

# Plot points

# plt.plot( * data_array[0:header_array[0],0:2].T , 'bo', markersize=12)
# plt.plot( * data_array[header_array[0]:,0:2].T , 'ro--')

# Plot arrows

# magnitude
r = 0.1

# convert polar (theta, r) to cartesian
u = r * np.cos(data_array[:,2])
v = r * np.sin(data_array[:,2])

plt.quiver(
    data_array[0:header_array[0],0],
    data_array[0:header_array[0],1],
    u[0:header_array[0]],
    v[0:header_array[0]],
    color='b',
    scale=1.5,
)
plt.quiver(
    data_array[header_array[0]:,0],
    data_array[header_array[0]:,1],
    u[header_array[0]:],
    v[header_array[0]:],
    color='r',
)

# ax = plt.gca()
# ax.axes.xaxis.set_visible(False)
# ax.axes.yaxis.set_visible(False)

plt.show()