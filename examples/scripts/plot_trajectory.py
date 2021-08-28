#!/usr/bin/python3

import csv
import fileinput
import numpy as np
import matplotlib.pyplot as plt

with fileinput.input() as f_input:
    data_iter = csv.reader(f_input, delimiter = ',', quotechar = '"')
    data = [data for data in data_iter]
data_array = np.asarray(data, dtype=np.float64)

plt.plot( * data_array[0:16,0:2].T , 'bo', markersize=12)
plt.plot( * data_array[16:,0:2].T , 'ro--')

# ax = plt.gca()
# ax.axes.xaxis.set_visible(False)
# ax.axes.yaxis.set_visible(False)

plt.show()