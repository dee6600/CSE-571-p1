import matplotlib.pyplot as plt
import pandas as pd
import os

# Load data
data = pd.read_csv('hw1_results.csv')

# only keep the data[' Time'] column, data['Dimension'] column, data['Algorithm'] column

data2 = data[['Dimension', ' Algorithm', ' Time']]

# #calculate mean of data[' Time'] column for same data['algorithm'] and data['dimension']

data3 = data2.groupby(['Dimension', ' Algorithm'])[' Time'].mean()


# put all astar data into a list

astar_list = []
for i in range(1, len(data3)):
    if data3.index[i][1] == 'astar':
        astar_list.append(data3.index[i][0])


print(astar_list)

    


# data2.plot(x=' Algorithm', y='Time', kind='line')
# data2.plot(x=' Dimension', y='Time', kind='line')
plt.legend(loc='upper right')
#plt.show()