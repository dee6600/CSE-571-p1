import matplotlib.pyplot as plt
import pandas as pd


# Load data
data = pd.read_csv('hw1_results.csv')

# get sum of data['time'] column from data frame for same data['algorithm']

#data2 = data.groupby(['Dimension',' Algorithm'])[' Time'].mean()

# only keep the data[' Time'] column, data['Dimension'] column, data['Algorithm'] column

data2 = data[['Dimension', ' Algorithm', ' Time']]

#calculate mean of data[' Time'] column for same data['algorithm'] and data['dimension']

data3 = data2.groupby(['Dimension', ' Algorithm'])[' Time'].mean()

data3 = data3.to_frame().reset_index()

#print(data3)

#create seperate data frame for each algorithm

bfs_df = data3[data3[' Algorithm'] == 'bfs']


print(bfs_df)

    


# data2.plot(x=' Algorithm', y='Time', kind='line')
# data2.plot(x=' Dimension', y='Time', kind='line')
plt.legend(loc='upper right')
#plt.show()