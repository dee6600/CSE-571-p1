import matplotlib.pyplot as plt
import pandas as pd


# Load data
data = pd.read_csv('hw1_results.csv')

# get sum of data['time'] column from data frame for same data['algorithm']

data2 = data.groupby(['Dimension',' Algorithm'])[' Time'].mean()

print(data2)

data2.plot(x=' Algorithm', y='Time', kind='line')
data2.plot(x=' Dimension', y='Time', kind='line')
plt.show()