#import modules
import matplotlib.pyplot as plt
import pandas as pd


# Load data_t3
data = pd.read_csv('hw1_results.csv')

# only keep the data_t3[' Nodes Expanded'] column, data_t3['Dimension'] column, data_t3['Algorithm'] column

data = data[['Dimension', ' Algorithm', ' Nodes Expanded', ' Time']]

# #calculate mean of data_t3[' Nodes Expanded'] column for same data_t3['algorithm'] and data_t3['dimension']

data_t2 = data.groupby(['Dimension', ' Algorithm'])[' Time'].mean()
data_t3= data.groupby(['Dimension', ' Algorithm'])[' Nodes Expanded'].mean()

data_t2 = data_t2.reset_index()
data_t3 = data_t3.reset_index()


# put all algorithms data_t3 into a seperate lists
bfs_list_t2 = []
ucs_list_t2 = []
gbfs_list_t2 = []
astar_list_t2 = []
custom_astar_list_t2 = []

bfs_list_t3 = []
ucs_list_t3 = []
gbfs_list_t3 = []
astar_list_t3 = []
custom_astar_list_t3 = []




for i in range(len(data_t2)):
    if data_t2.iloc[i][' Algorithm'] == ' bfs':
        bfs_list_t2.append((data_t2.iloc[i]['Dimension'], data_t2.iloc[i][' Time']))
    if data_t2.iloc[i][' Algorithm'] == ' ucs':
        ucs_list_t2.append((data_t2.iloc[i]['Dimension'], data_t2.iloc[i][' Time']))
    if data_t2.iloc[i][' Algorithm'] == ' gbfs':
        gbfs_list_t2.append((data_t2.iloc[i]['Dimension'], data_t2.iloc[i][' Time']))
    if data_t2.iloc[i][' Algorithm'] == ' astar':
        astar_list_t2.append((data_t2.iloc[i]['Dimension'], data_t2.iloc[i][' Time']))
    if data_t2.iloc[i][' Algorithm'] == ' custom-astar':
        custom_astar_list_t2.append((data_t2.iloc[i]['Dimension'], data_t2.iloc[i][' Time']))


for i in range(len(data_t3)):
    if data_t3.iloc[i][' Algorithm'] == ' bfs':
        bfs_list_t3.append((data_t3.iloc[i]['Dimension'], data_t3.iloc[i][' Nodes Expanded']))
    if data_t3.iloc[i][' Algorithm'] == ' ucs':
        ucs_list_t3.append((data_t3.iloc[i]['Dimension'], data_t3.iloc[i][' Nodes Expanded']))
    if data_t3.iloc[i][' Algorithm'] == ' gbfs':
        gbfs_list_t3.append((data_t3.iloc[i]['Dimension'], data_t3.iloc[i][' Nodes Expanded']))
    if data_t3.iloc[i][' Algorithm'] == ' astar':
        astar_list_t3.append((data_t3.iloc[i]['Dimension'], data_t3.iloc[i][' Nodes Expanded']))
    if data_t3.iloc[i][' Algorithm'] == ' custom-astar':
        custom_astar_list_t3.append((data_t3.iloc[i]['Dimension'], data_t3.iloc[i][' Nodes Expanded']))


SORT_ORDER = {"4x4": 0, "3x6": 1, "8x8": 2, "12x12": 3, "16x16": 4}

bfs_list_t2.sort(key=lambda val: SORT_ORDER[val[0]])
ucs_list_t2.sort(key=lambda val: SORT_ORDER[val[0]])
gbfs_list_t2.sort(key=lambda val: SORT_ORDER[val[0]])
astar_list_t2.sort(key=lambda val: SORT_ORDER[val[0]])
custom_astar_list_t2.sort(key=lambda val: SORT_ORDER[val[0]])

bfs_list_t3.sort(key=lambda val: SORT_ORDER[val[0]])
ucs_list_t3.sort(key=lambda val: SORT_ORDER[val[0]])
gbfs_list_t3.sort(key=lambda val: SORT_ORDER[val[0]])
astar_list_t3.sort(key=lambda val: SORT_ORDER[val[0]])
custom_astar_list_t3.sort(key=lambda val: SORT_ORDER[val[0]])

#plot two line plots

#plot t2

# time vs dimesntion for all algorithms
plot1 = plt.figure(1)

plt.plot([i[0] for i in bfs_list_t2], [i[1] for i in bfs_list_t2], label='bfs')
plt.plot([i[0] for i in ucs_list_t2], [i[1] for i in ucs_list_t2], label='ucs')
plt.plot([i[0] for i in gbfs_list_t2], [i[1] for i in gbfs_list_t2], label='gbfs')
plt.plot([i[0] for i in astar_list_t2], [i[1] for i in astar_list_t2], label='astar')
plt.plot([i[0] for i in custom_astar_list_t2], [i[1] for i in custom_astar_list_t2], label='custom-astar')
plt.legend()
plt.xlabel('Dimension')
plt.ylabel('Time')
plt.title('Time vs Dimension')

# plot t3
# nodes vs dimesntion for all algorithms

plot2 = plt.figure(2)
plt.plot([i[0] for i in bfs_list_t3], [i[1] for i in bfs_list_t3], label='bfs')
plt.plot([i[0] for i in ucs_list_t3], [i[1] for i in ucs_list_t3], label='ucs')
plt.plot([i[0] for i in gbfs_list_t3], [i[1] for i in gbfs_list_t3], label='gbfs')
plt.plot([i[0] for i in astar_list_t3], [i[1] for i in astar_list_t3], label='astar')
plt.plot([i[0] for i in custom_astar_list_t3], [i[1] for i in custom_astar_list_t3], label='custom-astar')
plt.legend()
plt.xlabel('Dimension')
plt.ylabel('Nodes Expanded')
plt.title('Nodes Expanded vs Dimension')

#show plots
plt.show()

#coment for git check
