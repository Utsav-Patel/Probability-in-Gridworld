import pickle
from helpers.agent6 import plot_boxplot, plot_histogram

key_str = 'total_actions'
agent6_data = pickle.load(open('../Final Data Files/agent6_100_grids_100x100_forest_target.pkl', 'rb'))
agent7_data = pickle.load(open('../Final Data Files/agent7_100_grids_100x100_forest_target.pkl', 'rb'))
agent8_data = pickle.load(open('../Final Data Files/agent8_100_grids_100x100_forest_target.pkl', 'rb'))

# for key in agent6_data:
#     print(key)
#     print('Agent6 :', (sum(agent6_data[key]) / len(agent6_data[key])))
#     print('Agent7 :', (sum(agent7_data[key]) / len(agent7_data[key])))
#     print('Agent8 :', (sum(agent8_data[key]) / len(agent8_data[key])))
# print(len(agent6_data[key_str]))
# plot_histogram(data)
plot_boxplot([agent6_data[key_str], agent7_data[key_str], agent8_data[key_str]], 'Box plots of ' + key_str,
             ['Agent 6', 'Agent 7', 'Agent 8'])
plot_histogram([agent6_data[key_str], agent7_data[key_str], agent8_data[key_str]], ['Agent 6', 'Agent 7', 'Agent 8'],
               'Histogram of ' + key_str)
