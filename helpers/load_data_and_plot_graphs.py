"""
This file is used to plot data for agent 9
"""
# Necessary imports
import pickle
from helpers.agent9 import plot_histogram, plot_boxplot

# open file to fetch data and plot after that
with open('../Final Data Files/agent9_data-300-100x100.pkl', 'rb') as f:
    data = pickle.load(f)
    plot_histogram(data)
    plot_boxplot(data, 'Box plots', ['Total Actions', 'Total Examinations', 'Total Movements'],
                        ['total_cost', 'total_examinations', 'total_movements'])
