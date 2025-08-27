#!/usr/bin/env python3

import matplotlib.pyplot as plt

# Data
tasks = ['Disturbance\nNone', 'Disturbance\n20 N', 'Disturbance\n40 N']
means = [52.64, 61.11, 70.10]
stds  = [12.51, 14.06, 7.21]

colors = ['skyblue', 'salmon', 'limegreen']

# Bar plot
plt.bar(tasks, means, yerr=stds, capsize=8, color=colors, edgecolor='black')
plt.ylabel('User Input Force (N)')
plt.title('User Input Force Across Tasks (Mean +/- SD)')

# Add significance annotations
plt.text(1, 10, 'T(59) = -2.44\np* = .0176', ha='center', fontsize=10)
plt.text(2, 10, 'T(41) = -3.92\np* = .0003', ha='center', fontsize=10)

# Caption (for report or manuscript)
plt.figtext(0.5, -0.1,
    'Bars show mean +/- SD. Significant differences tested via paired t-test.\n'
    'N = 13 participants. Each 3-4 repetitions. * p < .05, ** p < .01, *** p < .001.',
    wrap=True, horizontalalignment='center', fontsize=9)

plt.tight_layout()
plt.show()
