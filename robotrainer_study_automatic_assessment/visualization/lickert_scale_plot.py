import matplotlib.pyplot as plt
import numpy as np

# Data
questions = ['Effort', 'Intuitiveness', 'Safety']

# Number of responses for each question on a 5-point Likert scale
responses = np.array([
    [11, 25, 11, 13, 3],   # Q1 (Effort)
    [1, 2, 26, 47, 42],    # Q2 (Intuitiveness)
    [0., 0., 9, 17, 89],   # Q3 (Safety)
])

# Scale responses to percentages
responses = np.round((responses / responses.sum(axis=1, keepdims=True)) * 100, 1)

# Likert scale labels and colors (with greenish agree tones)
likert_points = ['Strongly Disagree', 'Disagree', 'Neutral', 'Agree', 'Strongly Agree']
colors = ['#d73027', '#fc8d59', '#fee08b', '#66c2a5', '#1b9e77']

# Setup
fig, ax = plt.subplots(figsize=(8, 4))
bottom = np.zeros(len(questions))

# Plot stacked bars
for i in range(responses.shape[1]):
    ax.barh(questions, responses[:, i], left=bottom, color=colors[i], label=likert_points[i])
    bottom += responses[:, i]

ax.axvline(50, color='gray', linestyle='--', linewidth=1)

# Add combined % of Agree + Strongly Agree inside the bars (rounded to int)
for i, (agree, strongly_agree) in enumerate(responses[:, 3:5]):
    total_agree = int(round(agree + strongly_agree))
    left_position = 100 - total_agree / 2
    ax.text(left_position, i, '{}% total agree'.format(total_agree), va='center', ha='center', color='white', weight='bold')

# Format plot
ax.set_xlabel('Percentage of Responses')

# Remove the frame (spines)
for spine in ax.spines.values():
    spine.set_visible(False)

# Remove title
ax.set_title('')

# Legend above the plot
ax.legend(bbox_to_anchor=(0.5, 1.15), loc='upper center', ncol=len(likert_points), frameon=False)

plt.tight_layout()
plt.show()
