# #!/usr/bin/env python3

# import matplotlib
# matplotlib.use('Agg')  # Use non-interactive backend
# import matplotlib.pyplot as plt
# import numpy as np

# # Selected data
# labels = [
#     "IMU Filtered Velocity",
#     "KF Velocity (LFN3)",
#     "KF Velocity (LK)"
# ]
# rmse_values = [0.1655, 0.0250, 0.0131]
# mae_values = [0.1322, 0.0179, 0.0083]

# # Set up the figure and axis
# plt.figure(figsize=(8, 6))
# x = np.arange(len(labels))  # Positions for the groups
# width = 0.35  # Width of the bars

# # Plot the bars
# plt.bar(x - width/2, rmse_values, width=width, label='RMSE', color='blue')
# plt.bar(x + width/2, mae_values, width=width, label='MAE', color='orange')

# # Add labels and title
# plt.xlabel('Method/Topic')
# plt.ylabel('Error')
# plt.title('RMSE and MAE for Kalman Filter Velocity and IMU Filtered Velocity')
# plt.xticks(x, labels, rotation=45, ha='right')

# # Add legend
# plt.legend()

# # Add value labels on top of each bar
# for i, v in enumerate(rmse_values):
#     plt.text(x[i] - width/2, v + 0.001, f'{v:.4f}', ha='center', va='bottom')
# for i, v in enumerate(mae_values):
#     plt.text(x[i] + width/2, v + 0.001, f'{v:.4f}', ha='center', va='bottom')

# # Adjust layout and save the figure
# plt.tight_layout()
# plt.savefig('bar_plot_selected_rmse_mae.png')


import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import numpy as np

# Data
labels = [
    "IMU Filtered Velocity",
    "Lucas Kanade",
    "KF Velocity (RAFT-Large)",
    "KF Velocity (LK)",
    "Raft-Large"
]
rmse_values = [0.0488, 0.0457, 0.0380, 0.0380, 0.0528]
mae_values = [0.0408, 0.0129, 0.0199, 0.0199, 0.0234]

# Set up the figure and axis
plt.figure(figsize=(10, 6))  # Adjusted width for five labels
x = np.arange(len(labels))  # Positions for the groups
width = 0.35  # Width of the bars

# Plot the bars
plt.bar(x - width/2, rmse_values, width=width, label='RMSE', color='blue')
plt.bar(x + width/2, mae_values, width=width, label='MAE', color='orange')

# Add labels and title
plt.xlabel('Method/Topic')
plt.ylabel('Error')
plt.title('RMSE and MAE for Kalman Filter Test using Lucas Kanade And RAFT')
plt.xticks(x, labels, rotation=45, ha='right')

# Add legend
plt.legend()

# Add value labels on top of each bar
for i, v in enumerate(rmse_values):
    plt.text(x[i] - width/2, v + 0.001, f'{v:.4f}', ha='center', va='bottom')
for i, v in enumerate(mae_values):
    plt.text(x[i] + width/2, v + 0.001, f'{v:.4f}', ha='center', va='bottom')

# Adjust layout and save the figure
plt.tight_layout()
plt.savefig('bar_plot_updated_rmse_mae.png')