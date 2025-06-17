# # #!/usr/bin/env python3

# # import matplotlib
# # matplotlib.use('Agg')  # Use non-interactive backend
# # import matplotlib.pyplot as plt
# # import numpy as np

# # # Selected data
# # labels = [
# #     "IMU Filtered Velocity",
# #     "KF Velocity (LFN3)",
# #     "KF Velocity (LK)"
# # ]
# # rmse_values = [0.1655, 0.0250, 0.0131]
# # mae_values = [0.1322, 0.0179, 0.0083]

# # # Set up the figure and axis
# # plt.figure(figsize=(8, 6))
# # x = np.arange(len(labels))  # Positions for the groups
# # width = 0.35  # Width of the bars

# # # Plot the bars
# # plt.bar(x - width/2, rmse_values, width=width, label='RMSE', color='blue')
# # plt.bar(x + width/2, mae_values, width=width, label='MAE', color='orange')

# # # Add labels and title
# # plt.xlabel('Method/Topic')
# # plt.ylabel('Error')
# # plt.title('RMSE and MAE for Kalman Filter Velocity and IMU Filtered Velocity')
# # plt.xticks(x, labels, rotation=45, ha='right')

# # # Add legend
# # plt.legend()

# # # Add value labels on top of each bar
# # for i, v in enumerate(rmse_values):
# #     plt.text(x[i] - width/2, v + 0.001, f'{v:.4f}', ha='center', va='bottom')
# # for i, v in enumerate(mae_values):
# #     plt.text(x[i] + width/2, v + 0.001, f'{v:.4f}', ha='center', va='bottom')

# # # Adjust layout and save the figure
# # plt.tight_layout()
# # plt.savefig('bar_plot_selected_rmse_mae.png')


# import matplotlib
# matplotlib.use('Agg')  # Use non-interactive backend
# import matplotlib.pyplot as plt
# import numpy as np

# # Data
# labels = [
#     "IMU Filtered Velocity",
#     "Lucas Kanade",
#     "KF Velocity using LFN3",
#     "KF Velocity using Lucas Kanade",
#     "Raft-Large"
# ]

# # labels = [
# #     "Lucas Kanade",
# #     "PWC-Net",
# #     "LiteFlowNet3",
# #     "RAFT-Small"
# #     "RAFT-Large"
# # ]


# rmse_values = [0.0488, 0.0457, 0.0380, 0.0380, 0.0528]
# mae_values = [0.0408, 0.0129, 0.0199, 0.0199, 0.0234]

# # Set up the figure and axis
# plt.figure(figsize=(10, 6))  # Adjusted width for five labels
# x = np.arange(len(labels))  # Positions for the groups
# width = 0.35  # Width of the bars

# # Plot the bars
# plt.bar(x - width/2, rmse_values, width=width, label='RMSE', color='#1f77b4')
# plt.bar(x + width/2, mae_values, width=width, label='MAE', color='#ff7f0e')

# # Add labels and title
# plt.xlabel('Method/Topic')
# plt.ylabel('Error')
# plt.title('RMSE and MAE for Kalman Filter Test using Lucas Kanade And RAFT')
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
# plt.savefig('bar_plot_updated_rmse_mae.png')


import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import numpy as np

# Define the labels for the methods
labels = [
    "Lucas Kanade",
    "PWC-Net",
    "LiteFlowNet3",
    "RAFT-Small",
    "RAFT-Large"
]

# Data extracted from the table, ordered according to the labels
rmse_640 = [0.0151, 0.0807, 0.0480, 0.0397, 0.0149]  # RMSE at 640×480
rmse_960 = [0.0141, 0.0355, 0.0253, 0.0196, 0.0274]  # RMSE at 960×540
mae_640 = [0.0082, 0.0507, 0.0239, 0.0277, 0.0090]   # MAE at 640×480
mae_960 = [0.0081, 0.0197, 0.0118, 0.0107, 0.0150]   # MAE at 960×540

# Set up positions and width for bars
x = np.arange(len(labels))
width = 0.35

# RMSE Plot
plt.figure(figsize=(10, 6))
plt.bar(x - width/2, rmse_640, width=width, label='640×480', color='#1f77b4')
plt.bar(x + width/2, rmse_960, width=width, label='960×540', color='#ff7f0e')

# Add value labels for RMSE
for i, v in enumerate(rmse_640):
    plt.text(x[i] - width/2, v + 0.001, f'{v:.4f}', ha='center', va='bottom')
for i, v in enumerate(rmse_960):
    plt.text(x[i] + width/2, v + 0.001, f'{v:.4f}', ha='center', va='bottom')

# Customize RMSE plot
plt.xlabel('Method')
plt.ylabel('RMSE')
plt.title('RMSE for Different Optical Flow Methods at Two Resolutions')
plt.xticks(x, labels, rotation=45, ha='right')
plt.legend()
plt.tight_layout()
plt.savefig('rmse_bar_plot.png')
plt.close()

# MAE Plot
plt.figure(figsize=(10, 6))
plt.bar(x - width/2, mae_640, width=width, label='640×480', color='#1f77b4')
plt.bar(x + width/2, mae_960, width=width, label='960×540', color='#ff7f0e')

# Add value labels for MAE
for i, v in enumerate(mae_640):
    plt.text(x[i] - width/2, v + 0.001, f'{v:.4f}', ha='center', va='bottom')
for i, v in enumerate(mae_960):
    plt.text(x[i] + width/2, v + 0.001, f'{v:.4f}', ha='center', va='bottom')

# Customize MAE plot
plt.xlabel('Method')
plt.ylabel('MAE')
plt.title('MAE for Different Optical Flow Methods at Two Resolutions')
plt.xticks(x, labels, rotation=45, ha='right')
plt.legend()
plt.tight_layout()
plt.savefig('mae_bar_plot.png')
plt.close()