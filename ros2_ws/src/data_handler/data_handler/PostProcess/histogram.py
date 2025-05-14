import matplotlib.pyplot as plt
import numpy as np

# Table 1 data
table1_data = [
    # {"Method": "Lucas–Kanade", "Resolution": "640×480", "Mean Inf. Time (s)": 0.00207, "FPS": 482, "Shift (s)": -0.030, "RMSE": 0.1146, "MAE": 0.0557},
    # {"Method": "Lucas–Kanade", "Resolution": "960×540", "Mean Inf. Time (s)": 0.00289, "FPS": 346, "Shift (s)": -0.040, "RMSE": 0.1304, "MAE": 0.0602},
    {"Method": "LiteFlowNet3", "Resolution": "640×480", "Mean Inf. Time (s)": 0.03488, "FPS": 29, "Shift (s)": -0.360, "RMSE": 0.0275, "MAE": 0.0119},
    {"Method": "PWC-Net", "Resolution": "640×480", "Mean Inf. Time (s)": 0.03475, "FPS": 29, "Shift (s)": -0.360, "RMSE": 0.0292, "MAE": 0.0130},
    {"Method": "RAFT-Small", "Resolution": "640×480", "Mean Inf. Time (s)": 0.04671, "FPS": 21, "Shift (s)": -0.380, "RMSE": 0.0323, "MAE": 0.0218},
    {"Method": "PWC-Net", "Resolution": "960×540", "Mean Inf. Time (s)": 0.05188, "FPS": 19, "Shift (s)": -0.390, "RMSE": 0.0300, "MAE": 0.0162},
    {"Method": "LiteFlowNet3", "Resolution": "960×540", "Mean Inf. Time (s)": 0.05806, "FPS": 17, "Shift (s)": -0.400, "RMSE": 0.0250, "MAE": 0.0106},
    {"Method": "RAFT-Small", "Resolution": "960×540", "Mean Inf. Time (s)": 0.08660, "FPS": 12, "Shift (s)": -0.440, "RMSE": 0.0202, "MAE": 0.0105},
    {"Method": "RAFT-Large", "Resolution": "640×480", "Mean Inf. Time (s)": 0.08776, "FPS": 11, "Shift (s)": -0.450, "RMSE": 0.0229, "MAE": 0.0107},
    {"Method": "RAFT-Large", "Resolution": "960×540", "Mean Inf. Time (s)": 0.15923, "FPS": 6, "Shift (s)": -0.560, "RMSE": 0.0247, "MAE": 0.0121}
]

# Table 2 data
table2_data = [
    {"Topic": "IMU (filtered)", "avg_rmse": 0.0902, "avg_mae": 0.0732, "avg_shift": None, "avg_rmse_shift": None, "avg_mae_shift": None},
    {"Topic": "LFN3 flow", "avg_rmse": 0.0622, "avg_mae": 0.0279, "avg_shift": -0.078, "avg_rmse_shift": 0.0281, "avg_mae_shift": 0.0112},
    {"Topic": "LFN3 smooth", "avg_rmse": 0.0771, "avg_mae": 0.036, "avg_shift": -0.113, "avg_rmse_shift": 0.0249, "avg_mae_shift": 0.0102},
    {"Topic": "KF state vel", "avg_rmse": 0.0545, "avg_mae": 0.0293, "avg_shift": None, "avg_rmse_shift": None, "avg_mae_shift": None}
]

# Plot 1: Bar plot of RMSE for different methods and resolutions
methods_resolutions = [f"{d['Method']} {d['Resolution']}" for d in table1_data]
rmse_values = [d['RMSE'] for d in table1_data]

plt.figure(figsize=(12, 6))
plt.bar(methods_resolutions, rmse_values, color='skyblue')
plt.xticks(rotation=45, ha='right')
plt.ylabel('RMSE')
plt.title('RMSE for Different Optical Flow Methods and Resolutions')
plt.tight_layout()
plt.savefig('rmse_methods.png')
plt.close()

# Plot 2: Bar plot of FPS for different methods and resolutions
fps_values = [d['FPS'] for d in table1_data]

plt.figure(figsize=(12, 6))
plt.bar(methods_resolutions, fps_values, color='green')
plt.xticks(rotation=45, ha='right')
plt.ylabel('FPS')
plt.title('FPS for Different Optical Flow Methods and Resolutions')
plt.tight_layout()
plt.savefig('fps_methods.png')
plt.close()

# Plot 3: Grouped bar plot of average RMSE before and after shift
topics = [d['Topic'] for d in table2_data]
avg_rmse = [d['avg_rmse'] for d in table2_data]
avg_rmse_shift = [d['avg_rmse_shift'] if d['avg_rmse_shift'] is not None else np.nan for d in table2_data]

x = np.arange(len(topics))
width = 0.35

plt.figure(figsize=(10, 6))
plt.bar(x - width/2, avg_rmse, width, label='RMSE', color='salmon')
plt.bar(x + width/2, avg_rmse_shift, width, label='RMSE: Optical Flow Shifted', color='lightblue')
plt.xticks(x, topics, rotation=45, ha='right')
plt.ylabel('Average RMSE')
plt.title('Average RMSE Before and After Shift for Different Topics')
plt.legend()
plt.tight_layout()
plt.savefig('avg_rmse_shift.png')
plt.close()

print("Plots have been saved as 'rmse_methods.png', 'fps_methods.png', and 'avg_rmse_shift.png'.")