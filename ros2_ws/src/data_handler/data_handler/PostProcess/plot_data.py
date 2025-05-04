#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

# Load the combined data from CSV
data = pd.read_csv('combined_data.csv')

# Extract columns
timestamps = data['timestamp']
velocity = data['velocity']
accel_x = data['accel_x']

# Create a figure with dual y-axes
fig, ax1 = plt.subplots(figsize=(12, 6))

# Plot velocity on the left y-axis
ax1.plot(timestamps, velocity, 'b-', label='Velocity (m/s)')
ax1.set_xlabel('Timestamp (s)')
ax1.set_ylabel('Velocity (m/s)', color='blue')
ax1.tick_params(axis='y', labelcolor='blue')

# Create a second y-axis for acceleration
ax2 = ax1.twinx()
ax2.plot(timestamps, accel_x, 'r-', label='Acceleration X (m/s²)')
ax2.set_ylabel('Acceleration X (m/s²)', color='red')
ax2.tick_params(axis='y', labelcolor='red')

# Add titles and legend
plt.title('Velocity and IMU Acceleration X vs. Time')
fig.legend(loc='upper right', bbox_to_anchor=(0.95, 0.95))

# Adjust layout to prevent overlap
plt.tight_layout()

# Show the plot
plt.show()