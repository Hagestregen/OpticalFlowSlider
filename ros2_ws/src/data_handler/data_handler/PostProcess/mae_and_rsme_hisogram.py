import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def main():
    # Define the paths to your CSV files
    csv1_path = '/home/docker/OpticalFlowSlider/ros2_ws/src/data_handler/data_handler/PostProcess/MasterThesisPlots/Experiment1/Test 1 RSME 640/metrics_average.csv'  # 640x480 resolution
    csv2_path = '/home/docker/OpticalFlowSlider/ros2_ws/src/data_handler/data_handler/PostProcess/MasterThesisPlots/Experiment1/Test 1 RSME 960/metrics_average.csv'  # 960x540 resolution

    # Load the CSV files into DataFrames
    df1 = pd.read_csv(csv1_path)
    df2 = pd.read_csv(csv2_path)

    # Combine the DataFrames into one
    df = pd.concat([df1, df2], ignore_index=True)

    # Define the order of methods for consistent plotting
    order = ['Lukas Kanade', 'PWC-Net', 'LiteFlowNet3', 'RAFT-Small', 'RAFT-Large']

    # Function to create and save a bar plot for a given metric
    def create_bar_plot(df, metric, filename):
        pivot = df.pivot(index='method', columns='resolution', values=metric)
        methods = pivot.index
        index = range(len(methods))
        bar_width = 0.35

        fig, ax = plt.subplots(figsize=(12, 6))
        ax.bar(index, pivot['640x480'], bar_width, label='640x480', color='#1f77b4')  # Updated to match script2
        ax.bar([i + bar_width for i in index], pivot['960x540'], bar_width, label='960x540', color='#ff7f0e')

        ax.set_xlabel('Method')
        ax.set_ylabel(metric.upper())
        ax.set_title(f'{metric.upper()} by Method and Resolution')
        ax.set_xticks([i + bar_width/2 for i in index])
        ax.set_xticklabels(methods, rotation=45, ha='right')
        ax.legend()

        plt.tight_layout()
        print(f"Saving bar plot to {filename}")
        plt.savefig(filename)
        plt.close()

    # Generate plots for RMSE and MAE
    create_bar_plot(df, 'rmse', 'rmse_comparison.png')
    create_bar_plot(df, 'mae', 'mae_comparison.png')

if __name__ == "__main__":
    main()