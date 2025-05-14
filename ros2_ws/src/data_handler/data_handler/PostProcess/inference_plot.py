import csv
import os
import matplotlib.pyplot as plt

def calculate_average_inference_time(file_path):
    """
    Calculate the average inference time from a CSV file.
    
    Args:
        file_path (str): Path to the CSV file
    Returns:
        float: Average inference time in seconds
    """
    with open(file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip header row
        inference_times = [float(row[1]) for row in reader]
        return sum(inference_times) / len(inference_times)

def process_files(file_paths):
    """
    Process multiple CSV files and calculate their average inference times.
    
    Args:
        file_paths (list): List of paths to CSV files
    Returns:
        dict: Dictionary with file names as keys and average inference times as values
    """
    averages = {}
    for file_path in file_paths:
        file_name = os.path.basename(file_path)
        average = calculate_average_inference_time(file_path)
        averages[file_name] = average
    return averages

def create_histogram(averages):
    """
    Create a histogram (bar chart) of average inference times.
    
    Args:
        averages (dict): Dictionary with file names and their average inference times
    """
    file_names = list(averages.keys())
    avg_times = list(averages.values())
    
    plt.figure(figsize=(10, 6))
    plt.bar(file_names, avg_times)
    plt.xlabel('File Name')
    plt.ylabel('Average Inference Time (s)')
    plt.title('Average Inference Time per CSV File')
    plt.xticks(rotation=45, ha='right')
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    # Replace with your actual CSV file paths
    file_paths = ['data1.csv', 'data2.csv', 'data3.csv']
    averages = process_files(file_paths)
    create_histogram(averages)