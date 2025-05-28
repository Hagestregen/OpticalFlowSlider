import csv
import os
import matplotlib.pyplot as plt
import numpy as np
from tabulate import tabulate
import argparse

def calculate_inference_times(file_path):
    """
    Calculate the average inference time and collect all inference times from a CSV file.
    
    Args:
        file_path (str): Path to the CSV file
    Returns:
        tuple: (average inference time in seconds, list of inference times)
    """
    with open(file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip header row
        inference_times = [float(row[1]) for row in reader]
        average = sum(inference_times) / len(inference_times)
        return average, inference_times

def get_files_for_resolution(base_dir, resolution):
    """
    Get a list of (method_name, file_path) tuples for CSV files matching the specified resolution.
    
    Args:
        base_dir (str): Base directory to search for CSV files
        resolution (str): Resolution string (e.g., '640x480' or '960x540')
    Returns:
        list: List of tuples containing (method_name, file_path)
    """
    result_files = []
    for root, dirs, file_list in os.walk(base_dir):
        for file in file_list:
            if file.endswith(f"_{resolution}.csv"):
                method_name = os.path.basename(root)
                result_files.append((method_name, os.path.join(root, file)))
    return result_files

def process_resolution_files(files, resolution):
    """
    Process a list of (method_name, file_path) tuples and calculate their average inference times.
    
    Args:
        files (list): List of tuples containing (method_name, file_path)
        resolution (str): Resolution string (e.g., '640x480' or '960x540')
    Returns:
        list: List of dictionaries containing method, resolution, average, and times
    """
    data = []
    for method, file_path in files:
        average, times = calculate_inference_times(file_path)
        data.append({
            'method': method,
            'resolution': resolution,
            'average': average,
            'times': times,
            'fps': 1 / average
        })
    return data

def create_summary_table(data):
    """
    Create and print a summary table of inference times and FPS for all methods and resolutions.
    
    Args:
        data (list): List of dictionaries containing method, resolution, average, and fps
    """
    table = [[d['method'], d['resolution'], f"{d['average']:.6f}", f"{d['fps']:.2f}"] for d in data]
    print(tabulate(table, headers=['Method', 'Resolution', 'Avg Time (s)', 'FPS'], tablefmt='grid'))

def save_summary_to_csv(data, filename="inference_summary.csv"):
    """
    Save the summary data to a CSV file.
    
    Args:
        data (list): List of dictionaries containing method, resolution, average, and fps
        filename (str): Name of the CSV file to save the data to
    """
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Method', 'Resolution', 'Avg Time (s)', 'FPS'])
        for d in data:
            writer.writerow([d['method'], d['resolution'], f"{d['average']:.6f}", f"{d['fps']:.2f}"])

def create_fps_histogram(data, resolution, show_labels=True, exclude_methods=None):
    """
    Create and save a bar chart of average FPS per method for the specified resolution.
    
    Args:
        data (list): List of dictionaries containing method, resolution, average, and fps
        resolution (str): Resolution string (e.g., '640x480' or '960x540')
        show_labels (bool): Whether to show data labels on the bars (default: True)
        exclude_methods (list, optional): List of method names to exclude
    """
    filtered_data = [d for d in data if d['resolution'] == resolution]
    if exclude_methods:
        filtered_data = [d for d in filtered_data if d['method'] not in exclude_methods]
    
    sorted_data = sorted(filtered_data, key=lambda x: x['fps'], reverse=True)
    fps_values = [d['fps'] for d in sorted_data]
    labels = [d['method'] for d in sorted_data]
    
    plt.figure(figsize=(12, 8))
    bars = plt.bar(labels, fps_values)
    
    if show_labels:
        for bar in bars:
            yval = bar.get_height()
            plt.text(bar.get_x() + bar.get_width()/2, yval, f'{yval:.2f}', ha='center', va='bottom')
    
    plt.xlabel('Method')
    plt.ylabel('Average Hz')
    plt.title(f'Average Hz Per Method ({resolution})')
    plt.xticks(rotation=45, ha='right')
    plt.yticks(np.arange(0, max(fps_values) + 10, step=10))
    plt.tight_layout()
    plt.savefig(f'average_hz_per_method_{resolution}{"_filtered" if exclude_methods else ""}.png')
    plt.show()

def create_inference_time_histogram(data, resolution, show_labels=True):
    """
    Create and save a bar chart of average inference time in milliseconds per method.
    
    Args:
        data (list): List of dictionaries containing method, resolution, and average
        resolution (str): Resolution string (e.g., '640x480' or '960x540')
        show_labels (bool): Whether to show data labels on the bars (default: True)
    """
    filtered_data = [d for d in data if d['resolution'] == resolution]
    sorted_data = sorted(filtered_data, key=lambda x: x['average'])
    inference_times_ms = [d['average'] * 1000 for d in sorted_data]
    labels = [d['method'] for d in sorted_data]
    
    plt.figure(figsize=(12, 8))
    bars = plt.bar(labels, inference_times_ms)
    
    if show_labels:
        for bar in bars:
            yval = bar.get_height()
            plt.text(bar.get_x() + bar.get_width()/2, yval, f'{yval:.2f}', ha='center', va='bottom')
    
    plt.xlabel('Method')
    plt.ylabel('Average Inference Time (ms)')
    plt.title(f'Average Inference Time per Method ({resolution})')
    plt.xticks(rotation=45, ha='right')
    plt.yticks(np.arange(0, max(inference_times_ms) + 10, step=10))
    plt.tight_layout()
    plt.savefig(f'average_inference_time_per_method_{resolution}.png')
    plt.show()

def create_combined_fps_histogram(data, show_labels=True, exclude_methods=None):
    """
    Create and save a grouped bar chart of average FPS per method for both resolutions,
    ordered from fastest to slowest based on average FPS across resolutions.
    
    Args:
        data (list): List of dictionaries containing method, resolution, average, and fps
        show_labels (bool): Whether to show data labels on the bars (default: True)
        exclude_methods (list, optional): List of method names to exclude
    """
    # Apply filtering if exclude_methods is provided
    if exclude_methods:
        data = [d for d in data if d['method'] not in exclude_methods]
    
    # Organize FPS data by method and resolution
    method_data = {}
    for d in data:
        method = d['method']
        if method not in method_data:
            method_data[method] = {}
        method_data[method][d['resolution']] = d['fps']
    
    # Calculate average FPS for each method across all resolutions
    method_avg_fps = {}
    for method, res_data in method_data.items():
        fps_values = list(res_data.values())
        method_avg_fps[method] = np.mean(fps_values) if fps_values else 0
    
    # Sort methods by average FPS in descending order (fastest to slowest)
    sorted_methods = sorted(method_avg_fps.keys(), key=lambda m: method_avg_fps[m], reverse=True)
    
    # Define resolutions and plotting parameters
    resolutions = ['640x480', '960x540']
    bar_width = 0.35
    index = np.arange(len(sorted_methods))
    
    # Create the plot
    plt.figure(figsize=(14, 8))
    
    for i, res in enumerate(resolutions):
        fps_values = [method_data[method].get(res, 0) for method in sorted_methods]
        bars = plt.bar(index + i * bar_width, fps_values, bar_width, label=res)
        
        if show_labels:
            for bar in bars:
                yval = bar.get_height()
                if yval > 0:
                    plt.text(bar.get_x() + bar.get_width()/2, yval, f'{yval:.2f}', 
                            ha='center', va='bottom')
    
    # Customize the plot
    plt.xlabel('Method')
    plt.ylabel('Average Hz')
    plt.title('Average Hz Per Method')
    plt.xticks(index + bar_width / 2, sorted_methods, rotation=45, ha='right')
    plt.legend()
    plt.tight_layout()
    
    # Save and display the plot
    plt.savefig(f'average_hz_combined{"_filtered" if exclude_methods else ""}.png')
    plt.show()

def create_combined_inference_time_histogram(data, show_labels=True, exclude_methods=None):
    """
    Create and save a grouped bar chart of average inference time in milliseconds per method for both resolutions.
    
    Args:
        data (list): List of dictionaries containing method, resolution, and average
        show_labels (bool): Whether to show data labels on the bars (default: True)
        exclude_methods (list, optional): List of method names to exclude
    """
    if exclude_methods:
        data = [d for d in data if d['method'] not in exclude_methods]
    
    method_data = {}
    for d in data:
        method = d['method']
        if method not in method_data:
            method_data[method] = {}
        method_data[method][d['resolution']] = d['average'] * 1000  # Convert to ms
    
    # Calculate average inference time across resolutions for sorting
    method_avg = {method: np.mean(list(res.values())) for method, res in method_data.items() if res}
    sorted_methods = sorted(method_avg.keys(), key=lambda m: method_avg[m])
    
    resolutions = ['640x480', '960x540']
    bar_width = 0.35
    index = np.arange(len(sorted_methods))
    
    plt.figure(figsize=(14, 8))
    
    for i, res in enumerate(resolutions):
        inference_times_ms = [method_data[method].get(res, 0) for method in sorted_methods]
        bars = plt.bar(index + i * bar_width, inference_times_ms, bar_width, label=res)
        
        if show_labels:
            for bar in bars:
                yval = bar.get_height()
                if yval > 0:
                    plt.text(bar.get_x() + bar.get_width()/2, yval, f'{yval:.2f}', ha='center', va='bottom')
    
    plt.xlabel('Method')
    plt.ylabel('Average Inference Time (ms)')
    plt.title('Average Inference Time Per Method')
    plt.xticks(index + bar_width / 2, sorted_methods, rotation=45, ha='right')
    plt.legend()
    plt.tight_layout()
    plt.savefig(f'average_inference_time_combined{"_filtered" if exclude_methods else ""}.png')
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process inference times and generate histograms.')
    parser.add_argument('--no-labels', action='store_true', help='Do not show data labels on histograms')
    parser.add_argument('--filter', action='store_true', help='Exclude specified methods from plots')
    args = parser.parse_args()
    
    show_labels = not args.no_labels
    
    base_dir = '/home/docker/OpticalFlowSlider/ros2_ws/inferenceTimes'
    
    files_640 = get_files_for_resolution(base_dir, '640x480')
    files_960 = get_files_for_resolution(base_dir, '960x540')
    
    data_640 = process_resolution_files(files_640, '640x480')
    data_960 = process_resolution_files(files_960, '960x540')
    
    all_data = data_640 + data_960
    
    create_summary_table(all_data)
    save_summary_to_csv(all_data)
    
    exclude_methods = ['LucasKanade_Medium', 'LucasKanade_Heavy'] if args.filter else None
    
    # Generate separate FPS histograms
    create_fps_histogram(all_data, '640x480', show_labels=show_labels, exclude_methods=exclude_methods)
    create_fps_histogram(all_data, '960x540', show_labels=show_labels, exclude_methods=exclude_methods)
    
    # Generate separate inference time histograms
    create_inference_time_histogram(all_data, '640x480', show_labels=show_labels)
    create_inference_time_histogram(all_data, '960x540', show_labels=show_labels)
    
    # Generate combined FPS histogram
    create_combined_fps_histogram(all_data, show_labels=show_labels, exclude_methods=exclude_methods)
    
    # Generate combined inference time histogram
    create_combined_inference_time_histogram(all_data, show_labels=show_labels, exclude_methods=exclude_methods)
# import csv
# import os
# import matplotlib.pyplot as plt
# from tabulate import tabulate

# def calculate_inference_times(file_path):
#     """
#     Calculate the average inference time and collect all inference times from a CSV file.
    
#     Args:
#         file_path (str): Path to the CSV file
#     Returns:
#         tuple: (average inference time in seconds, list of inference times)
#     """
#     with open(file_path, 'r') as csvfile:
#         reader = csv.reader(csvfile)
#         next(reader)  # Skip header row
#         inference_times = [float(row[1]) for row in reader]
#         average = sum(inference_times) / len(inference_times)
#         return average, inference_times

# def get_files_for_resolution(base_dir, resolution):
#     """
#     Get a list of (method_name, file_path) tuples for CSV files matching the specified resolution.
    
#     Args:
#         base_dir (str): Base directory to search for CSV files
#         resolution (str): Resolution string (e.g., '640x480' or '960x540')
#     Returns:
#         list: List of tuples containing (method_name, file_path)
#     """
#     result_files = []
#     for root, dirs, file_list in os.walk(base_dir):
#         for file in file_list:
#             if file.endswith(f"_{resolution}.csv"):
#                 method_name = os.path.basename(root)
#                 result_files.append((method_name, os.path.join(root, file)))
#     return result_files

# def process_resolution_files(files, resolution):
#     """
#     Process a list of (method_name, file_path) tuples and calculate their average inference times.
    
#     Args:
#         files (list): List of tuples containing (method_name, file_path)
#         resolution (str): Resolution string (e.g., '640x480' or '960x540')
#     Returns:
#         list: List of dictionaries containing method, resolution, average, and times
#     """
#     data = []
#     for method, file_path in files:
#         average, times = calculate_inference_times(file_path)
#         data.append({
#             'method': method,
#             'resolution': resolution,
#             'average': average,
#             'times': times,
#             'fps': 1 / average
#         })
#     return data

# def create_summary_table(data):
#     """
#     Create and print a summary table of inference times and FPS for all methods and resolutions.
    
#     Args:
#         data (list): List of dictionaries containing method, resolution, average, and fps
#     """
#     table = [[d['method'], d['resolution'], f"{d['average']:.6f}", f"{d['fps']:.2f}"] for d in data]
#     print(tabulate(table, headers=['Method', 'Resolution', 'Avg Time (s)', 'FPS'], tablefmt='grid'))

# def create_fps_histogram(data, resolution, exclude_methods=None):
#     """
#     Create and save a bar chart of average FPS per method for the specified resolution.
    
#     Args:
#         data (list): List of dictionaries containing method, resolution, average, and fps
#         resolution (str): Resolution string (e.g., '640x480' or '960x540')
#         exclude_methods (list, optional): List of method names to exclude
#     """
#     filtered_data = [d for d in data if d['resolution'] == resolution]
#     if exclude_methods:
#         filtered_data = [d for d in filtered_data if d['method'] not in exclude_methods]
    
#     sorted_data = sorted(filtered_data, key=lambda x: x['fps'], reverse=True)
#     fps_values = [d['fps'] for d in sorted_data]
#     labels = [d['method'] for d in sorted_data]
    
#     plt.figure(figsize=(10, 6))
#     plt.bar(labels, fps_values)
#     plt.xlabel('Method')
#     plt.ylabel('Average Hz')
#     plt.title(f'Average Hz Per Method ({resolution})')
#     plt.xticks(rotation=45, ha='right')
#     plt.tight_layout()
#     plt.savefig(f'average_hz_per_method_{resolution}{"_filtered" if exclude_methods else ""}.png')
#     plt.show()

# def create_inference_time_histogram(data, resolution):
#     """
#     Create and save a bar chart of average inference time in milliseconds per method.
    
#     Args:
#         data (list): List of dictionaries containing method, resolution, and average
#         resolution (str): Resolution string (e.g., '640x480' or '960x540')
#     """
#     filtered_data = [d for d in data if d['resolution'] == resolution]
#     sorted_data = sorted(filtered_data, key=lambda x: x['average'])
#     inference_times_ms = [d['average'] * 1000 for d in sorted_data]  # Convert to ms
#     labels = [d['method'] for d in sorted_data]
    
#     plt.figure(figsize=(10, 6))
#     plt.bar(labels, inference_times_ms)
#     plt.xlabel('Method')
#     plt.ylabel('Average Inference Time (ms)')
#     plt.title(f'Average Inference Time per Method ({resolution})')
#     plt.xticks(rotation=45, ha='right')
#     plt.tight_layout()
#     plt.savefig(f'average_inference_time_per_method_{resolution}.png')
#     plt.show()

# if __name__ == '__main__':
#     base_dir = '/home/docker/OpticalFlowSlider/ros2_ws/inferenceTimes'
    
#     # Get files for each resolution
#     files_640 = get_files_for_resolution(base_dir, '640x480')
#     files_960 = get_files_for_resolution(base_dir, '960x540')
    
#     # Process data for each resolution
#     data_640 = process_resolution_files(files_640, '640x480')
#     data_960 = process_resolution_files(files_960, '960x540')
    
#     # Combine data for table and histograms
#     all_data = data_640 + data_960
    
#     # Create and print summary table
#     create_summary_table(all_data)
    
#     # Create FPS histograms for each resolution
#     create_fps_histogram(all_data, '640x480')
#     create_fps_histogram(all_data, '960x540')
    
#     # Create inference time histograms for each resolution
#     create_inference_time_histogram(all_data, '640x480')
#     create_inference_time_histogram(all_data, '960x540')
    
#     # Create FPS histograms excluding "Lucas-Kanade Light" and "Lucas-Kanade Medium"
#     exclude_methods = ['Lucas_Kanade_Light', 'LucasKanade_Medium']  # Adjust based on actual method names
#     create_fps_histogram(all_data, '640x480', exclude_methods=exclude_methods)
#     create_fps_histogram(all_data, '960x540', exclude_methods=exclude_methods)

# import csv
# import os
# import matplotlib.pyplot as plt
# from tabulate import tabulate

# def calculate_inference_times(file_path):
#     """
#     Calculate the average inference time and collect all inference times from a CSV file.
    
#     Args:
#         file_path (str): Path to the CSV file
#     Returns:
#         tuple: (average inference time in seconds, list of inference times)
#     """
#     with open(file_path, 'r') as csvfile:
#         reader = csv.reader(csvfile)
#         next(reader)  # Skip header row
#         inference_times = [float(row[1]) for row in reader]
#         average = sum(inference_times) / len(inference_times)
#         return average, inference_times

# def extract_average_cpu_usage(log_file_path):
#     """
#     Extract the average CPU usage from the log file.
    
#     Args:
#         log_file_path (str): Path to the CPU usage log file
#     Returns:
#         float: Average CPU usage percentage, or None if not found
#     """
#     with open(log_file_path, 'r') as log_file:
#         for line in log_file:
#             if line.startswith("Average:"):
#                 parts = line.split()
#                 try:
#                     cpu_usage = float(parts[7])
#                     return cpu_usage
#                 except (IndexError, ValueError):
#                     print(f"Error parsing CPU usage from {log_file_path}")
#                     return None
#     print(f"No average CPU usage found in {log_file_path}")
#     return None

# def process_files(file_paths, log_file_paths):
#     """
#     Process multiple CSV files and calculate their average inference times and collect all times.
#     Also, extract average CPU usage from corresponding log files.
    
#     Args:
#         file_paths (list): List of paths to CSV files
#         log_file_paths (dict): Dictionary mapping file names to log file paths
#     Returns:
#         dict: Dictionary with file names as keys and dicts of average, times, and cpu_usage as values
#     """
#     data = {}
#     for file_path in file_paths:
#         file_name = os.path.basename(file_path)
#         average, times = calculate_inference_times(file_path)
#         log_file_path = log_file_paths.get(file_name)
#         cpu_usage = extract_average_cpu_usage(log_file_path) if log_file_path else None
#         data[file_name] = {'average': average, 'times': times, 'cpu_usage': cpu_usage}
#     return data

# # def create_fps_histogram(data, method_names, file_order):
# #     """
# #     Create and save a bar chart of average FPS per method in the specified order.
    
# #     Args:
# #         data (dict): Dictionary with file names and their data (average)
# #         method_names (dict): Mapping of file names to readable method names
# #         file_order (list): List of file names in the desired order
# #     """
# #     fps_values = [1 / data[file_name]['average'] for file_name in file_order]
# #     labels = [method_names[file_name] for file_name in file_order]
    
# #     plt.figure(figsize=(10, 6))
# #     plt.bar(labels, fps_values)
# #     plt.xlabel('Method')
# #     plt.ylabel('Average FPS')
# #     plt.title('Average Frames Per Second per Method')
# #     plt.xticks(rotation=45, ha='right')
# #     plt.tight_layout()
# #     plt.savefig('average_fps_per_method.png')

# def create_fps_histogram(data, method_names, file_order):
#     """
#     Create and save a bar chart of average FPS per method in the specified order.
    
#     Args:
#         data (dict): Dictionary with file names and their average inference times
#         method_names (dict): Mapping of file names to readable method names
#         file_order (list): List of file names in the desired order
#     """
#     fps_values = [1 / data[file_name] for file_name in file_order]
#     labels = [method_names[file_name] for file_name in file_order]
    
#     plt.figure(figsize=(10, 6))
#     plt.bar(labels, fps_values)
#     plt.xlabel('Method')
#     plt.ylabel('Average FPS')
#     plt.title('Average Frames Per Second per Method')
#     plt.xticks(rotation=45, ha='right')
#     plt.tight_layout()
#     plt.savefig('average_fps_per_method.png')
#     plt.show()

# def create_cpu_histogram(data, method_names, file_order):
#     """
#     Create and save a bar chart of average CPU usage per method in the specified order.
    
#     Args:
#         data (dict): Dictionary with file names and their data (cpu_usage)
#         method_names (dict): Mapping of file names to readable method names
#         file_order (list): List of file names in the desired order
#     """
#     cpu_values = [data[file_name]['cpu_usage'] for file_name in file_order]
#     labels = [method_names[file_name] for file_name in file_order]
    
#     plt.figure(figsize=(10, 6))
#     plt.bar(labels, cpu_values)
#     plt.xlabel('Method')
#     plt.ylabel('Average CPU Usage (%)')
#     plt.title('Average CPU Usage per Method')
#     plt.xticks(rotation=45, ha='right')
#     plt.tight_layout()
#     plt.savefig('average_cpu_usage_per_method.png')

# def create_inference_time_histogram(data, method_names, file_order):
#     """
#     Create and save a bar chart of average inference time in milliseconds per method in the specified order.
    
#     Args:
#         data (dict): Dictionary with file names and their data (average)
#         method_names (dict): Mapping of file names to readable method names
#         file_order (list): List of file names in the desired order
#     """
#     inference_times_ms = [data[file_name]['average'] * 1000 for file_name in file_order]  # Convert seconds to ms
#     labels = [method_names[file_name] for file_name in file_order]
    
#     plt.figure(figsize=(10, 6))
#     plt.bar(labels, inference_times_ms)
#     plt.xlabel('Method')
#     plt.ylabel('Average Inference Time (ms)')
#     plt.title('Average Inference Time per Method')
#     plt.xticks(rotation=45, ha='right')
#     plt.tight_layout()
#     plt.savefig('average_inference_time_per_method.png')

# if __name__ == '__main__':
#     file_paths = [
#         '/home/docker/OpticalFlowSlider/ros2_ws/inferenceTimes/LucasKanade/lk_heavy_640x480.csv',
#         '/home/docker/OpticalFlowSlider/ros2_ws/inferenceTimes/LucasKanade/lk_light_640x480.csv',
#         '/home/docker/OpticalFlowSlider/ros2_ws/inferenceTimes/LucasKanade/lk_medium_640x480.csv'
#     ]
    
#     # Define readable method names
#     method_names = {
#         'lk_light_640x480.csv': 'LK Light',
#         'lk_medium_640x480.csv': 'LK Medium',
#         'lk_heavy_640x480.csv': 'LK Heavy',
#     }
    
#     # Map CSV file names to their corresponding CPU usage log files
#     log_file_paths = {
#         'lk_heavy_640x480.csv': '/home/docker/OpticalFlowSlider/ros2_ws/CPU_Performance/Heavy/cpu_usage_lucas_kanade_accurate_node.log',
#         'lk_light_640x480.csv': '/home/docker/OpticalFlowSlider/ros2_ws/CPU_Performance/Light/cpu_usage_lucas_kanade_light_node.log',
#         'lk_medium_640x480.csv': '/home/docker/OpticalFlowSlider/ros2_ws/CPU_Performance/Medium/cpu_usage_lucas_kanade_node.log',
#     }
    
#     # Process data
#     data = process_files(file_paths, log_file_paths)
    
#     # Sort file names by FPS descending (fastest to slowest)
#     sorted_file_names = sorted(data.keys(), key=lambda x: 1 / data[x]['average'], reverse=True)
    
#     # Create and print table in sorted order with CPU usage
#     table = [[method_names[file_name], 
#               f"{data[file_name]['average']:.6f}", 
#               f"{1 / data[file_name]['average']:.2f}", 
#               f"{data[file_name]['cpu_usage']:.2f}%" if data[file_name]['cpu_usage'] is not None else "N/A"] 
#              for file_name in sorted_file_names]
#     print(tabulate(table, headers=['Method', 'Avg Time (s)', 'FPS', 'CPU Usage'], tablefmt='grid'))
    
#     # Create and save histograms in sorted order
#     averages = {file_name: data[file_name]['average'] for file_name in sorted_file_names}
#     create_fps_histogram(averages, method_names, sorted_file_names)
#     create_cpu_histogram(data, method_names, sorted_file_names)
#     create_inference_time_histogram(data, method_names, sorted_file_names)

# import csv
# import os
# import matplotlib.pyplot as plt
# from tabulate import tabulate

# def calculate_inference_times(file_path):
#     """
#     Calculate the average inference time and collect all inference times from a CSV file.
    
#     Args:
#         file_path (str): Path to the CSV file
#     Returns:
#         tuple: (average inference time in seconds, list of inference times)
#     """
#     with open(file_path, 'r') as csvfile:
#         reader = csv.reader(csvfile)
#         next(reader)  # Skip header row
#         inference_times = [float(row[1]) for row in reader]
#         average = sum(inference_times) / len(inference_times)
#         return average, inference_times

# def process_files(file_paths):
#     """
#     Process multiple CSV files and calculate their average inference times and collect all times.
    
#     Args:
#         file_paths (list): List of paths to CSV files
#     Returns:
#         dict: Dictionary with file names as keys and dicts of average and times as values
#     """
#     data = {}
#     for file_path in file_paths:
#         file_name = os.path.basename(file_path)
#         average, times = calculate_inference_times(file_path)
#         data[file_name] = {'average': average, 'times': times}
#     return data

# def create_histogram(averages, method_names, file_order):
#     """
#     Create and save a bar chart of average FPS per method in the specified order.
    
#     Args:
#         averages (dict): Dictionary with file names and their average inference times
#         method_names (dict): Mapping of file names to readable method names
#         file_order (list): List of file names in the desired order
#     """
#     avg_times = [averages[file_name] for file_name in file_order]
#     fps_values = [1 / time for time in avg_times]
#     labels = [method_names[file_name] for file_name in file_order]
    
#     plt.figure(figsize=(10, 6))
#     plt.bar(labels, fps_values)
#     plt.xlabel('Method')
#     plt.ylabel('Average FPS')
#     plt.title('Average Frames Per Second per Method')
#     plt.xticks(rotation=45, ha='right')
#     plt.tight_layout()
#     plt.savefig('average_fps_per_method.png')
#     plt.show()

# if __name__ == '__main__':
#     file_paths = [
#         '/home/docker/OpticalFlowSlider/ros2_ws/inferenceTimes/LucasKanade/lk_heavy_640x480.csv',
#         '/home/docker/OpticalFlowSlider/ros2_ws/inferenceTimes/LucasKanade/lk_light_640x480.csv',
#         '/home/docker/OpticalFlowSlider/ros2_ws/inferenceTimes/LucasKanade/lk_medium_640x480.csv'
#     ]
    
#     # Define readable method names
#     method_names = {
#         'lk_light_640x480.csv': 'LK Light',
#         'lk_medium_640x480.csv': 'LK Medium',
#         'lk_heavy_640x480.csv': 'LK Heavy',
#     }
    
#     # Process data
#     data = process_files(file_paths)
    
#     # Sort file names by FPS descending (fastest to slowest)
#     sorted_file_names = sorted(data.keys(), key=lambda x: 1 / data[x]['average'], reverse=True)
    
#     # Create and print table in sorted order
#     table = [[method_names[file_name], f"{data[file_name]['average']:.6f}", f"{1 / data[file_name]['average']:.2f}"] 
#              for file_name in sorted_file_names]
#     print(tabulate(table, headers=['Method', 'Avg Time (s)', 'FPS'], tablefmt='grid'))
    
#     # Create and save histogram in sorted order
#     averages = {file_name: data[file_name]['average'] for file_name in sorted_file_names}
#     create_histogram(averages, method_names, sorted_file_names)

