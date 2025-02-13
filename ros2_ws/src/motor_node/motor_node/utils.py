#!/usr/bin/env python3
"""
utils.py

Utility function for converting motor RPM to linear velocity in m/s.
"""

def rpm_to_linear_velocity_mps(rpm: float) -> float:
    """
    Convert a motor's velocity from RPM to linear velocity in m/s.

    Based on:
      - Gear ratio: 8.7 (87 teeth / 10 teeth)
      - GT2 Pulley diameter: 32 mm (circumference ≈ 3.14159 * 32 ≈ 100.53 mm)
    
    The linear displacement per motor revolution (in mm):
      displacement = gear_ratio * pulley_circumference
                 ≈ 8.7 * 100.53 ≈ 875.61 mm

    Convert mm/min to m/s:
      linear_velocity (m/s) = (RPM * displacement in mm) / (60 * 1000)
    
    Args:
        rpm (float): The motor's speed in RPM.

    Returns:
        float: The linear velocity in m/s.
    """
    gear_ratio = 8.7
    pulley_diameter_mm = 32
    circumference_mm = 3.14159 * pulley_diameter_mm
    displacement_per_rev_mm = gear_ratio * circumference_mm
    linear_velocity_mps = (rpm * displacement_per_rev_mm) / (60 * 1000)
    return linear_velocity_mps

def steps_to_m(steps: float) -> float:
    """
    Convert a number of motor steps into a linear distance in meters.

    Based on:
      - 4096 steps per revolution
      - Gear ratio = 8.7 (87-tooth gear on motor / 10-tooth gear on driven side)
      - GT2 pulley diameter = 32 mm (circumference ≈ π * 32 ≈ 100.53 mm)
      - Distance per motor revolution = 8.7 * 100.53 mm ≈ 875.61 mm
      - Distance per step = 875.61 mm / 4096 ≈ 0.2136 mm = 0.0002136 m

    Args:
        steps (float): The number of steps.

    Returns:
        float: The distance in meters.
    """
    conversion_factor = 875.61 / 4096000  # (875.61 mm per revolution) / (4096 steps * 1000 mm/m)
    return steps * conversion_factor

def calc_goal_differences_in_m(goal_positions: list) -> list:
    """
    Calculate the distances (in meters) between consecutive goal positions given in steps.

    Args:
        goal_positions (list of int or float): A list of goal positions (in steps).

    Returns:
        list of float: A list of distances (in meters) between each consecutive goal position.
    """
    differences = []
    for i in range(1, len(goal_positions)):
        diff_steps = abs(goal_positions[i] - goal_positions[i - 1])
        diff_m = steps_to_m(diff_steps)
        differences.append(diff_m)
    return differences

def calc_total_distance_in_m(goal_positions: list) -> float:
    """
    Calculate the total distance (in meters) traveled given a list of goal positions in steps.
    This is the sum of the absolute differences between each consecutive goal position.

    Args:
        goal_positions (list of int or float): A list of goal positions in steps.

    Returns:
        float: The total distance traveled in meters.
    """
    differences = calc_goal_differences_in_m(goal_positions)
    return sum(differences)


if __name__ == "__main__":
    test_rpm = 100
    print(f"{test_rpm} RPM -> {rpm_to_linear_velocity_mps(test_rpm):.4f} m/s")
