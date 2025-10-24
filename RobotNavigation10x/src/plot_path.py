"""
Trajectory Visualization Script

Generates two plots:
1. Path smoothing comparison (original waypoints vs smoothed path)
2. Time-stamped trajectory with markers

Saves plots to the package src directory regardless of execution location.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

print("Starting trajectory visualization...")

# Determine the script's directory (where CSV files are located)
script_dir = os.path.dirname(os.path.abspath(__file__))
print(f"Script directory: {script_dir}")

# Define file paths relative to script location
traj_file = os.path.join(script_dir, 'trajectory.csv')
waypoints_file = os.path.join(script_dir, 'original_waypoints.csv')
smooth_path_file = os.path.join(script_dir, 'smooth_path.csv')

# Check if required files exist
files_to_check = [traj_file, waypoints_file, smooth_path_file]
missing_files = [f for f in files_to_check if not os.path.exists(f)]

if missing_files:
    print("Error: Missing required CSV files. Please run the tracker node first.")
    for f in missing_files:
        print(f"  - {os.path.basename(f)} (MISSING)")
    sys.exit(1)

# Load CSV data
try:
    df_traj = pd.read_csv(traj_file)
    df_waypoints = pd.read_csv(waypoints_file)
    df_smooth = pd.read_csv(smooth_path_file)
    print("Successfully loaded all CSV files")
except Exception as e:
    print(f"Error loading CSV files: {e}")
    sys.exit(1)

# Validate data
if len(df_traj) < 2 or len(df_smooth) < 2:
    print("Error: Insufficient data points in trajectory files")
    sys.exit(1)

# ============================================================================
# Plot 1: Path Smoothing Comparison
# ============================================================================
print("Generating path smoothing visualization...")
fig1, ax1 = plt.subplots(figsize=(10, 8))

ax1.plot(df_waypoints['x'], df_waypoints['y'], 'ro--', 
         label='Original Waypoints', markersize=8, linewidth=2)
ax1.plot(df_smooth['x'], df_smooth['y'], 'b-', 
         label=f'Smoothed Path ({len(df_smooth)} points)', linewidth=2.5)

ax1.set_title('Task 1: Path Smoothing Visualization', fontsize=14, fontweight='bold')
ax1.set_xlabel('X (meters)', fontsize=12)
ax1.set_ylabel('Y (meters)', fontsize=12)
ax1.legend(fontsize=11)
ax1.grid(True, alpha=0.3)
ax1.axis('equal')

# Save to script directory
output_file_1 = os.path.join(script_dir, 'task_1_smoothing.png')
plt.savefig(output_file_1, dpi=150, bbox_inches='tight')
print(f"Saved: {output_file_1}")

# ============================================================================
# Plot 2: Trajectory with Time Markers
# ============================================================================
print("Generating trajectory timestamp visualization...")
fig2, ax2 = plt.subplots(figsize=(12, 9))

# Generate evenly spaced time markers
num_markers = 10
total_time = df_traj['time'].max()
target_times = np.linspace(0, total_time, num=num_markers)

marker_points_x = []
marker_points_y = []
marker_labels = []

for target_time in target_times:
    idx = (df_traj['time'] - target_time).abs().idxmin()
    point = df_traj.iloc[idx]
    marker_points_x.append(point['x'])
    marker_points_y.append(point['y'])
    marker_labels.append(f"{point['time']:.2f}s")

# Plot trajectory path
ax2.plot(df_traj['x'], df_traj['y'], color='gray', linestyle=':', 
         label='Trajectory Path', linewidth=2, zorder=2)

# Plot time markers
ax2.plot(marker_points_x, marker_points_y, 'ks', markersize=10, 
         label='Time Markers', zorder=4)

# Add timestamp labels
for i, txt in enumerate(marker_labels):
    ax2.text(marker_points_x[i] + 0.1, marker_points_y[i] + 0.1, 
             txt, fontsize=10, fontweight='bold', color='black', zorder=5,
             bbox=dict(facecolor='white', alpha=0.8, boxstyle='round,pad=0.3', edgecolor='gray'))

ax2.set_title('Task 2: Time-Parameterized Trajectory', fontsize=14, fontweight='bold')
ax2.set_xlabel('X (meters)', fontsize=12)
ax2.set_ylabel('Y (meters)', fontsize=12)
ax2.legend(fontsize=11)
ax2.grid(True, alpha=0.3)
ax2.axis('equal')

# Save to script directory
output_file_2 = os.path.join(script_dir, 'task_2_timestamp_path.png')
plt.savefig(output_file_2, dpi=150, bbox_inches='tight')
print(f"Saved: {output_file_2}")

print("\nVisualization complete!")
print(f"Total trajectory time: {total_time:.2f} seconds")
print("Close the plot windows to exit.")
plt.show()
