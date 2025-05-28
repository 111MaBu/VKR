import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# --- Parameter Settings ---
a4_width_mm = 210  # A4 paper width in millimeters
a4_height_mm = 297 # A4 paper height in millimeters
circle_diameter_mm = 20 # Diameter of the calibration circles in millimeters
circle_radius_mm = circle_diameter_mm / 2.0 # Radius of the calibration circles
circle_spacing_mm = 80 # Center-to-center distance between consecutive circles (adjustable here)
points_per_mm = 5 # Path sampling density: number of points per millimeter for smooth drawing and distance calculation

# Convert millimeters to inches for matplotlib's figsize (1 inch = 25.4 millimeters)
mm_to_inches = 1.0 / 25.4
a4_width_inches = a4_width_mm * mm_to_inches
a4_height_inches = a4_height_mm * mm_to_inches

# --- Define Trajectory Waypoints (Example: An S-shaped path) ---
# These are the control points that define the desired path.
# Coordinates are in millimeters, with the origin (0,0) at the bottom-left corner of the A4 paper.
# Ensure sufficient margins so the path and circles remain within the paper boundaries.
path_control_points = np.array([
    [30, 30],   # Starting point
    [80, 60],
    [150, 120],
    [100, 200],
    [180, 260]  # Example End point
])

# --- Generate Dense Path Points for Drawing and Distance Calculation ---
# A simple linear interpolation function to create a smooth, dense path.
def generate_dense_path(control_points, points_per_mm):
    path_points = []
    cumulative_distances = [0] # Keep track of cumulative distance along the path
    total_distance = 0

    for i in range(len(control_points) - 1):
        p1 = control_points[i]
        p2 = control_points[i+1]
        segment_length = np.linalg.norm(p2 - p1) # Calculate length of the current segment
        num_points_in_segment = int(segment_length * points_per_mm) # Determine number of points for this segment
        if num_points_in_segment < 2: # Ensure at least two points for a segment
             num_points_in_segment = 2

        t = np.linspace(0, 1, num_points_in_segment) # Parameter for interpolation
        segment_points = p1 + t[:, np.newaxis] * (p2 - p1) # Generate points along the segment

        if i > 0: # Avoid duplicating points at segment junctions
            segment_points = segment_points[1:]

        # Add generated points to the overall path and update cumulative distances
        for j in range(segment_points.shape[0]):
            if path_points: # If path_points is not empty
                dist = np.linalg.norm(segment_points[j] - path_points[-1]) # Distance from previous point
                total_distance += dist
                cumulative_distances.append(total_distance)
            path_points.append(segment_points[j])

    return np.array(path_points), np.array(cumulative_distances)

# Generate the dense path and its cumulative distances
path_points_dense, cumulative_distances = generate_dense_path(path_control_points, points_per_mm)

# --- Determine Circle Center Positions ---
# We want to place circles along the path at regular intervals.
circle_centers = []
next_circle_distance = circle_spacing_mm # Target distance for the next circle

# Place the first circle at the very beginning of the path
circle_centers.append(path_points_dense[0])

# Iterate through the dense path to find points for subsequent circle placements
for i in range(1, len(cumulative_distances)):
    if cumulative_distances[i] >= next_circle_distance:
        # We've reached or passed the target distance for the next circle.
        # For simplicity, we just take the current point;
        # for higher precision, one could interpolate between the current and previous point.
        circle_centers.append(path_points_dense[i])
        next_circle_distance += circle_spacing_mm # Set the target distance for the circle after this one

# Optional: If the end point is far enough from the last placed circle, add one at the end.
# This ensures the entire path is covered for calibration purposes.
# if np.linalg.norm(path_points_dense[-1] - circle_centers[-1]) > circle_spacing_mm / 2:
#      circle_centers.append(path_points_dense[-1])


# --- Plotting ---
fig, ax = plt.subplots(figsize=(a4_width_inches, a4_height_inches)) # Create a figure with A4 dimensions

# Plot the desired path line
ax.plot(path_points_dense[:, 0], path_points_dense[:, 1], 'k-', linewidth=2, label='Desired Path') # 'k-' means black solid line

# Plot the calibration circles
for center in circle_centers:
    # Create a circle patch
    circle = patches.Circle(center, radius=circle_radius_mm, edgecolor='red', facecolor='none', linewidth=1.5)
    ax.add_patch(circle) # Add the circle to the plot
    # Optionally, add a small dot at the center of each circle for clarity
    ax.plot(center[0], center[1], 'ro', markersize=4) # 'ro' means red circle marker

# Set axis limits to match A4 paper dimensions
ax.set_xlim(0, a4_width_mm)
ax.set_ylim(0, a4_height_mm)

# Ensure aspect ratio is 'equal' so circles appear perfectly round and proportions are correct
ax.set_aspect('equal', adjustable='box')

# Set axis labels and title
ax.set_xlabel('X Position (mm)')
ax.set_ylabel('Y Position (mm)')
ax.set_title('Robot Tracking Test Path with Calibration Circles')
ax.grid(True, linestyle='--', alpha=0.6) # Add a grid for better spatial reference

# If you prefer the origin (0,0) to be at the top-left (common in some image processing), uncomment:
# ax.invert_yaxis() # (For A4 printing, bottom-left origin is typical, so it's commented out)

# Add a legend to the plot
ax.legend()

# Optional: Hide axis ticks for a cleaner plot, keeping only the bounding box as A4 paper indicator
# ax.set_xticks([])
# ax.set_yticks([])

# Adjust layout to minimize whitespace around the plot
plt.tight_layout()

# --- Save the figure as a PDF file ---
output_filename = 'robot_test_path_A4.pdf'
# bbox_inches='tight' and pad_inches=0 help ensure the output file size exactly matches the figsize.
plt.savefig(output_filename, bbox_inches='tight', pad_inches=0)

plt.show() # Display the plot

print(f"Path plot saved as {output_filename}")
print(f"Estimated total path length: {cumulative_distances[-1]:.2f} mm")
print(f"Number of calibration circles plotted: {len(circle_centers)}")