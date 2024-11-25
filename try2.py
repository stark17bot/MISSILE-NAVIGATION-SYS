import tkinter as tk
from tkinter import messagebox, ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import folium
import webbrowser
from geopy.distance import geodesic
from mpl_toolkits.mplot3d import Axes3D
import threading

# Constants
GRAVITY = 9.81  # Gravity (m/s^2)

# Function to simulate missile trajectory in 2D
def simulate_missile_trajectory_2d(v0, angle, wind_speed, duration=10, steps=100):
    angle_rad = np.radians(angle)
    t = np.linspace(0, duration, steps)  # Time intervals

    x = v0 * np.cos(angle_rad) * t + wind_speed * t  # Horizontal distance
    y = (v0 * np.sin(angle_rad) * t) - (0.5 * GRAVITY * t**2)  # Vertical height
    y = np.maximum(y, 0)  # Prevent negative height

    return x, y, t

# Function to simulate missile trajectory in 3D
def simulate_missile_trajectory_3d(v0, angle, wind_speed, duration=10, steps=100):
    angle_rad = np.radians(angle)
    t = np.linspace(0, duration, steps)

    x = v0 * np.cos(angle_rad) * t + wind_speed * t
    y = (v0 * np.sin(angle_rad) * t) - (0.5 * GRAVITY * t**2)
    z = np.linspace(0, 1000, steps)  # Simulated height variation for 3D trajectory

    return x, y, z, t

# Function to plot 2D trajectory
def plot_trajectory_2d(v0, angle, wind_speed):
    x, y, t = simulate_missile_trajectory_2d(v0, angle, wind_speed)

    fig, ax = plt.subplots()
    ax.plot(x, y, label="Missile Trajectory (2D)", color="green")
    ax.set_title("Missile Trajectory - 2D")
    ax.set_xlabel("Horizontal Distance (m)")
    ax.set_ylabel("Vertical Distance (m)")
    ax.legend()
    ax.grid()

    return fig

# Function to plot 3D trajectory
def plot_trajectory_3d(v0, angle, wind_speed):
    x, y, z, t = simulate_missile_trajectory_3d(v0, angle, wind_speed)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x, y, z, label="Missile Trajectory (3D)", color="blue")
    ax.set_title("Missile Trajectory - 3D")
    ax.set_xlabel("Horizontal Distance (m)")
    ax.set_ylabel("Vertical Distance (m)")
    ax.set_zlabel("Altitude (m)")
    ax.legend()
    ax.grid()

    return fig

# Function to update map with missile trajectory and blast impact
def update_map(launch_lat, launch_lon, target_lat, target_lon, blast_radius, trajectory_coords):
    missile_map = folium.Map(location=[launch_lat, launch_lon], zoom_start=6)

    # Add launch and target markers
    folium.Marker([launch_lat, launch_lon], popup="Launch Site", icon=folium.Icon(color="green")).add_to(missile_map)
    folium.Marker([target_lat, target_lon], popup="Target Site", icon=folium.Icon(color="red")).add_to(missile_map)

    # Add blast radius (in km)
    folium.Circle(
        location=[target_lat, target_lon],
        radius=blast_radius * 1000,  # Convert km to meters
        color="red",
        fill=True,
        fill_opacity=0.4,
        popup="Blast Impact Area"
    ).add_to(missile_map)

    # Add trajectory path (convert to lat/lon)
    folium.PolyLine(trajectory_coords, color="blue", weight=2.5, opacity=0.8, popup="Missile Trajectory").add_to(missile_map)

    missile_map.save("real_time_missile_map.html")
    webbrowser.open("real_time_missile_map.html")

# Function to run the simulation and update GUI
def run_simulation():
    try:
        # Get inputs
        launch_lat = float(entry_launch_lat.get())
        launch_lon = float(entry_launch_lon.get())
        target_lat = float(entry_target_lat.get())
        target_lon = float(entry_target_lon.get())
        v0 = float(entry_velocity.get())
        angle = float(entry_angle.get())
        wind_speed = float(entry_wind.get())

        # Simulate 2D trajectory
        x_2d, y_2d, t_2d = simulate_missile_trajectory_2d(v0, angle, wind_speed)
        trajectory_coords_2d = [(launch_lat + (target_lat - launch_lat) * (x_2d[i] / max(x_2d)),
                                 launch_lon + (target_lon - launch_lon) * (x_2d[i] / max(x_2d))) for i in range(len(x_2d))]

        # Simulate 3D trajectory
        x_3d, y_3d, z_3d, t_3d = simulate_missile_trajectory_3d(v0, angle, wind_speed)

        # Predict blast radius
        blast_radius = np.sqrt(v0) / 10  # Example formula

        # Update map
        update_map(launch_lat, launch_lon, target_lat, target_lon, blast_radius, trajectory_coords_2d)

        # Plot 2D and 3D trajectories
        fig_2d = plot_trajectory_2d(v0, angle, wind_speed)
        canvas_2d = FigureCanvasTkAgg(fig_2d, master=frame_2d_trajectory)
        canvas_2d.draw()
        canvas_2d.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        fig_3d = plot_trajectory_3d(v0, angle, wind_speed)
        canvas_3d = FigureCanvasTkAgg(fig_3d, master=frame_3d_trajectory)
        canvas_3d.draw()
        canvas_3d.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    except Exception as e:
        messagebox.showerror("Error", f"An error occurred: {e}")

# GUI setup
root = tk.Tk()
root.title("Missile Navigation System with 2D/3D Trajectory and Mapping")
root.geometry("1200x800")

# Tabs
tab_control = ttk.Notebook(root)
tab_simulation = ttk.Frame(tab_control)
tab_2d_trajectory = ttk.Frame(tab_control)
tab_3d_trajectory = ttk.Frame(tab_control)
tab_control.add(tab_simulation, text="Simulation")
tab_control.add(tab_2d_trajectory, text="2D Trajectory")
tab_control.add(tab_3d_trajectory, text="3D Trajectory")
tab_control.pack(expand=1, fill="both")

# Simulation Tab
frame_inputs = tk.Frame(tab_simulation, padx=10, pady=10)
frame_inputs.pack(side=tk.LEFT, fill=tk.Y)

frame_inputs.columnconfigure(1, weight=1)
frame_inputs.rowconfigure(7, weight=1)

tk.Label(frame_inputs, text="Launch Latitude:").grid(row=0, column=0, sticky=tk.W, pady=5)
entry_launch_lat = tk.Entry(frame_inputs)
entry_launch_lat.grid(row=0, column=1, pady=5)

tk.Label(frame_inputs, text="Launch Longitude:").grid(row=1, column=0, sticky=tk.W, pady=5)
entry_launch_lon = tk.Entry(frame_inputs)
entry_launch_lon.grid(row=1, column=1, pady=5)

tk.Label(frame_inputs, text="Target Latitude:").grid(row=2, column=0, sticky=tk.W, pady=5)
entry_target_lat = tk.Entry(frame_inputs)
entry_target_lat.grid(row=2, column=1, pady=5)

tk.Label(frame_inputs, text="Target Longitude:").grid(row=3, column=0, sticky=tk.W, pady=5)
entry_target_lon = tk.Entry(frame_inputs)
entry_target_lon.grid(row=3, column=1, pady=5)

tk.Label(frame_inputs, text="Initial Velocity (m/s):").grid(row=4, column=0, sticky=tk.W, pady=5)
entry_velocity = tk.Entry(frame_inputs)
entry_velocity.grid(row=4, column=1, pady=5)

tk.Label(frame_inputs, text="Launch Angle (degrees):").grid(row=5, column=0, sticky=tk.W, pady=5)
entry_angle = tk.Entry(frame_inputs)
entry_angle.grid(row=5, column=1, pady=5)

tk.Label(frame_inputs, text="Wind Speed (m/s):").grid(row=6, column=0, sticky=tk.W, pady=5)
entry_wind = tk.Entry(frame_inputs)
entry_wind.grid(row=6, column=1, pady=5)

btn_simulate = tk.Button(frame_inputs, text="Run Simulation", command=lambda: threading.Thread(target=run_simulation).start())
btn_simulate.grid(row=7, column=0, columnspan=2, pady=10)

# 2D Trajectory Tab
frame_2d_trajectory = tk.Frame(tab_2d_trajectory)
frame_2d_trajectory.pack(fill=tk.BOTH, expand=True)

# 3D Trajectory Tab
frame_3d_trajectory = tk.Frame(tab_3d_trajectory)
frame_3d_trajectory.pack(fill=tk.BOTH, expand=True)

# Run the app
root.mainloop()
