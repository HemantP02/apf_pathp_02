#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt


from math import sqrt, pow, atan2, exp, cos, sin, tan

# Define a class for a point in 2D space
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    # Overload addition operator for Point class
    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

# Function to calculate the distance between two points
def distance(p1, p2):
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2))

# Function to calculate the attractive force
def attractive_force(current, goal, k_att):
    distance_to_goal = distance(current, goal)
    return Point((k_att * (goal.x - current.x)) / distance_to_goal, (k_att * (goal.y - current.y)) / distance_to_goal)

# Function to calculate the dynamic repulsive force
def dynamic_repulsive_force(current, obstacles_with_velocities, k_rep, d_0, vehicle_velocity, front_dynamic_radius, rear_dynamic_radius, k_v, angle_to_obstacle):
    total_force = Point(0, 0)

    # Initialize variables for closest obstacles and relative velocities
    closest_front_distance = float('inf')
    closest_rear_distance = float('inf')
    front_obstacle_radius = 0
    rear_obstacle_radius = 0
    relative_front_velocity = 0
    relative_rear_velocity = 0

    # Predict future position of the robot based on its velocity
    future_robot_position = Point(current.x + vehicle_velocity, current.y)

    # Determine closest obstacles within dynamic regions and predict their future positions
    for obstacle_with_velocity in obstacles_with_velocities:
        obstacle, obstacle_velocity = obstacle_with_velocity

        # Predict future position of the obstacle based on its velocity
        future_obstacle_position = Point(obstacle.x + obstacle_velocity.x, obstacle.y)

        # Calculate relative velocity between obstacle and robot
        relative_velocity_x = obstacle_velocity.x - vehicle_velocity

        angle_to_obstacle = atan2(obstacle.y - current.y, obstacle.x - current.x)
        distance_to_obstacle = distance(current, obstacle)
        obstacle_radius = 0.5 # You need to define this based on your obstacle definition

        # Front detection zone (120 degrees)
        if -np.pi / 3 <= angle_to_obstacle <= np.pi / 3:
            if distance_to_obstacle < closest_front_distance:
                closest_front_distance = distance_to_obstacle
                front_obstacle_radius = obstacle_radius
                relative_front_velocity = relative_velocity_x # Use relative velocity along x-axis

        # Rear detection zone (240 degrees)
        if -2 * np.pi / 3 <= angle_to_obstacle <= 2 * np.pi / 3:
            if distance_to_obstacle < closest_rear_distance:
                closest_rear_distance = distance_to_obstacle
                rear_obstacle_radius = obstacle_radius
                relative_rear_velocity = relative_velocity_x # Use relative velocity along x-axis

    # Calculate dynamic radii for repulsion
    if relative_front_velocity > 10:
        front_dynamic_radius = closest_front_distance + 10 * front_obstacle_radius
    elif relative_front_velocity > 4:
        front_dynamic_radius = closest_front_distance + relative_front_velocity * front_obstacle_radius
    else:
        front_dynamic_radius = closest_front_distance + 4 * front_obstacle_radius

    if relative_rear_velocity < -6:
        rear_dynamic_radius = closest_rear_distance + 6 * rear_obstacle_radius
    elif relative_rear_velocity <= 2:
        rear_dynamic_radius = closest_rear_distance + abs(relative_rear_velocity) * rear_obstacle_radius
    else:
        rear_dynamic_radius = closest_rear_distance + 2 * rear_obstacle_radius

    # Calculate repulsive force within dynamic regions using future points
    for obstacle_with_velocity in obstacles_with_velocities:
        obstacle, obstacle_velocity = obstacle_with_velocity

        # Predict future position of the obstacle based on its velocity
        future_obstacle_position = Point(obstacle.x + obstacle_velocity.x, obstacle.y)

        distance_to_obstacle = distance(current, future_obstacle_position)
        obstacle_radius = 0.5 # You need to define this based on your obstacle definition

        # Check if obstacle is within dynamic front or rear region
        if distance_to_obstacle <= front_dynamic_radius or distance_to_obstacle <= rear_dynamic_radius:
            # Calculate repulsive strength
            repulsive_strength = k_rep * (1 / distance_to_obstacle - 1 / d_0) * (1 / pow(distance_to_obstacle, 2))

            # Calculate repulsive force direction
            repulsive_force_x = repulsive_strength * (current.x - future_obstacle_position.x) / distance_to_obstacle
            repulsive_force_y = repulsive_strength * (current.y - future_obstacle_position.y) / distance_to_obstacle

            # Add repulsive force to total force
            total_force.x += repulsive_force_x
            total_force.y += repulsive_force_y

    # Velocity repulsive force calculation
    if relative_front_velocity > 0 and -np.pi / 3 <= angle_to_obstacle <= np.pi / 3:
        total_force.x += k_v * relative_front_velocity # Front obstacle velocity repulsion

    if relative_rear_velocity < 0 and -2 * np.pi / 3 <= angle_to_obstacle <= 2 * np.pi / 3:
        total_force.x += k_v * relative_rear_velocity # Rear obstacle velocity repulsion

    return total_force

# Function to calculate the road repulsive force
def road_repulsive_force(x, x_l, x_r, L, k_road1, k_road2, k_road3):
    if x <= L / 4:
        return k_road1 * (exp((x - x_l) / L) - 1)
    elif x < 3 * L / 4:
        return 2 * k_road2 * cos(2 * (x - x_l) / L)
    else:
        return k_road3 * (exp((x - x_r) / L) - 1)

# Function to perform APF path planning
def apf_path_planning(start, goal, obstacles_with_velocities, k_att, k_rep, d_0, step_size, max_iterations, vehicle_velocity, k_road1, k_road2, k_road3, L, x_l, x_r, k_v):
    path = [start]
    current = start

    front_dynamic_radius = 0
    rear_dynamic_radius = 0

    for _ in range(max_iterations):
        for obstacle_with_velocity in obstacles_with_velocities:
            obstacle, obstacle_velocity = obstacle_with_velocity
            
            angle_to_obstacle = atan2(obstacle.y - current.y, obstacle.x - current.x)
            force = attractive_force(current, goal, k_att) + \
                    dynamic_repulsive_force(current, obstacles_with_velocities, k_rep, d_0, vehicle_velocity, front_dynamic_radius, rear_dynamic_radius, k_v, angle_to_obstacle)

            # Calculate lateral position of the robot relative to the road boundaries
            lateral_position = current.y

            # Calculate road repulsive force
            road_force = road_repulsive_force(lateral_position, x_l, x_r, L, k_road1, k_road2, k_road3)

            # Add road repulsive force to the total force
            force.y += road_force

            force_magnitude = sqrt(pow(force.x, 2) + pow(force.y, 2))
            if force_magnitude < 0.001:
                # If force is very small, consider it as convergence
                break
            # Normalize the force vector
            norm_x = force.x / force_magnitude
            norm_y = force.y / force_magnitude
            # Update the current position
            current.x += step_size * norm_x
            current.y += step_size * norm_y
            path.append(Point(current.x, current.y))
            # Check if reached the goal
            if distance(current, goal) < step_size:
                path.append(goal)
                break

    return path


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

def bicycle_model(x, y, theta, v, phi, L, dt):
    theta = np.arctan2(np.sin(theta), np.cos(theta))  # Normalize theta to the range [-pi, pi]
    x_prime = x + v * np.cos(theta) * dt
    y_prime = y + v * np.sin(theta) * dt
    theta_prime = theta + (v / L) * np.tan(phi) * dt
    return x_prime, y_prime, theta_prime

def main():


    # Define the start and goal points
    start = Point(8, 0)
    goal = Point(12, 0.0)
    start1 = Point(18,0)
    goal1 = Point(22,0)

    # Define the obstacles with velocities and initial positions
    obstacles_with_velocities = [
        # (Point(10, 0), Point(0, 0.0)),    # obstacle at (9, 9) with velocity (1, 0)
        (Point(10, -0.000005), Point(0, 0)),    # obstacle at (7, 6) with velocity (0, 0)
        # (Point(30, 0), Point(0., 0)),
        # (Point(1,1), Point(0., 0)),
        # (Point(-1, -0.2), Point(0., 0))
    ]
    obstacles_with_velocities1 = [
        # (Point(10, 0), Point(0, 0.0)),    # obstacle at (9, 9) with velocity (1, 0)
        (Point(20, -0.000005), Point(0, 0)),    # obstacle at (7, 6) with velocity (0, 0)
        # (Point(30, 0), Point(0., 0)),
        # (Point(1,1), Point(0., 0)),
        # (Point(-1, -0.2), Point(0., 0))
    ]

    # Parameters
    k_att = 4.5
    k_rep = -0.0001
    k_v = .250  # Velocity gain coefficient for velocity repulsive force
    d_0 = .250
    step_size = 0.0025
    max_iterations = 5000
    vehicle_velocity = 0.50  # Fake vehicle velocity

    # Road parameters
    k_road1 = 02.0
    k_road2 = 0.0
    k_road3 = 0.0
    L = 01.0  # Width of the road
    x_l = -L / 8  # Horizontal position of the centerline of the left lane
    x_r = L / 8 # Horizontal position of the centerline of the right lane

    x = start.x
    y = start.y
    x1 = start1.x
    y1 = start1.y

    # Perform APF path planning
    path = apf_path_planning(start, goal, obstacles_with_velocities, k_att, k_rep, d_0, step_size, max_iterations, vehicle_velocity,
    
                            k_road1, k_road2, k_road3, L, x_l, x_r, k_v)
    path1 = apf_path_planning(start1, goal1, obstacles_with_velocities1, k_att, k_rep, d_0, step_size, max_iterations, vehicle_velocity,
    
                            k_road1, k_road2, k_road3, L, x_l, x_r, k_v)
    path.pop(0)
    path.insert(0, Point(x, y))
    path1.pop(0)
    path1.insert(0, Point(x1, y1))
    
    path_x = [point.x for point in path]
    path_y = [point.y for point in path]
    path_x1 = [point.x for point in path1]
    path_y1 = [point.y for point in path1]
    # Define the coordinates of the two points
    x1, y1 = 0, 0
    x2, y2 = 8, 0

    # Create lists of x and y values
    x_values = np.linspace(x1, x2, 50)
    y_values = np.linspace(y1, y2, 50)
    plt.plot(x_values, y_values, "ks-")
    x3, y3 = 12, 0
    x4, y4 = 18, 0

    # Create lists of x and y values
    x_value = np.linspace(x3, x4, 100)
    y_value = np.linspace(y3, y4, 100)

    x5, y5 = 22, 0
    x6, y6 = 30, 0
    x_value1 = np.linspace(x5, x6, 100)
    y_value1 = np.linspace(y5, y6, 100)
    # Plot the line
    plt.plot(x_value, y_value, "ks-")
    plt.plot(x_value1, y_value1, "ks-")
    plt.plot(path_x, path_y, "gs-",label = "APF_Trajectory_1")
    plt.plot(path_x1, path_y1, "cs-",label = "APF_Trajectory_2")


    # Define PID controller parameters
    kp_v = 4.0
    ki_v = 0.001
    kd_v = 0.51

    kp_phi = 4.0
    ki_phi = 0.001
    kd_phi = 0.5
    threshold = 0.5

    # Initialize robot state

    v = 0      # Initial linear velocity
    phi = 0    # Initial steering angle
    dt = 0.05   # Time step

    # Update robot's state to match the starting point of the APF path
    theta = np.arctan2(path[1].y - path[0].y, path[1].x - path[0].x)

    # Initialize PID controllers
    velocity_controller = PIDController(kp_v, ki_v, kd_v)
    steering_controller = PIDController(kp_phi, ki_phi, kd_phi)


    # Robot trajectory storage
    robot_trajectory_x = [x]
    robot_trajectory_y = [y]

    # Initialize obstacle trajectories storage
    obstacle_trajectories = {i: ([obstacle[0].x], [obstacle[0].y]) for i, obstacle in enumerate(obstacles_with_velocities)}

    # Iterate through the trajectory and apply control inputs
    for i in range(len(path) - 1):
        error_x = path[i + 1].x - x
        error_y = path[i + 1].y - y
        distance_error = sqrt(error_x * error_x + error_y * error_y)

        # Compute control output for linear velocity
        control_output_v = velocity_controller.compute(distance_error, dt)
        v = control_output_v
    
        # If the distance to the next point is smaller than a threshold, stop

        # Compute desired orientation
        desired_theta = atan2(error_y, error_x)
        angle_error = desired_theta - theta

        # Ensure angle error is within range [-pi, pi]
        if angle_error > np.pi:
            angle_error -= 2 * np.pi
        if angle_error < -np.pi:
            angle_error += 2 * np.pi

        # Compute control output for steering angle
        control_output_phi = steering_controller.compute(angle_error, dt)
        phi = control_output_phi
        
        if distance_error > threshold:
            v = 0
        else:
            v = control_output_v
        
        # Update robot state using bicycle model kinematics
        x, y, theta = bicycle_model(x, y, theta, v, phi, L, dt)

        # Store robot trajectory
        robot_trajectory_x.append(x)
        robot_trajectory_y.append(y)

        # Update and store obstacle positions
        for j, (obstacle, velocity) in enumerate(obstacles_with_velocities):
            obstacle.x += velocity.x * dt
            obstacle.y += velocity.y * dt
            obstacle_trajectories[j][0].append(obstacle.x)
            obstacle_trajectories[j][1].append(obstacle.y)

        if distance_error > threshold:
            break

    # Plot the robot's trajectory
    # plt.plot(robot_trajectory_x, robot_trajectory_y, "r-", label="Robot_PID_Trajectory")

    # Plot the obstacle trajectories
    for i, (obs_x, obs_y) in obstacle_trajectories.items():
        plt.plot(obs_x, obs_y,"rs", label=f"Obstacle_{i}_Trajectory")

    # Show plot
    plt.plot(20, 0,"rs", label=f"Obstacle_1_Trajectory")
    plt.plot(0, 0,"go", label=f"Start")
    plt.plot(30, 0,"ro", label=f"Goal")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("APF Path Planning with Bicycle Model and Obstacle Trajectories")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()