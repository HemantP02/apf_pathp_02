#include <iostream>
#include <vector>
#include <cmath>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// Define a structure for a point in 2D space
struct Point {
    double x;
    double y;

    // Overload addition operator for Point struct
    Point operator+(const Point& other) const {
        return {x + other.x, y + other.y};
    }
};

// Function to calculate the distance between two points
double distance(const Point& p1, const Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// Function to calculate the attractive force
Point attractiveForce(const Point& current, const Point& goal, double k_att) {
    double distance_to_goal = distance(current, goal);
    return {(k_att * (goal.x - current.x)) / distance_to_goal, (k_att * (goal.y - current.y)) / distance_to_goal};
}

// Function to calculate the repulsive force from obstacles within the dynamic regions
Point dynamicRepulsiveForce(const Point& current, const std::vector<std::pair<Point, Point>>& obstacles_with_velocities, 
                            double k_rep, double d_0, double vehicle_velocity, double& front_dynamic_radius, double& rear_dynamic_radius) {
    Point total_force = {0, 0};

    // Initialize variables for closest obstacles and relative velocities
    double closest_front_distance = std::numeric_limits<double>::max();
    double closest_rear_distance = std::numeric_limits<double>::max();
    double front_obstacle_radius = 0;
    double rear_obstacle_radius = 0;
    double relative_front_velocity = 0;
    double relative_rear_velocity = 0;

    // Determine closest obstacles within dynamic regions
    for (const auto& obstacle_with_velocity : obstacles_with_velocities) {
        const Point& obstacle = obstacle_with_velocity.first;
        const Point& obstacle_velocity = obstacle_with_velocity.second;

        double angle_to_obstacle = std::atan2(obstacle.y - current.y, obstacle.x - current.x);
        double distance_to_obstacle = distance(current, obstacle);
        double obstacle_radius = 0.2; // You need to define this based on your obstacle definition

        // Front detection zone (120 degrees)
        if (angle_to_obstacle >= -M_PI / 6 && angle_to_obstacle <= M_PI / 6) {
            if (distance_to_obstacle < closest_front_distance) {
                closest_front_distance = distance_to_obstacle;
                front_obstacle_radius = obstacle_radius;
                relative_front_velocity = std::sqrt(std::pow(vehicle_velocity - obstacle_velocity.x, 2) + std::pow(-obstacle_velocity.y, 2)); // Assuming obstacle moves in the same direction
            }
        }

        // Rear detection zone (240 degrees)
        if (angle_to_obstacle >= -2 * M_PI / 3 && angle_to_obstacle <= 2 * M_PI / 3) {
            if (distance_to_obstacle < closest_rear_distance) {
                closest_rear_distance = distance_to_obstacle;
                rear_obstacle_radius = obstacle_radius;
                relative_rear_velocity = std::sqrt(std::pow(vehicle_velocity - obstacle_velocity.x, 2) + std::pow(-obstacle_velocity.y, 2)); // Assuming obstacle moves in the same direction
            }
        }
    }

    // Calculate dynamic radii for repulsion
    if (relative_front_velocity > 10) {
        front_dynamic_radius = closest_front_distance + 10 * front_obstacle_radius;
    } else if (relative_front_velocity > 4) {
        front_dynamic_radius = closest_front_distance + relative_front_velocity * front_obstacle_radius;
    } else {
        front_dynamic_radius = closest_front_distance + 4 * front_obstacle_radius;
    }

    if (relative_rear_velocity < -6) {
        rear_dynamic_radius = closest_rear_distance + 6 * rear_obstacle_radius;
    } else if (relative_rear_velocity <= 2) {
        rear_dynamic_radius = closest_rear_distance + std::abs(relative_rear_velocity) * rear_obstacle_radius;
    } else {
        rear_dynamic_radius = closest_rear_distance + 2 * rear_obstacle_radius;
    }

    // Calculate repulsive force within dynamic regions
    for (const auto& obstacle_with_velocity : obstacles_with_velocities) {
        const Point& obstacle = obstacle_with_velocity.first;
        double distance_to_obstacle = distance(current, obstacle);
        double obstacle_radius = 0.2; // You need to define this based on your obstacle definition

        // Check if obstacle is within dynamic front region
        if (distance_to_obstacle <= front_dynamic_radius) {
            double repulsive_strength = k_rep * (1 / distance_to_obstacle - 1 / d_0) * (1 / std::pow(distance_to_obstacle, 2));
            total_force.x += repulsive_strength * (current.x - obstacle.x);
            total_force.y += repulsive_strength * (current.y - obstacle.y);
        }

        // Check if obstacle is within dynamic rear region
        if (distance_to_obstacle <= rear_dynamic_radius) {
            double repulsive_strength = k_rep * (1 / distance_to_obstacle - 1 / d_0) * (1 / std::pow(distance_to_obstacle, 2));
            total_force.x += repulsive_strength * (current.x - obstacle.x);
            total_force.y += repulsive_strength * (current.y - obstacle.y);
        }
    }

    return total_force;
}

// Function to perform APF path planning
std::vector<Point> apfPathPlanning(const Point& start, const Point& goal, const std::vector<std::pair<Point, Point>>& obstacles_with_velocities,
                                   double k_att, double k_rep, double d_0, double step_size, int max_iterations, double vehicle_velocity) {
    std::vector<Point> path;
    path.push_back(start);
    Point current = start;

    double front_dynamic_radius = 0;
    double rear_dynamic_radius = 0;

    for (int i = 0; i < max_iterations; ++i) {
        Point force = attractiveForce(current, goal, k_att) +
                      dynamicRepulsiveForce(current, obstacles_with_velocities, k_rep, d_0, vehicle_velocity, front_dynamic_radius, rear_dynamic_radius);

        double force_magnitude = std::sqrt(std::pow(force.x, 2) + std::pow(force.y, 2));
        if (force_magnitude < 0.001) {
            // If force is very small, consider it as convergence
            break;
        }
        // Normalize the force vector
        double norm_x = force.x / force_magnitude;
        double norm_y = force.y / force_magnitude;
        // Update the current position
        current.x += step_size * norm_x;
        current.y += step_size * norm_y;
        path.push_back(current);
        // Check if reached the goal
        if (distance(current, goal) < step_size) {
            path.push_back(goal);
            break;
        }
    }

    return path;
}

int main() {
    // Define the start and goal points
    Point start = {-4, 0};
    Point goal = {15, 0};

    // Define the obstacles with velocities and initial positions
    std::vector<std::pair<Point, Point>> obstacles_with_velocities = {
        {{9, 0}, {-0.1, 0}},    // obstacle at (9, 9) with velocity (1, 0)
        {{7, -0.1}, {0.1, 0}},    // obstacle at (7, 6) with velocity (0, 0)
        {{5, -0.5}, {0.1, 0}},
        {{-1,-0.1}, {0.1, 0}},
        {{-1, -0.2}, {0.1, 0}}
    };

    // Parameters
    double k_att = 2.0;
    double k_rep = 1.0;
    double d_0 = 1.0;
    double step_size = 0.1;
    int max_iterations = 1000;
    double vehicle_velocity = 0.20; // Fake vehicle velocity

    // Perform APF path planning
    std::vector<Point> path = apfPathPlanning(start, goal, obstacles_with_velocities, k_att, k_rep, d_0, step_size, max_iterations, vehicle_velocity);

    // Plot the path
    std::vector<double> x_path;
    std::vector<double> y_path;
    for (const auto& p : path) {
        x_path.push_back(p.x);
        y_path.push_back(p.y);
    }
    plt::plot(x_path, y_path, "ro-");

    // Plot the obstacles and their trajectories
    for (const auto& obstacle_with_velocity : obstacles_with_velocities) {
        const Point& obstacle = obstacle_with_velocity.first;
        const Point& velocity = obstacle_with_velocity.second;

        // Plot obstacle
        plt::plot({obstacle.x}, {obstacle.y}, "bs");

        // Plot obstacle trajectory
        std::vector<double> x_trajectory;
        std::vector<double> y_trajectory;
        Point current_obstacle_position = obstacle;
        for (int i = 0; i < max_iterations; ++i) {
            current_obstacle_position.x += velocity.x * step_size;
            current_obstacle_position.y += velocity.y * step_size;
            x_trajectory.push_back(current_obstacle_position.x);
            y_trajectory.push_back(current_obstacle_position.y);
        }
        plt::plot(x_trajectory, y_trajectory, "g--");
    }

    // Plot dynamic detection zones at each point along the path
    for (const auto& vehicle_position : path) {
        double front_dynamic_radius = 0;
        double rear_dynamic_radius = 0;
        dynamicRepulsiveForce(vehicle_position, obstacles_with_velocities, k_rep, d_0, vehicle_velocity, front_dynamic_radius, rear_dynamic_radius);

        // Plot the obstacles within the dynamic detection zones
        for (const auto& obstacle_with_velocity : obstacles_with_velocities) {
            const Point& obstacle = obstacle_with_velocity.first;
            double distance_to_obstacle = distance(vehicle_position, obstacle);
            double obstacle_radius = 0.5; // You need to define this based on your obstacle definition

            // Check if obstacle is within dynamic front region
            if (distance_to_obstacle <= front_dynamic_radius) {
                plt::plot({obstacle.x}, {obstacle.y}, "ro");
            }

            // Check if obstacle is within dynamic rear region
            if (distance_to_obstacle <= rear_dynamic_radius) {
                plt::plot({obstacle.x}, {obstacle.y}, "bo");
            }
        }
    }

    // Set labels and title
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::title("Path Planning with Artificial Potential Field");

    // Show the plot
    plt::show();

    return 0;
}
