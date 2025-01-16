#include <iostream>
#include <vector>
#include <cmath>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// Define a structure for a point in 2D space
struct Point {
    double x;
    double y;
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

// Function to calculate the repulsive force from all obstacles
Point repulsiveForce(const Point& current, const std::vector<Point>& obstacles, double k_rep, double d_0) {
    Point total_force = {0, 0};
    for (const auto& obstacle : obstacles) {
        double distance_to_obstacle = distance(current, obstacle);
        if (distance_to_obstacle <= d_0) {
            double repulsive_strength = k_rep * (1 / distance_to_obstacle - 1 / d_0) * (1 / std::pow(distance_to_obstacle, 2));
            total_force.x += repulsive_strength * (current.x - obstacle.x);
            total_force.y += repulsive_strength * (current.y - obstacle.y);
        }
    }
    return total_force;
}

// Function to calculate the total force
Point totalForce(const Point& current, const Point& goal, const std::vector<Point>& obstacles, double k_att, double k_rep, double d_0) {
    Point att_force = attractiveForce(current, goal, k_att);
    Point rep_force = repulsiveForce(current, obstacles, k_rep, d_0);
    return {att_force.x + rep_force.x, att_force.y + rep_force.y};
}

// Function to perform APF path planning
std::vector<Point> apfPathPlanning(const Point& start, const Point& goal, const std::vector<Point>& obstacles, double k_att, double k_rep, double d_0, double step_size, int max_iterations) {
    std::vector<Point> path;
    path.push_back(start);
    Point current = start;

    for (int i = 0; i < max_iterations; ++i) {
        Point force = totalForce(current, goal, obstacles, k_att, k_rep, d_0);
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
    Point goal = {10, 0};

    // Define the obstacles
    std::vector<Point> obstacles = {{9, 0}, {7, -0.1}, {5, -0.5}, {-1,-0.1}, {-1, -0.2}};
    // std::vector<Point> obstacles = {{3, 0}};
    // Parameters
    double k_att = 2.0;
    double k_rep = 1.0;
    double d_0 = 1.0;
    double step_size = 0.1;
    int max_iterations = 1000;

    // Perform APF path planning
    std::vector<Point> path = apfPathPlanning(start, goal, obstacles, k_att, k_rep, d_0, step_size, max_iterations);

    // Plot the path
    std::vector<double> x_path;
    std::vector<double> y_path;
    for (const auto& p : path) {
        x_path.push_back(p.x);
        y_path.push_back(p.y);
    }
    plt::plot(x_path, y_path, "ro-");
    // Plot the obstacles
    for (Point q_obstacle : obstacles) {
        plt::plot({q_obstacle.x}, {q_obstacle.y}, "bs");
    }
    // Plot the goal
    plt::plot({goal.x}, {goal.y}, "ys");
    // Set labels and title
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::title("Path Planning with Artificial Potential Field");

    // Show the plot
    plt::show();

    return 0;
}
