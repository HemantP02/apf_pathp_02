# APF Path Planning with Velocity Integration

This repository demonstrates path planning using the Artificial Potential Field (APF) method. Enhancements have been made to incorporate velocity considerations into the planning algorithm, improving dynamic response and real-time applicability for robotic systems.

## Key Features

- **Artificial Potential Field (APF) Method**: Employs attractive and repulsive potential fields to guide robots around obstacles toward a goal.
- **Velocity Consideration**: Integrates velocity dynamics, enhancing responsiveness and realism in path planning.
- **Dynamic Path Planning**: Adapts to changes in the environment and robot state.

## Directory Overview

- `apft_01.cpp` to `apft_06.cpp`: Versions of the APF algorithm, each incorporating iterative improvements in handling velocity and dynamic constraints.
- `apftc_real.py`: Python script for real-time control, integrating velocity-based tracking.


## Mathematical Model for Tracking

The path tracking controller uses dynamic modeling in lateral and longitudinal directions, represented by:

### 1. Side-Slip Angle and Yaw Rate Equations

$$
\dot{\beta} = \frac{(k_1 + k_2)}{m v_x} \beta + \left( \frac{a k_1 - b k_2}{m v_x^2} - 1 \right) \omega - \frac{k_1}{m v_x} \delta_f
$$

$$
\dot{\omega} = \frac{(a k_1 - b k_2)}{I_z} \beta + \frac{a^2 k_1 + b^2 k_2}{I_z v_x} \omega - \frac{a k_1}{I_z} \delta_f
$$

Where:
- $\beta$: Side slip angle  
- $\omega$: Yaw rate  
- $k_1, k_2$: Side slip stiffness of front and rear wheels  
- $m$: Vehicle mass  
- $a, b$: Distances from the center of mass to front and rear axles  
- $v_x, v_y$: Longitudinal and lateral velocities  
- $\delta_f$: Front wheel steering angle  
- $I_z$: Rotational inertia around the z-axis  

### 2. Position in Geodetic and Vehicle Coordinates

$$
X = X_0 + \int_0^t v \cos(\phi) dt
$$

$$
Y = Y_0 + \int_0^t v \sin(\phi) dt
$$

### 3. Heading Error

$$
\delta_\phi = \phi - \phi_r
$$

### 4. Lateral Error and Reference Yaw Rate

$$
\dot{e_d} = v_x \delta_\phi + v_y
$$

$$
\omega_r = \frac{v_x}{R} = v_x k_r
$$

### 5. Error Derivatives

$$
\ddot{e_d} = \dot{v_x} \delta_\phi + v_x \dot{\delta_\phi} + \dot{v_y}
$$

$$
\ddot{\delta_\phi} = \dot{\omega} - \dot{v_x} k_r - v_x \dot{k_r}
$$

## Usage Instructions

1. Compile and run individual C++ implementations for specific APF configurations:
   ```bash
   g++ apft_01.cpp -o apft_01
   ./apft_01
   ```
2. Execute Python-based real-time control:
   ```bash
   python3 apftc_real.py
   ```

## Planned Enhancements

- **Full Integration of Trajectory Tracking**: Incorporating complete longitudinal and lateral controllers.
- **Dynamic Obstacle Avoidance**: Real-time detection and navigation.
- **PID-Based Control Tuning**: Optimization for smooth path following.

## Contribution Guidelines

Contributions to improve algorithmic performance or add features are welcome. Fork the repository and submit a pull request with a detailed explanation of your enhancements.

## Licensing

This project is available under the MIT License.

## Support and Contact

For detailed questions or feedback, reach out via [GitHub Issues](https://github.com/HemantP02/apf_pathp_02/issues).
