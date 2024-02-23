# apf_pathp_02
Path planning using APF method with new changes in the method to incorporate the velocity also
Now I want to add the path tracking controller to accomplish tracking control of planned path.Tell me how to integrate following into current simulation:

// so taking a model and performing dynamic analysis on the lateral and longitudinal directions of 
// model following dynamic equation of 2 degrees of freedom is generated 
// beta_dot = ((k_1+k_2)/m*v_x)*beta + ((a*k_1 - b*k_2)/(m*v_x^2) - 1)*omega -(k_1/m*v_x)*delta_f)  and 
// omega_dot = ((a*k_1 - b*k_2)/I_z)*beta + ((a^2*k_1 + b^2*k_2)/I_z*v_x)*omega - ((a*k_1)/I_z)*delta_f
// beta and omega respectively represent the side slip angle and yaw rate of the vehicle’s center of mass. The symbols k_1
// and k_2 represent the side slip stiffness of the front and rear wheels, respectively, while represents the mass of the vehicle. The 
// distances from the center of mass of the vehicle to the front and rear axles are represented by a and b
// The longitudinal and lateral velocities of the vehicle are denoted as v_x and v_y, respectively.
// Additionally, delta_f represents the front wheel angle of the car, and I_z represents the car’s rotational inertia around the z-axis.
// The vehicle tracking model serves as the foundation for designing the trajectory-tracking controller. The trajectory-tracking controller
// enhances the accuracy of vehicle trajectory tracking by dynamically adjusting the front wheel angle based on the car's lateral and heading 
// errors in real time p represents the centroid of the car, p' represents the projection point of the centroid toward the center of the road,
// indicates the curvature of the road, and e_d indicates the lateral error.
// The position of car in the vehicle coordinate system is (x,y) while it's position in geodetic  coordinate system is (X,Y).
// the origin of vehicle coordinate system, representing the vehicle center of mass, is (X_0,Y_0).
// the linear velocity of vehicle is represented by v, and the heading angle of vehicle is phi.
// Now the position of the vehicle in the geodetic coordinate is expressed as:
// X = X_0 + integration(0 to t) of vcos(phi)dt and Y = Y_0 + integration(0 to t) of vsin(phi)dt
// The heading error delta_phi is calculated by subtracting referece value phi_r from the heading angle phi as delta_phi = phi - phi_r.
// The lateral error is calculated as e_d_dot = v_x*(delta_phi) + v_y where e_d_dot is 1st derivative of e_d, the referece trajectory yaw rate is calclulated as:
// omega_r = phi_r_dot = v_x/R = v_x*k_r where r is turning radius and phi_r_dot is 1st derivative of phi_r
// now delta_phi_dot = omega - omega_r = omega - v_x*k_r where delta_phi_dot is 1st dervative of delta_phi
// now the derivative of reciprocal of both lateral error and heading error is :
// e_d_doubledot = v_x_dot*delta_phi + v_x*delta_phi_dot + v_y_dot
// delta_phi_doubledot = omega_dot - v_x_dot*k_r - v_x*k_r_dot
// where doubledot means second derivative and dot means 1st derivative for variables.
// now dynamic equations combined with above 2 equations gives us:
// e_d_doubledot = ((k_1+k_2)/(m*v_x))*e_d_dot - ((k_1 + k_2)/m)*delta_phi + ((a*k_1 + b*k_2)/(m*v_x))*delta_phi_dot - ((a*k_1)/m)*delta_f + ((a*k_1 - b*k_2)/m)*k_r - k_r*(v_x)^2 + v_x_dot*delta_phi
// delta_phi_doubledot = ((a*k_1 - b*k_2)/I_z*v_x)*e_d - ((a*k_1 - b*k_2)/I_z)*delta_phi + ((a^2*k_1 + b^2*k_2)/I_z*v_x)*delta_phi_dot - (a*k_1/I_z)*delta_f + ((a^2*k_1 + b^2*k_2)/I_z)*k_r - v_x_dot*k_r - v_x*k_r
// here also doubledot is second derivative and dot is 1st.
// the state variable of the system are defined as x_1 = e_d, x_2 = e_d_dot, x_3 = delta_phi, x_4 = delta_phi_dot, and input variables u = delta_f. here delta_f is front wheel angle of the vehicle.
// The path-tracking method for controlling the general path is based on the analysis of lateral error and heading error.
// Additionally, a third control method utilizes the fusion of lateral error and heading error for control.
// we use the comprehensive approach of considering both lateral and heading errors in trajectory tracking.
// The designed fusion error function is presented as e_m = x_m1*e_d + x_m2*delta_phi
// x_m1 is lateral error coefficient and x_m2 is heading error coefficient.
// the design of the sliding mode function for SMC controller using fusion error e_m is
// s = lambda_1*e_m + lamdba_2*e_m_dot + lambda_3*integral(0 to t)(e_mdt) where lamda_1, lambda_2, lambda_3 are sliding mode coefficients.
// s_dot = lambda_1*e_m_dot + lamdba_2*e_m_doubledot + lambda_3*e_m
// The drawback of SMC is the generation of chattering. Thus, to enhance the system’s stability,
// a convergence rate is used. The commonly used convergence rate, aiming to improve system stability,
//  is selected. so s_dot = -epsilon_1*sgn(s) - epsilon_2*s
// epsilon_1 and epsilon_2 are general convergence rate coefficients.
// to reduce chattering more hyperbolic tangent function tanh(s) replaces the sign function sgn(s)
// now above equations are amalgamated to obtain the following expressions
// e_m_dot = omega_1 + omega_2 + omega_3*delta_f
// omega_1 = x_m1*[((k_1+k_2)/(m*v_x))*e_dot - ((k_1 + k_2)/m)*delta_phi + ((a*k_1 - b*k_2)/m*v_x)*delta_phi_dot] + x_m2*[((a*k_1 - b*k_2)/m*v_x)*e - ((a*k_1 - b*k_2)/I_z)*delta_phi + ((a^2*k_1 + b^2*k_2)/I_z*v_x)*delta_phi]
// omega_2 = x_m1*[((a*k_1 - b*k_2)/m)*R - R*v_x^2 + v_x_dot*delta_phi] + x_m2*[((a^2*k_1 + b^2*k_2)/I_z)*R - v_x_dot*R - v_x*R_dot]
// omega_3 = -(a*k_1/m)*x_m1 - (a*k_1/I_z)*xm2
// now delta_f = u = -(1/lambda_2*omega_3)*(lambda_2*omega_1 + lambda_2*omega_2 + lambda_1*e_m_dot + lambda_3*e_m + epsilon_1*tanh(s) + epsilon_2*s)
