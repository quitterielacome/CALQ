%   Manipulability
%   @author         Mirroyal Ismayilov
%   @organisation   King's College London
%   @module         Applied Medical Robotics
%   @year           2024

close all
clear all
clc

% Parameters for robot arm
r1 = 1.15;    % Length of first link
r2 = 1.15;    % Length of second link
t1 = 0;   % Initial angle of joint 1
t2 = 0;     % Initial angle of joint 2

% Desired position
x_d = [-0.9; 0.9]; % Desired x and y coordinates of the end-effector

% Control parameters
damping_factor = 0.01;   % Damping factor for DLS
step_limit = 0.1;        % Maximum step size for target clamping
gain = 0.3;              % Gain for Jacobian transpose method
tolerance = 1e-3;        % Position error tolerance
max_iterations = 100;   % Maximum number of iterations
dt = 0.01;               % Time step for plotting manipulability
ellipsoid_scale = 0.25;  % Scaling factor for ellipsoids

% Initialize array to store manipulability values
manipulability_values = zeros(max_iterations, 1);

figure;
hold on;
axis equal;
xlim([-2.5, 2.5]);
ylim([-0.5, 2.5]);
xlabel('X (cm)');
ylabel('Y (cm)');
title('Robot Arm Motion and Manipulability Ellipsoids');

for i = 1:max_iterations
    % Calculate forward kinematics to get the current end-effector position
    T = forwardKinematics(r1, r2, t1, t2);
    x = T(1, 4);
    y = T(2, 4);
    current_position = [x; y];
    
    % Calculate position error between desired and current end-effector position
    error = x_d - current_position;
    
    % Target clamping: Limit the step size to avoid large jumps
    if norm(error) > step_limit
        error = (error / norm(error)) * step_limit;
    end
    
    % Compute Jacobian for the current joint angles
    J = ik_jacobian(r1, r2, t1, t2);
    
    % Calculate manipulability as the square root of the determinant of (J*J')
    manipulability = sqrt(det(J * J'));
    manipulability_values(i) = manipulability;
    
    % Damped Least Squares (DLS) method for stable inverse calculation
    inv_J_damped = (J' * J + damping_factor * eye(2)) \ J';
    delta_q_dls = inv_J_damped * error;
    
    % Jacobian Transpose method as an alternative to inverse Jacobian
    delta_q_transpose = gain * J' * error;
    
    % Select method: uncomment one of the two lines below
    % delta_q = delta_q_dls;     % Damped Least Squares method
    delta_q = delta_q_transpose; % Jacobian Transpose method
    
    % Update joint angles using calculated change
    t1 = t1 + delta_q(1);
    t2 = t2 + delta_q(2);
    
    % Plot current robot arm position
    plot([0, r1*cos(t1), r1*cos(t1) + r2*cos(t1 + t2)], ...
         [0, r1*sin(t1), r1*sin(t1) + r2*sin(t1 + t2)], '-o');
    
    % Plot the manipulability ellipsoid at the current configuration
    [U, S, ~] = svd(J * J');
    theta = linspace(0, 2*pi, 100);
    ellipse_x = ellipsoid_scale * sqrt(S(1, 1)) * cos(theta);
    ellipse_y = ellipsoid_scale * sqrt(S(2, 2)) * sin(theta);
    ellipse = U * [ellipse_x; ellipse_y];
    
    % Plot the ellipsoid around the current end-effector position
    plot(ellipse(1, :) + x, ellipse(2, :) + y, 'm');
    
    pause(0.01);  % Pause for visualization
    
    % Check if the end-effector is within the desired tolerance
    if norm(error) < tolerance
        disp('Desired position reached!');
        break;
    end
end

% Define trajectories: Low-manipulability vs. High-manipulability
trajectory_low = [linspace(-0.5, 1, max_iterations); linspace(0.8, 0.5, max_iterations)];
trajectory_high = [linspace(-0.5, -0.2, max_iterations); linspace(0.8, 1, max_iterations)];

% Initialize metrics for comparison
metrics_low = struct('manipulability', zeros(max_iterations, 1), ...
                     'error', zeros(max_iterations, 1), ...
                     'effort', zeros(max_iterations, 1));

metrics_high = struct('manipulability', zeros(max_iterations, 1), ...
                      'error', zeros(max_iterations, 1), ...
                      'effort', zeros(max_iterations, 1));

% Function to simulate a trajectory
simulate_trajectory = @(trajectory, metrics) simulate(r1, r2, t1, t2, trajectory, damping_factor, step_limit, ...
                                                      gain, tolerance, max_iterations, dt, ellipsoid_scale, ...
                                                      metrics);

% Simulate both trajectories
metrics_low = simulate_trajectory(trajectory_low, metrics_low);
metrics_high = simulate_trajectory(trajectory_high, metrics_high);

% Plot comparison
figure;
subplot(3, 1, 1);
plot(metrics_low.manipulability, 'r', 'DisplayName', 'Low Manipulability');
hold on;
plot(metrics_high.manipulability, 'g', 'DisplayName', 'High Manipulability');
legend;
title('Manipulability Comparison');
xlabel('Iteration');
ylabel('Manipulability Index (\mu)');

subplot(3, 1, 2);
plot(metrics_low.error, 'r', 'DisplayName', 'Low Manipulability');
hold on;
plot(metrics_high.error, 'g', 'DisplayName', 'High Manipulability');
legend;
title('Position Error Comparison');
xlabel('Iteration');
ylabel('Error Norm');

subplot(3, 1, 3);
plot(metrics_low.effort, 'r', 'DisplayName', 'Low Manipulability');
hold on;
plot(metrics_high.effort, 'g', 'DisplayName', 'High Manipulability');
legend;
title('Motor Effort Comparison');
xlabel('Iteration');
ylabel('Effort Norm');


% Plot manipulability over the trajectory
figure;
plot(manipulability_values(1:i));
xlabel('Iteration');
ylabel('Manipulability');
title('Manipulability across the trajectory');

% --- Supporting Functions ---
function T = forwardKinematics(r1, r2, t1, t2)
    % Compute the forward kinematics for a 2-link planar robot
    % Returns the transformation matrix T of the end-effector
    T = [cos(t1 + t2), -sin(t1 + t2), 0, r1*cos(t1) + r2*cos(t1 + t2);
         sin(t1 + t2),  cos(t1 + t2), 0, r1*sin(t1) + r2*sin(t1 + t2);
         0,             0,            1, 0;
         0,             0,            0, 1];
end

function J = ik_jacobian(r1, r2, t1, t2)
    % Compute the Jacobian matrix for a 2-link planar robot
    % Returns a 2x2 Jacobian matrix
    J11 = -r1*sin(t1) - r2*sin(t1 + t2);
    J12 = -r2*sin(t1 + t2);
    J21 =  r1*cos(t1) + r2*cos(t1 + t2);
    J22 =  r2*cos(t1 + t2);
    
    J = [J11, J12;
         J21, J22];
end

function metrics = simulate(r1, r2, t1, t2, trajectory, damping_factor, step_limit, gain, tolerance, max_iterations, dt, ellipsoid_scale, metrics)
    for i = 1:max_iterations
        % Set desired position for current step
        x_d = trajectory(:, i);
        
        % Calculate forward kinematics
        T = forwardKinematics(r1, r2, t1, t2);
        x = T(1, 4);
        y = T(2, 4);
        current_position = [x; y];
        
        % Position error
        error = x_d - current_position;
        metrics.error(i) = norm(error);
        
        % Target clamping
        if norm(error) > step_limit
            error = (error / norm(error)) * step_limit;
        end
        
        % Compute Jacobian
        J = ik_jacobian(r1, r2, t1, t2);
        
        % Manipulability
        metrics.manipulability(i) = sqrt(det(J * J'));
        
        % Damped Least Squares
        inv_J_damped = (J' * J + damping_factor * eye(2)) \ J';
        delta_q = inv_J_damped * error;
        
        % Update joint angles
        t1 = t1 + delta_q(1);
        t2 = t2 + delta_q(2);
        
        % Motor effort
        metrics.effort(i) = norm(delta_q);
    end
end

