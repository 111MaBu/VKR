%% mpc_enhanced_vector_control_smoothed.m
clear; clc; close all;

%% 1) Constant Definitions
wheelbase = 0.2;            % Wheelbase (l)
sampling_time = 0.5;        % Sampling Time (delta_T)
obstacle_radius_fixed = 2;  % Detection radius for fixed obstacles
avoidance_strength = 1.5;
num_dynamic_obstacles = 50; % Number of dynamic obstacles
total_steps = 1000;         % Total number of simulation steps (N)
prediction_steps_pf = 10;  % Prediction steps for potential field

% MPC Parameters
prediction_horizon_mpc = 5;   % MPC Prediction Horizon (Hp_mpc)
Q_mpc = eye(2);               % MPC State Error Weight Matrix
R_mpc = 0.01*eye(2*prediction_horizon_mpc); % MPC Control Input Weight Matrix
max_linear_velocity = 1.2;    % Maximum linear velocity magnitude (U_max)

%% 2) Initial and Desired Trajectory
current_pose = [0; 12; 3/2*pi]; % [x; y; theta]
time_vector = linspace(0, 2*pi, total_steps);
A = 12; B = 10; alpha = 3; beta = 4; phi = pi/3;
desired_pose = [A*sin(alpha*time_vector+phi); B*cos(beta*time_vector)];

%% 3) Obstacle Initialization
fixed_obstacles = [5,5; -5,-5];
dynamic_obstacles_pos = rand(num_dynamic_obstacles, 2) * 10 - 5;
dynamic_obstacles_vel = rand(num_dynamic_obstacles, 2) * 0.1 - 0.05;

%% 4) Plot Initialization
figure('Color','w'); hold on; axis equal;
plot(desired_pose(1,:), desired_pose(2,:), 'r--', 'LineWidth', 1.2);
for i = 1:size(fixed_obstacles, 1)
    rectangle('Position', [fixed_obstacles(i,:) - obstacle_radius_fixed, 2*obstacle_radius_fixed, 2*obstacle_radius_fixed], ...
              'Curvature', [1,1], 'FaceColor', [0.8, 1, 0.8], 'EdgeColor', 'k');
end
trajectory_handle = animatedline('Color', 'b', 'LineWidth', 1.5);
reference_handle = animatedline('Color', 'r', 'LineStyle', '--');
robot_handle = plot(current_pose(1), current_pose(2), 'ko', 'MarkerSize', 8, ...
                  'MarkerFaceColor', 'none', 'MarkerEdgeColor', 'k');
xlim([-15, 15]); ylim([-15, 15]);

dynamic_obstacle_handles = gobjects(num_dynamic_obstacles, 1);
for i = 1:num_dynamic_obstacles
    dynamic_obstacle_handles(i) = plot(dynamic_obstacles_pos(i, 1), dynamic_obstacles_pos(i, 2), 'ko', 'MarkerFaceColor', 'k');
end

%% 5) Variable Recording & MPC Matrix Pre-calculation
tracking_errors = zeros(2, total_steps);
collision_count = 0;

S_mpc = zeros(2*prediction_horizon_mpc, 2*prediction_horizon_mpc);
for i = 1:prediction_horizon_mpc
    for j = 1:i
        S_mpc(2*i-1:2*i, 2*j-1:2*j) = sampling_time * eye(2);
    end
end

Qbar_mpc = kron(eye(prediction_horizon_mpc), Q_mpc);

% Smoothing Filter Parameter
smoothing_alpha = 0.45;     % Recommended range: 0.2 ~ 0.6
previous_control_input = [0; 0]; % Control input from the previous step

%% 6) Main Simulation Loop
gif_filename = 'mpc_enhanced_vector_control_smoothed.gif';
for k = 1:total_steps
    %—— Visualize Reference and Actual Trajectory
    addpoints(reference_handle, desired_pose(1, k), desired_pose(2, k));
    addpoints(trajectory_handle, current_pose(1), current_pose(2));
    
    %—— 1) MPC Target Attraction
    reference_vector_mpc = zeros(2*prediction_horizon_mpc, 1);
    current_xy_pose = current_pose(1:2);
    for i = 1:prediction_horizon_mpc
        index = min(k + i, total_steps);
        reference_vector_mpc(2*i-1:2*i) = desired_pose(:, index);
    end
    initial_state_vector_mpc = repmat(current_xy_pose, prediction_horizon_mpc, 1);
    
    H_mpc = 2*(S_mpc' * Qbar_mpc * S_mpc + R_mpc);
    f_mpc = 2*(S_mpc' * Qbar_mpc * (initial_state_vector_mpc - reference_vector_mpc));
    
    lower_bound_mpc = -max_linear_velocity * ones(2*prediction_horizon_mpc, 1);
    upper_bound_mpc =  max_linear_velocity * ones(2*prediction_horizon_mpc, 1);
    options = optimoptions('quadprog', 'Display', 'none');
    optimal_control_mpc = quadprog(H_mpc, f_mpc, [], [], [], [], lower_bound_mpc, upper_bound_mpc, [], options);
    
    u_x_mpc = optimal_control_mpc(1);
    u_y_mpc = optimal_control_mpc(2);
    
    %—— 2) Predictive Fixed Obstacle Avoidance (Original Logic)
    predicted_pose = current_pose;
    predicted_vx = u_x_mpc; predicted_vy = u_y_mpc;
    for t_pred = 1:prediction_steps_pf
        for i = 1:size(fixed_obstacles, 1)
            distance_fixed = norm(predicted_pose(1:2) - fixed_obstacles(i,:)');
            if distance_fixed < obstacle_radius_fixed + 1
                angle_fixed = atan2(predicted_pose(2) - fixed_obstacles(i, 2), predicted_pose(1) - fixed_obstacles(i, 1));
                repulsive_force_fixed = avoidance_strength * exp(-distance_fixed);
                predicted_vx = predicted_vx + repulsive_force_fixed * cos(angle_fixed);
                predicted_vy = predicted_vy + repulsive_force_fixed * sin(angle_fixed);
            end
        end
        predicted_pose(1) = predicted_pose(1) + predicted_vx * sampling_time;
        predicted_pose(2) = predicted_pose(2) + predicted_vy * sampling_time;
    end
    u_x = predicted_vx;
    u_y = predicted_vy;
    
    %—— 3) Dynamic Obstacle Avoidance (Original Logic)
    for i = 1:num_dynamic_obstacles
        distance_dynamic = hypot(current_pose(1) - dynamic_obstacles_pos(i, 1), current_pose(2) - dynamic_obstacles_pos(i, 2));
        if distance_dynamic < obstacle_radius_fixed + 2
            angle_dynamic = atan2(current_pose(2) - dynamic_obstacles_pos(i, 2), current_pose(1) - dynamic_obstacles_pos(i, 1));
            repulsive_force_dynamic = avoidance_strength * exp(-distance_dynamic);
            u_x = u_x + repulsive_force_dynamic * cos(angle_dynamic);
            u_y = u_y + repulsive_force_dynamic * sin(angle_dynamic);
        end
    end
    
    %—— 4) Input Saturation
    velocity_magnitude = norm([u_x; u_y]);
    if velocity_magnitude > max_linear_velocity
        u_x = u_x * max_linear_velocity / velocity_magnitude;
        u_y = u_y * max_linear_velocity / velocity_magnitude;
    end
    
    %—— 5) Velocity Smoothing (Low-Pass Filter)
    velocity_vector = [u_x; u_y];
    velocity_vector = smoothing_alpha * previous_control_input + (1 - smoothing_alpha) * velocity_vector;
    previous_control_input = velocity_vector;
    u_x = velocity_vector(1);
    u_y = velocity_vector(2);
    
    %—— 6) Inverse Kinematics for Angular Velocity
    angular_velocity = ( -sin(current_pose(3))*u_x + cos(current_pose(3))*u_y ) / wheelbase;
    
    %—— 7) Update State
    current_pose(1) = current_pose(1) + u_x * sampling_time;
    current_pose(2) = current_pose(2) + u_y * sampling_time;
    current_pose(3) = wrapToPi(current_pose(3) + angular_velocity * sampling_time);
    
    %—— 8) Update Dynamic Obstacles & Collision Count
    dynamic_obstacles_pos = dynamic_obstacles_pos + dynamic_obstacles_vel;
    for i = 1:num_dynamic_obstacles
        set(dynamic_obstacle_handles(i), 'XData', dynamic_obstacles_pos(i, 1), 'YData', dynamic_obstacles_pos(i, 2));
        if hypot(current_pose(1) - dynamic_obstacles_pos(i, 1), current_pose(2) - dynamic_obstacles_pos(i, 2)) < obstacle_radius_fixed + 0.5
            collision_count = collision_count + 1;
        end
    end
    
    %—— 9) Update Robot Plot
    set(robot_handle, 'XData', current_pose(1), 'YData', current_pose(2));
    
    %—— 10) (Optional) Save GIF
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    if k == 1
        imwrite(imind, cm, gif_filename, 'gif', 'LoopCount', inf, 'DelayTime', 0.01);
    elseif rem(k, 2) == 0
        imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.01);
    end
    
    drawnow;
    
    % Record Errors
    tracking_errors(1, k) = desired_pose(1, k) - current_pose(1);
    tracking_errors(2, k) = desired_pose(2, k) - current_pose(2);
end

%% 7) Error Evaluation
figure('Color','w');

subplot(2,1,1);
plot(time_vector, tracking_errors(1,:), 'r', time_vector, tracking_errors(2,:), 'b', 'LineWidth', 1.2);
legend('Error\_x', 'Error\_y'); xlabel('Time'); ylabel('Error (m)');
title('Tracking Error');

subplot(2,1,2);
rmse = sqrt(mean(tracking_errors.^2, 2));
bar(rmse);
set(gca, 'XTickLabel', {'RMSE\_x', 'RMSE\_y'});
ylabel('Error (m)');
title('Root Mean Square Error');

disp(['Total Collisions: ', num2str(collision_count)]);