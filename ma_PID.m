clear all; clc;

%% 1️Define Constants
l = 0.2; % Robot wheel axle length
K = 0.1; % Control gain
delta_T = 0.5; % Time step
obstacle_radius = 2; % Obstacle detection radius
avoidance_strength = 1.5; % Obstacle avoidance strength
num_dynamic_obstacles = 50; % Number of dynamic obstacles
N = 1000; % Number of simulation steps
prediction_steps = 10; % Number of prediction steps

%% 2️Robot Initialization
Pos = [0, 12, 3/2 * pi]; % Initial position (X, Y, Angle)
T = linspace(0, 2*pi, N); % Time vector
A = 12; B = 10;
alpha = 3; beta = 4;
phase_shift = pi/3;
desire_Pos = [A * sin(alpha * T + phase_shift); B * cos(beta * T)];

%% 3️Obstacle Initialization
% Static obstacles
obstacle_positions = [5, 5; -5, -5];

% Dynamic obstacles (randomly generated)
dynamic_obstacles = rand(num_dynamic_obstacles, 2) * 10 - 5;
dynamic_obstacle_velocities = rand(num_dynamic_obstacles, 2) * 0.1 - 0.05;

%% 4️Plot Setup
figure; hold on;
plot(desire_Pos(1,:), desire_Pos(2,:), 'r--'); % Desired trajectory
for i = 1:size(obstacle_positions, 1)
    rectangle('Position', [obstacle_positions(i,:) - obstacle_radius, 2*obstacle_radius, 2*obstacle_radius], ...
              'Curvature', [1, 1], 'FaceColor', 'g', 'EdgeColor', 'k');
end
h = animatedline('color','r','LineStyle','--');
h_car = animatedline('color','b');
h_car_model = animatedline('Marker','o','MaximumNumPoints',1);
axis([-15 15 -15 15]);

% Handles for dynamic obstacles
dynamic_obstacles_plots = gobjects(num_dynamic_obstacles, 1);
for i = 1:num_dynamic_obstacles
    dynamic_obstacles_plots(i) = plot(dynamic_obstacles(i,1), dynamic_obstacles(i,2), 'ko', 'MarkerFaceColor', 'k');
end

%% 5️Initialize Data Logging
errors_x = zeros(1, N);
errors_y = zeros(1, N);
collision_count = 0;

%% 6️Run Simulation
filename = 'complex_experiment_with_path_prediction.gif';

for k = 1:N
    addpoints(h_car, Pos(1), Pos(2));
    addpoints(h_car_model, Pos(1), Pos(2));
    
    % Compute tracking errors
    e_x = Pos(1) - desire_Pos(1, k);
    e_y = Pos(2) - desire_Pos(2, k);
    errors_x(k) = e_x;
    errors_y(k) = e_y;
    u_x = -K * e_x;
    u_y = -K * e_y;
    
    % Update dynamic obstacles' positions
    dynamic_obstacles = dynamic_obstacles + dynamic_obstacle_velocities;
    for i = 1:num_dynamic_obstacles
        set(dynamic_obstacles_plots(i), 'XData', dynamic_obstacles(i, 1), 'YData', dynamic_obstacles(i, 2));
    end

    % 🚀 **Path Prediction and Obstacle Avoidance:**
    predicted_Pos = Pos;
    for step = 1:prediction_steps
        for i = 1:size(obstacle_positions, 1)
            dist = sqrt((predicted_Pos(1) - obstacle_positions(i,1))^2 + (predicted_Pos(2) - obstacle_positions(i,2))^2);
            if dist < obstacle_radius + 1
                % Compute avoidance direction
                angle_to_obstacle = atan2(predicted_Pos(2) - obstacle_positions(i,2), predicted_Pos(1) - obstacle_positions(i,1));
                avoidance_factor = avoidance_strength * exp(-dist); % Exponential decay function
                u_x = u_x + avoidance_factor * cos(angle_to_obstacle); % Adjust control input
                u_y = u_y + avoidance_factor * sin(angle_to_obstacle);
            end
        end
        % Update predicted position
        predicted_Pos(1) = predicted_Pos(1) + u_x * delta_T;
        predicted_Pos(2) = predicted_Pos(2) + u_y * delta_T;
    end
    
    % ?**Avoidance of Dynamic Obstacles:**
    for i = 1:num_dynamic_obstacles
        dist = sqrt((Pos(1) - dynamic_obstacles(i,1))^2 + (Pos(2) - dynamic_obstacles(i,2))^2);
        if dist < obstacle_radius + 0.5
            % Compute avoidance direction
            angle_to_obstacle = atan2(Pos(2) - dynamic_obstacles(i,2), Pos(1) - dynamic_obstacles(i,1));
            avoidance_factor = avoidance_strength * exp(-dist); % Compute avoidance force
            u_x = u_x + avoidance_factor * cos(angle_to_obstacle);
            u_y = u_y + avoidance_factor * sin(angle_to_obstacle);
        end
    end
    
    % Robot motion update
    A_mat = [cos(Pos(3)) -l*sin(Pos(3)); sin(Pos(3)) l*cos(Pos(3))];
    U_mat = [u_x; u_y];
    v_w_mat = A_mat \ U_mat;
    Pos(1) = Pos(1) + v_w_mat(1) * cos(Pos(3)) - v_w_mat(2) * l * sin(Pos(3));
    Pos(2) = Pos(2) + v_w_mat(1) * sin(Pos(3)) + v_w_mat(2) * l * cos(Pos(3));
    Pos(3) = Pos(3) + v_w_mat(2) * delta_T;

    addpoints(h, desire_Pos(1, k), desire_Pos(2, k));

    % Collision detection
    for i = 1:num_dynamic_obstacles
        dist = sqrt((Pos(1) - dynamic_obstacles(i,1))^2 + (Pos(2) - dynamic_obstacles(i,2))^2);
        if dist < obstacle_radius + 0.5
            collision_count = collision_count + 1; % Count collisions
        end
    end

    % Save GIF
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    if k == 1
         imwrite(imind, cm, filename, 'gif', 'LoopCount', inf, 'DelayTime', 0.01);
    elseif rem(k,2) == 0
         imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.01);
    end
    drawnow;
end

%% 7 Error Evaluation
figure;
subplot(2,1,1);
plot(T, errors_x, 'r', 'DisplayName', 'X Error');
hold on;
plot(T, errors_y, 'b', 'DisplayName', 'Y Error');
xlabel('Time');
ylabel('Error');
legend;
title('Tracking Error Over Time');

subplot(2,1,2);
bar([sqrt(mean(errors_x.^2)), sqrt(mean(errors_y.^2))]);
set(gca, 'XTickLabel', {'RMSE_x', 'RMSE_y'});
ylabel('Error');
title('Root Mean Square Error (RMSE)');
