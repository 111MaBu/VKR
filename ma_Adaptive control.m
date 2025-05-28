%% advanced_vector_control.m
clear; clc; close all;
%% 1) Constant Definitions
l = 0.2;               % Wheelbase
delta_T = 0.5;         % Sampling time
obstacle_radius = 2;   % Fixed obstacle detection radius
avoidance_strength = 1.5;
num_dynamic = 50;      % Number of dynamic obstacles
N = 1000;              % Total steps
prediction_steps = 10; % Prediction steps
% Control input saturation
U_max = 1.2;           % Maximum linear velocity magnitude
% (Angular velocity is calculated from world velocity, no separate saturation needed, but W_max can be added if required)
%% 2) Initial and Desired Trajectory
Pos = [0; 12; 3/2*pi];    % [x; y; theta]
T = linspace(0,2*pi,N);
A = 12; B = 10; alpha = 3; beta = 4; phi = pi/3;
desire_Pos = [A*sin(alpha*T+phi); B*cos(beta*T)];
%% 3) Obstacle Initialization
obs_fixed = [5,5; -5,-5];
dyn_obs = rand(num_dynamic,2)*10 - 5;
dyn_vel = rand(num_dynamic,2)*0.1 - 0.05;
%% 4) Plot Initialization
figure('Color','w'); hold on; axis equal;
plot(desire_Pos(1,:),desire_Pos(2,:),'r--','LineWidth',1.2);
for i=1:size(obs_fixed,1)
    rectangle('Position',[obs_fixed(i,:)-obstacle_radius,2*obstacle_radius,2*obstacle_radius],...
              'Curvature',[1,1],'FaceColor',[0.8,1,0.8],'EdgeColor','k');
end
h_traj = animatedline('Color','b','LineWidth',1.5);
h_ref  = animatedline('Color','r','LineStyle','--');
h_robot = plot(Pos(1), Pos(2), 'ko','MarkerFaceColor','none','MarkerSize',8); % Empty circle for the robot
xlim([-15,15]); ylim([-15,15]);
h_dyn = gobjects(num_dynamic,1);
for i=1:num_dynamic
    h_dyn(i) = plot(dyn_obs(i,1), dyn_obs(i,2), 'ko','MarkerFaceColor','k');
end
%% 5) Record Variables
errors = zeros(2,N);
collisions = 0;
%% 6) Main Simulation Loop
filename = 'vector_control.gif';
for k = 1:N
    % Trajectory Visualization
    addpoints(h_ref, desire_Pos(1,k), desire_Pos(2,k));
    addpoints(h_traj, Pos(1), Pos(2));
    set(h_robot, 'XData', Pos(1), 'YData', Pos(2)); % Update robot position

    %—— 1) Target Attraction (Current position pointing to the desired point)
    e_x = desire_Pos(1,k) - Pos(1);
    e_y = desire_Pos(2,k) - Pos(2);
    u_x = 0.5 * e_x;    % Adjustable attraction gain
    u_y = 0.5 * e_y;

    %—— 2) Predictive Fixed Obstacle Avoidance (Keep original logic)
    p = Pos;
    ux_p = u_x; uy_p = u_y;
    for t_pred = 1:prediction_steps
        for i = 1:size(obs_fixed,1)
            d = norm(p(1:2) - obs_fixed(i,:)');
            if d < obstacle_radius + 1
                ang = atan2(p(2)-obs_fixed(i,2), p(1)-obs_fixed(i,1));
                f = avoidance_strength * exp(-d);
                ux_p = ux_p + f*cos(ang);
                uy_p = uy_p + f*sin(ang);
            end
        end
        % Prediction step
        p(1) = p(1) + ux_p*delta_T;
        p(2) = p(2) + uy_p*delta_T;
    end
    u_x = ux_p;  u_y = uy_p;

    %—— 3) Dynamic Obstacle Avoidance (Keep original logic)
    for i = 1:num_dynamic
        d = hypot(Pos(1)-dyn_obs(i,1), Pos(2)-dyn_obs(i,2));
        if d < obstacle_radius + 0.5
            ang = atan2(Pos(2)-dyn_obs(i,2), Pos(1)-dyn_obs(i,1));
            f = avoidance_strength * exp(-d);
            u_x = u_x + f*cos(ang);
            u_y = u_y + f*sin(ang);
        end
    end

    %—— 4) Input Saturation
    mag = norm([u_x;u_y]);
    if mag > U_max
        u_x = u_x * U_max/mag;
        u_y = u_y * U_max/mag;
    end

    %—— 5) Calculate Angular Velocity from World Velocity
    w = ( -sin(Pos(3))*u_x + cos(Pos(3))*u_y ) / l;

    %—— 6) Update State
    Pos(1) = Pos(1) + u_x * delta_T;
    Pos(2) = Pos(2) + u_y * delta_T;
    Pos(3) = wrapToPi(Pos(3) + w * delta_T);

    %—— 7) Update Dynamic Obstacles
    dyn_obs = dyn_obs + dyn_vel;
    for i=1:num_dynamic
        set(h_dyn(i),'XData',dyn_obs(i,1),'YData',dyn_obs(i,2));
        if hypot(Pos(1)-dyn_obs(i,1), Pos(2)-dyn_obs(i,2)) < obstacle_radius + 0.5
            collisions = collisions + 1;
        end
    end

    %—— 8) Save GIF (Optional)
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if k==1
        imwrite(imind,cm,filename,'gif','LoopCount',inf,'DelayTime',0.01);
    elseif rem(k,2)==0
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.01);
    end

    drawnow;

    % Record Errors
    errors(1,k) = e_x;
    errors(2,k) = e_y;
end
%% 7) Error Evaluation
figure('Color','w');
subplot(2,1,1);
plot(T, errors(1,:),'r', T, errors(2,:),'b','LineWidth',1.2);
legend('e_x','e_y'); xlabel('Time'); ylabel('Error (m)');
title('Tracking Error');
subplot(2,1,2);
rmse = sqrt(mean(errors.^2,2));
bar(rmse);
set(gca,'XTickLabel',{'RMSE_x','RMSE_y'});
ylabel('Error (m)');
title('Root Mean Square Error');
% disp(['Total Collisions: ', num2str(collisions)]);