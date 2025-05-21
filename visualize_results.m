function visualize_results(t, q, q_d, q_dot, q_d_dot, tau, alpha, tau_d, tau_d_hat)
% VISUALIZE_RESULTS Creates plots of simulation results
%
% Inputs:
%   t         - Time vector
%   q         - Joint positions
%   q_d       - Desired joint positions
%   q_dot     - Joint velocities
%   q_d_dot   - Desired joint velocities
%   tau       - Control torques
%   alpha     - Adaptive weights
%   tau_d     - External disturbances
%   tau_d_hat - Estimated disturbances

% Calculate tracking errors
e = q_d - q;

% Figure 1: Joint positions
figure('Name', 'Joint Positions', 'Position', [100, 100, 800, 600]);
for i = 1:5
    subplot(3, 2, i);
    plot(t, rad2deg(q(i, :)), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(t, rad2deg(q_d(i, :)), 'r--', 'LineWidth', 1.5);
    grid on;
    title(['Joint ', num2str(i), ' Position']);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    legend('Actual', 'Desired');
end

% Figure 2: Tracking errors
figure('Name', 'Tracking Errors', 'Position', [150, 150, 800, 600]);
for i = 1:5
    subplot(3, 2, i);
    plot(t, rad2deg(e(i, :)), 'b-', 'LineWidth', 1.5);
    grid on;
    title(['Joint ', num2str(i), ' Error']);
    xlabel('Time (s)');
    ylabel('Error (deg)');
end

% Figure 3: Control torques
figure('Name', 'Control Torques', 'Position', [200, 200, 800, 600]);
for i = 1:5
    subplot(3, 2, i);
    plot(t, tau(i, :), 'b-', 'LineWidth', 1.5);
    grid on;
    title(['Joint ', num2str(i), ' Torque']);
    xlabel('Time (s)');
    ylabel('Torque (N·m)');
end

% Figure 4: Adaptive weights
figure('Name', 'Adaptive Weights', 'Position', [250, 250, 800, 300]);
for i = 1:4
    subplot(1, 4, i);
    plot(t, alpha(i, :), 'LineWidth', 1.5);
    grid on;
    title(['Weight \alpha_', num2str(i)]);
    xlabel('Time (s)');
    ylabel('Weight');
    ylim([0, 1]);
end

% Figure 5: Disturbances and estimation
figure('Name', 'Disturbance Estimation', 'Position', [300, 300, 800, 600]);
for i = 1:3  % Show only the first 3 joints for clarity
    subplot(3, 1, i);
    plot(t, tau_d(i, :), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(t, tau_d_hat(i, :), 'r--', 'LineWidth', 1.5);
    grid on;
    title(['Joint ', num2str(i), ' Disturbance']);
    xlabel('Time (s)');
    ylabel('Torque (N·m)');
    legend('Actual', 'Estimated');
end

% Figure 6: Trajectory in Cartesian space
figure('Name', 'Trajectory in Cartesian Space', 'Position', [350, 350, 800, 600]);

% For this we need forward kinematics
xyz_actual = zeros(3, length(t));
xyz_desired = zeros(3, length(t));

% DH parameters
DH = [0, pi/2, 28, 0;     % Joint 1
      150, 0, 0, 0;       % Joint 2
      85, 0, 0, 0;        % Joint 3
      50, pi/2, 0, 0;     % Joint 4
      0, 0, 40, 0];       % Joint 5

for k = 1:length(t)
    % Forward kinematics for actual position
    T = eye(4);
    for i = 1:5
        T = T * DH_transform(DH(i, 1), DH(i, 2), DH(i, 3), q(i, k));
    end
    xyz_actual(:, k) = T(1:3, 4) * 1000;  % Convert to mm
    
    % Forward kinematics for desired position
    T = eye(4);
    for i = 1:5
        T = T * DH_transform(DH(i, 1), DH(i, 2), DH(i, 3), q_d(i, k));
    end
    xyz_desired(:, k) = T(1:3, 4) * 1000;  % Convert to mm
end

% 3D plot
subplot(1, 2, 1);
plot3(xyz_actual(1, :), xyz_actual(2, :), xyz_actual(3, :), 'b-', 'LineWidth', 2);
hold on;
plot3(xyz_desired(1, :), xyz_desired(2, :), xyz_desired(3, :), 'r--', 'LineWidth', 2);
grid on;
title('Trajectory in 3D Space');
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
legend('Actual', 'Desired');
axis equal;

% XY projection
subplot(1, 2, 2);
plot(xyz_actual(1, :), xyz_actual(2, :), 'b-', 'LineWidth', 2);
hold on;
plot(xyz_desired(1, :), xyz_desired(2, :), 'r--', 'LineWidth', 2);
grid on;
title('Trajectory Projection on XY Plane');
xlabel('X (mm)');
ylabel('Y (mm)');
legend('Actual', 'Desired');
axis equal;

% Figure 7: Performance evaluation
figure('Name', 'Performance Evaluation', 'Position', [400, 400, 800, 400]);

% RMS Tracking Error
subplot(1, 2, 1);
rms_error = sqrt(mean(e.^2, 2));
bar(rms_error);
grid on;
title('RMS Tracking Error');
xlabel('Joint');
ylabel('Error (rad)');
set(gca, 'XTick', 1:5);

% Control Effort
subplot(1, 2, 2);
control_effort = sum(tau.^2, 2) * (t(2) - t(1));
bar(control_effort);
grid on;
title('Control Effort');
xlabel('Joint');
ylabel('Effort (N²·m²·s)');
set(gca, 'XTick', 1:5);

end
