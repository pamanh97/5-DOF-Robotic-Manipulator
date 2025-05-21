function [q_d, q_d_dot, q_d_ddot] = generate_trajectory(traj_type, t, robot)
% GENERATE_TRAJECTORY Generates desired joint trajectories for the robot
%
% Inputs:
%   traj_type - Type of trajectory ('circular', 'square', 'pick_and_place')
%   t         - Time vector
%   robot     - Robot structure
%
% Outputs:
%   q_d       - Desired joint positions
%   q_d_dot   - Desired joint velocities
%   q_d_ddot  - Desired joint accelerations

N = length(t);
n = robot.n;
q_d = zeros(n, N);
q_d_dot = zeros(n, N);
q_d_ddot = zeros(n, N);

switch traj_type
    case 'circular'
        % Circular trajectory in XYZ space as specified in the paper
        x_d = zeros(1, N);
        y_d = zeros(1, N);
        z_d = zeros(1, N);
        
        for i = 1:N
            % From the paper: circular trajectory equations
            x_d(i) = 150 + 50 * sin(0.5 * t(i));   % mm
            y_d(i) = 150 + 50 * cos(0.5 * t(i));   % mm
            z_d(i) = 100 + 30 * sin(0.2 * t(i));   % mm
        end
        
        % Compute joint trajectories via inverse kinematics
        for i = 1:N
            % Compute target position in Cartesian space
            target_pos = [x_d(i); y_d(i); z_d(i)];
            
            % Inverse kinematics for 5DOF manipulator
            % Note: This is a simplified inverse kinematics.
            % A complete implementation would require solving the full IK problem
            q_d(:, i) = inverse_kinematics(target_pos, robot);
            
            % Compute joint velocities and accelerations via numerical differentiation
            if i > 1
                dt = t(i) - t(i-1);
                q_d_dot(:, i) = (q_d(:, i) - q_d(:, i-1)) / dt;
                
                if i > 2
                    q_d_ddot(:, i-1) = (q_d_dot(:, i) - q_d_dot(:, i-1)) / dt;
                end
            end
        end
        
        % Handle boundary cases for derivatives
        q_d_dot(:, 1) = q_d_dot(:, 2);
        q_d_ddot(:, 1) = q_d_ddot(:, 2);
        q_d_ddot(:, N) = q_d_ddot(:, N-1);
        
    case 'square'
        % Square trajectory with smooth angles
        % From the paper
        x_d = zeros(1, N);
        y_d = zeros(1, N);
        z_d = zeros(1, N);
        
        for i = 1:N
            x_d(i) = 150 + 50 * sign(sin(0.25 * t(i))) * tanh(5 * sin(0.25 * t(i)));
            y_d(i) = 150 + 50 * sign(cos(0.25 * t(i))) * tanh(5 * cos(0.25 * t(i)));
            z_d(i) = 100;  % Fixed height
        end
        
        % Same inverse kinematics process as for circular trajectory
        for i = 1:N
            target_pos = [x_d(i); y_d(i); z_d(i)];
            q_d(:, i) = inverse_kinematics(target_pos, robot);
            
            if i > 1
                dt = t(i) - t(i-1);
                q_d_dot(:, i) = (q_d(:, i) - q_d(:, i-1)) / dt;
                
                if i > 2
                    q_d_ddot(:, i-1) = (q_d_dot(:, i) - q_d_dot(:, i-1)) / dt;
                end
            end
        end
        
        % Handle boundary cases
        q_d_dot(:, 1) = q_d_dot(:, 2);
        q_d_ddot(:, 1) = q_d_ddot(:, 2);
        q_d_ddot(:, N) = q_d_ddot(:, N-1);
        
    case 'pick_and_place'
        % Pick-and-place trajectory
        % From the paper: piecewise linear with smooth transitions
        waypoints = {
            [0, 100, 150, 50],    % [time, x, y, z]
            [5, 200, 100, 50],
            [10, 200, 100, 150],
            [15, 100, 150, 150],
            [20, 100, 150, 50]
        };
        
        % Generate Cartesian trajectory from waypoints
        num_waypoints = length(waypoints);
        time_points = zeros(1, num_waypoints);
        x_waypoints = zeros(1, num_waypoints);
        y_waypoints = zeros(1, num_waypoints);
        z_waypoints = zeros(1, num_waypoints);
        
        for i = 1:num_waypoints
            time_points(i) = waypoints{i}(1);
            x_waypoints(i) = waypoints{i}(2);
            y_waypoints(i) = waypoints{i}(3);
            z_waypoints(i) = waypoints{i}(4);
        end
        
        % Generate smooth trajectory using spline interpolation
        x_d = interp1(time_points, x_waypoints, t, 'pchip');
        y_d = interp1(time_points, y_waypoints, t, 'pchip');
        z_d = interp1(time_points, z_waypoints, t, 'pchip');
        
        % Convert to joint space using inverse kinematics
        for i = 1:N
            target_pos = [x_d(i); y_d(i); z_d(i)];
            q_d(:, i) = inverse_kinematics(target_pos, robot);
            
            if i > 1
                dt = t(i) - t(i-1);
                q_d_dot(:, i) = (q_d(:, i) - q_d(:, i-1)) / dt;
                
                if i > 2
                    q_d_ddot(:, i-1) = (q_d_dot(:, i) - q_d_dot(:, i-1)) / dt;
                end
            end
        end
        
        % Handle boundary cases
        q_d_dot(:, 1) = q_d_dot(:, 2);
        q_d_ddot(:, 1) = q_d_ddot(:, 2);
        q_d_ddot(:, N) = q_d_ddot(:, N-1);
        
    otherwise
        error('Unknown trajectory type');
end

end
