function Y = compute_regressor(q, q_dot, q_r_dot, q_r_ddot, robot)
% COMPUTE_REGRESSOR Computes the regressor matrix for parameter adaptation
%
% Inputs:
%   q        - Joint positions
%   q_dot    - Joint velocities
%   q_r_dot  - Reference velocities
%   q_r_ddot - Reference accelerations
%   robot    - Robot structure
%
% Output:
%   Y        - Regressor matrix

n = robot.n;

% In this implementation, we'll use a simplified regressor with 27 parameters:
% - 15 inertial parameters (3 per link)
% - 5 link masses
% - 5 gravity components
% - 2 friction parameters per joint (10 total)

Y = zeros(n, 27);

% Compute key kinematic transformations
T = cell(n, 1);
T{1} = DH_transform(robot.DH(1, 1), robot.DH(1, 2), robot.DH(1, 3), q(1));
for i = 2:n
    T{i} = T{i-1} * DH_transform(robot.DH(i, 1), robot.DH(i, 2), robot.DH(i, 3), q(i));
end

% Fill the regressor matrix columns
col_idx = 0;

% Inertial parameters (3 per link)
for i = 1:n
    for j = 1:3
        col_idx = col_idx + 1;
        
        for k = 1:n
            if k <= i
                % Compute the effect of each parameter on each joint
                Y(k, col_idx) = compute_inertial_regressor(k, i, j, q, q_r_ddot, T);
            end
        end
    end
end

% Link masses
for i = 1:n
    col_idx = col_idx + 1;
    
    for k = 1:n
        if k <= i
            % Compute the effect of each mass on each joint
            Y(k, col_idx) = compute_mass_regressor(k, i, q, q_r_ddot, T);
        end
    end
end

% Gravity components
for i = 1:n
    col_idx = col_idx + 1;
    
    for k = 1:n
        if k <= i
            % Compute the effect of gravity on each joint
            Y(k, col_idx) = compute_gravity_regressor(k, i, q, T);
        end
    end
end

% Friction parameters (Coulomb and viscous)
for i = 1:n
    % Coulomb friction
    col_idx = col_idx + 1;
    Y(i, col_idx) = sign(q_dot(i));
    
    % Viscous friction
    col_idx = col_idx + 1;
    Y(i, col_idx) = q_dot(i);
end

end

function val = compute_inertial_regressor(joint, link, param, q, q_r_ddot, T)
% Compute the regressor component for inertial parameters
% This is a simplified implementation
    if joint == link
        val = q_r_ddot(joint);
    else
        % Approximate coupling effects
        R = T{link}(1:3, 1:3);
        val = 0.1 * norm(R) * q_r_ddot(joint);
    end
end

function val = compute_mass_regressor(joint, link, q, q_r_ddot, T)
% Compute the regressor component for masses
% This is a simplified implementation
    if joint == link
        val = q_r_ddot(joint);
    else
        % Approximate coupling effects
        val = 0.05 * q_r_ddot(joint);
    end
end

function val = compute_gravity_regressor(joint, link, q, T)
% Compute the regressor component for gravity
% This is a simplified implementation
    g = [0; 0; 9.81]; % Gravity vector
    
    % Transform gravity to joint frame
    if joint == 1
        R_0_j = eye(3);
    else
        R_0_j = T{joint-1}(1:3, 1:3);
    end
    
    g_j = R_0_j' * g;
    
    if joint == link
        val = g_j(3); % z-component
    else
        val = 0.1 * g_j(3);
    end
end
