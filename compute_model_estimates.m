function [M_hat, C_hat, G_hat, F_hat] = compute_model_estimates(q, q_dot, theta_hat, robot)
% COMPUTE_MODEL_ESTIMATES Computes model matrices using estimated parameters
%
% Inputs:
%   q         - Joint positions
%   q_dot     - Joint velocities
%   theta_hat - Estimated parameters
%   robot     - Robot structure
%
% Outputs:
%   M_hat     - Estimated inertia matrix
%   C_hat     - Estimated Coriolis matrix
%   G_hat     - Estimated gravity vector
%   F_hat     - Estimated friction vector

n = robot.n;

% Initialize matrices
M_hat = zeros(n, n);
C_hat = zeros(n, n);
G_hat = zeros(n, 1);
F_hat = zeros(n, 1);

% Extract parameters from theta_hat
% In this implementation, using structured approach for clarity

% Inertial parameters (15 parameters, 3 per link)
inertial_params = theta_hat(1:15);

% Link masses (5 parameters)
mass_params = theta_hat(16:20);

% Gravity components (5 parameters)
gravity_params = theta_hat(21:25);

% Friction parameters (2 per joint, 10 total)
friction_params = theta_hat(26:35);

% Compute transformation matrices for each joint
T = cell(n, 1);
T{1} = DH_transform(robot.DH(1, 1), robot.DH(1, 2), robot.DH(1, 3), q(1));
for i = 2:n
    T{i} = T{i-1} * DH_transform(robot.DH(i, 1), robot.DH(i, 2), robot.DH(i, 3), q(i));
end

% Construct inertia matrix using estimated parameters
for i = 1:n
    for j = 1:n
        if i >= j
            M_hat(i, j) = 0;
            
            % Add inertial contributions
            for k = 1:n
                if k >= max(i, j)
                    % Contribution from inertial parameters
                    idx = (k-1)*3;
                    M_hat(i, j) = M_hat(i, j) + ...
                        inertial_params(idx+1) * compute_inertial_regressor(i, k, 1, q, eye(n,1), T) * ...
                        compute_inertial_regressor(j, k, 1, q, eye(n,1), T);
                    
                    M_hat(i, j) = M_hat(i, j) + ...
                        inertial_params(idx+2) * compute_inertial_regressor(i, k, 2, q, eye(n,1), T) * ...
                        compute_inertial_regressor(j, k, 2, q, eye(n,1), T);
                    
                    M_hat(i, j) = M_hat(i, j) + ...
                        inertial_params(idx+3) * compute_inertial_regressor(i, k, 3, q, eye(n,1), T) * ...
                        compute_inertial_regressor(j, k, 3, q, eye(n,1), T);
                    
                    % Contribution from masses
                    M_hat(i, j) = M_hat(i, j) + ...
                        mass_params(k) * compute_mass_regressor(i, k, q, eye(n,1), T) * ...
                        compute_mass_regressor(j, k, q, eye(n,1), T);
                end
            end
        else
            % Ensure symmetry
            M_hat(i, j) = M_hat(j, i);
        end
    end
end

% Construct Coriolis matrix using estimated parameters
% Using numerical differentiation of M_hat
for i = 1:n
    for j = 1:n
        C_hat(i, j) = 0;
        
        for k = 1:n
            % Approximate Christoffel symbols
            delta = 1e-6;
            q_plus = q;
            q_plus(k) = q(k) + delta;
            
            % Recompute M_hat at perturbed position
            [M_hat_plus, ~, ~, ~] = compute_model_estimates(q_plus, q_dot, theta_hat, robot);
            
            % Numerical approximation of partial derivative
            dM_dq = (M_hat_plus - M_hat) / delta;
            
            % Christoffel symbols
            Gamma = 0.5 * (dM_dq(i, j) + dM_dq(i, k) - dM_dq(j, k));
            
            % Coriolis matrix element
            C_hat(i, j) = C_hat(i, j) + Gamma * q_dot(k);
        end
    end
end

% Construct gravity vector using estimated parameters
for i = 1:n
    G_hat(i) = 0;
    
    for k = 1:n
        if k >= i
            G_hat(i) = G_hat(i) + gravity_params(k) * compute_gravity_regressor(i, k, q, T);
        end
    end
end

% Construct friction vector using estimated parameters
for i = 1:n
    % Coulomb friction
    F_hat(i) = friction_params(2*i-1) * sign(q_dot(i));
    
    % Viscous friction
    F_hat(i) = F_hat(i) + friction_params(2*i) * q_dot(i);
end

end
