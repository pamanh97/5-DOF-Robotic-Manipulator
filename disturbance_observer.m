function [tau_d_hat, z_next] = disturbance_observer(q, q_dot, tau, z, L, K_p, robot)
% DISTURBANCE_OBSERVER Estimates external disturbances
%
% Inputs:
%   q       - Joint positions
%   q_dot   - Joint velocities
%   tau     - Control torques
%   z       - Observer state
%   L       - Observer gain matrix
%   K_p     - Auxiliary function parameter
%   robot   - Robot structure
%
% Outputs:
%   tau_d_hat - Estimated disturbances
%   z_next    - Updated observer state

% Compute manipulator matrices
[M, C, G, F] = manipulator_matrices(q, q_dot, robot);

% Auxiliary function p(q, q_dot) = -K_p * q as in the paper
p = -K_p * q;

% Observer dynamics (Eq. from paper)
z_dot = -L * z - L * (p + M \ (tau - C * q_dot - G - F));

% Integrate using Euler method
Ts = 0.001; % Assuming 1ms sample time
z_next = z + z_dot * Ts;

% Estimate disturbance
tau_d_hat = z_next + p;

end
