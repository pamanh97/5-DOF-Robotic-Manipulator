function [tau, theta_hat] = hinf_controller(q, q_dot, q_d, q_d_dot, q_d_ddot, e, e_dot, r, theta_hat, K_e, K_r, Gamma, robot)
% HINF_CONTROLLER Computes control input using adaptive H-infinity method
%
% Inputs:
%   q         - Joint positions
%   q_dot     - Joint velocities
%   q_d       - Desired joint positions
%   q_d_dot   - Desired joint velocities
%   q_d_ddot  - Desired joint accelerations
%   e         - Position error
%   e_dot     - Velocity error
%   r         - Sliding variable
%   theta_hat - Estimated parameters
%   K_e       - Error gain
%   K_r       - Reference gain
%   Gamma     - Adaptation gain
%   robot     - Robot structure
%
% Outputs:
%   tau       - Control torque
%   theta_hat - Updated parameter estimates

% Compute reference velocity and acceleration
Lambda = diag([10, 10, 10, 10, 10]);  % Sliding surface parameter
q_r_dot = q_d_dot + Lambda * e;
q_r_ddot = q_d_ddot + Lambda * e_dot;

% Compute regressor matrix Y
Y = compute_regressor(q, q_dot, q_r_dot, q_r_ddot, robot);

% Update parameter estimates
theta_hat_dot = -Gamma * Y' * r;
Ts = 0.001; % Assuming 1ms sample time
theta_hat = theta_hat + theta_hat_dot * Ts;

% Compute dynamic model estimates
[M_hat, C_hat, G_hat, ~] = compute_model_estimates(q, q_dot, theta_hat, robot);

% Compute H-infinity control law
tau = M_hat * q_r_ddot + C_hat * q_r_dot + G_hat - K_e * e - K_r * r;

end
