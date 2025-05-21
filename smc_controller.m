function tau = smc_controller(q, q_dot, q_d, q_d_dot, q_d_ddot, s, tau_d_hat, K_s, delta, robot)
% SMC_CONTROLLER Computes control input using sliding mode control
%
% Inputs:
%   q         - Joint positions
%   q_dot     - Joint velocities
%   q_d       - Desired joint positions
%   q_d_dot   - Desired joint velocities
%   q_d_ddot  - Desired joint accelerations
%   s         - Sliding variable
%   tau_d_hat - Estimated disturbances
%   K_s       - Sliding gain
%   delta     - Boundary layer thickness
%   robot     - Robot structure
%
% Output:
%   tau       - Control torque

% Compute manipulator matrices
[M, C, G, F] = manipulator_matrices(q, q_dot, robot);

% Compute sliding mode control with disturbance compensation
Lambda = diag([10, 10, 10, 10, 10]);  % Sliding surface parameter
e = q_d - q;
e_dot = q_d_dot - q_dot;

% Use continuous approximation of the sign function to mitigate chattering
sat_s = s ./ (abs(s) + delta);

% SMC control law (from paper)
tau = M * (q_d_ddot + Lambda * e_dot) + C * q_dot + G + F - tau_d_hat - K_s * sat_s;

end
