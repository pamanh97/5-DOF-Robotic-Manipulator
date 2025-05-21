function [q_next, q_dot_next, q_ddot] = robot_dynamics(q, q_dot, tau, tau_d, Ts, robot)
% ROBOT_DYNAMICS Simulates robot dynamics for one time step
%
% Inputs:
%   q       - Current joint positions
%   q_dot   - Current joint velocities
%   tau     - Control torques
%   tau_d   - External disturbances
%   Ts      - Sample time
%   robot   - Robot structure
%
% Outputs:
%   q_next     - Next joint positions
%   q_dot_next - Next joint velocities
%   q_ddot     - Joint accelerations

% Compute manipulator matrices
[M, C, G, F] = manipulator_matrices(q, q_dot, robot);

% Compute joint accelerations from dynamic equation
% M(q)q̈ + C(q,q̇)q̇ + G(q) + F(q̇) + τ_d = τ
q_ddot = M \ (tau - C * q_dot - G - F - tau_d);

% Apply joint acceleration limits
q_ddot_max = deg2rad([800, 800, 800, 800, 800]);
q_ddot = min(q_ddot_max', max(-q_ddot_max', q_ddot));

% Integrate using Euler method
q_dot_next = q_dot + q_ddot * Ts;

% Apply joint velocity limits
q_dot_max = deg2rad([180, 180, 180, 180, 180]);
q_dot_next = min(q_dot_max', max(-q_dot_max', q_dot_next));

q_next = q + q_dot_next * Ts;

% Apply joint position limits
q_next = min(robot.q_max', max(robot.q_min', q_next));

end
