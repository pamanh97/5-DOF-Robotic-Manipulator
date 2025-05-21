function [A, B] = linearize_model(q, q_dot, u)
% LINEARIZE_MODEL Linearizes the robot model around the current state
%
% Inputs:
%   q       - Joint positions
%   q_dot   - Joint velocities
%   u       - Control input
%
% Outputs:
%   A       - State matrix
%   B       - Input matrix

% Number of joints
n = length(q);

% State dimensions
nx = 2*n;
nu = n;

% Create state vector
x = [q; q_dot];

% Initialize matrices
A = zeros(nx, nx);
B = zeros(nx, nu);

% Set up identity matrices for position-velocity relationship
A(1:n, n+1:2*n) = eye(n);

% Create a robot structure for manipulator matrices calculation
robot.n = n;
robot.DH = [0, pi/2, 28, q(1);     % Joint 1
           150, 0, 0, q(2);       % Joint 2
           85, 0, 0, q(3);        % Joint 3
           50, pi/2, 0, q(4);     % Joint 4
           0, 0, 40, q(5)];       % Joint 5
robot.m = [0.40, 0.25, 0.20, 0.15, 0.20];
robot.I = {diag([0.007, 0.007, 0.0005]), ...
           diag([0.005, 0.005, 0.0003]), ...
           diag([0.004, 0.004, 0.0002]), ...
           diag([0.003, 0.003, 0.0002]), ...
           diag([0.002, 0.002, 0.0001])};
robot.g = 9.81;
robot.F_c = 0.02 * ones(n, 1);
robot.F_v = 0.001 * ones(n, 1);
robot.r_com = zeros(3, n);
for i = 1:n
    if i < n
        robot.r_com(:, i) = [robot.DH(i,1)/2; 0; robot.DH(i,3)/2];
    else
        robot.r_com(:, i) = [0; 0; robot.DH(i,3)/2];
    end
end

% Compute manipulator matrices at current state
[M, C, G, F] = manipulator_matrices(q, q_dot, robot);

% Compute state-space matrices for velocity dynamics
A(n+1:2*n, 1:n) = -M \ (diff_G_q(q, robot));
A(n+1:2*n, n+1:2*n) = -M \ C;
B(n+1:2*n, :) = M \ eye(n);

% Apply small sample time (Ts=0.001) for discretization
Ts = 0.001;
A_d = eye(nx) + Ts * A;
B_d = Ts * B;

A = A_d;
B = B_d;

end

function dG_dq = diff_G_q(q, robot)
% Numerical approximation of the Jacobian of gravity vector
n = robot.n;
G = zeros(n, 1);
dG_dq = zeros(n, n);

% Compute gravity vector
[~, ~, G, ~] = manipulator_matrices(q, zeros(n, 1), robot);

% Numerical differentiation for each element
delta = 1e-6;
for i = 1:n
    q_perturbed = q;
    q_perturbed(i) = q(i) + delta;
    [~, ~, G_perturbed, ~] = manipulator_matrices(q_perturbed, zeros(n, 1), robot);
    dG_dq(:, i) = (G_perturbed - G) / delta;
end

end
