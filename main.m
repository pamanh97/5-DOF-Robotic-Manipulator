%% Adaptive H-Infinity MPC-Fuzzy Control with Sliding-Mode Disturbance Observer
% For Trajectory Tracking of a 5-DOF Robotic Manipulator with Gripper
% Implementation based on the research paper by Duc-Anh Pham and Seung-Hun Han

clear all; close all; clc;

%% Simulation Parameters
Ts = 0.001;            % Sample time: 1 ms
Tf = 20;               % Simulation time: 20 seconds
t = 0:Ts:Tf;           % Time vector
N = length(t);         % Number of samples

% Controller parameters
% MPC parameters
Np = 10;               % Prediction horizon
Nc = 3;                % Control horizon
Q_mpc = diag([100, 100, 100, 100, 100, 10, 10, 10, 10, 10]);  % State weight
R_mpc = diag([0.1, 0.1, 0.1, 0.1, 0.1]);                      % Control weight

% Adaptation parameters
Gamma = diag([5, 5, 5, 5, 5]);  % Adaptation gain

% H-infinity parameters
K_e = diag([15, 15, 15, 15, 15]);  % Error gain
K_r = diag([25, 25, 25, 25, 25]);  % Reference gain

% Sliding mode parameters
Lambda = diag([10, 10, 10, 10, 10]);  % Sliding surface parameter
K_s = diag([20, 20, 20, 20, 20]);     % Sliding gain
delta = 0.05;                         % Boundary layer thickness

% Disturbance observer parameters
L = 50 * eye(5);  % Observer gain
K_p = diag([10, 10, 10, 10, 10]);  % Auxiliary function parameter

% Fuzzy parameters
sigma = [0.1, 0.1, 0.1, 0.1, 0.1];          % Error scaling
sigma_d = [0.05, 0.05, 0.05, 0.05, 0.05];   % Error rate scaling

% Adaptive weighting parameters
beta = [10, 5, 15, 20];  % Weights for H-infinity, MPC, Fuzzy, SMC

%% Robot Model Parameters
% DH Parameters [a, alpha, d, theta]
% Table 1 from the paper
DH = [0, pi/2, 28, 0;     % Joint 1
      150, 0, 0, 0;       % Joint 2
      85, 0, 0, 0;        % Joint 3
      50, pi/2, 0, 0;     % Joint 4
      0, 0, 40, 0];       % Joint 5

% Link masses (kg)
m = [0.40, 0.25, 0.20, 0.15, 0.20];

% Link inertia tensors (kg·m^2)
I1 = diag([0.007, 0.007, 0.0005]);
I2 = diag([0.005, 0.005, 0.0003]);
I3 = diag([0.004, 0.004, 0.0002]);
I4 = diag([0.003, 0.003, 0.0002]);
I5 = diag([0.002, 0.002, 0.0001]);
I = {I1, I2, I3, I4, I5};

% Friction parameters
F_c = 0.02 * ones(5, 1);  % Coulomb friction (N·m)
F_v = 0.001 * ones(5, 1); % Viscous friction (N·m·s/deg)

% Joint limits
q_min = deg2rad([-180, -120, -120, -180, -120]);
q_max = deg2rad([180, 120, 120, 180, 120]);
q_dot_max = deg2rad([180, 180, 180, 180, 180]);
q_ddot_max = deg2rad([800, 800, 800, 800, 800]);
tau_max = [2, 2, 1.5, 1, 0.5];

% Initialize robot model
robot = initialize_robot(DH, m, I, F_c, F_v, q_min, q_max);

%% Generate Reference Trajectory
% Circular trajectory as in the paper
traj_type = 'circular';
[q_d, q_d_dot, q_d_ddot] = generate_trajectory(traj_type, t, robot);

%% Initialize State Variables
q = zeros(5, N);
q_dot = zeros(5, N);
q_ddot = zeros(5, N);
tau = zeros(5, N);

% Initial joint positions (assuming starting at trajectory beginning)
q(:, 1) = q_d(:, 1);
q_dot(:, 1) = q_d_dot(:, 1);

% Initial parameter estimates (nominal values)
theta_hat = zeros(27, 1); % Assuming 27 dynamic parameters to estimate

% Initial disturbance observer states
z = zeros(5, 1);
tau_d_hat = zeros(5, N);
tau_d = zeros(5, N); % Actual disturbance

% Initialize controllers
mpc_state = initialize_mpc(robot, Np, Nc, Q_mpc, R_mpc, Ts);

% Initialize adaptive weights
alpha = ones(4, 1) / 4; % Equal weights initially

%% Main Simulation Loop
for k = 1:N-1
    % Current time
    t_curr = t(k);
    
    % Apply external disturbance between 5-7 seconds
    if t_curr >= 5 && t_curr <= 7
        tau_d(:, k) = [5; 5; 5; 0; 0]; % 5N force on first 3 joints
    else
        tau_d(:, k) = zeros(5, 1);
    end
    
    % Current state
    q_k = q(:, k);
    q_dot_k = q_dot(:, k);
    
    % Calculate tracking error
    e = q_d(:, k) - q_k;
    e_dot = q_d_dot(:, k) - q_dot_k;
    
    % Reference signals for controller
    q_r_dot = q_d_dot(:, k) + Lambda * e;
    q_r_ddot = q_d_ddot(:, k) + Lambda * e_dot;
    r = e_dot + Lambda * e;
    
    % Sliding surface
    s = r;
    
    % Update disturbance observer
    [tau_d_hat(:, k), z] = disturbance_observer(q_k, q_dot_k, tau(:, max(k-1, 1)), z, L, K_p, robot);
    
    % Compute control inputs from each controller
    % 1. Adaptive H-infinity Controller
    [tau_hinf, theta_hat] = hinf_controller(q_k, q_dot_k, q_d(:, k), q_d_dot(:, k), q_d_ddot(:, k), ...
                                          e, e_dot, r, theta_hat, K_e, K_r, Gamma, robot);
    
    % 2. Model Predictive Controller
    [tau_mpc, mpc_state] = mpc_controller(q_k, q_dot_k, q_d(:, k:min(k+Np-1, N)), ...
                                        q_d_dot(:, k:min(k+Np-1, N)), mpc_state);
    
    % 3. Fuzzy Controller
    tau_flc = fuzzy_controller(e, e_dot, sigma, sigma_d);
    
    % 4. Sliding Mode Controller with disturbance observer
    tau_smc = smc_controller(q_k, q_dot_k, q_d(:, k), q_d_dot(:, k), q_d_ddot(:, k), ...
                           s, tau_d_hat(:, k), K_s, delta, robot);
    
    % Update adaptive weights
    alpha = update_weights(e, beta);
    
    % Compute integrated control input
    tau(:, k) = alpha(1) * tau_hinf + alpha(2) * tau_mpc + alpha(3) * tau_flc + alpha(4) * tau_smc;
    
    % Apply torque limits
    tau(:, k) = min(tau_max, max(-tau_max, tau(:, k)));
    
    % Simulate robot dynamics for one step
    [q(:, k+1), q_dot(:, k+1), q_ddot(:, k)] = robot_dynamics(q_k, q_dot_k, tau(:, k), tau_d(:, k), Ts, robot);
end

%% Calculate Performance Metrics
% RMS tracking error
rms_error = sqrt(mean(sum((q_d - q).^2, 1)));
fprintf('RMS Tracking Error: %.4f rad\n', rms_error);

% Control effort
control_effort = sum(sum(tau.^2)) * Ts;
fprintf('Control Effort: %.4f N²·m²·s\n', control_effort);

%% Visualize Results
visualize_results(t, q, q_d, q_dot, q_d_dot, tau, alpha, tau_d, tau_d_hat);
