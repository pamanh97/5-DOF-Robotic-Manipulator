function mpc_state = initialize_mpc(robot, Np, Nc, Q, R, Ts)
% INITIALIZE_MPC Initializes the MPC controller
%
% Inputs:
%   robot   - Robot structure
%   Np      - Prediction horizon
%   Nc      - Control horizon
%   Q       - State weight matrix
%   R       - Input weight matrix
%   Ts      - Sample time
%
% Output:
%   mpc_state - MPC controller state

n = robot.n;

% Initialize state with nominal model at zero position and velocity
q_init = zeros(n, 1);
q_dot_init = zeros(n, 1);
u_init = zeros(n, 1);

% Linearize the robot model
[A, B] = linearize_model(q_init, q_dot_init, u_init);

% Define MPC state
mpc_state.Np = Np;
mpc_state.Nc = Nc;
mpc_state.Q = Q;
mpc_state.R = R;
mpc_state.S = 0.1 * eye(n);  % Control increment weight
mpc_state.A = A;
mpc_state.B = B;
mpc_state.Ts = Ts;
mpc_state.u_prev = u_init;

% Define constraints
mpc_state.x_min = [robot.q_min'; -deg2rad(180)*ones(n, 1)];
mpc_state.x_max = [robot.q_max'; deg2rad(180)*ones(n, 1)];
mpc_state.u_min = -[2; 2; 1.5; 1; 0.5];
mpc_state.u_max = [2; 2; 1.5; 1; 0.5];
mpc_state.du_min = -[0.5; 0.5; 0.4; 0.3; 0.2];
mpc_state.du_max = [0.5; 0.5; 0.4; 0.3; 0.2];

end
