function [tau, mpc_state] = mpc_controller(q, q_dot, q_d, q_d_dot, mpc_state)
% MPC_CONTROLLER Computes control input using Model Predictive Control
%
% Inputs:
%   q          - Current joint positions
%   q_dot      - Current joint velocities
%   q_d        - Desired joint positions trajectory
%   q_d_dot    - Desired joint velocities trajectory
%   mpc_state  - MPC controller state
%
% Outputs:
%   tau        - Control torque
%   mpc_state  - Updated MPC state

% Extract MPC parameters
Np = mpc_state.Np;      % Prediction horizon
Nc = mpc_state.Nc;      % Control horizon
Q = mpc_state.Q;        % State weight matrix
R = mpc_state.R;        % Control weight matrix
S = mpc_state.S;        % Control increment weight matrix
A = mpc_state.A;        % State matrix
B = mpc_state.B;        % Input matrix
x_min = mpc_state.x_min;  % State constraints
x_max = mpc_state.x_max;
u_min = mpc_state.u_min;  % Input constraints
u_max = mpc_state.u_max;
du_min = mpc_state.du_min; % Input rate constraints
du_max = mpc_state.du_max;

% Current state
x = [q; q_dot];

% Update linearized model at current operating point
[A, B] = linearize_model(q, q_dot, mpc_state.u_prev);
mpc_state.A = A;
mpc_state.B = B;

% Formulate the QP problem
% Build prediction matrices
[F, H] = build_prediction_matrices(A, B, Np, Nc);

% Build cost function matrices
[H_qp, f_qp] = build_cost_matrices(H, F, Q, R, S, x, q_d, q_d_dot, mpc_state.u_prev);

% Build constraint matrices
[A_ineq, b_ineq] = build_constraint_matrices(F, x, x_min, x_max, u_min, u_max, du_min, du_max, mpc_state.u_prev, Nc);

% Solve QP problem
try
    % Using MATLAB's quadprog
    options = optimoptions('quadprog', 'Display', 'off');
    u_sequence = quadprog(H_qp, f_qp, A_ineq, b_ineq, [], [], [], [], [], options);
    
    % Extract first control action
    tau = u_sequence(1:5);
    
    % Update MPC state
    mpc_state.u_prev = tau;
catch
    % If optimization fails, use the previous control action
    warning('MPC optimization failed. Using previous control action.');
    tau = mpc_state.u_prev;
end

end
