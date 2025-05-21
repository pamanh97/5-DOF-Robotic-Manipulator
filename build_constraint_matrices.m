function [A_ineq, b_ineq] = build_constraint_matrices(F, x, x_min, x_max, u_min, u_max, du_min, du_max, u_prev, Nc)
% BUILD_CONSTRAINT_MATRICES Creates constraint matrices for MPC QP problem
%
% Inputs:
%   F       - Free response matrix
%   x       - Current state
%   x_min   - Minimum state constraints
%   x_max   - Maximum state constraints
%   u_min   - Minimum input constraints
%   u_max   - Maximum input constraints
%   du_min  - Minimum input rate constraints
%   du_max  - Maximum input rate constraints
%   u_prev  - Previous control input
%   Nc      - Control horizon
%
% Outputs:
%   A_ineq  - Inequality constraint matrix
%   b_ineq  - Inequality constraint vector

nx = size(F, 2);
nu = length(u_min);
Np = size(F, 1) / nx;

% Number of constraints
n_state_constraints = 2 * nx * Np;  % Min and max for each state over horizon
n_input_constraints = 2 * nu * Nc;  % Min and max for each input over horizon
n_rate_constraints = 2 * nu * Nc;   % Min and max for each input rate over horizon

% Total constraints
n_constraints = n_state_constraints + n_input_constraints + n_rate_constraints;

% Initialize constraint matrices
A_ineq = zeros(n_constraints, nu * Nc);
b_ineq = zeros(n_constraints, 1);

% Input constraints: u_min <= u <= u_max
% Lower bounds
for i = 1:Nc
    row_idx = (i-1) * nu + 1;
    col_idx = (i-1) * nu + 1;
    A_ineq(row_idx:row_idx+nu-1, col_idx:col_idx+nu-1) = -eye(nu);
    b_ineq(row_idx:row_idx+nu-1) = -u_min;
end

% Upper bounds
for i = 1:Nc
    row_idx = Nc * nu + (i-1) * nu + 1;
    col_idx = (i-1) * nu + 1;
    A_ineq(row_idx:row_idx+nu-1, col_idx:col_idx+nu-1) = eye(nu);
    b_ineq(row_idx:row_idx+nu-1) = u_max;
end

% Input rate constraints: du_min <= u(k) - u(k-1) <= du_max
% Lower bounds
for i = 1:Nc
    row_idx = 2 * Nc * nu + (i-1) * nu + 1;
    col_idx = (i-1) * nu + 1;
    A_ineq(row_idx:row_idx+nu-1, col_idx:col_idx+nu-1) = -eye(nu);
    if i == 1
        b_ineq(row_idx:row_idx+nu-1) = -du_min - u_prev;
    else
        A_ineq(row_idx:row_idx+nu-1, col_idx-nu:col_idx-1) = eye(nu);
        b_ineq(row_idx:row_idx+nu-1) = -du_min;
    end
end

% Upper bounds
for i = 1:Nc
    row_idx = 3 * Nc * nu + (i-1) * nu + 1;
    col_idx = (i-1) * nu + 1;
    A_ineq(row_idx:row_idx+nu-1, col_idx:col_idx+nu-1) = eye(nu);
    if i == 1
        b_ineq(row_idx:row_idx+nu-1) = du_max + u_prev;
    else
        A_ineq(row_idx:row_idx+nu-1, col_idx-nu:col_idx-1) = -eye(nu);
        b_ineq(row_idx:row_idx+nu-1) = du_max;
    end
end

% State constraints: x_min <= x <= x_max
% Need to convert to constraints on U using prediction model: x = F*x0 + H*U
% Extracting the H matrix (already computed in the MPC controller)
H = zeros(nx * Np, nu * Nc);  % Placeholder to compute from prediction model

% Compute H for state constraints (simplified approach)
% This would actually be passed in from the MPC controller
% For this example, we'll use a simplified approximation
A_temp = eye(nx);
B_temp = zeros(nx, nu);
B_temp(nx/2+1:nx, :) = eye(nu);  % Assuming state = [q; q_dot]

for i = 1:Np
    for j = 1:min(i, Nc)
        if i == j
            H(nx*(i-1)+1:nx*i, nu*(j-1)+1:nu*j) = B_temp;
        else
            H(nx*(i-1)+1:nx*i, nu*(j-1)+1:nu*j) = A_temp^(i-j) * B_temp;
        end
    end
    A_temp = A_temp * A_temp;
end

% Now set up state constraints
% Lower bounds
row_offset = 4 * Nc * nu;
for i = 1:Np
    row_idx = row_offset + (i-1) * nx + 1;
    A_ineq(row_idx:row_idx+nx-1, :) = -H(nx*(i-1)+1:nx*i, :);
    b_ineq(row_idx:row_idx+nx-1) = -x_min + F(nx*(i-1)+1:nx*i, :) * x;
end

% Upper bounds
row_offset = 4 * Nc * nu + Np * nx;
for i = 1:Np
    row_idx = row_offset + (i-1) * nx + 1;
    A_ineq(row_idx:row_idx+nx-1, :) = H(nx*(i-1)+1:nx*i, :);
    b_ineq(row_idx:row_idx+nx-1) = x_max - F(nx*(i-1)+1:nx*i, :) * x;
end

end
