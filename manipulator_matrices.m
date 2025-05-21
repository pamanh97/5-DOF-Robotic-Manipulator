function [M, C, G, F] = manipulator_matrices(q, q_dot, robot)
% MANIPULATOR_MATRICES Computes dynamic matrices for the robot manipulator
%
% Inputs:
%   q       - Joint positions
%   q_dot   - Joint velocities
%   robot   - Robot structure
%
% Outputs:
%   M       - Inertia matrix
%   C       - Coriolis and centrifugal matrix
%   G       - Gravity vector
%   F       - Friction vector

n = robot.n;
g = robot.g;  % Gravity acceleration

% Initialize matrices
M = zeros(n, n);
C = zeros(n, n);
G = zeros(n, 1);
F = zeros(n, 1);

% Compute forward kinematics and Jacobians for each link
T = cell(n, 1);
Jv = cell(n, n);
Jw = cell(n, n);

% Forward kinematics for each joint
T{1} = DH_transform(robot.DH(1, 1), robot.DH(1, 2), robot.DH(1, 3), q(1));
for i = 2:n
    T{i} = T{i-1} * DH_transform(robot.DH(i, 1), robot.DH(i, 2), robot.DH(i, 3), q(i));
end

% Calculate Jacobians
for i = 1:n
    for j = 1:n
        if j <= i
            if j == 1
                z_prev = [0; 0; 1];
                o_prev = [0; 0; 0];
            else
                z_prev = T{j-1}(1:3, 3);
                o_prev = T{j-1}(1:3, 4);
            end
            
            o_curr = T{i}(1:3, 4);
            
            % Linear velocity Jacobian
            Jv{i, j} = cross(z_prev, (o_curr - o_prev));
            
            % Angular velocity Jacobian
            Jw{i, j} = z_prev;
        else
            % Zero contribution for upper triangular elements
            Jv{i, j} = zeros(3, 1);
            Jw{i, j} = zeros(3, 1);
        end
    end
end

% Compute the inertia matrix
for i = 1:n
    for j = 1:n
        % Contribution from each link
        for k = max(i, j):n
            % Linear velocity component
            Jv_ij = Jv{k, i}' * Jv{k, j} * robot.m(k);
            
            % Angular velocity component
            Jw_ij = Jw{k, i}' * robot.I{k} * Jw{k, j};
            
            M(i, j) = M(i, j) + trace(Jv_ij + Jw_ij);
        end
        
        % Ensure symmetry
        M(j, i) = M(i, j);
    end
end

% Compute Coriolis matrix using Christoffel symbols
Gamma = zeros(n, n, n);

for i = 1:n
    for j = 1:n
        for k = 1:n
            dMij_dqk = 0;
            dMik_dqj = 0;
            dMjk_dqi = 0;
            
            % Numerical approximation of partial derivatives
            delta = 1e-6;
            q_plus = q;
            q_plus(k) = q(k) + delta;
            [M_plus, ~, ~, ~] = manipulator_matrices(q_plus, q_dot, robot);
            dMij_dqk = (M_plus(i, j) - M(i, j)) / delta;
            
            q_plus = q;
            q_plus(j) = q(j) + delta;
            [M_plus, ~, ~, ~] = manipulator_matrices(q_plus, q_dot, robot);
            dMik_dqj = (M_plus(i, k) - M(i, k)) / delta;
            
            q_plus = q;
            q_plus(i) = q(i) + delta;
            [M_plus, ~, ~, ~] = manipulator_matrices(q_plus, q_dot, robot);
            dMjk_dqi = (M_plus(j, k) - M(j, k)) / delta;
            
            Gamma(i, j, k) = 0.5 * (dMij_dqk + dMik_dqj - dMjk_dqi);
        end
    end
end

% Compute Coriolis matrix
for i = 1:n
    for j = 1:n
        for k = 1:n
            C(i, j) = C(i, j) + Gamma(i, j, k) * q_dot(k);
        end
    end
end

% Compute gravity vector
for i = 1:n
    for k = i:n
        % Center of mass of link k in world frame
        p_com = T{k}(1:3, 1:3) * robot.r_com(:, k) + T{k}(1:3, 4);
        
        % Height of center of mass
        h_k = p_com(3);  % z-component is height
        
        % Partial derivative of height with respect to joint angle q_i
        dh_dqi = dot(Jv{k, i}, [0; 0; 1]);
        
        % Contribution to gravity vector
        G(i) = G(i) + robot.m(k) * g * dh_dqi;
    end
end

% Compute friction vector (Coulomb + viscous)
for i = 1:n
    F(i) = robot.F_c(i) * sign(q_dot(i)) + robot.F_v(i) * q_dot(i);
end

end
