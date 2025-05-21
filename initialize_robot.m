function robot = initialize_robot(DH, m, I, F_c, F_v, q_min, q_max)
% INITIALIZE_ROBOT Creates the robot structure with all necessary parameters
%
% Inputs:
%   DH      - DH parameters [a, alpha, d, theta]
%   m       - Link masses vector
%   I       - Cell array of inertia tensors
%   F_c     - Coulomb friction coefficients
%   F_v     - Viscous friction coefficients
%   q_min   - Lower joint limits
%   q_max   - Upper joint limits
%
% Output:
%   robot   - Robot structure

robot.DH = DH;          % DH parameters
robot.m = m;            % Link masses
robot.I = I;            % Inertia tensors
robot.g = 9.81;         % Gravity acceleration [m/s^2]
robot.n = size(DH, 1);  % Number of joints
robot.F_c = F_c;        % Coulomb friction
robot.F_v = F_v;        % Viscous friction
robot.q_min = q_min;    % Lower joint limits
robot.q_max = q_max;    % Upper joint limits

% Link centers of mass (simplified assumption)
% Assuming COM at the middle of each link
robot.r_com = zeros(3, robot.n);
for i = 1:robot.n
    if i < robot.n
        robot.r_com(:, i) = [DH(i,1)/2; 0; DH(i,3)/2];
    else
        robot.r_com(:, i) = [0; 0; DH(i,3)/2];
    end
end

end
