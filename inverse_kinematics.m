function q = inverse_kinematics(target_pos, robot)
% INVERSE_KINEMATICS Computes inverse kinematics for 5DOF manipulator
%
% Inputs:
%   target_pos - Target position [x; y; z] in mm
%   robot      - Robot structure
%
% Output:
%   q          - Joint angles

% Target position in meters
x = target_pos(1) / 1000;
y = target_pos(2) / 1000;
z = target_pos(3) / 1000;

% DH parameters
a = robot.DH(:, 1) / 1000;  % Convert to meters
d = robot.DH(:, 3) / 1000;  % Convert to meters

% Inverse kinematics algorithm for 5DOF manipulator
% This is a simplified analytical inverse kinematics solution
% A complete implementation would need to handle all singularities and joint limits

% Compute joint 1 (base rotation)
q1 = atan2(y, x);

% Distance from base to wrist
r = sqrt(x^2 + y^2) - a(5);
s = z - d(1) - d(5);

% Distance from shoulder to wrist
D = sqrt(r^2 + s^2);

% Check if target is reachable
if D > (a(2) + a(3) + a(4))
    warning('Target position is outside the manipulator workspace.');
    D = a(2) + a(3) + a(4);
end

% Compute joint 3 (elbow angle)
cos_q3 = (D^2 - a(2)^2 - a(3)^2 - a(4)^2) / (2 * a(2) * (a(3) + a(4)));
cos_q3 = min(1, max(-1, cos_q3));  % Ensure within valid range
q3 = acos(cos_q3);

% Compute joint 2 (shoulder angle)
beta = atan2(s, r);
alpha = atan2((a(3) + a(4)) * sin(q3), a(2) + (a(3) + a(4)) * cos(q3));
q2 = beta - alpha;

% Compute joint 4 (wrist pitch)
q4 = -q2 - q3;  % Keep the end-effector level (simplified approach)

% Compute joint 5 (wrist roll) - assuming fixed orientation
q5 = 0;  % Could be adapted based on desired end-effector orientation

% Joint angles vector
q = [q1; q2; q3; q4; q5];

% Apply joint limits
q = min(robot.q_max', max(robot.q_min', q));

end
