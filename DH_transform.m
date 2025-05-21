function T = DH_transform(a, alpha, d, theta)
% DH_TRANSFORM Computes homogeneous transformation matrix from DH parameters
%
% Inputs:
%   a      - Link length
%   alpha  - Link twist
%   d      - Link offset
%   theta  - Joint angle
%
% Output:
%   T      - Homogeneous transformation matrix

% Convert to meters if provided in mm
a = a / 1000;
d = d / 1000;

% DH transformation matrix
T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
     sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
     0, sin(alpha), cos(alpha), d;
     0, 0, 0, 1];

end
