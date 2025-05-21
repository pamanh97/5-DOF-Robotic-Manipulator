function [F, H] = build_prediction_matrices(A, B, Np, Nc)
% BUILD_PREDICTION_MATRICES Creates prediction matrices for MPC
%
% Inputs:
%   A       - State matrix
%   B       - Input matrix
%   Np      - Prediction horizon
%   Nc      - Control horizon
%
% Outputs:
%   F       - Free response matrix
%   H       - Forced response matrix

nx = size(A, 1);
nu = size(B, 2);

% Initialize matrices
F = zeros(nx * Np, nx);
H = zeros(nx * Np, nu * Nc);

% First block row
F(1:nx, :) = A;
H(1:nx, 1:nu) = B;

% Build remaining block rows
for i = 2:Np
    % Update free response matrix
    F(nx*(i-1)+1:nx*i, :) = A * F(nx*(i-2)+1:nx*(i-1), :);
    
    % Update forced response matrix for control horizon
    for j = 1:min(i-1, Nc)
        H(nx*(i-1)+1:nx*i, nu*(j-1)+1:nu*j) = A * H(nx*(i-2)+1:nx*(i-1), nu*(j-1)+1:nu*j);
    end
    
    % Add new control action if within control horizon
    if i <= Nc
        H(nx*(i-1)+1:nx*i, nu*(i-1)+1:nu*i) = B;
    end
end

end
