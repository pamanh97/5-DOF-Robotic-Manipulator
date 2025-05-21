function alpha = update_weights(e, beta)
% UPDATE_WEIGHTS Updates the adaptive weighting coefficients
%
% Inputs:
%   e     - Position error
%   beta  - Weighting parameters [beta_1, beta_2, beta_3, beta_4]
%
% Output:
%   alpha - Updated weighting coefficients

% Compute error norm
e_norm = norm(e)^2;

% Compute exponential weights
w = zeros(4, 1);
for i = 1:4
    w(i) = exp(-beta(i) * e_norm);
end

% Normalize weights
alpha = w / sum(w);

end
