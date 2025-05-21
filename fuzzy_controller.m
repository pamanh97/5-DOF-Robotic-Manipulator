function tau = fuzzy_controller(e, e_dot, sigma, sigma_d)
% FUZZY_CONTROLLER Computes control input using fuzzy logic
%
% Inputs:
%   e        - Position error
%   e_dot    - Velocity error
%   sigma    - Error scaling factors
%   sigma_d  - Error rate scaling factors
%
% Output:
%   tau      - Control torque

% Number of joints
n = length(e);

% Initialize control torque
tau = zeros(n, 1);

% Fuzzy rule base (as in Table 2 from the paper)
% NL NS ZE PS PL
% NL PL PL PL PS ZE
% NS PL PS PS ZE NS
% ZE PL PS ZE NS NL
% PS PS ZE NS NS NL
% PL ZE NS NL NL NL

% Define output membership functions centers
output_centers = {
    [-2.0, -1.0, 0.0, 1.0, 2.0],  % Joint 1
    [-2.0, -1.0, 0.0, 1.0, 2.0],  % Joint 2
    [-1.5, -0.75, 0.0, 0.75, 1.5], % Joint 3
    [-1.0, -0.5, 0.0, 0.5, 1.0],  % Joint 4
    [-0.5, -0.25, 0.0, 0.25, 0.5]  % Joint 5
};

% Process each joint
for i = 1:n
    % Normalize inputs
    e_norm = e(i) / sigma(i);
    e_dot_norm = e_dot(i) / sigma_d(i);
    
    % Calculate membership degrees for error
    mu_e_NL = membership_NL(e_norm);
    mu_e_NS = membership_NS(e_norm);
    mu_e_ZE = membership_ZE(e_norm);
    mu_e_PS = membership_PS(e_norm);
    mu_e_PL = membership_PL(e_norm);
    
    % Calculate membership degrees for error rate
    mu_ed_NL = membership_NL(e_dot_norm);
    mu_ed_NS = membership_NS(e_dot_norm);
    mu_ed_ZE = membership_ZE(e_dot_norm);
    mu_ed_PS = membership_PS(e_dot_norm);
    mu_ed_PL = membership_PL(e_dot_norm);
    
    % Rule evaluation using min operation for AND
    rule_strengths = zeros(5, 5);
    for row = 1:5
        for col = 1:5
            switch row
                case 1, mu_e = mu_e_NL;
                case 2, mu_e = mu_e_NS;
                case 3, mu_e = mu_e_ZE;
                case 4, mu_e = mu_e_PS;
                case 5, mu_e = mu_e_PL;
            end
            
            switch col
                case 1, mu_ed = mu_ed_NL;
                case 2, mu_ed = mu_ed_NS;
                case 3, mu_ed = mu_ed_ZE;
                case 4, mu_ed = mu_ed_PS;
                case 5, mu_ed = mu_ed_PL;
            end
            
            rule_strengths(row, col) = min(mu_e, mu_ed);
        end
    end
    
    % Rule base mapping to output (as per Table 2)
    rule_outputs = [
        5, 5, 5, 4, 3;  % NL
        5, 4, 4, 3, 2;  % NS
        5, 4, 3, 2, 1;  % ZE
        4, 3, 2, 2, 1;  % PS
        3, 2, 1, 1, 1   % PL
    ];
    
    % Defuzzification using center of gravity method
    numerator = 0;
    denominator = 0;
    
    for row = 1:5
        for col = 1:5
            output_idx = rule_outputs(row, col);
            numerator = numerator + rule_strengths(row, col) * output_centers{i}(output_idx);
            denominator = denominator + rule_strengths(row, col);
        end
    end
    
    % Avoid division by zero
    if denominator > 0
        tau(i) = numerator / denominator;
    else
        tau(i) = 0;
    end
end

end

% Membership functions as defined in the paper
function mu = membership_NL(x)
    if x <= -2
        mu = 1;
    elseif x > -2 && x < -1
        mu = (-x - 1) / 1;
    else
        mu = 0;
    end
end

function mu = membership_NS(x)
    if x > -2 && x < -1
        mu = (x + 2) / 1;
    elseif x >= -1 && x < 0
        mu = (-x) / 1;
    else
        mu = 0;
    end
end

function mu = membership_ZE(x)
    if x > -1 && x < 0
        mu = (x + 1) / 1;
    elseif x >= 0 && x < 1
        mu = (1 - x) / 1;
    else
        mu = 0;
    end
end

function mu = membership_PS(x)
    if x > 0 && x < 1
        mu = x / 1;
    elseif x >= 1 && x < 2
        mu = (2 - x) / 1;
    else
        mu = 0;
    end
end

function mu = membership_PL(x)
    if x >= 2
        mu = 1;
    elseif x > 1 && x < 2
        mu = (x - 1) / 1;
    else
        mu = 0;
    end
end
