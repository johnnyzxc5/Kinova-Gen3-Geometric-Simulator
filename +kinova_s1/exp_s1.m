function X_next = exp_s1(dtheta, X, normalize_cols)
%EXP_S1 Exact group action of SO(2)^6 on configuration manifold (S1)^6.
%
% Rotates each circular configuration x_i = [c_i; s_i] by angle dtheta_i:
%   x_i' = R_z(dtheta_i) * x_i = [cos(dtheta_i)*c_i - sin(dtheta_i)*s_i;
%                                 sin(dtheta_i)*c_i + cos(dtheta_i)*s_i]
%
% Inputs:
%   dtheta: 6-by-1 or 1-by-6 angle increment vector [rad]
%   X: 2-by-6 configuration matrix on (S1)^6
%   normalize_cols: (optional, default true) renormalize columns to unit norm
% Output:
%   X_next: 2-by-6 updated configuration matrix on (S1)^6

if nargin < 3
    normalize_cols = true;
end

X = kinova_s1.config_matrix(X);
dtheta = kinova.joint_vector(dtheta);

cd = cos(dtheta)';
sd = sin(dtheta)';

c_new = cd .* X(1, :) - sd .* X(2, :);
s_new = sd .* X(1, :) + cd .* X(2, :);

X_next = [c_new; s_new];

if normalize_cols
    norms = sqrt(sum(X_next.^2, 1));
    X_next = X_next ./ norms;
end
end
