function dtheta = geodesic_diff(X2, X1)
%GEODESIC_DIFF Minimal Riemannian geodesic difference on configuration manifold (S1)^6.
%
% Computes the shortest circular error vector dtheta in (-pi, pi]^6 such that:
%   R_z(dtheta_i) * x1_i = x2_i
%
% Inputs:
%   X2: 2-by-6 target configuration matrix on (S1)^6
%   X1: 2-by-6 source configuration matrix on (S1)^6
% Output:
%   dtheta: 6-by-1 shortest angular displacement vector [rad]

X1 = kinova_s1.config_matrix(X1);
X2 = kinova_s1.config_matrix(X2);

c1 = X1(1, :); s1 = X1(2, :);
c2 = X2(1, :); s2 = X2(2, :);

% Relative rotation: R_z(dtheta) = R_z(theta2) * R_z(theta1)^T
% cos(dtheta) = c2*c1 + s2*s1
% sin(dtheta) = s2*c1 - c2*s1
dtheta = atan2(s2 .* c1 - c2 .* s1, c2 .* c1 + s2 .* s1)';
end
