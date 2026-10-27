function [M, g, V] = lagrange_reference(X, m)
%LAGRANGE_REFERENCE Independent COM Jacobians on (S1)^6; no spatial RNEA kernels.
%
% Evaluates M(X), g(X), and potential V(X) purely algebraically from the
% relative link chain on (S1)^6. Intended for independent mathematical
% validation of the spatial RNEA kernels on (S1)^6.
%
% Inputs:
%   X: 2-by-6 configuration matrix on (S1)^6
%   m: (optional) model struct from kinova_s1.model()
% Outputs:
%   M: 6-by-6 symmetric positive-definite mass matrix
%   g: 6-by-1 gravitational torque vector [N*m]
%   V: scalar gravitational potential energy [J]

if nargin < 2 || isempty(m)
    m = kinova_s1.model();
end

X = kinova_s1.config_matrix(X);
chain = kinova_s1.frames(X, m);

M = m.Japp;
g = zeros(m.n, 1);
V = 0;

for i = 1:m.n
    R = chain(1:3, 1:3, i);
    r = R * m.com(:, i) + chain(1:3, 4, i);
    Jv = zeros(3, m.n);
    Jw = zeros(3, m.n);
    for j = 1:i
        w = chain(1:3, 1:3, j) * m.axis_local(:, j);
        origin = chain(1:3, 4, j);
        Jv(:, j) = cross(w, r - origin);
        Jw(:, j) = w;
    end
    M = M + m.mass(i) * (Jv' * Jv) + Jw' * R * m.Icom(:, :, i) * R' * Jw;
    g = g - m.mass(i) * (Jv' * m.gravity);
    V = V - m.mass(i) * (m.gravity' * r);
end
end
