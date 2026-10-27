function T = fk(X, m)
%FK Space product of exponentials on (S1)^6: exp([S1],x1)...exp([S6],x6)*M_home.
%
% Evaluates the space PoE algebraically on (S1)^6 via Theorem 3.1 (screw_exp),
% completely independent of the relative body chain kinova_s1.frames(X, m).
%
% Inputs:
%   X: 2-by-6 configuration matrix on (S1)^6
%   m: (optional) model struct from kinova_s1.model()
% Output:
%   T: 4-by-4 homogeneous transformation of bare flange in base frame

if nargin < 2 || isempty(m)
    m = kinova_s1.model();
end

X = kinova_s1.config_matrix(X);
T = m.home;
has_mid = isfield(m, 'umid') && isfield(m, 'qmid');

for i = m.n:-1:1
    if has_mid
        Ti = kinova_s1.screw_exp(m.Slist(:, i), X(:, i), m.umid(:, i), m.qmid(i));
    else
        Ti = kinova_s1.screw_exp(m.Slist(:, i), X(:, i));
    end
    T = Ti * T;
end
end
