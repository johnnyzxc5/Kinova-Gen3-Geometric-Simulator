function J = jacobian(X, m, frame)
%JACOBIAN Space or flange-body twist Jacobian on (S1)^6, rows [angular; linear].
%
% Evaluates the twist Jacobian purely algebraically on (S1)^6 via Theorem 3.1
% without lifting to joint angles q or calling cos/sin/atan2.
%
% Inputs:
%   X: 2-by-6 configuration matrix on (S1)^6
%   m: (optional) model struct from kinova_s1.model()
%   frame: (optional, default 'space') 'space' or 'body'
% Output:
%   J: 6-by-6 twist Jacobian matrix

if nargin < 3
    frame = 'space';
end
if nargin < 2 || isempty(m)
    m = kinova_s1.model();
end

X = kinova_s1.config_matrix(X);
J = m.Slist;
T = eye(4);
has_mid = isfield(m, 'umid') && isfield(m, 'qmid');

for i = 2:m.n
    if has_mid
        Ti = kinova_s1.screw_exp(m.Slist(:, i - 1), X(:, i - 1), m.umid(:, i - 1), m.qmid(i - 1));
    else
        Ti = kinova_s1.screw_exp(m.Slist(:, i - 1), X(:, i - 1));
    end
    T = T * Ti;
    J(:, i) = Adjoint(T) * m.Slist(:, i);
end

switch char(frame)
    case 'space'
    case 'body'
        if has_mid
            Tn = kinova_s1.screw_exp(m.Slist(:, m.n), X(:, m.n), m.umid(:, m.n), m.qmid(m.n));
        else
            Tn = kinova_s1.screw_exp(m.Slist(:, m.n), X(:, m.n));
        end
        T_flange = T * Tn * m.home;
        J = Adjoint(TransInv(T_flange)) * J;
    otherwise
        error('kinova_s1:frame', 'Frame must be space or body');
end
end
