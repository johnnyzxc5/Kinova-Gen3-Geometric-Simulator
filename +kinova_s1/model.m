function m = model(options)
%MODEL Kinova Gen3 six-link model augmented for global (S1)^6 Lie-group dynamics.
%
% Wraps kinova.model() and adds geometric arc limits on S^1, midpoints,
% half-spans, and precomputed local screw axis verification.
%
% Joint classification:
%   - Continuous joints (1, 4, 6): unbounded configuration on S^1, no mechanical stops.
%   - Revolute joints (2, 3, 5): bounded physical arcs on S^1 with hard mechanical limits.

if nargin < 1
    options = struct();
end

m = kinova.model(options);

% Identify continuous vs bounded revolute joints
m.is_continuous = isinf(m.qmin) & isinf(m.qmax); % [true; false; false; true; false; true]
m.is_bounded = ~m.is_continuous;

% Precompute geometric limit arc parameters on S^1
m.qmid = zeros(m.n, 1);
m.qhalf = inf(m.n, 1);
m.umid = repmat([1; 0], 1, m.n); % Default center along x-axis (0 rad)
m.cos_half = -ones(m.n, 1);

for i = 1:m.n
    if m.is_bounded(i)
        m.qmid(i) = 0.5 * (m.qmin(i) + m.qmax(i));
        m.qhalf(i) = 0.5 * (m.qmax(i) - m.qmin(i));
        m.umid(:, i) = [cos(m.qmid(i)); sin(m.qmid(i))];
        m.cos_half(i) = cos(m.qhalf(i));
    end
end

% Verify local screw axes A_i = Ad_{inv(M_1...M_i)} * S_i
m.is_pure_z = false(1, m.n);
Mi = eye(4);
m.Alist = zeros(6, m.n);
for i = 1:m.n
    Mi = Mi * m.Mlist(:, :, i);
    Ai = Adjoint(TransInv(Mi)) * m.Slist(:, i);
    m.Alist(:, i) = Ai;
    m.is_pure_z(i) = norm(Ai - [0; 0; 1; 0; 0; 0], inf) < 1e-10;
end

m.representation = 'S1';
m.scope = [m.scope, '; Global Lie-group formulation on (S^1)^6'];
end
