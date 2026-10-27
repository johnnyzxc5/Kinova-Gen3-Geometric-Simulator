function [in_limits, margins, max_norm_err, report] = check_limits(X, m, tol, tol_norm)
%CHECK_LIMITS Check joint limit arc inequalities and unit norms on (S1)^6.
%
% Mathematically rigorous admissibility condition for joint i on (S1)^6:
%   1. Manifold membership (all joints): |x_i^T * x_i - 1| <= tol_norm
%   2. Bounded revolute joints (2, 3, 5):
%        u_i^T * x_hat_i >= cos(h_i) - tol  AND  mu_i >= -tol
%      where x_hat_i = x_i / ||x_i|| is the radially projected unit direction,
%      u_i = [cos(m_i); sin(m_i)] is the arc center, and h_i = (qmax_i - qmin_i)/2.
%   3. Continuous joints (1, 4, 6):
%      Unbounded full-circle S^1 manifold without mechanical hard stops:
%        in_arc_i = true, margin_i = +Inf.
%
% Inputs:
%   X: 2-by-6 configuration matrix on (S1)^6 (or 12-element vector)
%   m: (optional) model struct from kinova_s1.model()
%   tol: (optional, default 1e-10) numerical tolerance for arc inequality [rad]
%   tol_norm: (optional, default 1e-10) tolerance for unit-norm manifold condition
% Outputs:
%   in_limits: 6-by-1 logical array (true if joint satisfies both manifold & arc)
%   margins: 6-by-1 vector of signed distance to nearest limit in radians (>=0 is safe)
%   max_norm_err: maximum column norm deviation |x_i^T * x_i - 1|
%   report: struct with detailed diagnostics (on_manifold, in_arc, etc.)

if nargin < 2 || isempty(m) || ~isfield(m, 'qmid')
    m = kinova_s1.model();
end
if nargin < 3 || isempty(tol)
    tol = 1e-10;
end
if nargin < 4 || isempty(tol_norm)
    tol_norm = 1e-10;
end

X = kinova_s1.config_matrix(X);

c = X(1, :)';
s = X(2, :)';
cm = m.umid(1, :)';
sm = m.umid(2, :)';

% 1. Manifold condition on (S1)^6: |c_i^2 + s_i^2 - 1| <= tol_norm
norm_sq = c.^2 + s.^2;
r = sqrt(norm_sq);
unit_norm_err = abs(norm_sq - 1);
max_norm_err = max(unit_norm_err);
on_manifold = (unit_norm_err <= tol_norm);

% 2. Normalized direction for scale-invariant angular evaluation
valid_norm = (r > eps);
c_hat = zeros(6, 1);
s_hat = zeros(6, 1);
c_hat(valid_norm) = c(valid_norm) ./ r(valid_norm);
s_hat(valid_norm) = s(valid_norm) ./ r(valid_norm);

% Dot product and wedge product with arc center unit vector u_i
u_dot_x_hat = c_hat .* cm + s_hat .* sm;
u_wedge_x_hat = s_hat .* cm - c_hat .* sm;
u_dot_x_hat(~valid_norm) = -Inf;

% Exact signed offset from arc center in radians: delta_i in (-pi, pi]
delta = atan2(u_wedge_x_hat, u_dot_x_hat);
delta(~valid_norm) = NaN;

% 3. Margins and arc admissibility
margins = inf(6, 1);
in_arc = valid_norm;

for i = 1:6
    if m.is_bounded(i)
        margins(i) = m.qhalf(i) - abs(delta(i));
        if ~valid_norm(i)
            margins(i) = -Inf;
            in_arc(i) = false;
        else
            in_arc(i) = (u_dot_x_hat(i) >= (m.cos_half(i) - tol)) && (margins(i) >= -tol);
        end
    else
        % Continuous joints: unbounded
        margins(i) = Inf;
        in_arc(i) = valid_norm(i);
    end
end

% 4. Strict combined admissibility: on manifold AND within arc
in_limits = on_manifold & in_arc;

if nargout > 3
    report = struct();
    report.in_limits = in_limits;
    report.all_in_limits = all(in_limits);
    report.on_manifold = on_manifold;
    report.all_on_manifold = all(on_manifold);
    report.in_arc = in_arc;
    report.all_in_arc = all(in_arc);
    report.margins_rad = margins;
    report.min_margin_rad = min(margins(m.is_bounded));
    report.max_limit_violation_rad = max([0; -margins(m.is_bounded)]);
    report.unit_norm_error = unit_norm_err;
    report.max_unit_norm_error = max_norm_err;
    report.u_dot_x = c .* cm + s .* sm;
    report.u_dot_x_normalized = u_dot_x_hat;
    report.cos_half = m.cos_half;
    report.delta_from_center_rad = delta;
    report.is_continuous = m.is_continuous;
end
end
