function q = to_q(X, m, q_prev)
%TO_Q Lift from (S1)^6 configuration manifold to joint coordinates q in R^6.
%
% Formal Mathematical Contract:
%   1. Bounded joints (2, 3, 5):
%      The physical arc is bounded: q_i in [qmin_i, qmax_i] with half-span h_i < pi.
%      The centered branch-cut-safe lift:
%        delta_i = atan2(s_i*cos(m_i) - c_i*sin(m_i), c_i*cos(m_i) + s_i*sin(m_i))
%        q_i = m_i + delta_i
%      places the branch cut strictly in the forbidden mechanical sector S^1 \ A_i.
%
%   2. Continuous joints (1, 4, 6):
%      These joints have no mechanical stops (unbounded full-circle S^1).
%      - If q_prev is provided (e.g. from continuous trajectory tracking, controller
%        history, or unwrapped telemetries):
%          delta_unwrapped = atan2(s_i*cos(q_prev_i) - c_i*sin(q_prev_i), ...
%                                  c_i*cos(q_prev_i) + s_i*sin(q_prev_i))
%          q_i = q_prev_i + delta_unwrapped
%        This recovers the continuous accumulated trajectory q(t) in R without
%        artificial 2*pi branch cuts or loss of winding count.
%      - If q_prev is omitted or empty:
%        q_i = atan2(s_i, c_i) in (-pi, pi] (principal branch).
%        Notice that X on (S^1)^6 alone cannot reconstruct the integer winding
%        number k in Z such that q_i = atan2(s_i, c_i) + 2*pi*k; callers requiring
%        specific winding preservation must supply q_prev or a winding count.
%
% Inputs:
%   X: 2-by-6 configuration matrix or 12-element vector on (S1)^6
%   m: (optional) model struct from kinova_s1.model()
%   q_prev: (optional) previous continuous joint angles (6x1) for phase unwrapping
% Output:
%   q: 6-by-1 physical joint vector in radians

if nargin < 2 || isempty(m) || ~isfield(m, 'qmid')
    m = kinova_s1.model();
end
if nargin < 3
    q_prev = [];
end

X = kinova_s1.config_matrix(X);

c = X(1, :)';
s = X(2, :)';
cm = m.umid(1, :)';
sm = m.umid(2, :)';

q = zeros(6, 1);

for i = 1:6
    if m.is_bounded(i)
        % Centered arc lift
        delta = atan2(s(i) * cm(i) - c(i) * sm(i), c(i) * cm(i) + s(i) * sm(i));
        q(i) = m.qmid(i) + delta;
    else
        % Continuous joint
        if ~isempty(q_prev) && isfinite(q_prev(i))
            % Unwrapped relative to previous continuous state
            cp = cos(q_prev(i));
            sp = sin(q_prev(i));
            d_rel = atan2(s(i) * cp - c(i) * sp, c(i) * cp + s(i) * sp);
            q(i) = q_prev(i) + d_rel;
        else
            % Principal branch in (-pi, pi]
            q(i) = atan2(s(i), c(i));
        end
    end
end
end
