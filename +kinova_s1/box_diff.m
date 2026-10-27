function dq_box = box_diff(X2, X1, m, winding)
%BOX_DIFF Admissible-arc displacement between configurations on (S1)^6.
%
% Computes configuration displacement between X1 and X2:
%
%   1. Bounded joints (2, 3, 5):
%      Computes the unique continuous displacement along the physical admissible
%      arc A_i = [qmin_i, qmax_i] without wrapping across the mechanical hard stop:
%        dq_box_i = delta_i(X2) - delta_i(X1) in [-2*h_i, 2*h_i]
%      where delta_i is the centered arc angle relative to midpoint m_i.
%      This guarantees kinematic homotopy consistency: the path remains strictly
%      within the admissible arc.
%
%   2. Continuous joints (1, 4, 6):
%      These joints have no mechanical hard stops.
%      - If winding is provided (6x1 integer vector):
%        dq_box_i = geodesic_diff_i(X2, X1) + 2*pi*winding_i
%      - If winding is omitted:
%        dq_box_i = geodesic_diff_i(X2, X1) in (-pi, pi] (shortest geodesic).
%
% Inputs:
%   X2: 2-by-6 target configuration on (S1)^6
%   X1: 2-by-6 source configuration on (S1)^6
%   m: (optional) model struct from kinova_s1.model()
%   winding: (optional) 6-by-1 integer vector of extra complete turns for continuous joints
% Output:
%   dq_box: 6-by-1 physical joint displacement vector [rad]

if nargin < 3 || isempty(m) || ~isfield(m, 'qmid')
    m = kinova_s1.model();
end
if nargin < 4
    winding = zeros(6, 1);
end

X1 = kinova_s1.config_matrix(X1);
X2 = kinova_s1.config_matrix(X2);

c1 = X1(1, :); s1 = X1(2, :);
c2 = X2(1, :); s2 = X2(2, :);
cm = m.umid(1, :); sm = m.umid(2, :);

dq_box = zeros(6, 1);

for i = 1:6
    if m.is_bounded(i)
        delta1 = atan2(s1(i) * cm(i) - c1(i) * sm(i), c1(i) * cm(i) + s1(i) * sm(i));
        delta2 = atan2(s2(i) * cm(i) - c2(i) * sm(i), c2(i) * cm(i) + s2(i) * sm(i));
        dq_box(i) = delta2 - delta1;
    else
        % Continuous joint: shortest geodesic on S^1 + optional winding turns
        d_geo = atan2(s2(i) * c1(i) - c2(i) * s1(i), c2(i) * c1(i) + s2(i) * s1(i));
        dq_box(i) = d_geo + 2 * pi * winding(i);
    end
end
end
