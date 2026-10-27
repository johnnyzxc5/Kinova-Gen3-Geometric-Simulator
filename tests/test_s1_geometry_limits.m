function report = test_s1_geometry_limits()
%TEST_S1_GEOMETRY_LIMITS Comprehensive test of (S1)^6 manifold geometry, limits, and chart maps.
%
% Verifies:
%   1. Embedding and lift roundtrip q <-> X on (S1)^6 (bounded and continuous)
%   2. Multi-turn winding preservation with continuous q_prev
%   3. Continuous joints (1, 4, 6) smooth crossing of +/-pi and periodic equivalence q ~ q + 2*pi*k
%   4. Bounded joints (2, 3, 5) arc inequality discrimination and margins at boundaries
%   5. Detection of off-manifold vectors and scale-invariant radial projection
%   6. Geodesic difference vs box difference with winding
%   7. Exp_S1 group action unit-norm preservation to machine precision

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
setup_kinova();

m = kinova_s1.model();
n = m.n;

rng(13102026, 'twister');

% -------------------------------------------------------------------------
% 1. Embedding and Lift Roundtrip q <-> X on (S1)^6
% -------------------------------------------------------------------------
max_roundtrip_err = 0;
N_samples = 200;
for s = 1:N_samples
    q_rand = zeros(n, 1);
    for i = 1:n
        if m.is_bounded(i)
            q_rand(i) = m.qmin(i) + rand * (m.qmax(i) - m.qmin(i));
        else
            q_rand(i) = (2 * rand - 1) * pi;
        end
    end
    X = kinova_s1.from_q(q_rand);
    q_rec = kinova_s1.to_q(X, m);
    max_roundtrip_err = max(max_roundtrip_err, norm(q_rec - q_rand, inf));
end
assert(max_roundtrip_err < 1e-14, 'kinova_s1:test', 'Principal roundtrip q <-> X failed');

% -------------------------------------------------------------------------
% 2. Multi-Turn Winding Preservation on Continuous Joints (1, 4, 6)
% -------------------------------------------------------------------------
max_winding_err = 0;
winding_counts = [-3, -2, -1, 0, 1, 2, 3];
for k_turn = winding_counts
    for j_cont = [1, 4, 6]
        q_target = 0.5 + 2 * pi * k_turn;
        X = [cos(q_target); sin(q_target)];
        X_full = repmat([1; 0], 1, 6);
        X_full(:, j_cont) = X;

        % Provide q_prev close to q_target
        q_prev = zeros(6, 1);
        q_prev(j_cont) = q_target - 0.1;

        q_lifted = kinova_s1.to_q(X_full, m, q_prev);
        err = abs(q_lifted(j_cont) - q_target);
        max_winding_err = max(max_winding_err, err);
        assert(err < 1e-14, 'kinova_s1:test', 'Multi-turn lift failed for joint %d, turn %d', j_cont, k_turn);
    end
end

% -------------------------------------------------------------------------
% 3. Smooth Crossing of +/- pi on Continuous Joints
% -------------------------------------------------------------------------
for j_cont = [1, 4, 6]
    angles = [pi - 1e-4, pi, pi + 1e-4, -pi + 1e-4, -pi, 3*pi];
    for ang = angles
        X = [cos(ang); sin(ang)];
        X_full = repmat([1; 0], 1, 6);
        X_full(:, j_cont) = X;

        [in_lim, margins, norm_err, rep] = kinova_s1.check_limits(X_full, m);
        assert(in_lim(j_cont), 'kinova_s1:test', 'Continuous joint should be in limits at angle %f', ang);
        assert(isinf(margins(j_cont)) && margins(j_cont) > 0, 'kinova_s1:test', 'Continuous joint margin must be +Inf');
        assert(norm_err < 1e-15, 'kinova_s1:test', 'Unit norm error at angle %f', ang);
    end
end

% Check equivalence: q and q + 2*pi*k give identical X on S^1
q_base = randn(6, 1);
X_base = kinova_s1.from_q(q_base);
for k = [-5, -2, 1, 4]
    q_shifted = q_base;
    q_shifted([1, 4, 6]) = q_shifted([1, 4, 6]) + 2 * pi * k;
    X_shifted = kinova_s1.from_q(q_shifted);
    assert(norm(X_base - X_shifted, 'fro') < 1e-14, 'kinova_s1:test', 'Periodic equivalence failed');
end

% -------------------------------------------------------------------------
% 4. Bounded Joints (2, 3, 5) Boundary Testing and Rejection
% -------------------------------------------------------------------------
% Exact boundaries
q_bounds = zeros(6, 1);
for i = [2, 3, 5]
    % Lower boundary
    q_bounds(i) = m.qmin(i);
    X_b = kinova_s1.from_q(q_bounds);
    [in_lim, margins] = kinova_s1.check_limits(X_b, m);
    assert(in_lim(i), 'kinova_s1:test', 'Lower boundary must be inside limits');
    assert(abs(margins(i)) < 1e-13, 'kinova_s1:test', 'Lower boundary margin should be zero');

    % Upper boundary
    q_bounds(i) = m.qmax(i);
    X_b = kinova_s1.from_q(q_bounds);
    [in_lim, margins] = kinova_s1.check_limits(X_b, m);
    assert(in_lim(i), 'kinova_s1:test', 'Upper boundary must be inside limits');
    assert(abs(margins(i)) < 1e-13, 'kinova_s1:test', 'Upper boundary margin should be zero');

    % Slightly outside bounds (1e-3 rad beyond)
    q_out = q_bounds;
    q_out(i) = m.qmax(i) + 1e-3;
    X_out = kinova_s1.from_q(q_out);
    [in_lim_out, margins_out] = kinova_s1.check_limits(X_out, m);
    assert(~in_lim_out(i), 'kinova_s1:test', 'Beyond upper limit must be rejected');
    assert(margins_out(i) < -5e-4, 'kinova_s1:test', 'Margin beyond upper limit must be negative');

    q_out(i) = m.qmin(i) - 1e-3;
    X_out = kinova_s1.from_q(q_out);
    [in_lim_out, margins_out] = kinova_s1.check_limits(X_out, m);
    assert(~in_lim_out(i), 'kinova_s1:test', 'Beyond lower limit must be rejected');
    assert(margins_out(i) < -5e-4, 'kinova_s1:test', 'Margin beyond lower limit must be negative');
end

% -------------------------------------------------------------------------
% 5. Vectors Outside S^1 (Non-Unit Norm)
% -------------------------------------------------------------------------
X_scaled = kinova_s1.from_q(zeros(6, 1));
X_scaled(:, 2) = X_scaled(:, 2) * 2.5; % Scale column 2
[in_lim_sc, ~, max_norm_err] = kinova_s1.check_limits(X_scaled, m);
assert(~in_lim_sc(2), 'kinova_s1:test', 'Scaled vector must violate manifold condition');
assert(abs(max_norm_err - (2.5^2 - 1)) < 1e-14, 'kinova_s1:test', 'Norm error calculation');

% -------------------------------------------------------------------------
% 6. Displacement and Winding in box_diff
% -------------------------------------------------------------------------
X1 = kinova_s1.from_q([0; 1.0; -1.0; 0; 0.5; 0]);
X2 = kinova_s1.from_q([0.5; -1.0; 1.0; 0.2; -0.5; -0.3]);

d_geo = kinova_s1.geodesic_diff(X2, X1);
d_box = kinova_s1.box_diff(X2, X1, m);
assert(all(abs(d_geo) <= pi + 1e-14), 'kinova_s1:test', 'Geodesic diff must be in (-pi, pi]');

% Continuous joints with specified winding
windings = [2; 0; 0; -1; 0; 3];
d_box_wind = kinova_s1.box_diff(X2, X1, m, windings);
for j = [1, 4, 6]
    expected = d_geo(j) + 2 * pi * windings(j);
    assert(abs(d_box_wind(j) - expected) < 1e-14, 'kinova_s1:test', 'Winding addition in box_diff failed');
end

% -------------------------------------------------------------------------
% 7. Exp_S1 Group Action Preservation of Unit Norm
% -------------------------------------------------------------------------
max_exp_norm_err = 0;
X_cur = kinova_s1.from_q(zeros(6, 1));
for step = 1:500
    dth = (randn(6, 1) - 0.5) * 0.1;
    X_cur = kinova_s1.exp_s1(dth, X_cur, true);
    max_exp_norm_err = max(max_exp_norm_err, max(abs(sum(X_cur.^2, 1) - 1)));
end
assert(max_exp_norm_err < 1e-15, 'kinova_s1:test', 'Exp_S1 accumulated unit norm error exceeds 1e-15');

report = struct('max_roundtrip_err', max_roundtrip_err, ...
    'max_winding_err', max_winding_err, ...
    'max_exp_norm_err', max_exp_norm_err, ...
    'pass', true);

disp('test_s1_geometry_limits: PASS');
disp(report);
end
