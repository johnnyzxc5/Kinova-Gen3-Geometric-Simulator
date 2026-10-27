function report = validate_kinova_s1()
%VALIDATE_KINOVA_S1 Independent mechanics, Lie identities, and Version A/B parity on (S1)^6.
%
% Verifies that the global (S1)^6 formulation (+kinova_s1) satisfies all intrinsic
% manifold identities (Lee et al., Ch. 4) and achieves machine-precision equivalence
% with the original joint-coordinate formulation (+kinova).

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
setup_kinova();

rng(13102026, 'twister');
m = kinova_s1.model();
m_orig = kinova.model();
n = m.n;
z = zeros(n, 1);
ep = 1e-5;

report = struct('seed', 13102026, 'states', 40, 'matlab_version', version, ...
    'mass_reference_error', 0, 'gravity_reference_error_Nm', 0, 'coriolis_reference_error_Nm', 0, ...
    'fk_chain_error', 0, 'jacobian_finite_difference_error', 0, 'mass_asymmetry', 0, ...
    'min_mass_eigenvalue', Inf, 'potential_gradient_error_Nm', 0, 'power_identity_error', 0, ...
    'version_a_fk_parity_error', 0, 'version_a_jacobian_parity_error', 0, ...
    'version_a_mass_parity_error', 0, 'version_a_inverse_dynamics_parity_error_Nm', 0, ...
    'version_a_forward_dynamics_parity_error', 0, 'version_a_energy_parity_error_J', 0, ...
    'forward_inverse_roundtrip_error', 0, 'wrench_duality_error', 0, ...
    'minimum_friction_power_W', Inf, 'adjoint_exponential_identity_error', 0);

for k = 1:report.states
    % Sample physically valid states: continuous joints in [-pi, pi], bounded joints in [qmin+0.05, qmax-0.05]
    q = zeros(n, 1);
    for i = 1:n
        if m.is_bounded(i)
            q(i) = m.qmin(i) + 0.05 + (m.qmax(i) - m.qmin(i) - 0.1) * rand;
        else
            q(i) = (2 * rand - 1) * pi;
        end
    end
    if k == 1
        % Explicit edge case with continuous joints crossing or at pi
        q(1) = 3.1415;
        q(4) = -3.1415;
        q(6) = pi;
    end
    omega = randn(n, 1);
    domega = randn(n, 1);
    F = randn(6, 1);

    X = kinova_s1.from_q(q);

    % 1. Intrinsic (S1)^6 dynamics, kinematics, and independent Lagrange oracle
    [M, b, parts] = kinova_s1.dynamics(X, omega, m);
    [ML, gL, VL] = kinova_s1.lagrange_reference(X, m);
    [~, Tchain] = kinova_s1.frames(X, m);
    T = kinova_s1.fk(X, m);

    report.fk_chain_error = max(report.fk_chain_error, norm(T - Tchain, 'fro'));
    report.mass_reference_error = max(report.mass_reference_error, norm(M - ML, 'fro') / norm(ML, 'fro'));
    report.gravity_reference_error_Nm = max(report.gravity_reference_error_Nm, norm(parts.gravity - gL));
    report.mass_asymmetry = max(report.mass_asymmetry, norm(M - M', 'fro'));
    report.min_mass_eigenvalue = min(report.min_mass_eigenvalue, min(eig((M + M') / 2)));

    Js = kinova_s1.jacobian(X, m, 'space');
    Jb = kinova_s1.jacobian(X, m, 'body');

    % 2. Tangential directional derivatives on (S1)^6 via Exp_S1 variations
    derivative = zeros(n, n, n);
    gradient = z;
    Jfd = zeros(6, n);
    for j = 1:n
        d = z;
        d(j) = ep;
        Xp = kinova_s1.exp_s1(d, X, true);
        Xm = kinova_s1.exp_s1(-d, X, true);
        [Mp, ~, Vp] = kinova_s1.lagrange_reference(Xp, m);
        [Mm, ~, Vm] = kinova_s1.lagrange_reference(Xm, m);
        derivative(:, :, j) = (Mp - Mm) / (2 * ep);
        gradient(j) = (Vp - Vm) / (2 * ep);

        Tp = kinova_s1.fk(Xp, m);
        Tm = kinova_s1.fk(Xm, m);
        twist = (Tp - Tm) / (2 * ep) * TransInv(T);
        Jfd(:, j) = [so3ToVec(twist(1:3, 1:3)); twist(1:3, 4)];
    end

    Mdot = zeros(n);
    gradK = z;
    for j = 1:n
        Mdot = Mdot + derivative(:, :, j) * omega(j);
        gradK(j) = omega' * derivative(:, :, j) * omega;
    end
    cL = Mdot * omega - 0.5 * gradK;

    report.coriolis_reference_error_Nm = max(report.coriolis_reference_error_Nm, norm(parts.coriolis - cL));
    report.potential_gradient_error_Nm = max(report.potential_gradient_error_Nm, norm(parts.gravity - gradient));
    report.power_identity_error = max(report.power_identity_error, abs(omega' * parts.coriolis - 0.5 * omega' * Mdot * omega));
    report.jacobian_finite_difference_error = max(report.jacobian_finite_difference_error, norm(Js - Jfd, 'fro'));

    % 3. Inverse/forward dynamics and wrench duality on (S1)^6
    tau_s1 = kinova_s1.inverse_dynamics(X, omega, domega, m, F);
    recovered = kinova_s1.forward_dynamics(X, omega, tau_s1, m, -F);
    report.forward_inverse_roundtrip_error = max(report.forward_inverse_roundtrip_error, norm(recovered - domega));
    report.wrench_duality_error = max(report.wrench_duality_error, abs(F' * (Jb * omega) - (Jb' * F)' * omega));
    report.minimum_friction_power_W = min(report.minimum_friction_power_W, omega' * parts.friction);

    % 4. Exact Parity against Version A (+kinova)
    T_a = kinova.fk(q, m_orig);
    Js_a = kinova.jacobian(q, m_orig, 'space');
    Jb_a = kinova.jacobian(q, m_orig, 'body');
    [M_a, b_a] = kinova.dynamics(q, omega, m_orig);
    tau_a = kinova.inverse_dynamics(q, omega, domega, m_orig, F);
    ddq_a = kinova.forward_dynamics(q, omega, tau_a, m_orig, -F);
    H_a = kinova.energy(q, omega, m_orig);
    H_b = kinova_s1.energy(X, omega, m);

    report.version_a_fk_parity_error = max(report.version_a_fk_parity_error, norm(T - T_a, 'fro'));
    report.version_a_jacobian_parity_error = max(report.version_a_jacobian_parity_error, ...
        max(norm(Js - Js_a, 'fro'), norm(Jb - Jb_a, 'fro')));
    report.version_a_mass_parity_error = max(report.version_a_mass_parity_error, norm(M - M_a, 'fro'));
    report.version_a_inverse_dynamics_parity_error_Nm = max(report.version_a_inverse_dynamics_parity_error_Nm, norm(tau_s1 - tau_a));
    report.version_a_forward_dynamics_parity_error = max(report.version_a_forward_dynamics_parity_error, norm(recovered - ddq_a));
    report.version_a_energy_parity_error_J = max(report.version_a_energy_parity_error_J, abs(H_b - H_a));

    % 5. Adjoint Lie group identity
    Xi = randn(6, 1) * 0.2;
    report.adjoint_exponential_identity_error = max(report.adjoint_exponential_identity_error, ...
        norm(T * MatrixExp6(VecTose3(Xi)) * TransInv(T) - MatrixExp6(VecTose3(Adjoint(T) * Xi)), 'fro'));
end

% 6. Verification of Actuator and Friction Profiles
for prof = {'cholet_apparent', 'rigid_links', 'custom'}
    opts = struct('actuator_profile', prof{1});
    if strcmp(prof{1}, 'custom')
        opts.custom_apparent_inertia = [0.05; 0.05; 0.05; 0.03; 0.03; 0.03];
    end
    m_act = kinova_s1.model(opts);
    m_act_a = kinova.model(opts);
    [M_b_act, ~] = kinova_s1.dynamics(X, omega, m_act);
    [M_a_act, ~] = kinova.dynamics(q, omega, m_act_a);
    assert(norm(M_b_act - M_a_act, 'fro') < 1e-12, 'kinova_s1:actuator', 'Actuator profile parity failed');
end

for fprof = {'muraccioli_smooth', 'shape_blend', 'none'}
    opts = struct('friction_profile', fprof{1});
    m_f = kinova_s1.model(opts);
    m_fa = kinova.model(opts);
    [~, b_b_f] = kinova_s1.dynamics(X, omega, m_f);
    [~, b_a_f] = kinova.dynamics(q, omega, m_fa);
    assert(norm(b_b_f - b_a_f) < 1e-12, 'kinova_s1:friction', 'Friction profile parity failed');
end

% 7. Acceptance criteria
report.pass = report.mass_reference_error < 1e-10 && ...
    report.gravity_reference_error_Nm < 1e-10 && ...
    report.coriolis_reference_error_Nm < 1e-6 && ...
    report.fk_chain_error < 1e-10 && ...
    report.jacobian_finite_difference_error < 1e-7 && ...
    report.mass_asymmetry < 1e-10 && ...
    report.min_mass_eigenvalue > 0 && ...
    report.potential_gradient_error_Nm < 1e-6 && ...
    report.power_identity_error < 1e-6 && ...
    report.version_a_fk_parity_error < 1e-12 && ...
    report.version_a_jacobian_parity_error < 1e-12 && ...
    report.version_a_mass_parity_error < 1e-12 && ...
    report.version_a_inverse_dynamics_parity_error_Nm < 1e-11 && ...
    report.version_a_forward_dynamics_parity_error < 1e-10 && ...
    report.version_a_energy_parity_error_J < 1e-12 && ...
    report.forward_inverse_roundtrip_error < 1e-9 && ...
    report.wrench_duality_error < 1e-12 && ...
    report.minimum_friction_power_W >= 0 && ...
    report.adjoint_exponential_identity_error < 1e-10;

kinova.write_json(fullfile(root, 'results', 'validation_s1.json'), report);
disp(jsonencode(report, PrettyPrint=true));
assert(report.pass, 'kinova_s1:test', 'Kinova S1 validation failed');
end
