function log = simulate(m, c)
%SIMULATE Munthe-Kaas Lie-group RK4 simulation on (S1)^6 manifold with ZOH control.
%
% Integrates dynamics on (S1)^6 x R^6 without storing q as an unconstrained state.
% Unit-norm constraints ||x_i|| = 1 are preserved to machine precision without
% phase distortion via Abelian Munthe-Kaas RK4.
%
% Joint handling:
%   - Bounded revolute joints (2, 3, 5): limits and margins are monitored at all
%     RK4 substeps. Reference and tracking use admissible-arc displacement box_diff.
%   - Continuous joints (1, 4, 6): full-circle S^1 manifold without mechanical limits.
%     Accumulated winding is tracked continuously along the integrated trajectory,
%     preserving multi-turn paths without branch-cut jumps.
%
% Inputs:
%   m: (optional) model struct from kinova_s1.model()
%   c: (optional) config struct from kinova.config()
% Output:
%   log: simulation record struct with X, lifted q, dq/omega, energies, and metrics

if nargin < 1 || isempty(m)
    m = kinova_s1.model();
elseif ~isfield(m, 'qmid')
    m_s1_base = kinova_s1.model(m.options);
    m.qmid = m_s1_base.qmid;
    m.qhalf = m_s1_base.qhalf;
    m.umid = m_s1_base.umid;
    m.cos_half = m_s1_base.cos_half;
    m.is_continuous = m_s1_base.is_continuous;
    m.is_bounded = m_s1_base.is_bounded;
    m.is_pure_z = m_s1_base.is_pure_z;
end
if nargin < 2 || isempty(c)
    c = kinova.config();
end

assert(c.control_dt > 0 && c.substeps >= 1 && c.substeps == floor(c.substeps));
assert(c.motion_time > 0 && c.end_time >= c.motion_time);
assert(abs(c.end_time / c.control_dt - round(c.end_time / c.control_dt)) < 1e-9);

q0 = kinova.joint_vector(c.q0);
goal = kinova.joint_vector(c.qgoal);
assert(all(q0 >= m.qmin & q0 <= m.qmax) && all(goal >= m.qmin & goal <= m.qmax));

X0 = kinova_s1.from_q(q0);
Xgoal = kinova_s1.from_q(goal);
assert(all(kinova_s1.check_limits(X0, m)) && all(kinova_s1.check_limits(Xgoal, m)), ...
    'kinova_s1:limits', 'Initial and goal configurations must be admissible');

n = m.n;
controller = m;
if ~isempty(c.controller_model)
    controller = c.controller_model;
end

phase = ftv5.phase_plan(0, 0, 0, c.motion_time);
d_box = goal - q0; % Preserves any multi-turn winding commanded in c.qgoal

clock = tic;
t = 0:c.control_dt:c.end_time;
N = numel(t);

% State on (S1)^6 x R^6: X (2x6), omega (6x1), q_unwrapped (6x1), and work integrals (3x1)
X = X0;
omega = zeros(n, 1);
q_unwrapped = q0;
work = zeros(3, 1); % [work_joint; friction_loss; work_env]

log = struct('t', t, 'X', zeros(2, n, N), 'q', zeros(n, N), 'dq', zeros(n, N), ...
    'omega', zeros(n, N), 'Xref', zeros(2, n, N), 'qref', zeros(n, N), ...
    'dqref', zeros(n, N), 'tau_command', zeros(n, N), 'tau_applied', zeros(n, N), ...
    'effectiveness', ones(n, N), 'saturated', false(n, N), 'H', zeros(1, N), ...
    'work_joint', zeros(1, N), 'friction_loss', zeros(1, N), ...
    'work_environment', zeros(1, N), 'position', zeros(3, N), ...
    'position_ref', zeros(3, N), 'max_unit_norm_error', zeros(1, N));

h = c.control_dt / c.substeps;
limit_violation = 0;
velocity_utilization = 0;
stage_min_margin = Inf;
max_unit_norm_ever = 0;

cm = m.umid(1, :)';
sm = m.umid(2, :)';

for k = 1:N
    [Xr, dqr, ddqr, qr] = kinova_s1.reference(t(k), c, m, phase, X0, d_box);
    q_lifted = kinova_s1.to_q(X, m, q_unwrapped);

    % Control computation using admissible displacement box_diff on (S1)^6
    if isempty(c.control_function)
        if isfield(c, 'continuous_control_mode') && strcmp(c.continuous_control_mode, 'modular_s1')
            w_ctrl = zeros(n, 1);
        else
            % Fiber winding offset w in Z preserves multi-turn tracking even when |qr - q| > pi
            d_geo = kinova_s1.geodesic_diff(Xr, X);
            w_ctrl = zeros(n, 1);
            w_ctrl(m.is_continuous) = round(((qr(m.is_continuous) - q_lifted(m.is_continuous)) ...
                - d_geo(m.is_continuous)) / (2 * pi));
        end
        err_box = kinova_s1.box_diff(Xr, X, m, w_ctrl);
        virtual = ddqr + c.kp(:) .* err_box + c.kd(:) .* (dqr - omega);
        command_raw = kinova_s1.inverse_dynamics(X, omega, virtual, controller);
    else
        command_raw = kinova.joint_vector(c.control_function(t(k), q_lifted, omega, qr, dqr, ddqr, controller));
    end

    command = command_raw;
    if c.command_saturation
        command = max(-m.taumax, min(m.taumax, command));
    end

    [alpha, ~] = inputs(t(k), X, omega, m, c, n, q_unwrapped);
    actual = alpha .* command;

    % Diagnostics and logging on (S1)^6 x R^6
    log.X(:, :, k) = X;
    log.q(:, k) = q_lifted;
    log.dq(:, k) = omega;
    log.omega(:, k) = omega;
    log.Xref(:, :, k) = Xr;
    log.qref(:, k) = qr;
    log.dqref(:, k) = dqr;
    log.tau_command(:, k) = command;
    log.tau_applied(:, k) = actual;
    log.effectiveness(:, k) = alpha;
    log.saturated(:, k) = abs(command_raw) > m.taumax + 1e-12;
    log.H(k) = kinova_s1.energy(X, omega, m);
    log.work_joint(k) = work(1);
    log.friction_loss(k) = work(2);
    log.work_environment(k) = work(3);

    norm_err_k = max(abs(sum(X.^2, 1) - 1));
    log.max_unit_norm_error(k) = norm_err_k;
    max_unit_norm_ever = max(max_unit_norm_ever, norm_err_k);

    T = kinova_s1.fk(X, m);
    Tr = kinova_s1.fk(Xr, m);
    log.position(:, k) = T(1:3, 4);
    log.position_ref(:, k) = Tr(1:3, 4);

    % Munthe-Kaas Lie-Group RK4 integration between control epochs
    if k < N
        for j = 1:c.substeps
            tj = t(k) + (j - 1) * h;
            X1 = X;
            omega1 = omega;
            q_stage1 = q_unwrapped;

            % Stage 1: at (X1, omega1)
            [dw1, k1_w, k1_work] = mk_rhs(tj, X1, omega1, command, m, c, q_stage1);

            % Stage 2: at Exp(h/2 * dw1) * X1
            dtheta2 = (h / 2) * dw1;
            X2 = kinova_s1.exp_s1(dtheta2, X1, true);
            omega2 = omega1 + (h / 2) * k1_w;
            q_stage2 = q_stage1 + dtheta2;
            [dw2, k2_w, k2_work] = mk_rhs(tj + h / 2, X2, omega2, command, m, c, q_stage2);

            % Stage 3: at Exp(h/2 * dw2) * X1
            dtheta3 = (h / 2) * dw2;
            X3 = kinova_s1.exp_s1(dtheta3, X1, true);
            omega3 = omega1 + (h / 2) * k2_w;
            q_stage3 = q_stage1 + dtheta3;
            [dw3, k3_w, k3_work] = mk_rhs(tj + h / 2, X3, omega3, command, m, c, q_stage3);

            % Stage 4: at Exp(h * dw3) * X1
            dtheta4 = h * dw3;
            X4 = kinova_s1.exp_s1(dtheta4, X1, true);
            omega4 = omega1 + h * k3_w;
            q_stage4 = q_stage1 + dtheta4;
            [dw4, k4_w, k4_work] = mk_rhs(tj + h, X4, omega4, command, m, c, q_stage4);

            % Step update via Lie group exponential action
            dtheta_step = (h / 6) * (dw1 + 2 * dw2 + 2 * dw3 + dw4);
            X = kinova_s1.exp_s1(dtheta_step, X1, true);
            omega = omega1 + (h / 6) * (k1_w + 2 * k2_w + 2 * k3_w + k4_w);
            work = work + (h / 6) * (k1_work + 2 * k2_work + 2 * k3_work + k4_work);
            q_unwrapped = q_stage1 + dtheta_step;

            if c.capture_substeps
                % Intrinsic S^1 stage monitoring: unit-norm and bounded-joint arc margins (2, 3, 5)
                C_st = [X1(1, :); X2(1, :); X3(1, :); X4(1, :); X(1, :)];
                S_st = [X1(2, :); X2(2, :); X3(2, :); X4(2, :); X(2, :)];
                st_norm_err = max(abs(C_st.^2 + S_st.^2 - 1), [], 'all');
                max_unit_norm_ever = max(max_unit_norm_ever, st_norm_err);

                b_idx = m.is_bounded;
                cm_b = cm(b_idx)';
                sm_b = sm(b_idx)';
                qhalf_b = m.qhalf(b_idx)';
                C_b = C_st(:, b_idx);
                S_b = S_st(:, b_idx);
                delta_b = atan2(S_b .* cm_b - C_b .* sm_b, C_b .* cm_b + S_b .* sm_b);
                margins = [repmat(qhalf_b, 5, 1) + delta_b; repmat(qhalf_b, 5, 1) - delta_b];
                stage_min_margin = min(stage_min_margin, min(margins, [], 'all'));
                limit_violation = max(limit_violation, max(-margins, [], 'all'));

                stage_omegas = [omega1, omega2, omega3, omega4, omega];
                velocity_utilization = max(velocity_utilization, ...
                    max(abs(stage_omegas) ./ m.dqmax, [], 'all'));
            end

            assert(all(isfinite(X(:))) && all(isfinite(omega)), ...
                'kinova_s1:integration', 'Nonfinite state: reduce step or check model');
        end
    end
end

error = log.q - log.qref;
log.energy_residual = log.H - log.H(1) - log.work_joint - log.work_environment + log.friction_loss;

log.metrics = struct('actuator_profile', m.actuator_profile, 'friction_profile', m.friction_profile, ...
    'control_dt_s', c.control_dt, 'plant_step_s', h, 'duration_s', c.end_time, ...
    'max_joint_error_deg', max(abs(error), [], 'all') * 180 / pi, ...
    'rms_joint_error_deg', sqrt(mean(error.^2, 'all')) * 180 / pi, ...
    'max_position_error_mm', max(vecnorm(log.position - log.position_ref)) * 1000, ...
    'energy_balance_max_error_J', max(abs(log.energy_residual)), ...
    'minimum_recorded_limit_margin_rad', min([log.q(m.is_bounded, :) - m.qmin(m.is_bounded); ...
                                              m.qmax(m.is_bounded) - log.q(m.is_bounded, :)], [], 'all'), ...
    'minimum_stage_limit_margin_rad', stage_min_margin, ...
    'maximum_stage_limit_violation_rad', limit_violation, ...
    'max_stage_velocity_utilization', velocity_utilization, ...
    'max_command_utilization', max(abs(log.tau_command) ./ m.taumax, [], 'all'), ...
    'saturated_joint_samples', nnz(log.saturated), ...
    'max_unit_norm_error', max_unit_norm_ever, ...
    'elapsed_seconds', toc(clock));
log.model_options = m.options;
log.urdf_revision = m.urdf_revision;
end

function [dw, k_w, k_work] = mk_rhs(t, X, omega, command, m, c, q_stage)
n = m.n;
[alpha, F] = inputs(t, X, omega, m, c, n, q_stage);
[M, b] = kinova_s1.dynamics(X, omega, m);
actual = alpha .* command;
environment = zeros(6, 1);
if any(F)
    environment = kinova_s1.jacobian(X, m, 'body')' * F;
end
domega = M \ (actual + environment - b);
f = kinova.friction(omega, m);
dw = omega;
k_w = domega;
k_work = [actual' * omega; f' * omega; environment' * omega];
end

function [alpha, F] = inputs(t, X, omega, m, c, n, q_stage)
alpha = ones(n, 1);
F = zeros(6, 1);
if ~isempty(c.effectiveness_function)
    alpha = kinova.joint_vector(c.effectiveness_function(t));
end
if ~isempty(c.external_wrench_function)
    q_lift = kinova_s1.to_q(X, m, q_stage);
    F = kinova.joint_vector(c.external_wrench_function(t, q_lift, omega));
end
assert(all(alpha >= 0 & alpha <= 1), 'kinova_s1:input', 'Effectiveness must be in [0,1]');
end
