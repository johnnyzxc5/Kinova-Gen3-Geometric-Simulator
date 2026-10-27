function report = benchmark_s1_comparison(varargin)
%BENCHMARK_S1_COMPARISON Rigorous four-way scientific benchmark and comparative profiling:
%   - Stock MR:      Pure textbook Modern Robotics (MatrixExp6, InverseDynamics, MassMatrix)
%   - Version A:     Original simulator (+kinova, q in R^6, MatrixExp6 + SHAPE RNEA)
%   - Variant B-alg: Angular-specialized Euclidean (q in R^6 -> [cos(q'); sin(q')] + S1 kernels)
%   - Version B:     Global Lie-group formulation (+kinova_s1, X in (S^1)^6 + S1 kernels + RK4-MK)
%
% Separating Variant B-alg from Version B cleanly distinguishes algorithmic
% specialization (replacing generic MatrixExp6/Adjoint with revolute z-rotations
% and hoisted adjoints) from the manifold representation itself (evolving on (S^1)^6).

p = inputParser;
addParameter(p, 'n_states', 100, @isnumeric);
addParameter(p, 'n_repeats', 10, @isnumeric);
addParameter(p, 'seed', 13102026, @isnumeric);
addParameter(p, 'save_results', true, @islogical);
parse(p, varargin{:});
opts = p.Results;

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
setup_kinova();

m_a = kinova.model();
m_b = kinova_s1.model();
n = m_a.n;

rng(opts.seed, 'twister');
N = opts.n_states;
R = opts.n_repeats;

fprintf('========================================================================================\n');
fprintf(' KINOVA GEN3 DYNAMICS BENCHMARK: STOCK MR vs VERSION A vs VARIANT B-ALG vs VERSION B\n');
fprintf(' MATLAB Version: %s\n', version);
fprintf(' States: %d, Repeats: %d, Seed: %d\n', N, R, opts.seed);
fprintf('========================================================================================\n\n');

% -------------------------------------------------------------------------
% 1. Sample Realistic Configurations and Velocities strictly within Physical Bounds
% -------------------------------------------------------------------------
margin = 0.08;
Q = zeros(n, N);
for i = 1:n
    if m_b.is_bounded(i)
        Q(i, :) = (m_a.qmin(i) + margin) + (m_a.qmax(i) - m_a.qmin(i) - 2 * margin) .* rand(1, N);
    else
        % Continuous joints: operational envelope [-pi, pi]
        Q(i, :) = (2 * rand(1, N) - 1) * pi;
    end
end

% Velocity sampling: normal distribution with sigma = 0.35*dqmax, truncated by rejection
% to ensure 100% of samples satisfy |dq_i| <= 0.95*dqmax (zero physical limit violations)
DQ = zeros(n, N);
for j = 1:n
    col_dq = [];
    while numel(col_dq) < N
        cand = randn(N, 1) * (0.35 * m_a.dqmax(j));
        valid = cand(abs(cand) <= 0.95 * m_a.dqmax(j));
        col_dq = [col_dq; valid]; %#ok<AGROW>
    end
    DQ(j, :) = col_dq(1:N)';
end

% Torque sampling: normal distribution with sigma = 0.30*taumax, truncated by rejection
% to ensure 100% of samples satisfy |tau_i| <= 0.95*taumax
TAU = zeros(n, N);
for j = 1:n
    col_tau = [];
    while numel(col_tau) < N
        cand = randn(N, 1) * (0.30 * m_a.taumax(j));
        valid = cand(abs(cand) <= 0.95 * m_a.taumax(j));
        col_tau = [col_tau; valid]; %#ok<AGROW>
    end
    TAU(j, :) = col_tau(1:N)';
end

% Acceleration sampling: normal distribution truncated to [-10, 10] rad/s^2
DDQ = zeros(n, N);
col_ddq = [];
while numel(col_ddq) < n * N
    cand = randn(n * N, 1) * 2.0;
    valid = cand(abs(cand) <= 10.0);
    col_ddq = [col_ddq; valid]; %#ok<AGROW>
end
DDQ = reshape(col_ddq(1:n*N), n, N);

% Strict physical verification asserts
assert(all(Q(m_b.is_bounded, :) >= m_a.qmin(m_b.is_bounded) & ...
           Q(m_b.is_bounded, :) <= m_a.qmax(m_b.is_bounded), 'all'), ...
       'benchmark:bounds', 'Q violates joint limits');
assert(all(abs(DQ) <= m_a.dqmax, 'all'), 'benchmark:bounds', 'DQ violates velocity limits');
assert(all(abs(TAU) <= m_a.taumax, 'all'), 'benchmark:bounds', 'TAU violates effort limits');

X_cells = cell(1, N);
for i = 1:N
    X_cells{i} = kinova_s1.from_q(Q(:, i));
end

% -------------------------------------------------------------------------
% 2. Warm-up Phase (Ensure JIT Compilation & CPU Warm-up)
% -------------------------------------------------------------------------
fprintf('[1/6] Warming up JIT and caches across all routines (%d passes)...\n', N);
for i = 1:N
    q_i = Q(:, i);
    dq_i = DQ(:, i);
    ddq_i = DDQ(:, i);
    tau_i = TAU(:, i);
    X_i = X_cells{i};

    % Stock
    stock_fk(q_i, m_a);
    JacobianSpace(m_a.Slist, q_i);
    JacobianBody(m_a.Blist, q_i);
    stock_inv_dyn(q_i, dq_i, ddq_i, m_a);
    stock_dyn(q_i, dq_i, m_a);
    stock_fwd_dyn(q_i, dq_i, tau_i, m_a);

    % Version A
    kinova.fk(q_i, m_a);
    kinova.frames(q_i, m_a);
    kinova.jacobian(q_i, m_a, 'space');
    kinova.jacobian(q_i, m_a, 'body');
    kinova.inverse_dynamics(q_i, dq_i, ddq_i, m_a);
    kinova.dynamics(q_i, dq_i, m_a);
    kinova.forward_dynamics(q_i, dq_i, tau_i, m_a);

    % Variant B-alg (q -> from_q(q) -> S1 algebraic kernels)
    kinova_s1.fk(kinova_s1.from_q(q_i), m_b);
    kinova_s1.frames(kinova_s1.from_q(q_i), m_b);
    kinova_s1.jacobian(kinova_s1.from_q(q_i), m_b, 'space');
    kinova_s1.jacobian(kinova_s1.from_q(q_i), m_b, 'body');
    kinova_s1.inverse_dynamics(kinova_s1.from_q(q_i), dq_i, ddq_i, m_b);
    kinova_s1.dynamics(kinova_s1.from_q(q_i), dq_i, m_b);
    kinova_s1.forward_dynamics(kinova_s1.from_q(q_i), dq_i, tau_i, m_b);

    % Version B (native (S1)^6 state X_i)
    kinova_s1.fk(X_i, m_b);
    kinova_s1.frames(X_i, m_b);
    kinova_s1.jacobian(X_i, m_b, 'space');
    kinova_s1.jacobian(X_i, m_b, 'body');
    kinova_s1.inverse_dynamics(X_i, dq_i, ddq_i, m_b);
    kinova_s1.dynamics(X_i, dq_i, m_b);
    kinova_s1.forward_dynamics(X_i, dq_i, tau_i, m_b);
    kinova_s1.to_q(X_i, m_b);
end

% -------------------------------------------------------------------------
% 3. Core Micro-Benchmarks (Execution Time per Call)
% -------------------------------------------------------------------------
fprintf('[2/6] Profiling core operations (%d repeats x %d states = %d calls each)...\n', R, N, R * N);

% Forward Kinematics (T_0e)
t_stock_fk = time_op(@() run_stock_fk(Q, m_a), R, N);
t_a_fk     = time_op(@() run_a_fk(Q, m_a), R, N);
t_balg_fk  = time_op(@() run_balg_fk(Q, m_b), R, N);
t_b_fk     = time_op(@() run_b_fk(X_cells, m_b), R, N);

% Link Frames (all poses)
t_stock_frames = time_op(@() run_stock_frames(Q, m_a), R, N);
t_a_frames     = time_op(@() run_a_frames(Q, m_a), R, N);
t_balg_frames  = time_op(@() run_balg_frames(Q, m_b), R, N);
t_b_frames     = time_op(@() run_b_frames(X_cells, m_b), R, N);

% Space Jacobian
t_stock_js = time_op(@() run_stock_js(Q, m_a), R, N);
t_a_js     = time_op(@() run_a_js(Q, m_a), R, N);
t_balg_js  = time_op(@() run_balg_js(Q, m_b), R, N);
t_b_js     = time_op(@() run_b_js(X_cells, m_b), R, N);

% Body Jacobian
t_stock_jb = time_op(@() run_stock_jb(Q, m_a), R, N);
t_a_jb     = time_op(@() run_a_jb(Q, m_a), R, N);
t_balg_jb  = time_op(@() run_balg_jb(Q, m_b), R, N);
t_b_jb     = time_op(@() run_b_jb(X_cells, m_b), R, N);

% Inverse Dynamics (RNEA)
t_stock_id = time_op(@() run_stock_id(Q, DQ, DDQ, m_a), R, N);
t_a_id     = time_op(@() run_a_id(Q, DQ, DDQ, m_a), R, N);
t_balg_id  = time_op(@() run_balg_id(Q, DQ, DDQ, m_b), R, N);
t_b_id     = time_op(@() run_b_id(X_cells, DQ, DDQ, m_b), R, N);

% Complete Dynamics (M and b: mass matrix + bias)
t_stock_dyn = time_op(@() run_stock_dyn(Q, DQ, m_a), R, N);
t_a_dyn     = time_op(@() run_a_dyn(Q, DQ, m_a), R, N);
t_balg_dyn  = time_op(@() run_balg_dyn(Q, DQ, m_b), R, N);
t_b_dyn     = time_op(@() run_b_dyn(X_cells, DQ, m_b), R, N);

% Forward Dynamics (q_ddot)
t_stock_fd = time_op(@() run_stock_fd(Q, DQ, TAU, m_a), R, N);
t_a_fd     = time_op(@() run_a_fd(Q, DQ, TAU, m_a), R, N);
t_balg_fd  = time_op(@() run_balg_fd(Q, DQ, TAU, m_b), R, N);
t_b_fd     = time_op(@() run_b_fd(X_cells, DQ, TAU, m_b), R, N);

% Representation Conversion
t_from_q = time_op(@() run_from_q(Q), R, N);
t_to_q   = time_op(@() run_to_q(X_cells, m_b), R, N);
t_exp_s1 = time_op(@() run_exp_s1(DQ, X_cells), R, N);

% -------------------------------------------------------------------------
% 4. Accuracy & Mathematical Parity across Sampled States
% -------------------------------------------------------------------------
fprintf('[3/6] Evaluating numerical parity across %d independent states...\n', N);
err_fk_a_vs_stock = 0; err_fk_b_vs_stock = 0; err_fk_b_vs_a = 0;
err_js_b_vs_a = 0;     err_jb_b_vs_a = 0;
err_id_b_vs_a = 0;     err_id_b_vs_stock = 0;
err_m_b_vs_a = 0;      err_m_b_vs_stock = 0;
err_b_bias_b_vs_a = 0;
err_fd_b_vs_a = 0;

for i = 1:N
    q_i = Q(:, i); dq_i = DQ(:, i); ddq_i = DDQ(:, i); tau_i = TAU(:, i);
    X_i = X_cells{i};

    % FK
    T_stock = stock_fk(q_i, m_a);
    T_a = kinova.fk(q_i, m_a);
    T_b = kinova_s1.fk(X_i, m_b);
    err_fk_a_vs_stock = max(err_fk_a_vs_stock, max(abs(T_a - T_stock), [], 'all'));
    err_fk_b_vs_stock = max(err_fk_b_vs_stock, max(abs(T_b - T_stock), [], 'all'));
    err_fk_b_vs_a     = max(err_fk_b_vs_a,     max(abs(T_b - T_a), [], 'all'));

    % Jacobians
    Js_a = kinova.jacobian(q_i, m_a, 'space');
    Js_b = kinova_s1.jacobian(X_i, m_b, 'space');
    Jb_a = kinova.jacobian(q_i, m_a, 'body');
    Jb_b = kinova_s1.jacobian(X_i, m_b, 'body');
    err_js_b_vs_a = max(err_js_b_vs_a, max(abs(Js_b - Js_a), [], 'all'));
    err_jb_b_vs_a = max(err_jb_b_vs_a, max(abs(Jb_b - Jb_a), [], 'all'));

    % ID
    tau_stock = stock_inv_dyn(q_i, dq_i, ddq_i, m_a);
    tau_a = kinova.inverse_dynamics(q_i, dq_i, ddq_i, m_a);
    tau_b = kinova_s1.inverse_dynamics(X_i, dq_i, ddq_i, m_b);
    err_id_b_vs_a     = max(err_id_b_vs_a,     max(abs(tau_b - tau_a)));
    err_id_b_vs_stock = max(err_id_b_vs_stock, max(abs(tau_b - tau_stock)));

    % Dynamics
    [M_stock, ~] = stock_dyn(q_i, dq_i, m_a);
    [M_a, b_a] = kinova.dynamics(q_i, dq_i, m_a);
    [M_b, b_b] = kinova_s1.dynamics(X_i, dq_i, m_b);
    err_m_b_vs_a      = max(err_m_b_vs_a,      max(abs(M_b - M_a), [], 'all'));
    err_m_b_vs_stock  = max(err_m_b_vs_stock,  max(abs(M_b - M_stock), [], 'all'));
    err_b_bias_b_vs_a = max(err_b_bias_b_vs_a, max(abs(b_b - b_a)));

    % FD
    ddq_a = kinova.forward_dynamics(q_i, dq_i, tau_i, m_a);
    ddq_b = kinova_s1.forward_dynamics(X_i, dq_i, tau_i, m_b);
    err_fd_b_vs_a = max(err_fd_b_vs_a, max(abs(ddq_b - ddq_a)));
end

% -------------------------------------------------------------------------
% 5. Macro-Benchmark: Full 3.6 s Trajectory Simulation
% -------------------------------------------------------------------------
fprintf('[4/6] Profiling full 3.6 s closed-loop trajectory simulation (A vs B-alg vs B)...\n');
c_sim = kinova.config();
c_sim.save_results = false;

% Warm-up simulation
kinova.simulate(m_a, c_sim);
simulate_balg(m_b, c_sim);
kinova_s1.simulate(m_b, c_sim);

t_sim_a = zeros(1, R);
t_sim_balg = zeros(1, R);
t_sim_b = zeros(1, R);

for r = 1:R
    tic; log_a_sim = kinova.simulate(m_a, c_sim); t_sim_a(r) = toc;
    tic; log_balg_sim = simulate_balg(m_b, c_sim); t_sim_balg(r) = toc;
    tic; log_b_sim = kinova_s1.simulate(m_b, c_sim); t_sim_b(r) = toc;
end

% Simulation trajectory parity
sim_max_q_err_b_vs_a = max(abs(log_b_sim.q - log_a_sim.q), [], 'all');
sim_max_pos_err_b_vs_a_mm = max(vecnorm(log_b_sim.position - log_a_sim.position)) * 1000;
sim_energy_err_a = log_a_sim.metrics.energy_balance_max_error_J;
sim_energy_err_balg = log_balg_sim.metrics.energy_balance_max_error_J;
sim_energy_err_b = log_b_sim.metrics.energy_balance_max_error_J;
sim_unit_norm_err_b = log_b_sim.metrics.max_unit_norm_error;

% -------------------------------------------------------------------------
% 6. Compile Structured Benchmark Report
% -------------------------------------------------------------------------
fprintf('[5/6] Compiling metrics and statistical distributions...\n');

ops = {'fk', 'frames', 'jacobian_space', 'jacobian_body', 'inverse_dynamics', 'dynamics', 'forward_dynamics'};
t_data = struct(...
    'fk',             struct('stock', t_stock_fk,     'a', t_a_fk,     'balg', t_balg_fk,     'b', t_b_fk), ...
    'frames',         struct('stock', t_stock_frames, 'a', t_a_frames, 'balg', t_balg_frames, 'b', t_b_frames), ...
    'jacobian_space', struct('stock', t_stock_js,     'a', t_a_js,     'balg', t_balg_js,     'b', t_b_js), ...
    'jacobian_body',  struct('stock', t_stock_jb,     'a', t_a_jb,     'balg', t_balg_jb,     'b', t_b_jb), ...
    'inverse_dynamics', struct('stock', t_stock_id,   'a', t_a_id,     'balg', t_balg_id,     'b', t_b_id), ...
    'dynamics',       struct('stock', t_stock_dyn,    'a', t_a_dyn,    'balg', t_balg_dyn,    'b', t_b_dyn), ...
    'forward_dynamics', struct('stock', t_stock_fd,   'a', t_a_fd,     'balg', t_balg_fd,     'b', t_b_fd));

micro_summary = struct();
for k = 1:numel(ops)
    op = ops{k};
    d = t_data.(op);
    s_med = median(d.stock);
    a_med = median(d.a);
    balg_med = median(d.balg);
    b_med = median(d.b);
    micro_summary.(op) = struct(...
        'stock_us', s_med, 'version_a_us', a_med, 'variant_balg_us', balg_med, 'version_b_us', b_med, ...
        'speedup_b_vs_stock', s_med / b_med, ...
        'speedup_b_vs_a', a_med / b_med, ...
        'speedup_balg_vs_a', a_med / balg_med, ...
        'overhead_b_vs_balg_pct', ((b_med - balg_med) / balg_med) * 100);
end

conversion_summary = struct(...
    'from_q_us', median(t_from_q), ...
    'to_q_us', median(t_to_q), ...
    'exp_s1_us', median(t_exp_s1));

macro_summary = struct(...
    'duration_s', c_sim.end_time, ...
    'control_dt_s', c_sim.control_dt, ...
    'plant_step_s', c_sim.control_dt / c_sim.substeps, ...
    'total_rk4_substeps', (c_sim.end_time / c_sim.control_dt) * c_sim.substeps, ...
    'version_a_elapsed_s', median(t_sim_a), ...
    'variant_balg_elapsed_s', median(t_sim_balg), ...
    'version_b_elapsed_s', median(t_sim_b), ...
    'sim_speedup_b_vs_a', median(t_sim_a) / median(t_sim_b), ...
    'sim_speedup_b_vs_balg', median(t_sim_balg) / median(t_sim_b), ...
    'sim_realtime_factor_a', c_sim.end_time / median(t_sim_a), ...
    'sim_realtime_factor_b', c_sim.end_time / median(t_sim_b), ...
    'sim_max_q_parity_error_rad', sim_max_q_err_b_vs_a, ...
    'sim_max_pos_parity_error_mm', sim_max_pos_err_b_vs_a_mm, ...
    'sim_energy_err_a_J', sim_energy_err_a, ...
    'sim_energy_err_balg_J', sim_energy_err_balg, ...
    'sim_energy_err_b_J', sim_energy_err_b, ...
    'sim_max_unit_norm_error', sim_unit_norm_err_b);

parity_summary = struct(...
    'max_fk_err_b_vs_a', err_fk_b_vs_a, ...
    'max_fk_err_b_vs_stock', err_fk_b_vs_stock, ...
    'max_js_err_b_vs_a', err_js_b_vs_a, ...
    'max_jb_err_b_vs_a', err_jb_b_vs_a, ...
    'max_id_err_b_vs_a', err_id_b_vs_a, ...
    'max_id_err_b_vs_stock', err_id_b_vs_stock, ...
    'max_mass_err_b_vs_a', err_m_b_vs_a, ...
    'max_mass_err_b_vs_stock', err_m_b_vs_stock, ...
    'max_bias_err_b_vs_a', err_b_bias_b_vs_a, ...
    'max_fd_err_b_vs_a', err_fd_b_vs_a);

envelope_summary = struct(...
    'admissible_envelope_verified', true, ...
    'dq_limit_violations', nnz(any(abs(DQ) > m_a.dqmax, 1)), ...
    'max_dq_utilization', max(abs(DQ) ./ m_a.dqmax, [], 'all'), ...
    'tau_limit_violations', nnz(any(abs(TAU) > m_a.taumax, 1)), ...
    'max_tau_utilization', max(abs(TAU) ./ m_a.taumax, [], 'all'));

report = struct(...
    'timestamp', datestr(now, 'yyyy-mm-dd HH:MM:SS'), ...
    'matlab_version', version, ...
    'n_states', N, ...
    'n_repeats', R, ...
    'seed', opts.seed, ...
    'envelope', envelope_summary, ...
    'micro_benchmarks', micro_summary, ...
    'conversions', conversion_summary, ...
    'macro_simulation', macro_summary, ...
    'parity', parity_summary);

% -------------------------------------------------------------------------
% 7. Generate 4-Panel Benchmark Visualization Figure
% -------------------------------------------------------------------------
fprintf('[6/6] Generating benchmark figure and saving artifacts...\n');
fig = figure('Name', 'Kinova Gen3 S1 Benchmark Comparison', ...
    'Units', 'pixels', 'Position', [100, 100, 1200, 850], 'Color', 'w', 'Visible', 'off');

% Color palette
col_stock = [0.45, 0.45, 0.45];
col_a     = [0.82, 0.28, 0.20];
col_balg  = [0.18, 0.55, 0.75];
col_b     = [0.13, 0.65, 0.35];

% Panel 1: Core Operations Execution Time (log scale)
subplot(2, 2, 1);
op_names = {'FK', 'Frames', 'Jac Space', 'Jac Body', 'Inv Dyn', 'Dynamics', 'Fwd Dyn'};
data_mat = zeros(7, 4);
for k = 1:7
    op = ops{k};
    data_mat(k, 1) = micro_summary.(op).stock_us;
    data_mat(k, 2) = micro_summary.(op).version_a_us;
    data_mat(k, 3) = micro_summary.(op).variant_balg_us;
    data_mat(k, 4) = micro_summary.(op).version_b_us;
end
b_plot = bar(data_mat, 'grouped');
b_plot(1).FaceColor = col_stock;
b_plot(2).FaceColor = col_a;
b_plot(3).FaceColor = col_balg;
b_plot(4).FaceColor = col_b;
set(gca, 'YScale', 'log', 'XTickLabel', op_names, 'XTickLabelRotation', 25, 'FontSize', 9);
ylabel('Latency per Call (\mus, log scale)');
title('(a) Core Operator Latency (N=100, R=10)');
legend({'Stock MR', 'Version A (q)', 'Variant B-alg', 'Version B ((S^1)^6)'}, 'Location', 'northwest');
grid on;

% Panel 2: Speedup relative to Stock MR and Version A
subplot(2, 2, 2);
sp_b_vs_stock = zeros(1, 7);
sp_b_vs_a = zeros(1, 7);
for k = 1:7
    op = ops{k};
    sp_b_vs_stock(k) = micro_summary.(op).speedup_b_vs_stock;
    sp_b_vs_a(k) = micro_summary.(op).speedup_b_vs_a;
end
b_sp = bar([sp_b_vs_stock; sp_b_vs_a]', 'grouped');
b_sp(1).FaceColor = col_stock;
b_sp(2).FaceColor = col_a;
set(gca, 'XTickLabel', op_names, 'XTickLabelRotation', 25, 'FontSize', 9);
ylabel('Speedup Factor (\times)');
title('(b) Speedup Factor of Version B');
legend({'vs Stock MR', 'vs Version A'}, 'Location', 'northeast');
grid on;

% Panel 3: Closed-Loop Simulation Runtime Breakdown
subplot(2, 2, 3);
sim_times = [macro_summary.version_a_elapsed_s, ...
             macro_summary.variant_balg_elapsed_s, ...
             macro_summary.version_b_elapsed_s];
b_sim = bar(sim_times, 0.55);
b_sim.FaceColor = 'flat';
b_sim.CData(1, :) = col_a;
b_sim.CData(2, :) = col_balg;
b_sim.CData(3, :) = col_b;
set(gca, 'XTickLabel', {'Version A (q)', 'Variant B-alg', 'Version B ((S^1)^6)'}, 'FontSize', 9);
ylabel('Execution Time (seconds)');
title('(c) 3.6 s Closed-Loop Simulation Time (7200 Substeps)');
for idx = 1:3
    text(idx, sim_times(idx) + 0.05 * max(sim_times), sprintf('%.2f s', sim_times(idx)), ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 9);
end
ylim([0, max(sim_times) * 1.25]);
grid on;

% Panel 4: Energy Balance Residual & Representation Overhead
subplot(2, 2, 4);
energy_errs = [macro_summary.sim_energy_err_a_J, ...
               macro_summary.sim_energy_err_balg_J, ...
               macro_summary.sim_energy_err_b_J];
b_n = bar(energy_errs, 0.55);
b_n.FaceColor = 'flat';
b_n.CData(1, :) = col_a;
b_n.CData(2, :) = col_balg;
b_n.CData(3, :) = col_b;
set(gca, 'XTickLabel', {'Version A', 'Variant B-alg', 'Version B'}, 'FontSize', 9);
ylabel('Max Energy Residual (J)');
title('(d) Energy Balance Max Error (h = 0.5 ms)');
for idx = 1:3
    text(idx, energy_errs(idx) * 1.05, sprintf('%.2e J', energy_errs(idx)), ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 8);
end
ylim([0, max(energy_errs) * 1.3]);
grid on;

if opts.save_results
    out_dir = fullfile(root, 'results');
    if ~isfolder(out_dir), mkdir(out_dir); end
    asset_dir = fullfile(root, 'docs', 'assets');
    if ~isfolder(asset_dir), mkdir(asset_dir); end

    % Save JSON report
    json_path = fullfile(out_dir, 'benchmark_s1_comparison.json');
    kinova.write_json(json_path, report);
    fprintf('Saved JSON benchmark: %s\n', json_path);

    % Save MAT file
    mat_path = fullfile(out_dir, 'benchmark_s1_comparison.mat');
    save(mat_path, 'report', 't_data', 'Q', 'DQ', 'TAU', 'DDQ');
    fprintf('Saved MAT benchmark: %s\n', mat_path);

    % Save PNG figure
    png_path = fullfile(asset_dir, 'benchmark_s1_comparison.png');
    exportgraphics(fig, png_path, 'Resolution', 300);
    fprintf('Saved figure: %s\n', png_path);
end
close(fig);

print_console_summary(report);
end

% =========================================================================
% Console Formatter
% =========================================================================
function print_console_summary(r)
m = r.micro_benchmarks;
fprintf('\n========================================================================================\n');
fprintf(' BENCHMARK SUMMARY (N=%d states, R=%d repeats)\n', r.n_states, r.n_repeats);
fprintf('----------------------------------------------------------------------------------------\n');
fprintf(' Operator           | Stock MR    | Version A   | Variant B-alg | Version B   | Speedup (B/A)\n');
fprintf('----------------------------------------------------------------------------------------\n');
ops = fieldnames(m);
for i = 1:numel(ops)
    op = ops{i};
    d = m.(op);
    fprintf(' %-18s | %9.2f us | %9.2f us | %9.2f us   | %9.2f us | %7.2fx\n', ...
        op, d.stock_us, d.version_a_us, d.variant_balg_us, d.version_b_us, d.speedup_b_vs_a);
end
fprintf('----------------------------------------------------------------------------------------\n');
c = r.conversions;
fprintf(' Conversions: from_q: %.2f us | to_q: %.2f us | exp_s1: %.2f us\n', ...
    c.from_q_us, c.to_q_us, c.exp_s1_us);
fprintf('----------------------------------------------------------------------------------------\n');
s = r.macro_simulation;
fprintf(' Closed-Loop 3.6 s Sim: Version A: %.3f s | Variant B-alg: %.3f s | Version B: %.3f s\n', ...
    s.version_a_elapsed_s, s.variant_balg_elapsed_s, s.version_b_elapsed_s);
fprintf(' Energy Balance Error:  Version A: %.2e J | Variant B-alg: %.2e J | Version B: %.2e J\n', ...
    s.sim_energy_err_a_J, s.sim_energy_err_balg_J, s.sim_energy_err_b_J);
fprintf(' Parity across states:  FK: %.2e | M: %.2e | ID: %.2e | FD: %.2e\n', ...
    r.parity.max_fk_err_b_vs_a, r.parity.max_mass_err_b_vs_a, ...
    r.parity.max_id_err_b_vs_a, r.parity.max_fd_err_b_vs_a);
fprintf('========================================================================================\n\n');
end

% =========================================================================
% Variant B-alg Euler-Lagrange Sim Backend
% =========================================================================
function log = simulate_balg(m, c)
n = m.n;
q0 = kinova.joint_vector(c.q0);
goal = kinova.joint_vector(c.qgoal);
controller = m;
if ~isempty(c.controller_model)
    controller = c.controller_model;
end
phase = ftv5.phase_plan(0, 0, 0, c.motion_time);
t = 0:c.control_dt:c.end_time;
N = numel(t);
state = [q0; zeros(n, 1); 0; 0; 0];
log = struct('t', t, 'q', zeros(n, N), 'dq', zeros(n, N), ...
    'qref', zeros(n, N), 'dqref', zeros(n, N), 'tau_command', zeros(n, N), ...
    'tau_applied', zeros(n, N), 'H', zeros(1, N), 'work_joint', zeros(1, N), ...
    'friction_loss', zeros(1, N), 'work_environment', zeros(1, N), ...
    'position', zeros(3, N), 'position_ref', zeros(3, N));
h = c.control_dt / c.substeps;
for k = 1:N
    q = state(1:n);
    dq = state(n+1:2*n);
    X = [cos(q'); sin(q')];
    [s, v, a] = ftv5.phase(phase, t(k));
    d = goal - q0;
    qr = q0 + d * s;
    dqr = d * v;
    ddqr = d * a;
    Xr = [cos(qr'); sin(qr')];
    virtual = ddqr + c.kp(:) .* (qr - q) + c.kd(:) .* (dqr - dq);
    command = kinova_s1.inverse_dynamics(X, dq, virtual, controller);
    if c.command_saturation
        command = max(-m.taumax, min(m.taumax, command));
    end
    log.q(:, k) = q;
    log.dq(:, k) = dq;
    log.qref(:, k) = qr;
    log.dqref(:, k) = dqr;
    log.tau_command(:, k) = command;
    log.tau_applied(:, k) = command;
    log.H(k) = kinova_s1.energy(X, dq, m);
    log.work_joint(k) = state(2*n+1);
    log.friction_loss(k) = state(2*n+2);
    log.work_environment(k) = state(2*n+3);
    T = kinova_s1.fk(X, m);
    Tr = kinova_s1.fk(Xr, m);
    log.position(:, k) = T(1:3, 4);
    log.position_ref(:, k) = Tr(1:3, 4);
    if k < N
        for j = 1:c.substeps
            k1 = balg_rhs(state, command, m);
            k2 = balg_rhs(state + (h / 2) * k1, command, m);
            k3 = balg_rhs(state + (h / 2) * k2, command, m);
            k4 = balg_rhs(state + h * k3, command, m);
            state = state + (h / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
        end
    end
end
log.energy_residual = log.H - log.H(1) - log.work_joint - log.work_environment + log.friction_loss;
log.metrics = struct('energy_balance_max_error_J', max(abs(log.energy_residual)));
end

function dx = balg_rhs(state, command, m)
n = m.n;
q = state(1:n);
dq = state(n+1:2*n);
X = [cos(q'); sin(q')];
[M, b] = kinova_s1.dynamics(X, dq, m);
ddq = M \ (command - b);
f = kinova.friction(dq, m);
dx = [dq; ddq; command' * dq; f' * dq; 0];
end

% =========================================================================
% Micro-benchmark Execution Runners
% =========================================================================
function t_per_call_us = time_op(op_func, repeats, n_calls)
t_runs = zeros(1, repeats);
for r = 1:repeats
    tic;
    op_func();
    t_runs(r) = toc;
end
t_per_call_us = (t_runs ./ n_calls) * 1e6;
end

function T = stock_fk(q, m)
T = m.home;
for j = m.n:-1:1
    T = MatrixExp6(VecTose3(m.Slist(:, j) * q(j))) * T;
end
end

function run_stock_fk(Q, m)
for i = 1:size(Q, 2)
    stock_fk(Q(:, i), m);
end
end

function run_a_fk(Q, m)
for i = 1:size(Q, 2)
    kinova.fk(Q(:, i), m);
end
end

function run_balg_fk(Q, m)
for i = 1:size(Q, 2)
    q = Q(:, i);
    kinova_s1.fk([cos(q'); sin(q')], m);
end
end

function run_b_fk(X_cells, m)
for i = 1:length(X_cells)
    kinova_s1.fk(X_cells{i}, m);
end
end

function run_stock_frames(Q, m)
for i = 1:size(Q, 2)
    T = eye(4);
    for j = 1:m.n+1
        T = T * m.Mlist(:, :, j);
        if j <= m.n
            T = T * MatrixExp6(VecTose3([m.axis_local(:, j); zeros(3, 1)] * Q(j, i)));
        end
    end
end
end

function run_a_frames(Q, m)
for i = 1:size(Q, 2)
    kinova.frames(Q(:, i), m);
end
end

function run_balg_frames(Q, m)
for i = 1:size(Q, 2)
    q = Q(:, i);
    kinova_s1.frames([cos(q'); sin(q')], m);
end
end

function run_b_frames(X_cells, m)
for i = 1:length(X_cells)
    kinova_s1.frames(X_cells{i}, m);
end
end

function run_stock_js(Q, m)
for i = 1:size(Q, 2)
    JacobianSpace(m.Slist, Q(:, i));
end
end

function run_a_js(Q, m)
for i = 1:size(Q, 2)
    kinova.jacobian(Q(:, i), m, 'space');
end
end

function run_balg_js(Q, m)
for i = 1:size(Q, 2)
    q = Q(:, i);
    kinova_s1.jacobian([cos(q'); sin(q')], m, 'space');
end
end

function run_b_js(X_cells, m)
for i = 1:length(X_cells)
    kinova_s1.jacobian(X_cells{i}, m, 'space');
end
end

function run_stock_jb(Q, m)
for i = 1:size(Q, 2)
    JacobianBody(m.Blist, Q(:, i));
end
end

function run_a_jb(Q, m)
for i = 1:size(Q, 2)
    kinova.jacobian(Q(:, i), m, 'body');
end
end

function run_balg_jb(Q, m)
for i = 1:size(Q, 2)
    q = Q(:, i);
    kinova_s1.jacobian([cos(q'); sin(q')], m, 'body');
end
end

function run_b_jb(X_cells, m)
for i = 1:length(X_cells)
    kinova_s1.jacobian(X_cells{i}, m, 'body');
end
end

function run_stock_id(Q, DQ, DDQ, m)
for i = 1:size(Q, 2)
    stock_inv_dyn(Q(:, i), DQ(:, i), DDQ(:, i), m);
end
end

function run_a_id(Q, DQ, DDQ, m)
for i = 1:size(Q, 2)
    kinova.inverse_dynamics(Q(:, i), DQ(:, i), DDQ(:, i), m);
end
end

function run_balg_id(Q, DQ, DDQ, m)
for i = 1:size(Q, 2)
    q = Q(:, i);
    kinova_s1.inverse_dynamics([cos(q'); sin(q')], DQ(:, i), DDQ(:, i), m);
end
end

function run_b_id(X_cells, DQ, DDQ, m)
for i = 1:length(X_cells)
    kinova_s1.inverse_dynamics(X_cells{i}, DQ(:, i), DDQ(:, i), m);
end
end

function run_stock_dyn(Q, DQ, m)
for i = 1:size(Q, 2)
    stock_dyn(Q(:, i), DQ(:, i), m);
end
end

function run_a_dyn(Q, DQ, m)
for i = 1:size(Q, 2)
    kinova.dynamics(Q(:, i), DQ(:, i), m);
end
end

function run_balg_dyn(Q, DQ, m)
for i = 1:size(Q, 2)
    q = Q(:, i);
    kinova_s1.dynamics([cos(q'); sin(q')], DQ(:, i), m);
end
end

function run_b_dyn(X_cells, DQ, m)
for i = 1:length(X_cells)
    kinova_s1.dynamics(X_cells{i}, DQ(:, i), m);
end
end

function run_stock_fd(Q, DQ, TAU, m)
for i = 1:size(Q, 2)
    stock_fwd_dyn(Q(:, i), DQ(:, i), TAU(:, i), m);
end
end

function run_a_fd(Q, DQ, TAU, m)
for i = 1:size(Q, 2)
    kinova.forward_dynamics(Q(:, i), DQ(:, i), TAU(:, i), m);
end
end

function run_balg_fd(Q, DQ, TAU, m)
for i = 1:size(Q, 2)
    q = Q(:, i);
    kinova_s1.forward_dynamics([cos(q'); sin(q')], DQ(:, i), TAU(:, i), m);
end
end

function run_b_fd(X_cells, DQ, TAU, m)
for i = 1:length(X_cells)
    kinova_s1.forward_dynamics(X_cells{i}, DQ(:, i), TAU(:, i), m);
end
end

function run_from_q(Q)
for i = 1:size(Q, 2)
    kinova_s1.from_q(Q(:, i));
end
end

function run_to_q(X_cells, m)
for i = 1:length(X_cells)
    kinova_s1.to_q(X_cells{i}, m);
end
end

function run_exp_s1(DQ, X_cells)
for i = 1:length(X_cells)
    kinova_s1.exp_s1(0.001 * DQ(:, i), X_cells{i}, true);
end
end

function tau = stock_inv_dyn(q, dq, ddq, m)
tau = InverseDynamics(q, dq, ddq, m.gravity, zeros(6, 1), m.Mlist, m.Glist, m.Slist) + ...
    m.Japp * ddq + kinova.friction(dq, m);
end

function [M, b] = stock_dyn(q, dq, m)
M = MassMatrix(q, m.Mlist, m.Glist, m.Slist) + m.Japp;
b = InverseDynamics(q, dq, zeros(6, 1), m.gravity, zeros(6, 1), m.Mlist, m.Glist, m.Slist) + ...
    kinova.friction(dq, m);
end

function ddq = stock_fwd_dyn(q, dq, tau, m)
[M, b] = stock_dyn(q, dq, m);
ddq = M \ (tau - b);
end
