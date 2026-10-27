function report = test_s1_simulation_consistency()
%TEST_S1_SIMULATION_CONSISTENCY Comprehensive verification of (S1)^6 simulation,
% Lie-group RK4-MK integration, multi-turn winding tracking, forcing, and energy balance.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
setup_kinova();

m_a = kinova.model();
m_b = kinova_s1.model();

% -------------------------------------------------------------------------
% 1. Nominal 3.6 s Trajectory Parity (Version A vs Version B) & Step Refinement
% -------------------------------------------------------------------------
c = kinova.config();
c.save_results = false;

log_a = kinova.simulate(m_a, c);
log_b = kinova_s1.simulate(m_b, c);

nom_q_parity = max(abs(log_a.q - log_b.q), [], 'all');
nom_q_end_parity = max(abs(log_a.q(:, end) - log_b.q(:, end)));
nom_dq_parity = max(abs(log_a.dq - log_b.omega), [], 'all');
nom_pos_parity_mm = max(vecnorm(log_a.position - log_b.position)) * 1000;
nom_tau_parity = max(abs(log_a.tau_applied - log_b.tau_applied), [], 'all');
nom_energy_err_a = log_a.metrics.energy_balance_max_error_J;
nom_energy_err_b = log_b.metrics.energy_balance_max_error_J;
nom_max_unit_norm_err = log_b.metrics.max_unit_norm_error;

assert(nom_q_parity < 1e-8, 'kinova_s1:sim', 'Nominal q trajectory parity failed');
assert(nom_q_end_parity < 1e-12, 'kinova_s1:sim', 'Nominal final q parity failed');
assert(nom_dq_parity < 1e-6, 'kinova_s1:sim', 'Nominal omega trajectory parity failed');
assert(nom_pos_parity_mm < 1e-3, 'kinova_s1:sim', 'Nominal position parity failed');
assert(nom_tau_parity < 2e-5, 'kinova_s1:sim', 'Nominal torque parity failed');
assert(nom_max_unit_norm_err < 1e-15, 'kinova_s1:sim', 'Unit norm violation on S^1');
assert(nom_energy_err_b < 5e-9, 'kinova_s1:sim', 'Nominal energy balance error exceeds bound');
assert(log_b.metrics.maximum_stage_limit_violation_rad == 0, 'kinova_s1:sim', 'Nominal limit violation');

% Step refinement check (h = 0.5 ms vs h = 0.25 ms on (S1)^6)
c_fine = c;
c_fine.substeps = 2 * c.substeps;
log_b_fine = kinova_s1.simulate(m_b, c_fine);
nom_energy_err_b_fine = log_b_fine.metrics.energy_balance_max_error_J;
rk4_order_ratio_b = nom_energy_err_b / nom_energy_err_b_fine;

step_q_diff = max(abs(log_b.q - log_b_fine.q), [], 'all');
step_dq_diff = max(abs(log_b.omega - log_b_fine.omega), [], 'all');
assert(step_q_diff < 1e-5 && step_dq_diff < 1e-4, 'kinova_s1:sim', 'Step refinement failed');
assert(rk4_order_ratio_b > 15.0, 'kinova_s1:sim', 'Expected 4th-order O(h^4) energy convergence on (S1)^6');
assert(log_b_fine.metrics.max_unit_norm_error < 1e-15, 'kinova_s1:sim', 'Fine step unit norm violation');

% -------------------------------------------------------------------------
% 2. Multi-Turn Winding Trajectory Simulation on Continuous Joints (1, 4, 6)
% -------------------------------------------------------------------------
c_turn = kinova.config();
c_turn.save_results = false;
c_turn.motion_time = 4.0;
c_turn.end_time = 4.4;
c_turn.q0 = [0; 0.35; -0.7; 0; 0.35; 0];
% Joint 1 makes two complete revolutions (4*pi rad) while bounded joints stay within limits
c_turn.qgoal = [4 * pi; 0.5; -0.8; 2 * pi; 0.4; -2 * pi];

log_turn_a = kinova.simulate(m_a, c_turn);
log_turn_b = kinova_s1.simulate(m_b, c_turn);

turn_q_parity = max(abs(log_turn_a.q - log_turn_b.q), [], 'all');
turn_end_diff = max(abs(log_turn_b.q(:, end) - c_turn.qgoal));
assert(turn_q_parity < 1e-8, 'kinova_s1:sim', 'Multi-turn trajectory parity failed');
assert(turn_end_diff < 5e-3, 'kinova_s1:sim', 'Multi-turn goal arrival failed');
assert(log_turn_b.metrics.maximum_stage_limit_violation_rad == 0, 'kinova_s1:sim', 'Multi-turn limit violation');
assert(log_turn_b.metrics.energy_balance_max_error_J < 1e-8, 'kinova_s1:sim', 'Multi-turn energy balance failed');
assert(abs(log_turn_a.metrics.energy_balance_max_error_J - log_turn_b.metrics.energy_balance_max_error_J) < 1e-11, ...
    'kinova_s1:sim', 'Multi-turn energy balance parity failed');
assert(log_turn_b.metrics.max_unit_norm_error < 1e-15, 'kinova_s1:sim', 'Multi-turn unit norm error');

% 2b. Large-Lag Winding Preservation (|qref - q| > pi) vs Modular S^1 Control
c_lag = kinova.config();
c_lag.save_results = false;
c_lag.motion_time = 0.04; % Fast ramp creates > pi rad tracking lag on Joint 6
c_lag.end_time = 0.20;
c_lag.q0 = [0; 0.35; -0.7; 0; 0.35; 0];
c_lag.qgoal = [0; 0.35; -0.7; 0; 0.35; 1.5 * pi];
log_lag_a = kinova.simulate(m_a, c_lag);
log_lag_b = kinova_s1.simulate(m_b, c_lag);
lag_q_parity = max(abs(log_lag_a.q - log_lag_b.q), [], 'all');
assert(lag_q_parity < 1e-8, 'kinova_s1:sim', 'Winding slip occurred under > pi tracking error');
assert(log_lag_b.q(6, end) > 0, 'kinova_s1:sim', 'Unwrapped control must advance in +theta direction');

c_mod = c_lag;
c_mod.continuous_control_mode = 'modular_s1';
log_mod_b = kinova_s1.simulate(m_b, c_mod);
assert(log_mod_b.q(6, end) < 0, 'kinova_s1:sim', 'Modular S^1 control must take shortest negative arc for +1.5*pi');

% -------------------------------------------------------------------------
% 3. Perturbation Testing: External Wrench (Constant & Continuous-Angle Dependent)
% -------------------------------------------------------------------------
c_forced = kinova.config();
c_forced.motion_time = 0.08;
c_forced.end_time = 0.12;
c_forced.qgoal = c_forced.q0;
c_forced.save_results = false;
c_forced.external_wrench_function = @(~,~,~)[.03;-.02;.01;.2;-.1;.15];
c_forced.effectiveness_function = @(t)[1; 1 - 0.1 * (t >= 0.04); 1; 1; 1; 1];

log_forced_a = kinova.simulate(m_a, c_forced);
log_forced_b = kinova_s1.simulate(m_b, c_forced);

forced_q_parity = max(abs(log_forced_a.q - log_forced_b.q), [], 'all');
forced_env_work_diff = abs(log_forced_a.work_environment(end) - log_forced_b.work_environment(end));
forced_energy_parity = abs(log_forced_a.metrics.energy_balance_max_error_J - log_forced_b.metrics.energy_balance_max_error_J);

assert(forced_q_parity < 1e-8, 'kinova_s1:sim', 'Forced simulation parity failed');
assert(forced_env_work_diff < 1e-12, 'kinova_s1:sim', 'External work parity failed');
assert(forced_energy_parity < 1e-10, 'kinova_s1:sim', 'Forced energy balance parity failed');
assert(log_forced_b.metrics.energy_balance_max_error_J < 1e-5, 'kinova_s1:sim', 'Forced energy balance violation');

% 3b. External wrench dependent on unwrapped continuous joint angle crossing pi
c_spring = kinova.config();
c_spring.motion_time = 0.20;
c_spring.end_time = 0.24;
c_spring.q0 = [0; 0.35; -0.7; 0; 0.35; 2.8];
c_spring.qgoal = [0; 0.35; -0.7; 0; 0.35; 3.5]; % crosses pi = 3.14159...
c_spring.save_results = false;
c_spring.external_wrench_function = @(~, q, ~)[0; 0; -0.25 * q(6); 0; 0; 0];

log_spring_a = kinova.simulate(m_a, c_spring);
log_spring_b = kinova_s1.simulate(m_b, c_spring);
spring_q_parity = max(abs(log_spring_a.q - log_spring_b.q), [], 'all');
assert(spring_q_parity < 1e-8, 'kinova_s1:sim', 'Continuous-angle external wrench parity failed across pi');

% -------------------------------------------------------------------------
% 4. Saturation Clamping Parity
% -------------------------------------------------------------------------
c_sat = kinova.config();
c_sat.end_time = 0.012;
c_sat.motion_time = 0.008;
c_sat.save_results = false;
c_sat.control_function = @(~,~,~,~,~,~,~) 1000 * ones(6, 1);

log_sat_a = kinova.simulate(m_a, c_sat);
log_sat_b = kinova_s1.simulate(m_b, c_sat);

assert(all(log_sat_b.saturated, 'all'), 'kinova_s1:sim', 'All joints must saturate');
assert(max(abs(log_sat_b.tau_command) ./ m_b.taumax, [], 'all') <= 1 + 1e-12, 'kinova_s1:sim', 'Torque clamping');
assert(norm(log_sat_a.tau_command - log_sat_b.tau_command, 'fro') < 1e-12, 'kinova_s1:sim', 'Saturation parity');

report = struct(...
    'nom_q_parity_rad', nom_q_parity, ...
    'nom_q_end_parity_rad', nom_q_end_parity, ...
    'nom_pos_parity_mm', nom_pos_parity_mm, ...
    'nom_energy_err_b_J', nom_energy_err_b, ...
    'fine_energy_err_b_J', nom_energy_err_b_fine, ...
    'rk4_order_ratio_b', rk4_order_ratio_b, ...
    'turn_q_parity_rad', turn_q_parity, ...
    'lag_q_parity_rad', lag_q_parity, ...
    'forced_q_parity_rad', forced_q_parity, ...
    'spring_q_parity_rad', spring_q_parity, ...
    'max_unit_norm_err', max([nom_max_unit_norm_err, log_turn_b.metrics.max_unit_norm_error]), ...
    'pass', true);

disp('test_s1_simulation_consistency: PASS');
disp(report);
end
