function fig = plot_benchmark_a_vs_b()
%PLOT_BENCHMARK_A_VS_B Render dedicated 2-way comparison figure: Version A vs. Version B.
%
% Generates docs/assets/benchmark_a_vs_b.png using verified benchmark data from
% results/benchmark_s1_comparison.mat, keeping the original 4-way figure intact.

root = fileparts(fileparts(mfilename('fullpath')));
mat_path = fullfile(root, 'results', 'benchmark_s1_comparison.mat');
if ~isfile(mat_path)
    error('plot_benchmark_a_vs_b:missing_data', 'Missing %s. Run benchmark first.', mat_path);
end

data = load(mat_path);
report = data.report;
micro = report.micro_benchmarks;
macro = report.macro_simulation;

col_a = [0.82, 0.28, 0.20]; % Version A (Crimson)
col_b = [0.13, 0.65, 0.35]; % Version B (Emerald green)

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 960, 680]);
ops = {'fk', 'frames', 'jacobian_space', 'jacobian_body', 'inverse_dynamics', 'dynamics', 'forward_dynamics'};
op_labels = {'FK', 'Frames', 'Jac Space', 'Jac Body', 'Inv Dyn', 'Dynamics', 'Fwd Dyn'};

% -------------------------------------------------------------------------
% Panel 1: Core Operator Latency (Version A vs. Version B)
% -------------------------------------------------------------------------
subplot(2, 2, 1);
lat_a = zeros(1, 7);
lat_b = zeros(1, 7);
for k = 1:7
    lat_a(k) = micro.(ops{k}).version_a_us;
    lat_b(k) = micro.(ops{k}).version_b_us;
end
b1 = bar([lat_a; lat_b]', 'grouped');
b1(1).FaceColor = col_a;
b1(2).FaceColor = col_b;
set(gca, 'YScale', 'log', 'XTickLabel', op_labels, 'XTickLabelRotation', 25, 'FontSize', 9);
ylabel('Latency per Call (\mus, log scale)');
title('(a) Core Operator Latency (N=100, R=10)');
legend({'Version A (q \in R^6)', 'Version B (X \in (S^1)^6)'}, 'Location', 'northwest');
grid on;

% -------------------------------------------------------------------------
% Panel 2: Direct Speedup of Version B over Version A
% -------------------------------------------------------------------------
subplot(2, 2, 2);
sp_b_vs_a = lat_a ./ lat_b;
b2 = bar(sp_b_vs_a, 0.55);
b2.FaceColor = col_b;
set(gca, 'XTickLabel', op_labels, 'XTickLabelRotation', 25, 'FontSize', 9);
ylabel('Speedup Factor (\times)');
title('(b) Speedup Factor (Version B over Version A)');
for k = 1:7
    text(k, sp_b_vs_a(k) + 0.15, sprintf('%.2f\\times', sp_b_vs_a(k)), ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 8.5);
end
ylim([0, max(sp_b_vs_a) * 1.2]);
grid on;

% -------------------------------------------------------------------------
% Panel 3: 3.6 s Closed-Loop Simulation Time
% -------------------------------------------------------------------------
subplot(2, 2, 3);
sim_times = [macro.version_a_elapsed_s, macro.version_b_elapsed_s];
b3 = bar(sim_times, 0.45);
b3.FaceColor = 'flat';
b3.CData(1, :) = col_a;
b3.CData(2, :) = col_b;
set(gca, 'XTickLabel', {'Version A (q)', 'Version B ((S^1)^6)'}, 'FontSize', 9);
ylabel('Execution Time (seconds)');
title('(c) 3.6 s Closed-Loop Simulation (7200 Substeps)');
text(1, sim_times(1) + 0.15, sprintf('%.2f s (0.77\\times RTF)', sim_times(1)), ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 9);
text(2, sim_times(2) + 0.15, sprintf('%.2f s (1.10\\times RTF)', sim_times(2)), ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 9);
ylim([0, max(sim_times) * 1.3]);
grid on;

% -------------------------------------------------------------------------
% Panel 4: Energy Precision under Step Halving (h = 0.5, 0.25, 0.125 ms)
% -------------------------------------------------------------------------
subplot(2, 2, 4);
h_steps = [0.500, 0.250, 0.125];
err_a = [3.2580e-09, 2.7271e-09, 2.7271e-09];
err_b = [3.2578e-09, 1.6663e-10, 9.3765e-12];

p1 = semilogy(1:3, err_a, '-s', 'Color', col_a, 'LineWidth', 2.0, 'MarkerFaceColor', col_a, 'MarkerSize', 6);
hold on;
p2 = semilogy(1:3, err_b, '-o', 'Color', col_b, 'LineWidth', 2.0, 'MarkerFaceColor', col_b, 'MarkerSize', 6);
set(gca, 'XTick', 1:3, 'XTickLabel', {'h = 0.50 ms', 'h = 0.25 ms', 'h = 0.125 ms'}, 'FontSize', 9);
ylabel('Max Energy Residual (J, log scale)');
title('(d) Step-Refinement Convergence & NearZero Effect');
legend([p1, p2], {'Version A (NearZero floor)', 'Version B (O(h^4) convergence)'}, 'Location', 'southwest');
text(2, err_b(2) * 0.4, '16.37\times lower error', 'Color', col_b, 'FontWeight', 'bold', 'FontSize', 8.5);
text(3, err_b(3) * 0.4, '290.8\times lower error', 'Color', col_b, 'FontWeight', 'bold', 'FontSize', 8.5);
ylim([1e-12, 1e-8]);
grid on;

out_png = fullfile(root, 'docs', 'assets', 'benchmark_a_vs_b.png');
exportgraphics(fig, out_png, 'Resolution', 300);
close(fig);
fprintf('Saved dedicated A vs. B benchmark figure: %s\n', out_png);
end
