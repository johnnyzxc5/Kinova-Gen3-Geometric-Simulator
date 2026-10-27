function plot_results(log, m, out)
%PLOT_RESULTS Scientific trajectory/effort/energy plots for (S1)^6 global simulation.
if ~isfolder(out), mkdir(out); end
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1250, 800]);
cleanup = onCleanup(@() delete(fig));
tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax = nexttile;
plot(ax, log.t, log.q * 180 / pi, 'LineWidth', 1.25); hold(ax, 'on');
ax.ColorOrderIndex = 1;
plot(ax, log.t, log.qref * 180 / pi, '--', 'LineWidth', 0.7); grid(ax, 'on');
title(ax, 'Joint motion (S^1)^6 lifted'); xlabel(ax, 'Time (s)'); ylabel(ax, 'Angle (deg)');
legend(ax, {'q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6'}, 'Location', 'eastoutside');

ax = nexttile;
plot(ax, log.t, (log.q - log.qref) * 180 / pi, 'LineWidth', 1.2); grid(ax, 'on');
title(ax, 'Tracking error'); xlabel(ax, 'Time (s)'); ylabel(ax, 'Error (deg)');

ax = nexttile;
plot(ax, log.t, log.tau_applied, 'LineWidth', 1.2); grid(ax, 'on');
title(ax, 'Applied joint effort'); xlabel(ax, 'Time (s)'); ylabel(ax, 'Torque (N m)');

ax = nexttile;
plot(ax, log.t, log.energy_residual, 'Color', [.07, .43, .54], 'LineWidth', 1.4); grid(ax, 'on');
title(ax, 'Mechanical energy balance residual'); xlabel(ax, 'Time (s)'); ylabel(ax, 'Residual (J)');

sgtitle(fig, 'Kinova Gen3 | Global Lie-Group Dynamics on (S^1)^6', 'FontWeight', 'bold');
exportgraphics(fig, fullfile(out, 'simulation_diagnostics_s1.png'), 'Resolution', 160);

fig2 = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1000, 760]);
cleanup2 = onCleanup(@() delete(fig2));
ax2 = axes(fig2);
kinova_s1.draw_robot(log.X(:, :, end), m, ax2);
hold(ax2, 'on');
plot3(ax2, log.position_ref(1, :), log.position_ref(2, :), log.position_ref(3, :), ...
    '--', 'Color', [.05, .48, .58], 'LineWidth', 1.7);
plot3(ax2, log.position(1, :), log.position(2, :), log.position(3, :), ...
    '-', 'Color', [.95, .46, .17], 'LineWidth', 1.2);
title(ax2, {'KINOVA GEN3 | GLOBAL (S^1)^6 DYNAMICS', 'Official visual meshes + Lie-group RK4-MK integration'}, 'FontWeight', 'bold');
exportgraphics(fig2, fullfile(out, 'kinova_s1_preview.png'), 'Resolution', 160);
end
