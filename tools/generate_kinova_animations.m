function generate_kinova_animations()
%GENERATE_KINOVA_ANIMATIONS Render and export 3D animated GIFs of Kinova Gen3:
%   - Side-by-side comparison: Version A (q in R^6) vs. Version B (X in (S1)^6)
%   - Individual animations for Version A and Version B
%
% Renders the official manufacturer visual meshes (STL) with end-effector traces,
% reference path, and dynamic time/energy/norm HUD overlays.
%
% Outputs:
%   - docs/assets/kinova_comparison_side_by_side.gif
%   - docs/assets/kinova_version_a.gif
%   - docs/assets/kinova_version_b.gif
%   - results/ copies for local reproducibility

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
setup_kinova();

assets_dir = fullfile(root, 'docs', 'assets');
if ~isfolder(assets_dir)
    mkdir(assets_dir);
end
results_dir = fullfile(root, 'results');
if ~isfolder(results_dir)
    mkdir(results_dir);
end

fprintf('========================================================================\n');
fprintf(' GENERATING KINOVA GEN3 3D ANIMATIONS (VERSION A vs VERSION B)\n');
fprintf('========================================================================\n\n');

m_a = kinova.model();
m_b = kinova_s1.model();
c = kinova.config();
c.save_results = false;

fprintf('[1/3] Simulating closed-loop 3.6 s trajectory for Version A and Version B...\n');
log_a = kinova.simulate(m_a, c);
log_b = kinova_s1.simulate(m_b, c);

% Sample 60 frames across 901 points (step = 15) for smooth ~16.7 fps looping GIF
frame_indices = 1:15:numel(log_a.t);
if frame_indices(end) ~= numel(log_a.t)
    frame_indices = [frame_indices, numel(log_a.t)];
end
n_frames = length(frame_indices);
delay_time = 0.06; % ~16.7 fps

% -------------------------------------------------------------------------
% 1. Render Side-by-Side Comparison GIF
% -------------------------------------------------------------------------
fprintf('[2/3] Rendering side-by-side comparison animation (%d frames)...\n', n_frames);
fig_side = figure('Visible', 'off', 'Color', 'w', 'Position', [50, 50, 1020, 540]);
tlayout = tiledlayout(fig_side, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax_a = nexttile(tlayout, 1);
ax_b = nexttile(tlayout, 2);

% Pre-plot static trajectory reference traces
plot3(ax_a, log_a.position_ref(1, :), log_a.position_ref(2, :), log_a.position_ref(3, :), ...
    '--', 'Color', [0.05, 0.48, 0.58], 'LineWidth', 1.8);
hold(ax_a, 'on');
h_trace_a = plot3(ax_a, NaN, NaN, NaN, '-', 'Color', [0.95, 0.46, 0.17], 'LineWidth', 2.0);

plot3(ax_b, log_b.position_ref(1, :), log_b.position_ref(2, :), log_b.position_ref(3, :), ...
    '--', 'Color', [0.05, 0.48, 0.58], 'LineWidth', 1.8);
hold(ax_b, 'on');
h_trace_b = plot3(ax_b, NaN, NaN, NaN, '-', 'Color', [0.15, 0.68, 0.38], 'LineWidth', 2.0);

h_robot_a = [];
h_robot_b = [];

side_gif_path_assets = fullfile(assets_dir, 'kinova_comparison_side_by_side.gif');
side_gif_path_results = fullfile(results_dir, 'kinova_comparison_side_by_side.gif');

for idx = 1:n_frames
    k = frame_indices(idx);

    % Update Robot A
    h_robot_a = kinova.draw_robot(log_a.q(:, k), m_a, ax_a, h_robot_a);
    set(h_trace_a, 'XData', log_a.position(1, 1:k), ...
                   'YData', log_a.position(2, 1:k), ...
                   'ZData', log_a.position(3, 1:k));
    title(ax_a, { ...
        '\bf\fontsize{11}Version A: Joint Coordinates (q \in R^6)', ...
        sprintf('\\rm\\fontsize{9.5}t = %.2f s | r_E = %.1e J', log_a.t(k), abs(log_a.energy_residual(k)))}, ...
        'Interpreter', 'tex');

    % Update Robot B
    h_robot_b = kinova_s1.draw_robot(log_b.X(:, :, k), m_b, ax_b, h_robot_b);
    set(h_trace_b, 'XData', log_b.position(1, 1:k), ...
                   'YData', log_b.position(2, 1:k), ...
                   'ZData', log_b.position(3, 1:k));
    title(ax_b, { ...
        '\bf\fontsize{11}Version B: Global Lie-Group on (S^1)^6', ...
        sprintf('\\rm\\fontsize{9.5}t = %.2f s | r_E = %.1e J | ||x_i||_{err} = %.0e', ...
            log_b.t(k), abs(log_b.energy_residual(k)), log_b.max_unit_norm_error(k))}, ...
        'Interpreter', 'tex');

    drawnow;
    frame = getframe(fig_side);
    im = frame2im(frame);
    [A, map] = rgb2ind(im, 256, 'nodither');

    if idx == 1
        imwrite(A, map, side_gif_path_assets, 'gif', 'LoopCount', Inf, 'DelayTime', delay_time);
    else
        imwrite(A, map, side_gif_path_assets, 'gif', 'WriteMode', 'append', 'DelayTime', delay_time);
    end
end
close(fig_side);
copyfile(side_gif_path_assets, side_gif_path_results);
fprintf('  -> Saved: %s\n', side_gif_path_assets);

% -------------------------------------------------------------------------
% 2. Render Individual Version A & Version B GIFs
% -------------------------------------------------------------------------
fprintf('[3/3] Rendering individual Version A and Version B animations...\n');
render_single_gif(log_a, m_a, frame_indices, delay_time, 'A', ...
    fullfile(assets_dir, 'kinova_version_a.gif'), fullfile(results_dir, 'kinova_version_a.gif'));
render_single_gif(log_b, m_b, frame_indices, delay_time, 'B', ...
    fullfile(assets_dir, 'kinova_version_b.gif'), fullfile(results_dir, 'kinova_version_b.gif'));

fprintf('\nAll animations successfully rendered and persisted!\n');
end

function render_single_gif(log_data, m, frame_indices, delay_time, mode, out_assets, out_results)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 560, 500]);
ax = axes(fig);
hold(ax, 'on');

if strcmp(mode, 'A')
    trace_col = [0.95, 0.46, 0.17];
    title_str = '\bf\fontsize{11}Version A: Joint Coordinates (q \in R^6)';
else
    trace_col = [0.15, 0.68, 0.38];
    title_str = '\bf\fontsize{11}Version B: Global Lie-Group on (S^1)^6';
end

plot3(ax, log_data.position_ref(1, :), log_data.position_ref(2, :), log_data.position_ref(3, :), ...
    '--', 'Color', [0.05, 0.48, 0.58], 'LineWidth', 1.8);
h_trace = plot3(ax, NaN, NaN, NaN, '-', 'Color', trace_col, 'LineWidth', 2.2);

h_robot = [];
n_frames = length(frame_indices);

for idx = 1:n_frames
    k = frame_indices(idx);
    if strcmp(mode, 'A')
        h_robot = kinova.draw_robot(log_data.q(:, k), m, ax, h_robot);
    else
        h_robot = kinova_s1.draw_robot(log_data.X(:, :, k), m, ax, h_robot);
    end
    set(h_trace, 'XData', log_data.position(1, 1:k), ...
                 'YData', log_data.position(2, 1:k), ...
                 'ZData', log_data.position(3, 1:k));
    title(ax, {title_str, sprintf('\\rm\\fontsize{9.5}t = %.2f s | r_E = %.1e J', ...
        log_data.t(k), abs(log_data.energy_residual(k)))}, 'Interpreter', 'tex');

    drawnow;
    frame = getframe(fig);
    im = frame2im(frame);
    [A, map] = rgb2ind(im, 256, 'nodither');

    if idx == 1
        imwrite(A, map, out_assets, 'gif', 'LoopCount', Inf, 'DelayTime', delay_time);
    else
        imwrite(A, map, out_assets, 'gif', 'WriteMode', 'append', 'DelayTime', delay_time);
    end
end
close(fig);
copyfile(out_assets, out_results);
fprintf('  -> Saved: %s\n', out_assets);
end
