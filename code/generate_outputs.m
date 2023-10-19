config;
init_hinf_controller;

%% Plot scaled Plant OUTPLOT
if 0
    figure;
    bcol = copper(length(LPV_CONTROLLER_WIND_SPEEDS) + 1); % Define the color order based on the number of models
    bcol = flip(bcol(2:end, :), 1);
    % https://www.mathworks.com/help/matlab/colors-1.html?s_tid=CRUX_lftnav

    omega = logspace(-2, 2, 300);
    
    for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
        % Plot baseline and tuned controllers
        x = Plant(:, :, c_ws_idx);
        x.InputName = {'$\beta_d$ Control Input', '$\beta_q$ Control Input'};
        x.OutputName = {'$M_d$ Output', '$M_1$ Output'};
        bodeplot(x, 'b', omega, bode_plot_opt);

        % Find handles of all lines in the figure that have the color blue
        blineHandle = findobj(gcf,'Type','line','-and','Color','b');

        % Change the color to the one you defined
        set(blineHandle,'Color', bcol(c_ws_idx,:));
        set(blineHandle, 'LineWidth', 2);
        
        hold on
    end

    axh = findall(gcf, 'type', 'axes');
    xline(axh(3), omega_1P_rad * HARMONICS, 'k--', 'LineWidth', 2);
    xline(axh(5), omega_1P_rad * HARMONICS, 'k--', 'LineWidth', 2);
    xline(axh(7), omega_1P_rad * HARMONICS, 'k--', 'LineWidth', 2);
    xline(axh(9), omega_1P_rad * HARMONICS, 'k--', 'LineWidth', 2);
    
    axh(7).XTick = axh(7).XTick(1:end-1);
    
    % labels = {};
    % obj = findobj(gcf,'Type','hggroup');
    % for idx = 1:numel(obj)
    %     for jdx = 1:numel(obj(idx).Children)
    %         % obj(idx).Children(jdx).LineWidth = 2;
    %         labels{end + 1} = '';
    %     end
    % end
    
    harmonic_label = '';
    for h = 1:length(HARMONICS)
        if h < length(HARMONICS)
            harmonic_label = [harmonic_label, num2str(HARMONICS(h)), 'P, '];
        else
            harmonic_label = [harmonic_label, num2str(HARMONICS(h)), 'P Frequencies'];
        end
    end

    labels = {};
    obj = findobj(axh(9),'Type','hggroup');
    for idx = 1:numel(obj)
        for jdx = 1:numel(obj(idx).Children)
            labels{end + 1} = '';
        end
    end

    labels{end + 1} = harmonic_label;

    legend(axh(9), ...
        labels, ...
        'Interpreter', 'latex', 'FontSize', SUBPLOT_BIG_FONT_SIZE, ...
        'Position', [0.14205527750651 0.714450867052023 0.22694472249349 0.046242774566474]);

    colormap(bcol);
    for ax_idx = [3]
        cbar = colorbar(axh(ax_idx), ...
            'TickLabelsMode', 'manual', 'TicksMode', 'manual', ...
            'Ticks', linspace(0, 1, length(LPV_CONTROLLER_WIND_SPEEDS)), ...
        'TickLabels', arrayfun(@num2str, LPV_CONTROLLER_WIND_SPEEDS, 'UniformOutput', 0), ...
        'TickLabelInterpreter', 'latex', 'FontSize', SUBPLOT_SMALL_FONT_SIZE);
        cbar.Label.String = 'Linearized Wind Speed [m/s]';
        cbar.Label.Interpreter = 'latex';
        clim([LPV_CONTROLLER_WIND_SPEEDS(1) LPV_CONTROLLER_WIND_SPEEDS(end)]);
    end

    set(gcf, 'Position', [0 0 1500 900]);
    axh(7).Title.FontSize = SUBPLOT_BIG_FONT_SIZE;
    axh(9).Title.FontSize = SUBPLOT_BIG_FONT_SIZE;
    axh(9).YLabel.FontSize = SUBPLOT_BIG_FONT_SIZE;
    axh(5).YLabel.FontSize = SUBPLOT_BIG_FONT_SIZE;

    savefig(gcf, fullfile(fig_dir, 'Plant_bodemag.fig'));
    saveas(gcf, fullfile(fig_dir, 'Plant_bodemag.png'));

   end

%% Plot tuned controller for this weighting case OUTPLOT FIGURE 8
if 0
    % Plot baseline OUTPLOT
    % load(fullfile(mat_save_dir, 'K0'));

    % figure;
    % x = K_tmp1;
    % x.OutputName = {'$\beta_d$ Control Input', '$\beta_q$ Control Input'};
    % x.InputName = {'$M_d$ Tracking Error', '$M_q$ Tracking Error'};
    % bodeplot(x, bode_plot_opt);
    % 
    % axh = findall(gcf, 'type', 'axes');
    % xline(axh(3), omega_1P_rad * HARMONICS, 'k--', 'LineWidth', 2);
    % % xline(axh(5), omega_1P_rad * HARMONICS, 'k--', 'LineWidth', 2);
    % % xline(axh(7), omega_1P_rad * HARMONICS, 'k--', 'LineWidth', 2);
    % xline(axh(9), omega_1P_rad * HARMONICS, 'k--', 'LineWidth', 2);
    % 
    % obj = findobj(gcf,'Type','hggroup');
    % for idx = 1:numel(obj)
    %     for jdx = 1:numel(obj(idx).Children)
    %         obj(idx).Children(jdx).LineWidth = 2;
    %     end
    % end
    % 
    % set(gcf, 'Position', [0 0 1500 900]);
    % hold off;

    % savefig(gcf, fullfile(fig_dir, 'BaselineController_bodemag.fig'));
    % saveas(gcf, fullfile(fig_dir, 'BaselineController_bodemag.png'));

    load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_wu', '_full_controller_cases']));
    load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_wu', '_Controllers_case_list.mat']));
    
    omega = logspace(-2, 2, 300);

    cc = find(case_basis.WindSpeedIndex.x == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED));
    % first case = black = highest Wu_gain
    % make plot for each controller type, comparing to baseline
    n_weighting_cases = length(Controllers_case_list);
    controller_types = fieldnames(case_basis.W1Gain);
    for f = 1:length(controller_types)
        % ctrl_cond = contains(Controllers_case_table.("Case Desc."), f{1});

        figure(f);
        Wu_gains = [];
        ctrl_type = controller_types{f};

        clear controllers;
        % 
        for w_idx = 1:n_weighting_cases

            wu = Controllers_case_list(w_idx).WuGain.(ctrl_type).Numerator{1,1}(1);
            Wu_gains = [Wu_gains,  wu];
        
            K_tmp = Controllers_case_list(w_idx).Controller_scaled.(ctrl_type)(:, :, cc);
            K_tmp.InputName = {'$M_d$ Tracking Error', '$M_q$ Tracking Error'};
            K_tmp.OutputName = {'$\beta_d$ Control Input', '$\beta_q$ Control Input'};
            controllers(:, :, w_idx) = K_tmp;

        end
        % controllers(:, :, w_idx + 1) = K0; % add baseline controller

        bcol = copper(n_weighting_cases + 1); % Define the color order based on the number of models, darkest (0.2500    0.1562    0.0995) corresponding to highest Wu
        bcol = bcol(2:end, :); % remove black

        % Plot baseline and tuned controllers
        bodeplot(controllers, 'b', omega, bode_plot_opt); hold on;
        bodeplot(K0, 'r', omega, bode_plot_opt); hold on;

        % Find handles of all lines in the figure that have the color blue
        axh = findall(gcf, 'type', 'axes');
        
        % set colour of tuned controllers
        
        for ax_idx = [3, 5, 7, 9]
            % xline(axh(ax_idx), omega_1P_rad * HARMONICS);
            blineHandle = findobj(axh(ax_idx), 'Type', 'line', '-and', 'Color', 'b');
            for w_idx = 1:n_weighting_cases
                % Change the color of the tuned controllers to the one you defined
                set(blineHandle(w_idx), 'Color', bcol(w_idx, :), 'LineWidth', 2);
            end
            % set colour of baseline controller
            rlineHandle = findobj(axh(ax_idx), 'Type', 'line', '-and', 'Color', 'r');
            set(rlineHandle(1), 'Color', 'k', 'LineStyle', '--', 'LineWidth', 2);
        end

        % harmonic_label = '';
        % for h = 1:length(HARMONICS)
        %     if h < length(HARMONICS)
        %         harmonic_label = [harmonic_label, num2str(HARMONICS(h)), 'P, '];
        %     else
        %         harmonic_label = [harmonic_label, num2str(HARMONICS(h)), 'P Frequencies'];
        %     end
        % end
    
        labels = {};
        obj = findobj(axh(9),'Type','hggroup');
        for idx = 1:2
            labels{end + 1} = '';
            % for jdx = 1:numel(obj(idx).Children)
            %     labels{end + 1} = '';
            % end
        end
    
        % labels{end + 1} = harmonic_label;
        labels{end+1} = ['Baseline ', '$K_0$'];
        plot(axh(9), NaN, NaN, '--k', 'LineWidth', 2);
        legend(axh(9), ...
            labels, ...
            'Interpreter', 'latex', 'FontSize', SUBPLOT_BIG_FONT_SIZE);
    
        colormap(bcol);
        for ax_idx = [3]
            cbar = colorbar(axh(ax_idx), ...
                'TickLabelsMode', 'manual', 'TicksMode', 'manual', ...
                'Ticks', linspace(0, 1, length(Wu_gains)), ...
            'TickLabels', arrayfun(@num2str, Wu_gains, 'UniformOutput', 0), ...
            'TickLabelInterpreter', 'latex', 'FontSize', SUBPLOT_SMALL_FONT_SIZE, ...
            'Direction','reverse');
            cbar.Label.String = '$A_{W_u}$';
            cbar.Label.Interpreter = 'latex';
            % clim([Wu_gains(1) Wu_gains(end)]);
        end

        axh(7).XTick = axh(7).XTick(1:end-1);

        set(gcf, 'Position', [0 0 1500 900]);
        savefig(gcf, fullfile(fig_dir, [ctrl_type '_fullorder_controller_bodemag.fig']));
        saveas(gcf, fullfile(fig_dir, [ctrl_type '_fullorder_controller_bodemag.png']));
    end

end

%% Plot classical, disk margins for controller tuned for different wind speeds OUTPLOT FIGURE 10
if 0
    figure;
    load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_wu', '_full_controller_cases']));
    load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_wu', '_Controllers_case_table.mat']));
    ax_gm_2 = subplot(2, 2, 1);
    ax_gm_3 = subplot(2, 2, 2);
    ax_pm_2 = subplot(2, 2, 3);
    ax_pm_3 = subplot(2, 2, 4);

    linestyles = {'-', '--', ':'};
    markerstyles = {'o', 'square', 'diamond'};
    controller_types = fieldnames(case_basis.W1Gain);
    controller_types = {'y_mse', 'adc'};
    n_weighting_cases = length(case_basis.WuGain.x);

    for f = 1:length(controller_types)
        
        ctrl_type = controller_types{f};

        ctrl_cond = contains(Controllers_case_table.("Case Desc."), ctrl_type);
        bcol = copper(n_weighting_cases + 1);
        bcol = flip(bcol(2:end, :), 1); % remove black, flip direction since Wugains go from low to high
        
        % figure;
        
        % top row: gain margins vs wind speed for worst case single channel
        % classical margin, worst case single channel disk margin, multi
        % input/output disk margin
        % bottom row: phase margins vs wind speed for worst case single channel
        % classical margin, worst case single channel disk margin, multi
        % input/output disk margin
        % ax_gm_1 = subplot(2, 3, 1);
        % ax_gm_2 = subplot(2, 3, 2);
        % ax_gm_3 = subplot(2, 3, 3);
        % ax_pm_1 = subplot(2, 3, 4);
        % ax_pm_2 = subplot(2, 3, 5);
        % ax_pm_3 = subplot(2, 3, 6);
        
        Wu_gains = [];
        wu_labels = {};
        for w_idx = 1:n_weighting_cases
            wu = case_basis.WuGain.x{w_idx}.Numerator{1,1}(1);
            wu_cond = Controllers_case_table.("A_Wu") == wu;
            Wu_gains = [Wu_gains,  wu];
            wu_labels{end + 1} = '';

            % data_gm_1 = []; 
            data_gm_2 = []; data_gm_3 = [];
            % data_pm_1 = []; 
            data_pm_2 = []; data_pm_3 = [];
            for c_ws_idx = case_basis.WindSpeedIndex.x
                ws_cond = Controllers_case_table.("TunedWindSpeed") == LPV_CONTROLLER_WIND_SPEEDS(c_ws_idx);
                % x = Controllers_case_table(ctrl_cond & ws_cond, "WorstCase_SingleClassical_GM").Variables;
                % data_gm_1 = [data_gm_1 x(w_idx)];
                x = Controllers_case_table(ctrl_cond & ws_cond & wu_cond, "WorstCase_SingleDisk_GM").Variables;
                data_gm_2 = [data_gm_2 x];
                x = Controllers_case_table(ctrl_cond & ws_cond & wu_cond, "MultiDiskIO_GM").Variables;
                data_gm_3 = [data_gm_3 x];

                % x = Controllers_case_table(ctrl_cond & ws_cond, "WorstCase_SingleClassical_PM").Variables;
                % data_pm_1 = [data_pm_1 x(w_idx)];
                x = Controllers_case_table(ctrl_cond & ws_cond & wu_cond, "WorstCase_SingleDisk_PM").Variables;
                data_pm_2 = [data_pm_2 x];
                x = Controllers_case_table(ctrl_cond & ws_cond & wu_cond, "MultiDiskIO_PM").Variables;
                data_pm_3 = [data_pm_3 x];
            end
            % subplot(2, 3, 1); plot(LPV_CONTROLLER_WIND_SPEEDS, data_gm_1, 'LineWidth', 2, 'Color', bcol(w_idx, :)); hold on;
            % subplot(2, 3, 2); plot(LPV_CONTROLLER_WIND_SPEEDS, data_gm_2, 'LineWidth', 2, 'Color', bcol(w_idx, :)); hold on;
            % subplot(2, 3, 3); plot(LPV_CONTROLLER_WIND_SPEEDS, data_gm_3, 'LineWidth', 2, 'Color', bcol(w_idx, :)); hold on;
            % subplot(2, 3, 4); plot(LPV_CONTROLLER_WIND_SPEEDS, data_pm_1, 'LineWidth', 2, 'Color', bcol(w_idx, :)); hold on;
            % subplot(2, 3, 5); plot(LPV_CONTROLLER_WIND_SPEEDS, data_pm_2, 'LineWidth', 2, 'Color', bcol(w_idx, :)); hold on;
            % subplot(2, 3, 6); plot(LPV_CONTROLLER_WIND_SPEEDS, data_pm_3, 'LineWidth', 2, 'Color', bcol(w_idx, :)); hold on;

            subplot(2, 2, 1); plot(LPV_CONTROLLER_WIND_SPEEDS, data_gm_2, 'LineWidth', 2, 'Color', bcol(w_idx, :), 'LineStyle', linestyles{f}); hold on;
            subplot(2, 2, 2); plot(LPV_CONTROLLER_WIND_SPEEDS, data_gm_3, 'LineWidth', 2, 'Color', bcol(w_idx, :), 'LineStyle', linestyles{f}); hold on;
            subplot(2, 2, 3); plot(LPV_CONTROLLER_WIND_SPEEDS, data_pm_2, 'LineWidth', 2, 'Color', bcol(w_idx, :), 'LineStyle', linestyles{f}); hold on;
            subplot(2, 2, 4); plot(LPV_CONTROLLER_WIND_SPEEDS, data_pm_3, 'LineWidth', 2, 'Color', bcol(w_idx, :), 'LineStyle', linestyles{f}); hold on;

            % subplot(2, 2, 1); scatter(LPV_CONTROLLER_WIND_SPEEDS, data_gm_2, 150, bcol(w_idx, :), 'filled', markerstyles{f}); hold on;
            % subplot(2, 2, 2); scatter(LPV_CONTROLLER_WIND_SPEEDS, data_gm_3, 150, bcol(w_idx, :), 'filled', markerstyles{f}); hold on;
            % subplot(2, 2, 3); scatter(LPV_CONTROLLER_WIND_SPEEDS, data_pm_2, 150, bcol(w_idx, :), 'filled', markerstyles{f}); hold on;
            % subplot(2, 2, 4); scatter(LPV_CONTROLLER_WIND_SPEEDS, data_pm_3, 150, bcol(w_idx, :), 'filled', markerstyles{f}); hold on;


        end
        
        % ax = subplot(2, 3, 1); 
        % % legend([arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
        % %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex'); 
        % xticks(LPV_CONTROLLER_WIND_SPEEDS);
        % xlim([LPV_CONTROLLER_WIND_SPEEDS(1), LPV_CONTROLLER_WIND_SPEEDS(end)]);
        % xlabel('Wind Speed [m/s]', 'Interpreter', 'latex');
        % ax.XAxis.FontSize = SMALL_FONT_SIZE;
        % ax.YAxis.FontSize = SMALL_FONT_SIZE;
        % subtitle("Worst Case Classical Gain Margin [dB]", 'FontSize', BIG_FONT_SIZE, 'Interpreter', 'latex');

        % ax = subplot(2, 3, 2); 
        ax = subplot(2, 2, 1); 
        % legend([arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
        %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex'); 
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlim([LPV_CONTROLLER_WIND_SPEEDS(1), LPV_CONTROLLER_WIND_SPEEDS(end)]);
        xlabel('Wind Speed [m/s]', 'Interpreter', 'latex');
        ax.XAxis.FontSize = SMALL_FONT_SIZE;
        ax.YAxis.FontSize = SMALL_FONT_SIZE;
        subtitle("Worst Case Loop-at-a-Time Disk Gain Margin [dB]", 'FontSize', BIG_FONT_SIZE, 'Interpreter', 'latex');
        

        % ax = subplot(2, 3, 3); 
        ax = subplot(2, 2, 2); 
        % legend([arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
        %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex'); 
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlim([LPV_CONTROLLER_WIND_SPEEDS(1), LPV_CONTROLLER_WIND_SPEEDS(end)]);
        xlabel('Wind Speed [m/s]', 'Interpreter', 'latex');
        ax.XAxis.FontSize = SMALL_FONT_SIZE;
        ax.YAxis.FontSize = SMALL_FONT_SIZE;
        subtitle("Multiloop Disk Gain Margin [dB]", 'FontSize', BIG_FONT_SIZE, 'Interpreter', 'latex');

        % ax = subplot(2, 3, 4); 
        % % legend([arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
        % %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex'); 
        % xticks(LPV_CONTROLLER_WIND_SPEEDS);
        % xlim([LPV_CONTROLLER_WIND_SPEEDS(1), LPV_CONTROLLER_WIND_SPEEDS(end)]);
        % xlabel('Wind Speed [m/s]', 'Interpreter', 'latex');
        % ax.XAxis.FontSize = SMALL_FONT_SIZE;
        % ax.YAxis.FontSize = SMALL_FONT_SIZE;
        % subtitle("Worst Case Classical Phase Margin [deg]", 'FontSize', BIG_FONT_SIZE, 'Interpreter', 'latex');

        % ax = subplot(2, 3, 5); 
        ax = subplot(2, 2, 3); 
        % legend([arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
        %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex'); 
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlim([LPV_CONTROLLER_WIND_SPEEDS(1), LPV_CONTROLLER_WIND_SPEEDS(end)]);
        xlabel('Wind Speed [m/s]', 'Interpreter', 'latex');
        ax.XAxis.FontSize = SMALL_FONT_SIZE;
        ax.YAxis.FontSize = SMALL_FONT_SIZE;
        subtitle("Worst Case Loop-at-a-Time Disk Phase Margin [deg]", 'FontSize', BIG_FONT_SIZE, 'Interpreter', 'latex');
        
        h = zeros(length(Wu_gains) + 2, 1);
        for i = 1:length(Wu_gains)
            % wu_labels{end + 1} = ['$A_{W_u} = $', ' ', num2str(Wu_gains(i))];
            wu_labels{end + 1} = '';
            h(i) = plot(ax, NaN, NaN, 'Color', bcol(i,:), 'LineStyle', '-', 'LineWidth', 2); hold on;
        end
        h(end - 1) = plot(ax, NaN, NaN, 'Color', 'k', 'LineStyle', '-', 'LineWidth', 2); hold on;
        wu_labels{end + 1} = '$K_{err}$';
        h(end) = plot(ax, NaN, NaN, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 2);
        wu_labels{end + 1} = '$K_{adc}$';
        legend(wu_labels, 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);

        % ax = subplot(2, 3, 6); 
        ax = subplot(2, 2, 4); 
        % legend([arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
        %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex'); 
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlim([LPV_CONTROLLER_WIND_SPEEDS(1), LPV_CONTROLLER_WIND_SPEEDS(end)]);
        xlabel('Wind Speed [m/s]', 'Interpreter', 'latex');
        ax.XAxis.FontSize = SMALL_FONT_SIZE;
        ax.YAxis.FontSize = SMALL_FONT_SIZE;
        subtitle("Multiloop Disk Phase Margin [deg]", 'FontSize', BIG_FONT_SIZE, 'Interpreter', 'latex');
        colormap(bcol);
        cbar = colorbar(ax, ...
                'TickLabelsMode', 'manual', 'TicksMode', 'manual', ...
                'Ticks', linspace(0, 1, length(Wu_gains)), ...
            'TickLabels', arrayfun(@num2str, Wu_gains, 'UniformOutput', 0), ...
            'TickLabelInterpreter', 'latex', 'FontSize', SUBPLOT_SMALL_FONT_SIZE);
            cbar.Label.String = '$A_{W_u}$';
            cbar.Label.Interpreter = 'latex';

        % 
        % ax1 = subplot(2, 3, 1); ax2 = subplot(2, 3, 2); ax3 = subplot(2, 3, 3);
        % linkaxes([ax1, ax2, ax3]);
        % ax1 = subplot(2, 3, 4); ax2 = subplot(2, 3, 5); ax3 = subplot(2, 3, 6);
        % linkaxes([ax1, ax2, ax3]);

        % sgtitle([f{1}, ' Controller'], 'Interpreter', 'latex'); 
        % set(gcf, 'Position', [0 0 1500 900]);
        % savefig(gcf, fullfile(fig_dir, [ctrl_type, '_fullorder_rob_margins.fig']));
        % saveas(gcf, fullfile(fig_dir, [ctrl_type, '_fullorder_rob_margins.png']));
    end
    set(gcf, 'Position', [0 0 1500 900]);
    savefig(gcf, fullfile(fig_dir, 'fullorder_rob_margins.fig'));
    saveas(gcf, fullfile(fig_dir, 'fullorder_rob_margins.png'));
end

%% OUTPLOT Plot transfer functions and weighting functions for full-order controller
if 1

    load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_wu', '_full_controller_cases']));
    load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_wu', '_Controllers_case_list.mat']));

    cc = find(case_basis.WindSpeedIndex.x == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED));
    n_weighting_cases = length(case_basis.WuGain.x);

    % w_idx = full_controller_case_basis.WuGain.x;
    % w_idx = 1;

    %% Plot (1,1) channel of transfer functions of KSo, Ti; So, GSi vs f for single wind speed on
    % for open-loop vs closed-loop with weightings OUTPLOT
    % for each controller type: robustness, adc, y_mse
    figure;
    ax_KSo = subplot(2, 2, 2);
    ax_Ti = subplot(2, 2, 1);
    ax_So = subplot(2, 2, 4);
    ax_GSi = subplot(2, 2, 3);

    blue_color = [0, 0.4470, 0.7410];
    red_color = [0.8500, 0.3250, 0.0980];
    yellow_color = [0.9290, 0.6940, 0.1250];
    
        
    ax_KSo_del_idx = []; ax_Ti_del_idx = []; 
    ax_So_del_idx = []; ax_GSi_del_idx = [];
    Wu_gains = [];
    
    for f = fieldnames(case_basis.W1Gain)'
        for w_idx = 1:n_weighting_cases % for each value of A_Wu
            % only plot baseline value of A_Wu
            if Controllers_case_list(w_idx).WuGain.(f{1}).Numerator{1,1}(1) ~= VARY_WU_BASIS(1)
                continue;
            end
            Wu_tmp = Controllers_case_list(w_idx).WuGain.(f{1}) * Wu;
            We_tmp = Controllers_case_list(w_idx).WeGain.(f{1}) * We;
            W1_tmp = Controllers_case_list(w_idx).W1Gain.(f{1}) * W1;
            W2_tmp = Controllers_case_list(w_idx).W2Gain.(f{1}) * W2;
    
            SF = loopsens(-Plant(:, :, cc), ...
                -Controllers_case_list(w_idx).Controller_scaled.(f{1})(:, :, cc));
    
            sys_KSo.(f{1}) = SF.CSo * W2_tmp;
            bound_KSo.(f{1}) = inv(Wu_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc);

            sys_Ti.(f{1}) = SF.Ti * W1_tmp;
            bound_Ti.(f{1}) = inv(Wu_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc);
            
            sys_So.(f{1}) = SF.So * W2_tmp;
            bound_So.(f{1}) = inv(We_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc);
            ol_So.(f{1}) = eye(2) * W2_tmp;

            sys_GSi.(f{1}) = SF.PSi * W1_tmp;
            bound_GSi.(f{1}) = inv(We_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc);
            ol_GSi.(f{1}) = Plant(:, :, cc) * W1_tmp;
        end
    end
    
    % [s, w] = sigma(sys_KSi);
    % sigmaplot(ax_KSo, sys_KSo, sigma_plot_opt); hold on;
    omega = logspace(-4, 2, 200);
    bodeplot(ax_KSo, sys_KSo.rob(1, 1), 'b', sys_KSo.adc(1, 1), 'b', ...
        sys_KSo.y_mse(1, 1), 'b', omega, bode_plot_opt); hold on;
    % bodeplot(ax_KSo, sys_KSo.adc(1, 1), ...
    %     sys_KSo.y_mse(1, 1), bode_plot_opt); hold on;
    a = findobj(ax_KSo,'Type','line','-and','Color','b');
    set(a(1), 'Color', blue_color); set(a(1), 'LineStyle', '-', 'LineWidth', 2);
    set(a(2), 'Color', red_color); set(a(2), 'LineStyle', '-', 'LineWidth', 2);
    set(a(3), 'Color', yellow_color); set(a(3), 'LineStyle', '-', 'LineWidth', 2);

    bodeplot(ax_KSo, bound_KSo.rob(1, 1), 'b', bound_KSo.adc(1, 1), 'b', ...
        bound_KSo.y_mse(1, 1), 'b', omega, bode_plot_opt); hold off;
    % bodeplot(ax_KSo, bound_KSo.adc(1, 1), ...
    %     bound_KSo.y_mse(1, 1), bode_plot_opt); hold off;
    
    a = findobj(ax_KSo,'Type','line','-and','Color','b');
    set(a(1), 'Color', blue_color); set(a(1), 'LineStyle', '--', 'LineWidth', 2);
    set(a(2), 'Color', red_color); set(a(2), 'LineStyle', '--', 'LineWidth', 2);
    set(a(3), 'Color', yellow_color); set(a(3), 'LineStyle', '--', 'LineWidth', 2);

    % ax_KSo_del_idx = [ax_KSo_del_idx, length(ax_KSo_del_idx) + 2:2];
    % legend(ax_KSi, ['Wu = ', num2str(Wu_gains(end))], 'Bound', 'Location', 'westoutside');
    % subtitle(ax_KSo, '$(KS_oW_2)^{(1,1)}$', 'Interpreter','latex');
     subtitle(ax_KSo, '$KS_o^{(1,1)} = \frac{z_{u, d}}{d_{2, d}}$', 'Interpreter','latex');
    
    % [s, w] = sigma(sys_Ti);
    % sigmaplot(ax_Ti, sys_Ti, sigma_plot_opt); hold on;
    bodeplot(ax_Ti, sys_Ti.rob(1, 1), 'b', sys_Ti.adc(1, 1), 'b', ...
        sys_Ti.y_mse(1, 1), 'b', omega, bode_plot_opt); hold on;
    % bodeplot(ax_Ti, sys_Ti.adc(1, 1), ...
    %     sys_Ti.y_mse(1, 1), bode_plot_opt); hold on;
    a = findobj(ax_Ti,'Type','line','-and','Color','b');
    set(a(1), 'Color', blue_color); set(a(1), 'LineStyle', '-', 'LineWidth', 2);
    set(a(2), 'Color', red_color); set(a(2), 'LineStyle', '-', 'LineWidth', 2);
    set(a(3), 'Color', yellow_color); set(a(3), 'LineStyle', '-', 'LineWidth', 2);

    bodeplot(ax_Ti, bound_Ti.rob(1, 1), 'b', bound_Ti.adc(1, 1), 'b', ...
        bound_Ti.y_mse(1, 1), 'b', omega, bode_plot_opt); hold off;
    % bodeplot(ax_Ti, bound_Ti.adc(1, 1), ...
    %     bound_Ti.y_mse(1, 1), bode_plot_opt); hold off;

    a = findobj(ax_Ti,'Type','line','-and','Color','b');
    set(a(1), 'Color', blue_color); set(a(1), 'LineStyle', '--', 'LineWidth', 2);
    set(a(2), 'Color', red_color); set(a(2), 'LineStyle', '--', 'LineWidth', 2);
    set(a(3), 'Color', yellow_color); set(a(3), 'LineStyle', '--', 'LineWidth', 2);

    % ax_Ti_del_idx = [ax_Ti_del_idx, length(ax_Ti_del_idx) + 2:2];
    
    % legend(['Wu = ', num2str(Wu_gains(end))], 'Bound');
    % subtitle(ax_Ti, '$(T_iW_1)^{(1,1)}$', 'Interpreter','latex');
    subtitle(ax_Ti, '$T_i^{(1,1)} = \frac{z_{u, d}}{d_{1, d}}$', 'Interpreter','latex');
    
    % sigmaplot(ax_So, sys_So, sigma_plot_opt); hold on;
    bodeplot(ax_So, sys_So.rob(1, 1), 'b', ...
        sys_So.adc(1, 1), 'b', sys_So.y_mse(1, 1), 'b', ...
        omega, bode_plot_opt); hold on;
    a = findobj(ax_So,'Type','line','-and','Color','b');
    set(a(1), 'Color', blue_color); set(a(1), 'LineStyle', '-', 'LineWidth', 2);
    set(a(2), 'Color', red_color); set(a(2), 'LineStyle', '-', 'LineWidth', 2);
    set(a(3), 'Color', yellow_color); set(a(3), 'LineStyle', '-', 'LineWidth', 2);

    bodeplot(ax_So, bound_So.rob(1, 1), 'b', bound_So.adc(1, 1), 'b', ...
        bound_So.y_mse(1, 1), 'b', omega, bode_plot_opt); hold on;

    a = findobj(ax_So,'Type','line','-and','Color','b');
    set(a(1), 'Color', blue_color); set(a(1), 'LineStyle', '--', 'LineWidth', 2);
    set(a(2), 'Color', red_color); set(a(2), 'LineStyle', '--', 'LineWidth', 2);
    set(a(3), 'Color', yellow_color); set(a(3), 'LineStyle', '--', 'LineWidth', 2);
    
    bodeplot(ax_So, ol_So.rob(1, 1), 'b', omega, bode_plot_opt); hold on;
    a = findobj(ax_So,'Type','line','-and','Color','b');
    set(a(1), 'Color', 'k'); set(a(1), 'LineStyle', ':', 'LineWidth', 2);

    % ax_So_del_idx = [ax_So_del_idx, length(ax_So_del_idx) + 2:2];
    
    % legend(['Wu = ', num2str(Wu_gains(end))], 'Bound');
    % subtitle(ax_So, '$(S_oW_2)^{(1,1)}$', 'Interpreter','latex');
    subtitle(ax_So, '$S_o^{(1,1)} = \frac{z_{e, d}}{d_{2, d}}$', 'Interpreter','latex');

    % sigmaplot(ax_GSi, sys_GSi, sigma_plot_opt); hold on;
    bodeplot(ax_GSi, sys_GSi.rob(1, 1), 'b', sys_GSi.adc(1, 1), 'b', ...
        sys_GSi.y_mse(1, 1), 'b', omega, bode_plot_opt); hold on;
    a = findobj(ax_GSi,'Type','line','-and','Color','b');
    set(a(1), 'Color', blue_color); set(a(1), 'LineStyle', '-', 'LineWidth', 2);
    set(a(2), 'Color', red_color); set(a(2), 'LineStyle', '-', 'LineWidth', 2);
    set(a(3), 'Color', yellow_color); set(a(3), 'LineStyle', '-', 'LineWidth', 2);

    bodeplot(ax_GSi, bound_GSi.rob(1, 1), 'b', bound_GSi.adc(1, 1), 'b', ...
        bound_GSi.y_mse(1, 1), 'b', omega, bode_plot_opt); hold on;

    a = findobj(ax_GSi,'Type','line','-and','Color','b');
    set(a(1), 'Color', blue_color); set(a(1), 'LineStyle', '--', 'LineWidth', 2);
    set(a(2), 'Color', red_color); set(a(2), 'LineStyle', '--', 'LineWidth', 2);
    set(a(3), 'Color', yellow_color); set(a(3), 'LineStyle', '--', 'LineWidth', 2);
    
    bodeplot(ax_GSi, ol_GSi.rob(1, 1), 'b', omega, bode_plot_opt); hold on;
    a = findobj(ax_GSi,'Type','line','-and','Color','b');
    set(a(1), 'Color', 'k'); set(a(1), 'LineStyle', ':', 'LineWidth', 2);

    % legend(, 'Interpreter', 'latex')

    % ax_GSi_del_idx = [ax_GSi_del_idx, length(ax_GSi_del_idx) + 2:2];
    
    % legend(['Wu = ', num2str(Wu_gains(end))], 'Bound');
    % subtitle(ax_GSi, '$(GS_iW_2)^{(1,1)}$', 'Interpreter','latex');
    subtitle(ax_GSi, '$GS_i^{(1,1)} = \frac{z_{e, d}}{d_{1, d}}$', 'Interpreter','latex');

    % sigmaplot(ax_KSo, inv(Wu_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc), '--', sigma_plot_opt);
    % sigmaplot(ax_Ti, inv(Wu_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc), '--', sigma_plot_opt);
    % sigmaplot(ax_So, inv(We_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc), '--', sigma_plot_opt);
    % sigmaplot(ax_GSi, inv(We_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc), '--', sigma_plot_opt);
    % end
    % sgtitle([f{1}, ' Controller'], 'Interpreter', 'latex');
    % TODO no legend save seperately
    % legend(ax_KSo, [arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
    %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex');
    % legend(ax_Ti, [arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ... % 'Bound'], ...
    %     'NumColumns', 1, 'Location', 'eastoutside', 'Interpreter', 'latex');
    % legend(ax_So, [arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ... %
    %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex');
    % legend(ax_GSi, [arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ... %
    %     'NumColumns', 1, 'Location', 'eastoutside', 'Interpreter', 'latex');

    % axesHandlesToChildObjects = findobj(ax_KSo, 'Type', 'line');
    % delete(axesHandlesToChildObjects(ax_KSo_del_idx)); % remove first singluar value
    % axesHandlesToChildObjects = findobj(ax_Ti, 'Type', 'line');
    % delete(axesHandlesToChildObjects(ax_Ti_del_idx)); % remove first singluar value
    % axesHandlesToChildObjects = findobj(ax_So, 'Type', 'line');
    % delete(axesHandlesToChildObjects(ax_So_del_idx)); % remove first singluar value
    % axesHandlesToChildObjects = findobj(ax_GSi, 'Type', 'line');
    % delete(axesHandlesToChildObjects(ax_GSo_del_idx)); % remove first singluar value

    % for ch = 1:length(ax_KSo.Children)
    %     if isfield(ax_KSo.Children(ch), 'Children')
    %     ax_KSo.Children(ch).Children.LineWidth = 2; 
    %     end
    %     if isfield(ax_Ti.Children(ch), 'Children')
    %     ax_Ti.Children(ch).Children.LineWidth = 2; 
    %     end
    %     if isfield(ax_So.Children(ch), 'Children')
    %     ax_So.Children(ch).Children.LineWidth = 2;
    %     end
    %     if isfield(ax_GSi.Children(ch), 'Children')
    %     ax_GSi.Children(ch).Children.LineWidth = 2;
    %     end
    % end
    ax_KSo.Title.String = '';
    ax_Ti.Title.String = '';
    ax_So.Title.String = '';
    ax_GSi.Title.String = '';
  
    % generate legend
    % Find handles of all lines in the figure that have the color blue
    axh = findall(gcf, 'type', 'axes');
    labels = {};
    obj = findobj(ax_Ti,'Type','hggroup');
    for idx = 1:2
        labels{end + 1} = '';
        % for jdx = 1:numel(obj(idx).Children)
        %     labels{end + 1} = '';
        % end
    end
    
    plot(ax_Ti, NaN, NaN, 'Color', blue_color, 'LineStyle', '-', 'LineWidth', 2); hold on;
    plot(ax_Ti, NaN, NaN, 'Color', red_color, 'LineStyle', '-', 'LineWidth', 2);
    plot(ax_Ti, NaN, NaN, 'Color', blue_color, 'LineStyle', '--', 'LineWidth', 2);
    plot(ax_Ti, NaN, NaN, 'Color', red_color, 'LineStyle', '--', 'LineWidth', 2);
    legend(ax_Ti, ...
        '', '', '', '', ...
        ['$(T_iW_1)^{(1,1)}$', ' for ', '$K_{adc}$'], ...
        ['$(T_iW_1)^{(1,1)}$', ' for ', '$K_{err}$'], ...
        ['$(W_u^{-1}\gamma)^{(1,1)}$', ' for ', '$K_{adc}$'], ...
        ['$(W_u^{-1}\gamma)^{(1,1)}$', ' for ', '$K_{err}$'], ...
        'Interpreter', 'latex', 'FontSize', SUBPLOT_BIG_FONT_SIZE);

    h = zeros(4, 1);
    h(1) = plot(ax_KSo, NaN, NaN, 'Color', blue_color, 'LineStyle', '-', 'LineWidth', 2); hold on;
    h(2) = plot(ax_KSo, NaN, NaN, 'Color', red_color, 'LineStyle', '-', 'LineWidth', 2);
    h(3) = plot(ax_KSo, NaN, NaN, 'Color', blue_color, 'LineStyle', '--', 'LineWidth', 2);
    h(4) = plot(ax_KSo, NaN, NaN, 'Color', red_color, 'LineStyle', '--', 'LineWidth', 2); hold off;
    legend(ax_KSo, ...
        '', '', '', '', ...
        ['$(KS_oW_1)^{(1,1)}$', ' for ', '$K_{adc}$'], ...
        ['$(KS_oW_1)^{(1,1)}$', ' for ', '$K_{err}$'], ...
        ['$(W_u^{-1}\gamma)^{(1,1)}$', ' for ', '$K_{adc}$'], ...
        ['$(W_u^{-1}\gamma)^{(1,1)}$', ' for ', '$K_{err}$'], ...
        'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);
    

    h = zeros(5, 1);
    h(1) = plot(ax_GSi, NaN, NaN, 'Color', blue_color, 'LineStyle', '-', 'LineWidth', 2); hold on;
    h(2) = plot(ax_GSi, NaN, NaN, 'Color', red_color, 'LineStyle', '-', 'LineWidth', 2);
    h(3) = plot(ax_GSi, NaN, NaN, 'Color', blue_color, 'LineStyle', '--', 'LineWidth', 2);
    h(4) = plot(ax_GSi, NaN, NaN, 'Color', red_color, 'LineStyle', '--', 'LineWidth', 2);
    h(5) = plot(ax_GSi, NaN, NaN, 'Color', 'k', 'LineStyle', ':', 'LineWidth', 2);  hold off;
    legend(ax_GSi, ...
        '', '', '', '', ...
        ['$(GS_iW_1)^{(1,1)}$', ' for ', '$K_{adc}$'], ...
        ['$(GS_iW_1)^{(1,1)}$', ' for ', '$K_{err}$'], ...
        ['$(W_e^{-1}\gamma)^{(1,1)}$', ' for ', '$K_{adc}$'], ...
        ['$(W_e^{-1}\gamma)^{(1,1)}$', ' for ', '$K_{err}$'], ...
        ['$G^{(1,1)}$'], 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);

    h = zeros(1, 1);
    h(1) = plot(ax_So, NaN, NaN, 'Color', 'k', 'LineStyle', ':', 'LineWidth', 2); hold off;
    legend(ax_So, ...
        '', '', '', '', ...
        ['$(S_oW_2)^{(1,1)}$', ' for ', '$K_{adc}$'], ...
        ['$(S_oW_2)^{(1,1)}$', ' for ', '$K_{err}$'], ...
        ['$(W_e^{-1}\gamma)^{(1,1)}$', ' for ', '$K_{adc}$'], ...
        ['$(W_e^{-1}\gamma)^{(1,1)}$', ' for ', '$K_{err}$'], ...
        ['$I$'], 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);
    
     set(gcf, 'Position', [0 0 1500 900]);
     savefig(gcf, fullfile(fig_dir, 'fullorder_tfs_and_legend.fig'));
     saveas(gcf, fullfile(fig_dir, 'fullorder_tfs_and_legend.png'));

end    


%% Plot Weighting matrices OUTPLOT FIGURE 7
    % examination
if 0
    load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_wu', '_full_controller_cases']));

    blue_color = [0, 0.4470, 0.7410];
    red_color = [0.8500, 0.3250, 0.0980];

    fh = figure;
    % ax_w1 = subplot(2, 2, 1);
    ax_w2 = subplot(1, 3, 1);
    ax_we = subplot(1, 3, 2);
    ax_wu = subplot(1, 3, 3);

    % bodeplot(ax_w1, ...
    %     case_basis.W1Gain.adc{1}(1, 1) * W1(1,1),...
    %     case_basis.W1Gain.y_mse{1}(1, 1) * W1(1,1),...
    %     bode_plot_opt);
    % ax_w1.Title.String = '$A_{W_1}$';
    % for i = 1:3
    %     ax_w1.Children(i).Children.LineWidth = 2; 
    % end
    % legend(ax_w1, '$K_\text{rob}$', '$K_\text{adc}$', '$K_\text{err}$', 'Interpreter', 'latex');

    bodeplot(ax_w2, ...
        case_basis.W2Gain.adc{1}(1, 1) * W2(1,1),...
        case_basis.W2Gain.y_mse{1}(1, 1) * W2(1,1),...
        bode_plot_opt); hold on;
    ax_w2.Title.String = '$A_{W_2}$';
    for i = 1:length(ax_w2.Children) - 1
        ax_w2.Children(i).Children.LineWidth = 2; 
    end
    h = zeros(2, 1);
    h(1) = plot(ax_w2, NaN, NaN, 'Color', blue_color, 'LineStyle', '-', 'LineWidth', 2); 
    h(2) = plot(ax_w2, NaN, NaN, 'Color', red_color, 'LineStyle', '-', 'LineWidth', 2); hold off;
    legend(ax_w2, ...
        '', '', ...
        ['$A_{W_2}$', ' for ', '$K_{adc}$'], ...
        ['$A_{W_2}$', ' for ', '$K_{err}$'], ...
        'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);
    % legend('$K_\text{rob}$', '$K_\text{adc}$', '$K_\text{err}$', 'Interpreter', 'tex');
    % legend('K_\text{rob}', 'K_\text{adc}', 'K_\text{err}', 'Interpreter', 'latex');

    bodeplot(ax_we, ...
        case_basis.WeGain.adc{1}(1, 1) * We(1,1),...
        case_basis.WeGain.y_mse{1}(1, 1) * We(1,1),...
        bode_plot_opt); hold on;
    ax_we.Title.String = '$A_{W_e}$';
    for i = 1:length(ax_we.Children) - 1
        ax_we.Children(i).Children.LineWidth = 2; 
    end
    h = zeros(2, 1);
    h(1) = plot(ax_we, NaN, NaN, 'Color', blue_color, 'LineStyle', '-', 'LineWidth', 2); 
    h(2) = plot(ax_we, NaN, NaN, 'Color', red_color, 'LineStyle', '-', 'LineWidth', 2); hold off;
    legend(ax_we, ...
        '', '', ...
        ['$A_{W_e}$', ' for ', '$K_{adc}$'], ...
        ['$A_{W_e}$', ' for ', '$K_{err}$'], ...
        'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);
    % legend(ax_w1, '$K_\text{rob}$', '$K_\text{adc}$', '$K_\text{err}$', 'Interpreter', 'latex');
    wu_labels = {};
    bcol = flip(copper(length(case_basis.WuGain.x)), 1);
    for i = 1:length(case_basis.WuGain.x)
        wu_labels{end + 1} = '';
        W = case_basis.WuGain.x{i}(1, 1) * Wu(1,1);
        W.InputName = '';
        bodeplot(ax_wu, W, 'b', bode_plot_opt);
       
        % Find handles of all lines in the figure that have the color blue
        blineHandle = findobj(ax_wu,'Type','line','-and','Color','b');

        % Change the color to the one you defined
        set(blineHandle,'Color',bcol(i,:));
        
        hold on;
    end
    ax_wu.Title.String = '$A_{W_u}$';
    for i = 1:length(ax_wu.Children) - 1
        ax_wu.Children(i).Children.LineWidth = 2; 
    end

    h = zeros(length(case_basis.WuGain.x), 1);
    for i = 1:length(case_basis.WuGain.x)
        wu_labels{end + 1} = ['$A_{W_u} = $', num2str(case_basis.WuGain.x{i}.Numerator{1,1})];
        W = case_basis.WuGain.x{i}(1, 1) * Wu(1,1);
        h(i) = plot(ax_wu, NaN, NaN, 'Color', bcol(i,:), 'LineStyle', '-', 'LineWidth', 2); hold on;
    end

    legend(ax_wu, ...
        wu_labels, ...
        'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);

    set(fh, 'Position', [0 0 1500 900]);
    savefig(fh, fullfile(fig_dir, 'weighting_funcs.fig'));
    saveas(fh, fullfile(fig_dir, 'weighting_funcs.png'));
    
    end

%% 2x2 plot for each gain: A_w1, A_w2, A_we or A_wu showing OUTPLOT FIGURE 6
% 3 plots on each mulitIO_dm, adc, M1 RMSE (y-axis) vs. A_w for extreme controller sweep
 % one curve corresponds to increasing A_w1, A_w2, A_we or A_wu while
 % holding all others at default value of 1
 if 1
     load(fullfile(mat_save_dir, ['extreme_k_cases_turbsim', '_full_controller_cases']));
     load(fullfile(mat_save_dir, ['extreme_k_cases_turbsim', '_simulation_case_table.mat']));
     
     w2_const_idx = 3;
     % varying_w1_cond = ...
     %     (Controllers_simulation_case_table.("A_W2") == case_basis.W2Gain.x{4}.Numerator{1, 1}) & ...
     %     (Controllers_simulation_case_table.("A_We") == case_basis.WeGain.x{4}.Numerator{1, 1}) & ...
     %     (Controllers_simulation_case_table.("A_Wu") == case_basis.WuGain.x{4}.Numerator{1, 1});
     % sum(varying_w1_cond)
     wu_const_idx = 4; % 4
     we_const_idx = 4; % 4
     varying_w2_cond = ...
         (Controllers_simulation_case_table.("A_W1") == case_basis.W1Gain.x{1}.Numerator{1, 1}) & ...
         (Controllers_simulation_case_table.("A_We") == case_basis.WeGain.x{we_const_idx}.Numerator{1, 1}) & ...
         (Controllers_simulation_case_table.("A_Wu") == case_basis.WuGain.x{wu_const_idx}.Numerator{1, 1});
     sum(varying_w2_cond)

     % set We and Wu to minimum and vary w2 to vary w2/w1 ratio
     varying_w2w1_cond = ...
         (Controllers_simulation_case_table.("A_W1") == case_basis.W1Gain.x{end}.Numerator{1, 1}) & ...
         (Controllers_simulation_case_table.("A_We") == case_basis.WeGain.x{1}.Numerator{1, 1}) & ...
         (Controllers_simulation_case_table.("A_Wu") == case_basis.WuGain.x{1}.Numerator{1, 1});
    
     wu_const_idx = 6; % 4
     varying_we_cond = ...
         (Controllers_simulation_case_table.("A_W2") == case_basis.W2Gain.x{w2_const_idx}.Numerator{1, 1}) & ...
         (Controllers_simulation_case_table.("A_W1") == case_basis.W1Gain.x{1}.Numerator{1, 1}) & ...
         (Controllers_simulation_case_table.("A_Wu") == case_basis.WuGain.x{wu_const_idx}.Numerator{1, 1});
     sum(varying_we_cond)

     % set W2 to maximimum and Wu to minimum and vary We to vary we/w1 ratio
     varying_wew1_cond = ...
         (Controllers_simulation_case_table.("A_W2") == case_basis.W2Gain.x{end}.Numerator{1, 1}) & ...
         (Controllers_simulation_case_table.("A_W1") == case_basis.W1Gain.x{end}.Numerator{1, 1}) & ...
         (Controllers_simulation_case_table.("A_Wu") == case_basis.WuGain.x{1}.Numerator{1, 1});
     

     % const_idx = 1; % 4]
     we_const_idx = 1; % 4
     varying_wu_cond = ...
         (Controllers_simulation_case_table.("A_W2") == case_basis.W2Gain.x{w2_const_idx}.Numerator{1, 1}) & ...
         (Controllers_simulation_case_table.("A_We") == case_basis.WeGain.x{we_const_idx}.Numerator{1, 1}) & ...
         (Controllers_simulation_case_table.("A_W1") == case_basis.W1Gain.x{1}.Numerator{1, 1});

     % set W2 to maximimum and We to minimum and vary Wu to vary wu/w1 ratio
     varying_wuw1_cond = ...
         (Controllers_simulation_case_table.("A_W2") == case_basis.W2Gain.x{end}.Numerator{1, 1}) & ...
         (Controllers_simulation_case_table.("A_We") == case_basis.WeGain.x{1}.Numerator{1, 1}) & ...
         (Controllers_simulation_case_table.("A_W1") == case_basis.W1Gain.x{end}.Numerator{1, 1});

     varying_wuwe_cond = ...
         (Controllers_simulation_case_table.("A_W2") == case_basis.W2Gain.x{end}.Numerator{1, 1}) & ...
         (Controllers_simulation_case_table.("A_We") == case_basis.WeGain.x{4}.Numerator{1, 1}) & ...
         (Controllers_simulation_case_table.("A_W1") == case_basis.W1Gain.x{end}.Numerator{1, 1});

     varying_wuw2_cond = ...
         (Controllers_simulation_case_table.("A_W2") == case_basis.W2Gain.x{end}.Numerator{1, 1}) & ...
         (Controllers_simulation_case_table.("A_We") == case_basis.WeGain.x{1}.Numerator{1, 1}) & ...
         (Controllers_simulation_case_table.("A_W1") == case_basis.W1Gain.x{end}.Numerator{1, 1});

     varying_wew2_cond = ...
         (Controllers_simulation_case_table.("A_W2") == case_basis.W2Gain.x{end}.Numerator{1, 1}) & ...
         (Controllers_simulation_case_table.("A_Wu") == case_basis.WuGain.x{1}.Numerator{1, 1}) & ...
         (Controllers_simulation_case_table.("A_W1") == case_basis.W1Gain.x{end}.Numerator{1, 1});
     
     sum(varying_wu_cond)
     
     metrics = {'MultiDiskIO_DM', 'ADC', 'RootMycBlade1 RMSE'};
     % metric_scaling = [1, 1, 10];
     gains = {'A_W2', 'A_We', 'A_Wu'};
     conds = [varying_w2_cond, varying_we_cond, varying_wu_cond];
     titles = {['Varying ', '$A_{W_2}$'], ['Varying ', '$A_{W_e}$'], ['Varying ', '$A_{W_u}$']};
     xlabels = {'$A_{W_2}$', '$A_{W_e}$', '$A_{W_u}$'};
     
     rel_denom = [case_basis.W1Gain.x{end}.Numerator{1, 1}, ...
         case_basis.W1Gain.x{end}.Numerator{1, 1}, ...
         case_basis.W1Gain.x{end}.Numerator{1, 1}, ...
         case_basis.WeGain.x{4}.Numerator{1, 1}];
     rel_gains = {'A_W2', 'A_We', 'A_Wu', 'A_Wu'};
     rel_conds = [varying_w2w1_cond, varying_wuw1_cond, varying_wew1_cond, varying_wuwe_cond];
     rel_titles = {['Varying ', '$A_{W_2}/A_{W_1}$'], ...
         ['Varying ', '$A_{W_e}/A_{W_1}$'], ...
         ['Varying ', '$A_{W_u}/A_{W_1}$'], ...
         ['Varying ', '$A_{W_u}/A_{W_e}$']};
     rel_xlabels = {'$A_{W_2}/A_{W_1}$', '$A_{W_e}/A_{W_1}$', '$A_{W_u}/A_{W_1}$', '$A_{W_u}/A_{W_e}$'};
     
     for m = 1:length(metrics)
         % metric_min = min(Controllers_simulation_case_table(:, metrics{m})).Variables;
         % metric_max = max(Controllers_simulation_case_table(:, metrics{m})).Variables;
        figure(1);
         for g = 1:length(gains)
            ax = subplot(1, 3, g);
            tab = sortrows(Controllers_simulation_case_table(conds(:, g), {gains{g}, metrics{m}}), gains{g});
            x = tab.(gains{g});
            metric_min = min(Controllers_simulation_case_table(conds(:, g), metrics{m})).Variables;
            metric_max = max(Controllers_simulation_case_table(conds(:, g), metrics{m})).Variables;
            y = (tab.(metrics{m}) - metric_min) ./ (metric_max - metric_min);
            % plot(x, y, 'LineWidth', 2); hold on;
            scatter(x, y, 0.1, 'filled'); hold on;
            subtitle(titles{g}, 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);
            xticks(x);
            ax.XAxis.FontSize = SMALL_FONT_SIZE;
            ax.YAxis.FontSize = SMALL_FONT_SIZE;
            xlabel(xlabels{g}, 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE)
            % ylabel('$\%$', 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);
         end
        %  figure(2);
        % for g = 1:length(rel_gains)
        %     ax = subplot(2, 2, g);
        %     tab = sortrows(Controllers_simulation_case_table(rel_conds(:, g), {rel_gains{g}, metrics{m}}), rel_gains{g});
        %     x = tab.(rel_gains{g}) / rel_denom(g);
        %     y = (tab.(metrics{m}) - metric_min) ./ (metric_max - metric_min);
        %     plot(x, y, 'LineWidth', 2); hold on;
        %     subtitle(rel_titles{g}, 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);
        %     xticks(x);
        %     ax.XAxis.FontSize = SMALL_FONT_SIZE;
        %     ax.YAxis.FontSize = SMALL_FONT_SIZE;
        %     xlabel(rel_xlabels{g}, 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE)
        %     ylabel('$\%$', 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);
        % end
     end
     ax = subplot(1,3,1);
     legend(ax, 'Multi-Loop Disk Margin', 'ADC', ['$M_1$' ' RMSE'], 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE, 'Location', 'east');
     colormap(bcol);
    colorbar('Ticks', Wu_gains, ...
        'TickLabels', cellfun(@(x) ['$A_{W_u} = $', ' ', num2str(x)], Wu_gains, ...
        'UniformOutput', false));


    axes = []
    for g = 1:3
     ax = subplot(1,3,g);
     axes = [axes, ax];
    end
    linkaxes(axes, 'y');

     for g = 1:1
         ax = subplot(1,3,g);
         tab = sortrows(Controllers_simulation_case_table(conds(:, g), {gains{g}, metrics{m}}), gains{g});
         x = tab.(gains{g});
         xticks(ax, [x(1); x(4:end)]);
     end

     for g = 2:3
         ax = subplot(1,3,g);
         tab = sortrows(Controllers_simulation_case_table(conds(:, g), {gains{g}, metrics{m}}), gains{g});
         x = tab.(gains{g});
         xticks(ax, [x(1); x(3:end)]);
     end

     set(gcf, 'Position', [0 0 1500 900]);
     savefig(gcf, fullfile(fig_dir, 'fullorder_gain_sweep.fig'));
     saveas(gcf, fullfile(fig_dir, 'fullorder_gain_sweep.png'));

 end
 

%% 2x3 Plot for each controller type showing change in mean
 % ADC, M1 RMSE (y-axis) relative to no ipc case for each mean wind speed
 % (x-axis)
 if 0

    load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_wu', '_full_controller_cases']));
    load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_wu', '_Controllers_case_list.mat']));
    load(fullfile(mat_save_dir, 'untuned_k_turbsim_simulation_case_agg_table.mat'));
    load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_wu', '_simulation_case_agg_table.mat']));
    
    UntunedControllers_simulation_case_agg_table(:, ["Case Desc.", "WindMean", "mean_ADC", "mean_RootMycBlade1 RMSE"])
    Controllers_simulation_case_agg_table(:, ["Case Desc.", "WindMean", "mean_ADC", "mean_RootMycBlade1 RMSE"])

     figure;
     wind_speeds = sort(unique(Controllers_simulation_case_table.WindMean));
    
     metrics = {'ADC', 'RootMycBlade1 RMSE'};
     agg_metrics = {'mean_ADC', 'mean_RootMycBlade1 RMSE'};
     label_metrics = {'$ADC$', '$M_1$ RMSE'};
     label_controller_types = {'$K_{rob}$', '$K_{adc}$', '$K_{err}$'};
    
     for m = 1:length(metrics)
         for c = 1:length(controller_types)
             plot_idx = (m - 1) * length(controller_types) + c;
             ax = subplot(length(metrics), length(controller_types), plot_idx);
             data.noipc = sortrows(UntunedControllers_simulation_case_agg_table(strcmp(UntunedControllers_simulation_case_agg_table.("Case Desc."), "noipc"), :), 'WindMean');
             data.noipc = data.noipc(:, {'Case Desc.', 'WindMean', agg_metrics{m}});
             
             data.baseline_controller = sortrows(UntunedControllers_simulation_case_agg_table(strcmp(UntunedControllers_simulation_case_agg_table.("Case Desc."), "baseline_controller"), :), 'WindMean');
             data.baseline_controller = data.baseline_controller(:, {'Case Desc.', 'WindMean', agg_metrics{m}});
            
             y = 100 * (data.baseline_controller.(agg_metrics{m}) - data.noipc.(agg_metrics{m})) ./ data.noipc.(agg_metrics{m});
             if strcmp(metrics{m}, 'ADC')
                 y = y / 1000;
             end
             plot(wind_speeds, y, 'k--', 'LineWidth', 2); hold on;
             
             data.tuned = sortrows(Controllers_simulation_case_agg_table(contains(Controllers_simulation_case_agg_table.("Case Desc."), controller_types{c}), :), 'WindMean');
             % TODO change to gain_col
             data.tuned = sortrows(data.tuned, 'A_Wu');
             data.tuned = data.tuned(:, {'Case Desc.', 'WindMean', agg_metrics{m}});
    
             cases = unique(data.tuned.("Case Desc."));
             bcol = flip(copper(length(cases) + 1), 1);
             bcol = bcol(1:end-1, :); % remove blace
             for cc = 1:length(cases)
                 case_desc = cases{cc};
                 y = 100 * (data.tuned(strcmp(data.tuned.("Case Desc."), case_desc), agg_metrics{m}).Variables - data.noipc.(agg_metrics{m})) ./ data.noipc.(agg_metrics{m});
                 
                 if strcmp(metrics{m}, 'ADC')
                    y = y / 1000;
                 end
                 
                 plot(wind_speeds, y, 'Color', bcol(cc, :), 'LineWidth', 2); hold on;
             end
    
             xticks(wind_speeds); xlim([wind_speeds(1), wind_speeds(end)]);
             
             if strcmp(metrics{m}, 'ADC')
                 ylabel('$[1000\%]$', 'Interpreter', 'latex', 'FontSize', SMALL_FONT_SIZE);
             else
                 ylabel('$[\%]$', 'Interpreter', 'latex', 'FontSize', SMALL_FONT_SIZE);
             end
    
             if m == 2
                xlabel('Mean Wind Speed [m/s]', 'Interpreter', 'latex', 'FontSize', SMALL_FONT_SIZE);
             end
             l = ['Change in ', label_metrics{m}, ' for ' ,label_controller_types{c}];
             subtitle(l, 'FontSize', BIG_FONT_SIZE, 'Interpreter', 'latex');
             ax.XAxis.FontSize = SMALL_FONT_SIZE;
             ax.YAxis.FontSize = SMALL_FONT_SIZE;
         end
     end
     ax1 = subplot(length(metrics), length(controller_types), 1);
     ax2 = subplot(length(metrics), length(controller_types), 2);
     ax3 = subplot(length(metrics), length(controller_types), 3);
     linkaxes([ax1, ax2, ax3], 'y');
     % for ax = [ax1, ax2, ax3]
     %    ax.YTickLabel = cellfun(@(x) num2str(str2num(x) / 1000), ax.YTickLabel, 'UniformOutput', false);
     %    ylabel(ax, '$[1000\%]$', 'Interpreter', 'latex', 'FontSize', SMALL_FONT_SIZE);
     % end
     % 
     ax1 = subplot(length(metrics), length(controller_types), 4);
     ax2 = subplot(length(metrics), length(controller_types), 5);
     ax3 = subplot(length(metrics), length(controller_types), 6);
     linkaxes([ax1, ax2, ax3], 'y');
    
    set(gcf, 'Position', [0 0 1500 900]);
    savefig(gcf, fullfile(fig_dir, [sim_type, '_metric_change.fig']));
    saveas(gcf, fullfile(fig_dir, [sim_type, '_metric_change.png']));
 end

%% OUTPLOT 2x2 multibar plot for each load type (rows) showing greatest reduction  in 
% DEL of 3 different rows (first 3 plots) and ADC (last plot) relative to no ipc case for 
% baseline controller, best peforming (in terms of greatest decrease) 
% wu tuned controllers, reference tuned controllers, 
% and saturation tuned controllers (y-axis) 
% for different mean wind speeds (x-axis)
% for 3 different loads: Main Bearing Yaw Bending moment, Main Bearing Tilt
% Bending moment, Flap-wise blade root bending moment) (different
% plots)

if 0
    loads = {'RootMyc1', 'ADC', 'YawBrMyp', 'YawBrMzp'};
    labels = {'Blade Root Bending Moment DEL', 'ADC', 'Tower-Top Tilt Bending Moment DEL', 'Tower-Top Yaw Bending Moment DEL'};
    metric_cols = arrayfun(@num2str, LPV_CONTROLLER_WIND_SPEEDS, 'UniformOutput', 0);

    del_wu = load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_wu', '_del_table'])).del_out_table;
    del_ref = load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_ref', '_del_table'])).del_out_table;
    del_sat = load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_sat', '_del_table'])).del_out_table;

    % for each load type, get the mean over all wind speeds
    del_wu.("Mean") = mean(del_wu(:, metric_cols), 2).Variables;
    del_ref.("Mean") = mean(del_ref(:, metric_cols), 2).Variables;
    del_sat.("Mean") = mean(del_sat(:, metric_cols), 2).Variables;

    % del_wu.("Idx") = 1:height(del_wu);
    % del_ref.("Idx") = 1:height(del_ref);
    % del_sat.("Idx") = 1:height(del_sat);
    
    % for each load type, get the best mean reduction over all values of
    % Wu/Ref/Sat
    groupsummary(del_wu, 'Load', 'min', 'Mean');
    groupsummary(del_sat, 'Load', 'min', 'Mean');
    groupsummary(del_ref, 'Load', 'min', 'Mean');

    % get best performing from del_wu, del_sat, del_ref
    del_wu_red = table();
    del_ref_red = table();
    del_sat_red = table();
    for load_val = unique(del_wu.("Load"))'
        tmp = sortrows(del_wu(strcmp(del_wu.("Load"), load_val{1}) ...
            & ~contains(del_wu.("Case Desc."), 'baseline_controller'), :), "Mean", 'ascend');
        cond = strcmp(del_wu.("Case Desc."), tmp(1, "Case Desc.").Variables) & (del_wu.("A_Wu") == tmp(1, "A_Wu").Variables) & strcmp(del_wu.("Load"), load_val{1});
        del_wu_red = [del_wu_red; del_wu(cond, :)];

        tmp = sortrows(del_sat(strcmp(del_sat.("Load"), load_val{1}) ...
            & ~contains(del_sat.("Case Desc."), 'baseline_controller'), :), "Mean", 'ascend');
        cond = strcmp(del_sat.("Case Desc."), tmp(1, "Case Desc.").Variables) & (del_sat.("Saturation") == tmp(1, "Saturation").Variables) & strcmp(del_sat.("Load"), load_val{1});
        del_sat_red = [del_sat_red; del_sat(cond, :)];

        tmp = sortrows(del_ref(strcmp(del_ref.("Load"), load_val{1}) ...
            & ~contains(del_ref.("Case Desc."), 'baseline_controller'), :), "Mean", 'ascend');
        cond = strcmp(del_ref.("Case Desc."), tmp(1, "Case Desc.").Variables) & (del_ref.("Reference") == tmp(1, "Reference").Variables) & strcmp(del_ref.("Load"), load_val{1});
        del_ref_red = [del_ref_red; del_ref(cond, :)];
    end
    
    figure;
    x = LPV_CONTROLLER_WIND_SPEEDS;
    for l = 1:length(loads)
        ax = subplot(2, 2, l);
        y = 100 * [del_wu(strcmp(del_wu.Load, loads{l}) & strcmp(del_wu.('Case Desc.'), "baseline_controller"), metric_cols).Variables; ...
            del_wu_red(strcmp(del_wu_red.Load, loads{l}), metric_cols).Variables; ...
            del_ref_red(strcmp(del_ref_red.Load, loads{l}), metric_cols).Variables; ...
            del_sat_red(strcmp(del_sat_red.Load, loads{l}), metric_cols).Variables];
        bar(ax, x, y');
        xlabel('Wind Speed [m/s]', 'Interpreter', 'latex', 'FontSize', SMALL_FONT_SIZE);
        subtitle(['$\%$', ' Change in ', labels{l}], 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);
        ax.XAxis.FontSize = SMALL_FONT_SIZE;
        ax.YAxis.FontSize = SMALL_FONT_SIZE;
    end

    ax = subplot(2, 2, 1);
    legend(ax, ['Baseline ', '$K_0$'], ['Varying ', '$A_{W_u}$'], ...
        ['Varying ', '$M^\star_{dq}$'], ['Varying ', '$\overline{\beta}_{dq}$'], ...
        'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);

    set(gcf, 'Position', [0 0 1500 900]);
    savefig(gcf, fullfile(fig_dir, 'del_change.fig'));
    saveas(gcf, fullfile(fig_dir, 'del_change.png'));

end
%% 2x3 Plot for each controller type showing PSD for a single seed comparing no ipc, baseline and best of controller type over Wu, Reference, Saturation values (y-axis) for Myc1
if 0      
% blade_op_arr = {'OoPDefl1', 'IPDefl1', 'TwstDefl1', ...
%           'RotThrust', ...
%           'TTDspFA', 'TTDspSS', 'TTDspTwst', ... % Tower-top / yaw bearing fore-aft/side-to-side/angular torsion deflection
%           'RootMxb1', 'RootMyb1', ... % Blade 1 edgewise/flapwise moment 
%           'RootMyc1', 'RootMzc1', ... % Blade 1 out-of-plane/pitching moment
%           'LSSGagMya', 'LSSGagMza', ... % Rotating low-speed shaft bending moment at the shaft's strain gage (about ya/za axis)
%           'YawBrMxp', 'YawBrMyp', 'YawBrMzp', ... % Nonrotating tower-top / yaw bearing roll/pitch/yaw moment
%           'TwrBsMxt', 'TwrBsMyt', 'TwrBsMzt' ... % Tower base roll (or side-to-side)/pitching (or fore-aft)/yaw moment 
%           };
    
    loads = {"RootMyc1_1P", "YawBrMyp_0P", "YawBrMzp_0P", ...
             "RootMyc1_2P",  "YawBrMyp_3P", "YawBrMzp_3P"};

    labels = {'1P Blade Root Bending Moment', ...
        '0P Tower-Top Tilt Bending Moment', ...
        '0P Tower-Top Yaw Bending Moment', ...
        '2P Blade Root Bending Moment', ...
        '3P Tower-Top Tilt Bending Moment', ...
        '3P Tower-Top Yaw Bending Moment'};

    metric_cols = arrayfun(@num2str, LPV_CONTROLLER_WIND_SPEEDS, 'UniformOutput', 0);

    psd_wu = load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_wu', '_psd_table'])).psd_out_table;
    psd_ref = load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_ref', '_psd_table'])).psd_out_table;
    psd_sat = load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_sat', '_psd_table'])).psd_out_table;

    % for each load type, get the mean over all wind speeds
    psd_wu.("Mean") = mean(psd_wu(:, metric_cols), 2).Variables;
    psd_ref.("Mean") = mean(psd_ref(:, metric_cols), 2).Variables;
    psd_sat.("Mean") = mean(psd_sat(:, metric_cols), 2).Variables;
    
    % for each load type, get the best mean reduction over all values of
    % Wu/Ref/Sat
    groupsummary(psd_wu, 'Load', 'min', 'Mean');
    groupsummary(psd_sat, 'Load', 'min', 'Mean');
    groupsummary(psd_ref, 'Load', 'min', 'Mean');

    % get best performing from del_wu, del_sat, del_ref
    psd_wu_red = table();
    psd_ref_red = table();
    psd_sat_red = table();
    for load_val = unique(psd_wu.("Load"))'
        tmp = sortrows(psd_wu(strcmp(psd_wu.("Load"), load_val{1}) ...
            & ~contains(psd_wu.("Case Desc."), 'baseline_controller'), :), "Mean", 'ascend');
        cond = strcmp(psd_wu.("Case Desc."), tmp(1, "Case Desc.").Variables) & (psd_wu.("A_Wu") == tmp(1, "A_Wu").Variables) & strcmp(psd_wu.("Load"), load_val{1});
        psd_wu_red = [psd_wu_red; psd_wu(cond, :)];
    
        tmp = sortrows(psd_sat(strcmp(psd_sat.("Load"), load_val{1}) ...
            & ~contains(psd_sat.("Case Desc."), 'baseline_controller'), :), "Mean", 'ascend');
        cond = strcmp(psd_sat.("Case Desc."), tmp(1, "Case Desc.").Variables) & (psd_sat.("Saturation") == tmp(1, "Saturation").Variables) & strcmp(psd_sat.("Load"), load_val{1});
        psd_sat_red = [psd_sat_red; psd_sat(cond, :)];
    
        tmp = sortrows(psd_ref(strcmp(psd_ref.("Load"), load_val{1}) ...
            & ~contains(psd_ref.("Case Desc."), 'baseline_controller'), :), "Mean", 'ascend');
        cond = strcmp(psd_ref.("Case Desc."), tmp(1, "Case Desc.").Variables) & (psd_ref.("Reference") == tmp(1, "Reference").Variables) & strcmp(psd_ref.("Load"), load_val{1});
        psd_ref_red = [psd_ref_red; psd_ref(cond, :)];
    end

    figure(1);
    x = LPV_CONTROLLER_WIND_SPEEDS;
    for l = 1:length(loads)
        ax = subplot(2, 3, l);
        y = 100 * [psd_wu(strcmp(psd_wu.Load, loads{l}) & strcmp(psd_wu.('Case Desc.'), "baseline_controller"), metric_cols).Variables; ...
            psd_wu_red(strcmp(psd_wu_red.Load, loads{l}), metric_cols).Variables; ...
            psd_ref_red(strcmp(psd_ref_red.Load, loads{l}), metric_cols).Variables; ...
            psd_sat_red(strcmp(psd_sat_red.Load, loads{l}), metric_cols).Variables];
        bar(x, y');
        xlabel('Wind Speed [m/s]', 'Interpreter', 'latex', 'FontSize', SMALL_FONT_SIZE);
        subtitle({['$\%$', ' Change in ', labels{l}(1:3)], labels{l}(4:end)}, 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);
        ax.XAxis.FontSize = SMALL_FONT_SIZE;
        ax.YAxis.FontSize = SMALL_FONT_SIZE;
    end

    ax = subplot(2, 3, 1);
    legend(ax, ['Baseline ', '$K_0$'], ['Varying ', '$A_{W_u}$'], ...
        ['Varying ', '$M^\star_{dq}$'], ['Varying ', '$\overline{\beta}_{dq}$'], ...
        'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);

    set(gcf, 'Position', [0 0 1500 900]);
    savefig(gcf, fullfile(fig_dir, 'psd_change.fig'));
    saveas(gcf, fullfile(fig_dir, 'psd_change.png'));
end

% TODO Plot PSD for three different load for best wu, sat, ref controllers
if 0
    loads = {"RootMyc1", "YawBrMyp", "YawBrMzp"};

    labels = {'Blade Root Bending Moment', ...
        'Tower-Top Tilt Bending Moment', ...
        'Tower-Top Yaw Bending Moment'};

    figure(2);
    L = size(dqValues, 1);
    f = (1 / (DT * L)) * (0:L/2) * 2 * pi;
    x = (1 / L) * abs(x);
    x = x(1:L/2 + 1, :);
    x(2:end-1, :) = 2 * x(2:end-1, :);
    plot(f, P1,"LineWidth", 2);
    title("DQ Single-Sided Amplitude Spectrum", 'Interpreter', 'latex')

    xline(harmonics);
    xlim([0, harmonics(end) * 1.25])
    xlabel(['$f$', ' [Hz]'], 'Interpreter', 'latex')

    linkaxes([ax1, ax2], 'xy')

    plotSpectra(...
        x, ...
        {'RootMyc1'}, {'RootMycD', 'RootMycQ'}, ...
        'psd', DT, omega_1P_rad * [0, 3] * (1 / (2*pi)));
end

    
%% Plot Blade-Pitch Actuation, dBetadt and Loads in Time-Domain over a single rotation of Azimuth OUTPLOT
if 0

    sim_processed_data_wu = load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_wu', '_simulation_case_table.mat'])).Controllers_simulation_case_table;
    sim_processed_data_ref = load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_ref', '_simulation_case_table.mat'])).Controllers_simulation_case_table;
    sim_processed_data_sat = load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_sat', '_simulation_case_table.mat'])).Controllers_simulation_case_table;

    % get cases for 16m/s seed=1, lowest adc for wu, ref and sat
    infw_fn = 'A_16_5';
    tmp = sortrows(sim_processed_data_wu(strcmp(sim_processed_data_wu.("WindField"), infw_fn) & ...
        ~strcmp(sim_processed_data_wu.("Case Desc."), "noipc"), :), "ADC");
    best_wu_case = tmp(1, "Case Desc.").Variables;
    tmp = sortrows(sim_processed_data_ref(strcmp(sim_processed_data_ref.("WindField"), infw_fn) & ...
        ~strcmp(sim_processed_data_ref.("Case Desc."), "noipc"), :), "ADC");
    best_ref_case = tmp(1, "Case Desc.").Variables;
    tmp = sortrows(sim_processed_data_sat(strcmp(sim_processed_data_sat.("WindField"), infw_fn) & ...
        ~strcmp(sim_processed_data_sat.("Case Desc."), "noipc"), :), "ADC");
    best_sat_case = tmp(1, "Case Desc.").Variables;
    
    sim_raw_data_baseline = load(fullfile(sl_metadata_save_dir, ['sim_out_list_', 'baseline_k_turbsim', '.mat'])).sim_out_list;
    sim_raw_data_wu = load(fullfile(sl_metadata_save_dir, ['sim_out_list_', 'optimal_k_cases_turbsim_wu', '.mat'])).sim_out_list;
    sim_raw_data_ref = load(fullfile(sl_metadata_save_dir, ['sim_out_list_', 'optimal_k_cases_turbsim_ref', '.mat'])).sim_out_list;
    sim_raw_data_sat = load(fullfile(sl_metadata_save_dir, ['sim_out_list_', 'optimal_k_cases_turbsim_sat', '.mat'])).sim_out_list;
    
    time = struct;
    az = struct;
    beta = struct;
    beta_do = struct;

    for case_idx = 1:length(sim_raw_data_baseline)
        if strcmp(sim_raw_data_baseline(case_idx).CaseDesc, 'baseline_controller') & contains(sim_raw_data_baseline(case_idx).InflowWind.FileName_BTS, infw_fn)
            case_idx
            values = load([sim_raw_data_baseline(case_idx).outdata_save_fn '_' num2str(case_idx)]);
            values = values.OutData';
            values = values(floor(cut_transients / DT):end, 2:end); % remove extra time column
            time.baseline = values(:, ismember(OutList, 'Time'));
            az.baseline = values(:, ismember(OutList, 'Azimuth'));
            beta.baseline = values(:, ismember(OutList, 'BldPitch1'));
            beta_dot.baseline = diff(values(:, ismember(OutList, 'BldPitch1'))) / DT;

            break;
        end
    end

    for case_idx = 1:length(sim_raw_data_wu)
        if strcmp(sim_raw_data_wu(case_idx).CaseDesc, best_wu_case) & contains(sim_raw_data_wu(case_idx).InflowWind.FileName_BTS, infw_fn)
            case_idx
            values = load([sim_raw_data_wu(case_idx).outdata_save_fn '_' num2str(case_idx)]);
            values = values.OutData';
            values = values(floor(cut_transients / DT):end, 2:end); % remove extra time column
            time.wu = values(:, ismember(OutList, 'Time'));
            az.wu = values(:, ismember(OutList, 'Azimuth'));
            beta.wu = values(:, ismember(OutList, 'BldPitch1'));
            beta_dot.wu = diff(values(:, ismember(OutList, 'BldPitch1'))) / DT;
            break;
        end
    end

    for case_idx = 1:length(sim_raw_data_ref)
        if strcmp(sim_raw_data_ref(case_idx).CaseDesc, best_ref_case) & contains(sim_raw_data_ref(case_idx).InflowWind.FileName_BTS, infw_fn)
            values = load([sim_raw_data_ref(case_idx).outdata_save_fn '_' num2str(case_idx)]);
            values = values.OutData';
            values = values(floor(cut_transients / DT):end, 2:end); % remove extra time column
            time.ref = values(:, ismember(OutList, 'Time'));
            az.ref = values(:, ismember(OutList, 'Azimuth'));
            beta.ref = values(:, ismember(OutList, 'BldPitch1'));
            beta_dot.ref = diff(values(:, ismember(OutList, 'BldPitch1'))) / DT;

            break;
        end
    end

    for case_idx = 1:length(sim_raw_data_sat)
        if strcmp(sim_raw_data_sat(case_idx).CaseDesc, best_sat_case) & contains(sim_raw_data_sat(case_idx).InflowWind.FileName_BTS, infw_fn)
            values = load([sim_raw_data_sat(case_idx).outdata_save_fn '_' num2str(case_idx)]);
            values = values.OutData';
            values = values(floor(cut_transients / DT):end, 2:end); % remove extra time column
            time.sat = values(:, ismember(OutList, 'Time'));
            az.sat = values(:, ismember(OutList, 'Azimuth'));
            beta.sat = values(:, ismember(OutList, 'BldPitch1'));
            beta_dot.sat = diff(values(:, ismember(OutList, 'BldPitch1'))) / DT;
           
            break;
        end
    end
    
    figure(1); 

    az_ax = subplot(3, 1, 1);
    plot(az_ax, time.baseline, az.baseline, ...
        time.wu, az.wu, ...
        time.ref, az.ref, ...
        time.sat, az.sat, 'LineWidth', 2);
    az_ax.XAxis.FontSize = SMALL_FONT_SIZE;
    az_ax.YAxis.FontSize = SMALL_FONT_SIZE;
    subtitle(['$\theta$', ' [deg]'], 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);
    legend(az_ax, ['Baseline, ', '$K_0$'], ...
            ['Varying , ', '$A_{W_u}$'], ...
            ['Varying , ', '$M^\star_{d}$'], ...
            ['Varying , ', '$\overline{\beta}_{dq}$'], ...
            'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);

    beta_ax = subplot(3, 1, 2);
    plot(beta_ax, time.baseline, beta.baseline, ...
        time.wu, beta.wu, ...
        time.ref, beta.ref, ...
        time.sat, beta.sat, 'LineWidth', 2);
    beta_ax.XAxis.FontSize = SMALL_FONT_SIZE;
    beta_ax.YAxis.FontSize = SMALL_FONT_SIZE;
    subtitle(['$\beta_1$', ' [deg]'], 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);


    beta_dot_ax = subplot(3, 1, 3); 
    plot(beta_dot_ax, time.baseline(1:end-1), beta_dot.baseline, ...
        time.wu(1:end-1), beta_dot.wu, ...
        time.ref(1:end-1), beta_dot.ref, ...
        time.sat(1:end-1), beta_dot.sat, ...
        'LineWidth', 2); hold on;
    beta_dot_ax.XAxis.FontSize = SMALL_FONT_SIZE;
    beta_dot_ax.YAxis.FontSize = SMALL_FONT_SIZE;
    subtitle(['$\frac{d\beta_1}{dt}$', ' [deg/s]'], 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE); 
    xlabel('Time [s]', 'FontSize', SMALL_FONT_SIZE);
    linkaxes([az_ax, beta_ax, beta_dot_ax], 'x');
    xlim([cut_transients, cut_transients + (2 * pi) / omega_1P_rad]);

    

    set(gcf, 'Position', [0 0 1500 900]);
    savefig(gcf, fullfile(fig_dir, 'ts.fig'));
    saveas(gcf, fullfile(fig_dir, 'ts.png'));
end

%% Plot Blade-Pitch DQ with saturation values, Md with reference values OUTPLOT
if 1

    blue_color = [0, 0.4470, 0.7410];
    red_color = [0.8500, 0.3250, 0.0980];

    load(fullfile(mat_save_dir, 'M_dq_reference.mat'));
    load(fullfile(mat_save_dir, 'Beta_dq_saturation.mat'));
    % load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_ref', '_full_controller_cases']));

    sim_processed_data_wu = load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_wu', '_simulation_case_table.mat'])).Controllers_simulation_case_table;
    sim_processed_data_ref = load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_ref', '_simulation_case_table.mat'])).Controllers_simulation_case_table;
    sim_processed_data_sat = load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_sat', '_simulation_case_table.mat'])).Controllers_simulation_case_table;

    % get cases for 16m/s seed=1, lowest adc for wu, ref and sat
    infw_fn = 'A_16_5';

    sim_raw_data_baseline = load(fullfile(sl_metadata_save_dir, ['sim_out_list_', 'baseline_k_turbsim', '.mat'])).sim_out_list;
    sim_raw_data_wu = load(fullfile(sl_metadata_save_dir, ['sim_out_list_', 'optimal_k_cases_turbsim_wu', '.mat'])).sim_out_list;
    sim_raw_data_ref = load(fullfile(sl_metadata_save_dir, ['sim_out_list_', 'optimal_k_cases_turbsim_ref', '.mat'])).sim_out_list;
    sim_raw_data_sat = load(fullfile(sl_metadata_save_dir, ['sim_out_list_', 'optimal_k_cases_turbsim_sat', '.mat'])).sim_out_list;

    tmp = sortrows(sim_processed_data_wu(strcmp(sim_processed_data_wu.("WindField"), infw_fn) & ...
        ~strcmp(sim_processed_data_wu.("Case Desc."), "noipc"), :), "ADC");
    best_wu_adc_case = tmp(7, "Case Desc.").Variables;
    tmp = sortrows(sim_processed_data_ref(strcmp(sim_processed_data_ref.("WindField"), infw_fn) & ...
        ~strcmp(sim_processed_data_ref.("Case Desc."), "noipc"), :), "ADC");
    best_ref_adc_case = tmp(7, "Case Desc.").Variables;
    tmp = sortrows(sim_processed_data_sat(strcmp(sim_processed_data_sat.("WindField"), infw_fn) & ...
        ~strcmp(sim_processed_data_sat.("Case Desc."), "noipc"), :), "ADC");
    best_sat_adc_case = tmp(7, "Case Desc.").Variables;

    tmp = sortrows(sim_processed_data_wu(strcmp(sim_processed_data_wu.("WindField"), infw_fn) & ...
        ~strcmp(sim_processed_data_wu.("Case Desc."), "noipc"), :), "RootMycBlade1 RMSE");
    best_wu_ymse_case = tmp(1, "Case Desc.").Variables;
    tmp = sortrows(sim_processed_data_ref(strcmp(sim_processed_data_ref.("WindField"), infw_fn) & ...
        ~strcmp(sim_processed_data_ref.("Case Desc."), "noipc"), :), "RootMycBlade1 RMSE");
    best_ref_ymse_case = tmp(1, "Case Desc.").Variables;
    tmp = sortrows(sim_processed_data_sat(strcmp(sim_processed_data_sat.("WindField"), infw_fn) & ...
        ~strcmp(sim_processed_data_sat.("Case Desc."), "noipc"), :), "RootMycBlade1 RMSE");
    best_sat_ymse_case = tmp(1, "Case Desc.").Variables;
    
    
    beta_dq = struct;
    M_dq = struct;

    % for case_idx = 1:length(sim_raw_data_baseline)
    %     if strcmp(sim_raw_data_baseline(case_idx).CaseDesc, 'baseline_controller') & contains(sim_raw_data_baseline(case_idx).InflowWind.FileName_BTS, infw_fn)
    %         case_idx
    %         values = load([sim_raw_data_baseline(case_idx).outdata_save_fn '_' num2str(case_idx)]);
    %         values = values.OutData';
    %         values = values(floor(cut_transients / DT):end, 2:end); % remove extra time column
    % 
    %         blpitch_values = load([sim_raw_data_baseline(case_idx).blpitch_save_fn '_' num2str(case_idx)]);
    %         blpitch_values = blpitch_values.BlPitch';
    %         blpitch_values = blpitch_values(floor(cut_transients / DT):end, 2:end); % remove extra time column
    % 
    %         time.baseline = values(:, ismember(OutList, 'Time'));
    %         beta_dq.baseline = blpitch_values(:, 1:2);
    % 
    %         rootmyc_values = load([sim_raw_data_baseline(case_idx).rootmyc_save_fn '_' num2str(case_idx)]);
    %         rootmyc_values = rootmyc_values.RootMyc';
    %         rootmyc_values = rootmyc_values(floor(cut_transients / DT):end, 2:end); % remove extra time column
    %         M_dq.baseline = rootmyc_values(:, 1:2);
    % 
    %         break;
    %     end
    % end

    for case_idx = 1:length(sim_raw_data_wu)
        if strcmp(sim_raw_data_wu(case_idx).CaseDesc, best_wu_adc_case) & contains(sim_raw_data_wu(case_idx).InflowWind.FileName_BTS, infw_fn)
            case_idx
            values = load([sim_raw_data_wu(case_idx).outdata_save_fn '_' num2str(case_idx)]);
            values = values.OutData';
            values = values(floor(cut_transients / DT):end, 2:end); % remove extra time column

            blpitch_values = load([sim_raw_data_wu(case_idx).blpitch_save_fn '_' num2str(case_idx)]);
            blpitch_values = blpitch_values.BlPitch';
            blpitch_values = blpitch_values(floor(cut_transients / DT):end, 2:end); % remove extra time column

            beta_dq.time = values(:, ismember(OutList, 'Time'));
            beta_dq.vals = blpitch_values(:, 1:2);
        end
        if strcmp(sim_raw_data_wu(case_idx).CaseDesc, best_wu_ymse_case) & contains(sim_raw_data_wu(case_idx).InflowWind.FileName_BTS, infw_fn)
            case_idx
            values = load([sim_raw_data_wu(case_idx).outdata_save_fn '_' num2str(case_idx)]);
            values = values.OutData';
            values = values(floor(cut_transients / DT):end, 2:end); % remove extra time column


            rootmyc_values = load([sim_raw_data_wu(case_idx).rootmyc_save_fn '_' num2str(case_idx)]);
            rootmyc_values = rootmyc_values.RootMyc';
            rootmyc_values = rootmyc_values(floor(cut_transients / DT):end, 2:end); % remove extra time column

            rootmyc_dq.time = values(:, ismember(OutList, 'Time'));
            rootmyc_dq.vals = rootmyc_values(:, 1:2);
        end
    end

    figure(1); 
    
    rootmyc_ax = subplot(2, 1, 1);
    rootmyc_labels = {};
    plot(rootmyc_ax, rootmyc_dq.time, rootmyc_dq.vals(:, 1), 'LineWidth', 2); hold on;
    rootmyc_labels{end+1} = '$M_d$';
    ref_basis = load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_ref', '_full_controller_cases'])).case_basis.Reference.x;
    bcol = flip(copper(length(ref_basis) + 1), 1); % light to dark, adding extra to remove black
    for r = 1:length(ref_basis)
        ref = VARY_REFERENCE_BASIS(ref_basis(r));
        rootmyc_labels{end+1} = ['$M^\star_d = $', ' ', num2str(ref), '$\underline{M}_d$'];
        plot(rootmyc_ax, rootmyc_dq.time, ref * M_dq_reference(1) * ones(length(rootmyc_dq.time), 1), 'Color', bcol(r, :), 'LineStyle', '--', 'LineWidth', 2);
    end
    legend(rootmyc_ax, ...
        rootmyc_labels, ...
        'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE)
    rootmyc_ax.XAxis.FontSize = SMALL_FONT_SIZE;
    rootmyc_ax.YAxis.FontSize = SMALL_FONT_SIZE;
    subtitle(['$M_d$', ' [kNm]'], 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);
    % legend(rootmyc_ax, ['Baseline, ', '$K_0$'], ...
    %         ['Varying , ', '$A_{W_u}$'], ...
    %         ['Varying , ', '$M^\star_{d}$'], ...
    %         ['Varying , ', '$\overline{\beta}_{dq}$'], ...
    %         'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);

    beta_ax = subplot(2, 1, 2);
    beta_labels = {};
    plot(beta_ax, beta_dq.time, rad2deg(beta_dq.vals), 'LineWidth', 2); hold on;
    beta_labels{end+1} = '$\beta_d$';
    beta_labels{end+1} = '$\beta_q$';
    sat_basis = load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_sat', '_full_controller_cases'])).case_basis.Saturation.x;
    for s = 1:length(sat_basis)
        sat = VARY_SATURATION_BASIS(sat_basis(s));
        beta_labels{end+1} = ['$\overline{\beta}_d = $', ' ', num2str(sat), '$\underline{\beta}_d$'];
        beta_labels{end+1} = ['$\overline{\beta}_q = $', ' ', num2str(sat), '$\underline{\beta}_q$'];
        plot(beta_ax, beta_dq.time, rad2deg(sat * Beta_dq_saturation(1) * ones(length(beta_dq.time), 1)), 'Color', blue_color, 'LineStyle', '--', 'LineWidth', 2);
        plot(beta_ax, beta_dq.time, rad2deg(sat * Beta_dq_saturation(2) * ones(length(beta_dq.time), 1)), 'Color', red_color, 'LineStyle', '--', 'LineWidth', 2);
    end
    legend(beta_ax, ...
        beta_labels, ...
        'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE, 'NumColumns', 3);
    beta_ax.XAxis.FontSize = SMALL_FONT_SIZE;
    beta_ax.YAxis.FontSize = SMALL_FONT_SIZE;
    xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', SMALL_FONT_SIZE);
    subtitle(['$\beta_{dq}$', ' [deg]'], 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);
    % xlim([cut_transients, cut_transients + (2 * pi) / omega_1P_rad]);
    % xlim(rootmyc_ax, [beta_dq.time(end) - (2 * pi) / omega_1P_rad, beta_dq.time(end)])
    xlim(rootmyc_ax, [cut_transients, beta_dq.time(end)])
    linkaxes([rootmyc_ax, beta_ax], 'x');
    set(gcf, 'Position', [0 0 1500 900]);
    savefig(gcf, fullfile(fig_dir, 'ref_sat_ts.fig'));
    saveas(gcf, fullfile(fig_dir, 'ref_sat_ts.png'));
end

%%
if 0
    % sim_out_list.noipc(20).InflowWind.FileName_BTS;
    % [~, x, ~] = fileparts(infw_fn);
    case_descs = Controllers_simulation_case_table(strcmp(Controllers_simulation_case_table.("WindField"), infw_fn), :).("Case Desc.")
    
    sim_infw_fn = cell(length(sim_out_list.controller), 1);
    [sim_infw_fn{:}] = sim_out_list.controller.InflowWind.FileName_BTS;

    wind_idx = find(strcmp(sim_infw_fn, infw_fn));
    ctrl_sim_data = sim_out_list.controller(wind_idx)
    sim_case_descs = cell(length(ctrl_sim_data), 1);
    [sim_case_descs{:}] = ctrl_sim_data.CaseDesc{1};

    ctrl_sim_table = Controllers_simulation_case_table(strcmp(Controllers_simulation_case_table.("WindField"), x), :)
    case_idx = find(contains(sim_case_descs, 'rob'));
    cases = ctrl_sim_data(case_idx);
    % TODO change to gain_col
    ordered_cases = sortrows(ctrl_sim_table(contains(ctrl_sim_table.("Case Desc."), 'rob'), :), 'A_Wu');
    bcol = flip(copper(length(cases) + 1), 1);
    
    figure;
    

    % Plot blade-pitch 1 for robust controllers
    ax1 = subplot(1, 3, 1);
    y = sim_out_list.noipc(20).beta_ipc.blade;
    t = 0:DT:(length(y)-1)*DT;
    t = t((100 / DT):(120 / DT) + 1);
    y = y((100 / DT):(120 / DT) + 1);
    plot(t, y, 'LineWidth', 2, 'LineStyle', ':');
    hold on;
    y = sim_out_list.baseline_controller(20).beta_ipc.blade;
    t = 0:DT:(length(y)-1)*DT;
    t = t((100 / DT):(120 / DT) + 1);
    y = y((100 / DT):(120 / DT) + 1);
    plot(t, y, 'LineWidth', 2, 'LineStyle', '--');
    case_idx = find(contains(sim_case_descs, 'rob'));
    cases = ctrl_sim_data(case_idx);
    % TODO change to gain_col
    ordered_cases = sortrows(ctrl_sim_table(contains(ctrl_sim_table.("Case Desc."), 'rob'), :), 'A_Wu');
    bcol = flip(copper(length(cases) + 1), 1);
    for cc = 1:height(ordered_cases)
        case_desc = ordered_cases(cc, 'Case Desc.').Variables;
        y = cases(strcmp(sim_case_descs(case_idx), case_desc)).beta.blade;
        t = 0:DT:(length(y)-1)*DT;
        t = t((100 / DT):(110 / DT) + 1);
        y = y((100 / DT):(110 / DT) + 1);
        plot(t, y, 'Color', bcol(cc, :), 'LineWidth', 2);
        xlim([t(1), t(end)]);
    end
    ax1.XAxis.FontSize = SMALL_FONT_SIZE;
    ax1.YAxis.FontSize = SMALL_FONT_SIZE;
    title(['$\beta_1$', ' [deg] for robust controller ', '$K_{rob}$'], 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);
    hold off;

    % Plot blade-pitch 1 for adc controllers
    ax1 = nexttile(2);
    y = sim_out_list.noipc(20).beta.blade;
    t = 0:DT:(length(y)-1)*DT;
    t = t((100 / DT):(120 / DT) + 1);
    y = y((100 / DT):(120 / DT) + 1);
    plot(t, y, 'LineWidth', 2, 'LineStyle', ':');
    hold on;
    y = sim_out_list.baseline_controller(20).beta.blade;
    t = 0:DT:(length(y)-1)*DT;
    t = t((100 / DT):(120 / DT) + 1);
    y = y((100 / DT):(120 / DT) + 1);
    plot(t, y, 'LineWidth', 2, 'LineStyle', '--');
    case_idx = find(contains(sim_case_descs, 'adc'));
    cases = ctrl_sim_data(case_idx);
    % TODO change to gain_col
    ordered_cases = sortrows(ctrl_sim_table(contains(ctrl_sim_table.("Case Desc."), 'adc'), :), 'A_Wu');
    bcol = flip(copper(length(cases) + 1), 1);
    for cc = 1:height(ordered_cases)
        case_desc = ordered_cases(cc, 'Case Desc.').Variables;
        y = cases(strcmp(sim_case_descs(case_idx), case_desc)).beta.blade;
        t = 0:DT:(length(y)-1)*DT;
        t = t((100 / DT):(110 / DT) + 1);
        y = y((100 / DT):(110 / DT) + 1);
        plot(t, y, 'Color', bcol(cc, :), 'LineWidth', 2);
        xlim([t(1), t(end)]);
    end
    ax1.XAxis.FontSize = 22;
    ax1.YAxis.FontSize = 22;
    title(['$\beta_1$', ' [deg] for ADC controller ', '$K_{adc}$'], 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);
    hold off;
    

    % Plot blade-pitch 1 for mse controllers
    ax1 = nexttile(3);
    y = sim_out_list.noipc(20).beta.blade;
    t = 0:DT:(length(y)-1)*DT;
    t = t((100 / DT):(120 / DT) + 1);
    y = y((100 / DT):(120 / DT) + 1);
    plot(t, y, 'LineWidth', 2, 'LineStyle', ':');
    hold on;
    y = sim_out_list.baseline_controller(20).beta.blade;
    t = 0:DT:(length(y)-1)*DT;
    t = t((100 / DT):(120 / DT) + 1);
    y = y((100 / DT):(120 / DT) + 1);
    plot(t, y, 'LineWidth', 2, 'LineStyle', '--');
    case_idx = find(contains(sim_case_descs, 'y_mse'));
    cases = ctrl_sim_data(case_idx);
    % TODO change to gain_col
    ordered_cases = sortrows(ctrl_sim_table(contains(ctrl_sim_table.("Case Desc."), 'y_mse'), :), 'A_Wu');
    bcol = flip(copper(length(cases) + 1), 1);
    for cc = 1:height(ordered_cases)
        case_desc = ordered_cases(cc, 'Case Desc.').Variables;
        y = cases(strcmp(sim_case_descs(case_idx), case_desc)).beta.blade;
        t = 0:DT:(length(y)-1)*DT;
        t = t((100 / DT):(110 / DT) + 1);
        y = y((100 / DT):(110 / DT) + 1);
        plot(t, y, 'Color', bcol(cc, :), 'LineWidth', 2);
        xlim([t(1), t(end)]);
    end
    ax1.XAxis.FontSize = 22;
    ax1.YAxis.FontSize = 22;
    title(['$\beta_1$', ' [deg] for Error controller ', '$K_{err}$'], 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);
    hold off;

    % Plot M1 for robust controllers
    ax1 = nexttile(4);
    y = sim_out_list.noipc(20).RootMyc.blade;
    t = 0:DT:(length(y)-1)*DT;
    t = t((100 / DT):(120 / DT) + 1);
    y = y((100 / DT):(120 / DT) + 1) * 1e-3;
    plot(t, y, 'LineWidth', 2, 'LineStyle', ':');
    hold on;
    y = sim_out_list.baseline_controller(20).RootMyc.blade;
    t = 0:DT:(length(y)-1)*DT;
    t = t((100 / DT):(120 / DT) + 1);
    y = y((100 / DT):(120 / DT) + 1) * 1e-3;
    plot(t, y, 'LineWidth', 2, 'LineStyle', '--');
    case_idx = find(contains(sim_case_descs, 'rob'));
    cases = ctrl_sim_data(case_idx);
    % TODO change to gain_col
    ordered_cases = sortrows(ctrl_sim_table(contains(ctrl_sim_table.("Case Desc."), 'rob'), :), 'A_Wu');
    bcol = flip(copper(length(cases) + 1), 1);
    for cc = 1:height(ordered_cases)
        case_desc = ordered_cases(cc, 'Case Desc.').Variables;
        y = cases(strcmp(sim_case_descs(case_idx), case_desc)).RootMyc.blade;
        t = 0:DT:(length(y)-1)*DT;
        t = t((100 / DT):(110 / DT) + 1);
        y = y((100 / DT):(110 / DT) + 1) * 1e-3;
        plot(t, y, 'Color', bcol(cc, :), 'LineWidth', 2);
        xlim([t(1), t(end)]);
    end
    ax1.XAxis.FontSize = 22;
    ax1.YAxis.FontSize = 22;
    title(['$M_1$', ' [MNm] for Error controller ', '$K_{rob}$'], 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);
    hold off;

    % Plot M1 for adc controllers
    ax1 = nexttile(5);
    y = sim_out_list.noipc(20).RootMyc.blade;
    t = 0:DT:(length(y)-1)*DT;
    t = t((100 / DT):(120 / DT) + 1);
    y = y((100 / DT):(120 / DT) + 1) * 1e-3;
    plot(t, y, 'LineWidth', 2, 'LineStyle', ':');
    hold on;
    y = sim_out_list.baseline_controller(20).RootMyc.blade;
    t = 0:DT:(length(y)-1)*DT;
    t = t((100 / DT):(120 / DT) + 1);
    y = y((100 / DT):(120 / DT) + 1) * 1e-3;
    plot(t, y, 'LineWidth', 2, 'LineStyle', '--');
    case_idx = find(contains(sim_case_descs, 'adc'));
    cases = ctrl_sim_data(case_idx);
    % TODO change to gain_col
    ordered_cases = sortrows(ctrl_sim_table(contains(ctrl_sim_table.("Case Desc."), 'adc'), :), 'A_Wu');
    bcol = flip(copper(length(cases) + 1), 1);
    for cc = 1:height(ordered_cases)
        case_desc = ordered_cases(cc, 'Case Desc.').Variables;
        y = cases(strcmp(sim_case_descs(case_idx), case_desc)).RootMyc.blade;
        t = 0:DT:(length(y)-1)*DT;
        t = t((100 / DT):(110 / DT) + 1);
        y = y((100 / DT):(110 / DT) + 1) * 1e-3;
        plot(t, y, 'Color', bcol(cc, :), 'LineWidth', 2);
        xlim([t(1), t(end)]);
    end
    ax1.XAxis.FontSize = 22;
    ax1.YAxis.FontSize = 22;
    title(['$M_1$', ' [MNm] for Error controller ', '$K_{adc}$'], 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);
    hold off;

    % Plot M1 for mse controllers
    ax1 = nexttile(6);
    y = sim_out_list.noipc(20).RootMyc.blade;
    t = 0:DT:(length(y)-1)*DT;
    t = t((100 / DT):(120 / DT) + 1);
    y = y((100 / DT):(120 / DT) + 1) * 1e-3;
    plot(t, y, 'LineWidth', 2, 'LineStyle', ':');
    hold on;
    y = sim_out_list.baseline_controller(20).RootMyc.blade;
    t = 0:DT:(length(y)-1)*DT;
    t = t((100 / DT):(120 / DT) + 1);
    y = y((100 / DT):(120 / DT) + 1) * 1e-3;
    plot(t, y, 'LineWidth', 2, 'LineStyle', '--');
    case_idx = find(contains(sim_case_descs, 'y_mse'));
    cases = ctrl_sim_data(case_idx);
    % TODO change to gain_col
    ordered_cases = sortrows(ctrl_sim_table(contains(ctrl_sim_table.("Case Desc."), 'y_mse'), :), 'A_Wu');
    bcol = flip(copper(length(cases) + 1), 1);
    for cc = 1:height(ordered_cases)
        case_desc = ordered_cases(cc, 'Case Desc.').Variables;
        y = cases(strcmp(sim_case_descs(case_idx), case_desc)).RootMyc.blade;
        t = 0:DT:(length(y)-1)*DT;
        t = t((100 / DT):(110 / DT) + 1);
        y = y((100 / DT):(110 / DT) + 1) * 1e-3;
        plot(t, y, 'Color', bcol(cc, :), 'LineWidth', 2);
        xlim([t(1), t(end)]);
    end
    ax1.XAxis.FontSize = 22;
    ax1.YAxis.FontSize = 22;
    title(['$M_1$', ' [MNm] for Error controller ', '$K_{err}$'], 'Interpreter', 'latex', 'FontSize', BIG_FONT_SIZE);
    hold off;

    set(gcf, 'Position', [0 0 1500 900]);
    savefig(gcf, fullfile(fig_dir, 'vary_wu_timeseries.fig'));
    saveas(gcf, fullfile(fig_dir, 'vary_wu_timeseries.png'));
end

if 0

% Plot DQ values of blade-pitch and blade root bending moment for
% particular wind field
% if (strcmp(case_basis.InflowWind.WindType, '3')) && (strcmp(case_basis.InflowWind.FileName_BTS{i}, ))
bts_filename = case_basis.InflowWind.FileName_BTS{1};
ux = case_basis.InflowWind.HWindSpeed(1);
figure;
tcf = tiledlayout(2, 2);
cc = 0;
for c = 1:length(sim_out_list.controller)
    if length(sim_out_list.controller(c).ErrorMessage)
        continue;
    end
    if ~strcmp(sim_out_list.controller(c).InflowWind.FileName_BTS, bts_filename) ...
            || (sim_out_list.controller(c).InflowWind.HWindSpeed ~= ux) ...
            || ~strcmp(sim_out_list.noipc(1).InflowWind.FileName_BTS, bts_filename) ...
            || (sim_out_list.noipc(1).InflowWind.HWindSpeed ~= ux)
        continue;
    end
    t = sim_out_list.controller(c).OutData.time;
    cc = cc + 1;
    nexttile(1);
    plot(t, sim_out_list.controller(cc).beta.dq(:, 1));
    hold on;
    nexttile(2);
    plot(t, sim_out_list.controller(cc).beta.dq(:, 2));
    hold on;
    nexttile(3);
    plot(t, sim_out_list.controller(cc).Myc.dq(:, 1));
    hold on;
    nexttile(4);
    plot(t, sim_out_list.controller(cc).Myc.dq(:, 2));
    hold on;
end
nexttile(1); plot(t, beta(1).noipc.dq(:, 1)); hold off;
nexttile(2); plot(t, beta(1).noipc.dq(:, 2)); hold off;
nexttile(3); plot(t, RootMyc(1).noipc.dq(:, 1)); hold off;
nexttile(4); plot(t, RootMyc(1).noipc.dq(:, 2)); hold off;

nexttile(1);
xlabel('Time [s]'); ylabel('Blade-Pitch DQ Actuation [deg]', 'Rotation', 0);
nexttile(3);
xlabel('Time [s]'); ylabel('Root Blade DQ Bending Moment [kNm]', 'Rotation', 0);
nexttile(4);

legend([arrayfun(@(n) ['', num2str(n)], 1:length(sim_out_list.controller), 'UniformOutput', false), 'None']);


% Plot blade values of blade-pitch and blade root bending moment for
% particular wind field
figure;
tcf = tiledlayout(2, 1);
cc = 0;
for c = 1:length(sim_out_list.controller)
    if length(sim_out_list.controller(c).ErrorMessage)
        continue;
    end
    if ~strcmp(sim_out_list.controller(c).InflowWind.FileName_BTS, bts_filename) ...
            || (sim_out_list.controller(c).InflowWind.HWindSpeed ~= ux) ...
            || ~strcmp(sim_out_list.noipc(1).InflowWind.FileName_BTS, bts_filename) ...
            || (sim_out_list.noipc(1).InflowWind.HWindSpeed ~= ux)
        continue;
    end
    t = sim_out_list.controller(c).OutData.time;
    cc = cc + 1;
    nexttile(1);
    plot(t, beta(cc).controller.blade);
    hold on;
    nexttile(2);
    plot(t, RootMyc(cc).controller.blade);
    hold on;
end
nexttile(1); plot(t, beta(1).noipc.blade); hold off;
nexttile(2); plot(t, RootMyc(1).noipc.blade); hold off;
legend([arrayfun(@(n) ['', num2str(n)], 1:length(sim_out_list.controller), 'UniformOutput', false), 'None']);
nexttile(1);
xlabel('Time [s]'); ylabel('Blade-Pitch Actuation [deg]', 'Rotation', 0);
nexttile(2);
xlabel('Time [s]'); ylabel('Root Blade 1 Bending Moment [kNm]', 'Rotation', 0);
set(gcf, 'Position', [0 0 1500 900]);%get(0, 'Screensize'));
savefig(gcf, fullfile(fig_dir, 'nonlin_ts.fig'));
saveas(gcf, fullfile(fig_dir, 'nonlin_ts.png'));
end
