%% Plot classical, disk margins for controller tuned for different wind speeds OUTPLOT FIGURE 10

SUBPLOT_BIG_FONT_SIZE = 30;
SUBPLOT_SMALL_FONT_SIZE = 27;
bode_plot_opt = bodeoptions;
bode_plot_opt.Title.FontSize = SUBPLOT_BIG_FONT_SIZE;
bode_plot_opt.Title.String = '';
bode_plot_opt.InputLabels.FontSize = SUBPLOT_BIG_FONT_SIZE;
bode_plot_opt.OutputLabels.FontSize = SUBPLOT_BIG_FONT_SIZE;
bode_plot_opt.XLabel.FontSize = SUBPLOT_BIG_FONT_SIZE;
bode_plot_opt.YLabel.FontSize = SUBPLOT_BIG_FONT_SIZE;
bode_plot_opt.TickLabel.FontSize = SUBPLOT_SMALL_FONT_SIZE;
bode_plot_opt.PhaseMatching = 'on';
bode_plot_opt.Grid = 'on';
bode_plot_opt.PhaseMatchingFreq = 0;
bode_plot_opt.PhaseMatchingValue = 0;
bode_plot_opt.PhaseVisible = 'off';
bode_plot_opt.Title.Interpreter = 'latex';
bode_plot_opt.InputLabels.Interpreter = 'latex';
bode_plot_opt.OutputLabels.Interpreter = 'latex';
bode_plot_opt.XLabel.Interpreter = 'latex';
bode_plot_opt.YLabel.Interpreter = 'latex';
%bode_plot_opt.XLim = [];
if 1
    LPV_CONTROLLER_WIND_SPEEDS = [12, 14, 16, 18, 20, 22];
    ws_cond = Controllers_case_table.("TunedWindSpeed") == LPV_CONTROLLER_WIND_SPEEDS(3);
     min(Controllers_case_table(ws_cond, ...
        ["ClassicalInput_GMD", "ClassicalInput_GMQ", ...
        "ClassicalOutput_GMD", "ClassicalOutput_GMQ"]), [], 2);


    low_adc_wu_case = {'adc W1 = 1 W2 = 0.01 ->  Wu = 1 We = 0.1'};
    low_ymse_wu_case = {'y_mse W1 = 1 W2 = 0.1 ->  Wu = 10 We = 10'}
    figure;
    % load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_wu', '_full_controller_cases']));
    % load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_wu', '_Controllers_case_table.mat']));
    ax_gm_2 = subplot(2, 2, 1);
    ax_gm_3 = subplot(2, 2, 2);
    ax_pm_2 = subplot(2, 2, 3);
    ax_pm_3 = subplot(2, 2, 4);

    linestyles = {'--', '-', ':'};
    markerstyles = {'o', 'square', 'diamond'};
    % controller_types = fieldnames(case_basis.W1Gain);
    controller_types = {'adc', 'y_mse'};
    weighting_cases = [1, 2.5, 5, 7.5, 10];
    n_weighting_cases = 5;%length(case_basis.WuGain.x);
    
    for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
        ws_cond = Controllers_case_table.("TunedWindSpeed") == LPV_CONTROLLER_WIND_SPEEDS(c_ws_idx);

        [M, I] = min(Controllers_case_table(ws_cond, ...
            ["SingleDiskInput_DMD", "SingleDiskInput_DMQ", ...
             "SingleDiskOutput_DMD", "SingleDiskOutput_DMQ"]), [], 2);

        y = Controllers_case_table(ws_cond, ["SingleDiskInput_GMD", "SingleDiskInput_GMQ", "SingleDiskOutput_GMD", "SingleDiskOutput_GMQ"]);
        y = table2array(y);
        Controllers_case_table(ws_cond, "WorstCase_SingleDisk_GM") = table(y(sub2ind(size(y),1:size(y,1),I.Variables'))', 'VariableNames', {'WorstCase_SingleDisk_GM'});
        
        y = Controllers_case_table(ws_cond, ["SingleDiskInput_PMD", "SingleDiskInput_PMQ", "SingleDiskOutput_PMD", "SingleDiskOutput_PMQ"]);
        y = table2array(y);
        Controllers_case_table(ws_cond, "WorstCase_SingleDisk_PM") = table(y(sub2ind(size(y),1:size(y,1),I.Variables'))', 'VariableNames', {'WorstCase_SingleDisk_PM'});
    end           
    
    for f = 1:length(controller_types)
        
        ctrl_type = controller_types{f};

        ctrl_cond = contains(Controllers_case_table.("Case Desc."), ctrl_type);
        bcol = copper(n_weighting_cases + 1);
        bcol = flip(bcol(2:end, :), 1); % remove black, flip direction since Wugains go from low to high

       
        Wu_gains = [];
        wu_labels = {};
        for w_idx = 1:n_weighting_cases
            % wu = case_basis.WuGain.x{w_idx}.Numerator{1,1}(1);
            wu = weighting_cases(w_idx);
            wu_cond = Controllers_case_table.("A_Wu") == wu;
            Wu_gains = [Wu_gains,  wu];
            wu_labels{end + 1} = '';

            % data_gm_1 = []; 
            data_gm_2 = []; data_gm_3 = [];
            % data_pm_1 = []; 
            data_pm_2 = []; data_pm_3 = [];
            for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
                ws_cond = Controllers_case_table.("TunedWindSpeed") == LPV_CONTROLLER_WIND_SPEEDS(c_ws_idx);
                % x = Controllers_case_table(ctrl_cond & ws_cond, "WorstCase_SingleClassical_GM").Variables;
                % data_gm_1 = [data_gm_1 x(w_idx)];
                
                Controllers_case_table(ctrl_cond & ws_cond & wu_cond, ["SingleDiskInput_GMD", "SingleDiskInput_GMQ", "SingleDiskOutput_GMD", "SingleDiskOutput_GMQ"])
                

                x = Controllers_case_table(ctrl_cond & ws_cond & wu_cond, "WorstCase_SingleDisk_GM").Variables
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

            subplot(2, 2, 1); plot(LPV_CONTROLLER_WIND_SPEEDS, data_gm_2, 'LineWidth', 2, 'Color', bcol(w_idx, :), 'LineStyle', linestyles{f}); hold on;
            subplot(2, 2, 2); plot(LPV_CONTROLLER_WIND_SPEEDS, data_gm_3, 'LineWidth', 2, 'Color', bcol(w_idx, :), 'LineStyle', linestyles{f}); hold on;
            subplot(2, 2, 3); plot(LPV_CONTROLLER_WIND_SPEEDS, data_pm_2, 'LineWidth', 2, 'Color', bcol(w_idx, :), 'LineStyle', linestyles{f}); hold on;
            subplot(2, 2, 4); plot(LPV_CONTROLLER_WIND_SPEEDS, data_pm_3, 'LineWidth', 2, 'Color', bcol(w_idx, :), 'LineStyle', linestyles{f}); hold on;

        end
        
        ax = subplot(2, 2, 1); 
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlim([LPV_CONTROLLER_WIND_SPEEDS(1), LPV_CONTROLLER_WIND_SPEEDS(end)]);
        % xlabel('Linearized Wind Speed [m/s]', 'Interpreter', 'latex');
        ax.XTick = [];
        ax.XAxis.FontSize = SUBPLOT_SMALL_FONT_SIZE;
        ax.YAxis.FontSize = SUBPLOT_SMALL_FONT_SIZE;
        subtitle("Worst Case Loop-at-a-Time Disk Gain Margin [dB]", 'FontSize', SUBPLOT_BIG_FONT_SIZE, 'Interpreter', 'latex');
        
        ax = subplot(2, 2, 2); 
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlim([LPV_CONTROLLER_WIND_SPEEDS(1), LPV_CONTROLLER_WIND_SPEEDS(end)]);
        % xlabel('Linearized Wind Speed [m/s]', 'Interpreter', 'latex');
        ax.XTick = [];
        ax.XAxis.FontSize = SUBPLOT_SMALL_FONT_SIZE;
        ax.YAxis.FontSize = SUBPLOT_SMALL_FONT_SIZE;
        subtitle("Multi-Loop Disk Gain Margin [dB]", 'FontSize', SUBPLOT_BIG_FONT_SIZE, 'Interpreter', 'latex');

        ax = subplot(2, 2, 3);
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlim([LPV_CONTROLLER_WIND_SPEEDS(1), LPV_CONTROLLER_WIND_SPEEDS(end)]);
        xlabel('Linearized Wind Speed [m/s]', 'Interpreter', 'latex');
        ax.XAxis.FontSize = SUBPLOT_SMALL_FONT_SIZE;
        ax.YAxis.FontSize = SUBPLOT_SMALL_FONT_SIZE;
        subtitle("Worst Case Loop-at-a-Time Disk Phase Margin [deg]", 'FontSize', SUBPLOT_BIG_FONT_SIZE, 'Interpreter', 'latex');
        
        h = zeros(length(Wu_gains) + 2, 1);
        for i = 1:length(Wu_gains)
            % wu_labels{end + 1} = ['$A_{W_u} = $', ' ', num2str(Wu_gains(i))];
            wu_labels{end + 1} = '';
            h(i) = plot(ax, NaN, NaN, 'Color', bcol(i,:), 'LineStyle', '-', 'LineWidth', 2); hold on;
        end
        h(end - 1) = plot(ax, NaN, NaN, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 2); hold on;
        wu_labels{end + 1} = '$K_{adc}$';
        h(end) = plot(ax, NaN, NaN, 'Color', 'k', 'LineStyle', '-', 'LineWidth', 2);
        wu_labels{end + 1} = '$K_{err}$';
        legend(wu_labels, 'Interpreter', 'latex', 'FontSize', SUBPLOT_BIG_FONT_SIZE);

        ax = subplot(2, 2, 4);
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlim([LPV_CONTROLLER_WIND_SPEEDS(1), LPV_CONTROLLER_WIND_SPEEDS(end)]);
        xlabel('Linearized Wind Speed [m/s]', 'Interpreter', 'latex');
        ax.XAxis.FontSize = SUBPLOT_SMALL_FONT_SIZE;
        ax.YAxis.FontSize = SUBPLOT_SMALL_FONT_SIZE;
        subtitle("Multi-Loop Disk Phase Margin [deg]", 'FontSize', SUBPLOT_BIG_FONT_SIZE, 'Interpreter', 'latex');
        colormap(bcol);
        cbar = colorbar(ax, ...
                'TickLabelsMode', 'manual', 'TicksMode', 'manual', ...
                'Ticks', linspace(0, 1, length(Wu_gains)), ...
            'TickLabels', arrayfun(@num2str, Wu_gains, 'UniformOutput', 0), ...
            'TickLabelInterpreter', 'latex', 'FontSize', SUBPLOT_SMALL_FONT_SIZE);
            cbar.Label.String = '$A_{W_u}$';
            cbar.Label.Interpreter = 'latex';

    end
    set(gcf, 'Position', [0 0 1500 900]);
    fig_dir = "/Users/ahenry/Documents/MATLAB";
    savefig(gcf, fullfile(fig_dir, 'fullorder_rob_margins.fig'));
    saveas(gcf, fullfile(fig_dir, 'fullorder_rob_margins.png'));
end


