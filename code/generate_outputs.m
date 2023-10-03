init_hinf_controller;

%% Load Data
load(fullfile(mat_save_dir, [sim_type, '_full_controller_cases']));
if EXTREME_K_COLLECTION
    load(fullfile(mat_save_dir, 'Extreme_Controllers_case_table.mat'));
    load(fullfile(mat_save_dir, 'Extreme_Controllers_case_list.mat'));
else
    load(fullfile(mat_save_dir, 'Optimal_Controllers_case_table.mat'));
    load(fullfile(mat_save_dir, 'Optimal_Controllers_case_list.mat'));
end

%% Plot transfer functions and weighting functions for full-order controller
if 0
    cc = find(full_controller_case_basis.WindSpeedIndex.x == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED));
    n_weighting_cases = (FullOrderControllers_n_cases / length(full_controller_case_basis.WindSpeedIndex.x));

    %% TODO Plot transfer functions of KSo, Ti; So, GSi vs f for single wind speed on
    % 2*2 bodeplot OR singular values for open-loop vs closed-loop OUTPLOT
    % QUESTION MANUEL do these plots make sense, bound on GSi seems
    % ineffective, should I choose a single singular value to plot, which
    % one? Messy to plot gamma bounds for all weighting cases, do you think
    % its necessary?
    % for each controller type: robustness, adc, y_mse
    % controller over wind speedfs = fieldnames(case_basis.W1Gain)';
    for f = fieldnames(full_controller_case_basis.W1Gain)'
        figure;
        ax_KSo = subplot(2, 2, 1);
        ax_Ti = subplot(2, 2, 2);
        ax_So = subplot(2, 2, 3);
        ax_GSi = subplot(2, 2, 4);
        ax_KSo_del_idx = []; ax_Ti_del_idx = []; 
        ax_So_del_idx = []; ax_GSi_del_idx = [];
        Wu_gains = [];
        for w_idx = 1:n_weighting_cases
            % bodemag(Wu_tmp, We_tmp, W1_tmp, W2_tmp); legend('Wu', 'We', 'W1', 'W2');
           
            Wu_gains = [Wu_gains,  ss(Controllers_case_list(w_idx).WuGain.x).D(1, 1)];
            Wu_tmp = Controllers_case_list(w_idx).WuGain.x * Wu;
            We_tmp = Controllers_case_list(w_idx).WeGain.(f{1}) * We;
            W1_tmp = Controllers_case_list(w_idx).W1Gain.(f{1}) * W1;
            W2_tmp = Controllers_case_list(w_idx).W2Gain.(f{1}) * W2;
    
            SF = loopsens(-Plant(:, :, cc), ...
                -Controllers_case_list(w_idx).Controller_scaled.(f{1})(:, :, cc));
    
            % sys_KSi = SF.Si * Controllers_case_list(w_idx).Controller_scaled.(f{1})(:, :, cc) * W1_tmp;
            sys_KSo = SF.CSo * W1_tmp;
            sys_Ti = SF.Ti * W2_tmp;
            % sys_Ti2 = (Controllers_case_list(w_idx).Controller_scaled(:, :, cc) * Plant(:, :, cc)) / ...
            %           (eye(size(Plant(:, :, cc))) + Controllers_case_list(w_idx).Controller_scaled(:, :, cc) * Plant(:, :, cc)) * W2_tmp;
            
            sys_So = SF.So * W1_tmp;
            % sys_So_cl2 = inv(eye(size(Plant(:, :, cc))) + Plant(:, :, cc) * Controllers_case_list(w_idx).Controller_scaled(:, :, cc)) * W1_tmp;
            % sys_GSi = Plant(:, :, cc) * SF.So * W2_tmp;
            sys_GSi = SF.PSi * W2_tmp;
            
            % [s, w] = sigma(sys_KSi);
            sigmaplot(ax_KSo, sys_KSo, sigma_plot_opt); hold on;
            a = findobj(ax_KSo, 'Type', 'line');
            ax_KSo_del_idx = [ax_KSo_del_idx, length(ax_KSo_del_idx) + 2:2];
            % legend(ax_KSi, ['Wu = ', num2str(Wu_gains(end))], 'Bound', 'Location', 'westoutside');
            subtitle(ax_KSo, '$KS_o$', 'Interpreter','latex');
            
            % [s, w] = sigma(sys_Ti);
            sigmaplot(ax_Ti, sys_Ti, sigma_plot_opt); hold on;
            a = findobj(ax_Ti, 'Type', 'line');
            ax_Ti_del_idx = [ax_Ti_del_idx, length(ax_Ti_del_idx) + 2:2];
            
            % legend(['Wu = ', num2str(Wu_gains(end))], 'Bound');
            subtitle(ax_Ti, '$T_i$', 'Interpreter','latex');
            
            sigmaplot(ax_So, sys_So, sigma_plot_opt); hold on;
            a = findobj(ax_So, 'Type', 'line');
            ax_So_del_idx = [ax_So_del_idx, length(ax_So_del_idx) + 2:2];
            
            % legend(['Wu = ', num2str(Wu_gains(end))], 'Bound');
            subtitle(ax_So, '$S_o$', 'Interpreter','latex');

            sigmaplot(ax_GSi, sys_GSi, sigma_plot_opt); hold on;
            a = findobj(ax_GSi, 'Type', 'line');
            ax_GSi_del_idx = [ax_GSi_del_idx, length(ax_GSi_del_idx) + 2:2];
            
            % legend(['Wu = ', num2str(Wu_gains(end))], 'Bound');
            subtitle(ax_GSi, '$GS_i$', 'Interpreter','latex');

            % sigmaplot(ax_KSo, inv(Wu_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc), '--', sigma_plot_opt);
            % sigmaplot(ax_Ti, inv(Wu_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc), '--', sigma_plot_opt);
            % sigmaplot(ax_So, inv(We_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc), '--', sigma_plot_opt);
            % sigmaplot(ax_GSi, inv(We_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc), '--', sigma_plot_opt);
        end
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

        axesHandlesToChildObjects = findobj(ax_KSo, 'Type', 'line');
        delete(axesHandlesToChildObjects(ax_KSo_del_idx)); % remove first singluar value
        axesHandlesToChildObjects = findobj(ax_Ti, 'Type', 'line');
        delete(axesHandlesToChildObjects(ax_Ti_del_idx)); % remove first singluar value
        axesHandlesToChildObjects = findobj(ax_So, 'Type', 'line');
        delete(axesHandlesToChildObjects(ax_So_del_idx)); % remove first singluar value
        axesHandlesToChildObjects = findobj(ax_GSi, 'Type', 'line');
        delete(axesHandlesToChildObjects(ax_GSo_del_idx)); % remove first singluar value
    
        set(gcf, 'Position', [0 0 1500 900]);
        savefig(gcf, fullfile(fig_dir, [f{1}, '_fullorder_singular_vals.fig']));
        saveas(gcf, fullfile(fig_dir, [f{1}, '_fullorder_singular_vals.png']));
    
    end
    
    %% Plot classical, disk margins for controller tuned for different wind speeds OUTPLOT
    for f = fieldnames(full_controller_case_basis.W1Gain)'
        ctrl_cond = contains(Controllers_case_table.("Case Desc."), f{1});

        figure;
        % top row: gain margins vs wind speed for worst case single channel
        % classical margin, worst case single channel disk margin, multi
        % input/output disk margin
        % bottom row: phase margins vs wind speed for worst case single channel
        % classical margin, worst case single channel disk margin, multi
        % input/output disk margin
        ax_gm_1 = subplot(2, 3, 1);
        ax_gm_2 = subplot(2, 3, 2);
        ax_gm_3 = subplot(2, 3, 3);
        ax_pm_1 = subplot(2, 3, 4);
        ax_pm_2 = subplot(2, 3, 5);
        ax_pm_3 = subplot(2, 3, 6);
        Wu_gains = [];
        for w_idx = 1:n_weighting_cases
            wu = ss(Controllers_case_list(w_idx).WuGain.x).D(1, 1);
            Wu_gains = [Wu_gains,  wu];

            data_gm_1 = []; data_gm_2 = []; data_gm_3 = [];
            data_pm_1 = []; data_pm_2 = []; data_pm_3 = [];
            for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
                ws_cond = Controllers_case_table.("TunedWindSpeed") == LPV_CONTROLLER_WIND_SPEEDS(c_ws_idx);
                x = Controllers_case_table(ctrl_cond & ws_cond, "WorstCase_SingleClassical_GM").Variables;
                data_gm_1 = [data_gm_1 x(w_idx)];
                x = Controllers_case_table(ctrl_cond & ws_cond, "WorstCase_SingleDisk_GM").Variables;
                data_gm_2 = [data_gm_2 x(w_idx)];
                x = Controllers_case_table(ctrl_cond & ws_cond, "MultiDiskIO_GM").Variables;
                data_gm_3 = [data_gm_3 x(w_idx)];

                x = Controllers_case_table(ctrl_cond & ws_cond, "WorstCase_SingleClassical_PM").Variables;
                data_pm_1 = [data_pm_1 x(w_idx)];
                x = Controllers_case_table(ctrl_cond & ws_cond, "WorstCase_SingleDisk_PM").Variables;
                data_pm_2 = [data_pm_2 x(w_idx)];
                x = Controllers_case_table(ctrl_cond & ws_cond, "MultiDiskIO_PM").Variables;
                data_pm_3 = [data_pm_3 x(w_idx)];
            end
            subplot(2, 3, 1); plot(LPV_CONTROLLER_WIND_SPEEDS, data_gm_1); hold on;
            subplot(2, 3, 2); plot(LPV_CONTROLLER_WIND_SPEEDS, data_gm_2); hold on;
            subplot(2, 3, 3); plot(LPV_CONTROLLER_WIND_SPEEDS, data_gm_3); hold on;
            subplot(2, 3, 4); plot(LPV_CONTROLLER_WIND_SPEEDS, data_pm_1); hold on;
            subplot(2, 3, 5); plot(LPV_CONTROLLER_WIND_SPEEDS, data_pm_2); hold on;
            subplot(2, 3, 6); plot(LPV_CONTROLLER_WIND_SPEEDS, data_pm_3); hold on;

        end
        % TODO no legend save seperately
        subplot(2, 3, 1); 
        % legend([arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
        %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex'); 
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlabel('Wind Speed [m/s]');
        subtitle("Worst Case Classical Gain Margin [dB]");

        subplot(2, 3, 2); 
        % legend([arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
        %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex'); 
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlabel('Wind Speed [m/s]');
        subtitle("Worst Case Loop-at-a-Time Disk Gain Margin [dB]");

        subplot(2, 3, 3); 
        % legend([arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
        %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex'); 
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlabel('Wind Speed [m/s]');
        subtitle("Multiloop Disk Gain Margin [dB]");

        subplot(2, 3, 4); 
        % legend([arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
        %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex'); 
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlabel('Wind Speed [m/s]');
        subtitle("Worst Case Classical Phase Margin [deg]");

        subplot(2, 3, 5); 
        % legend([arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
        %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex'); 
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlabel('Wind Speed [m/s]');
        subtitle("Worst Case Loop-at-a-Time Disk Phase Margin [deg]");

        subplot(2, 3, 6); 
        % legend([arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
        %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex'); 
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlabel('Wind Speed [m/s]');
        subtitle("Multiloop Disk Phase Margin [deg]");
        % 
        % ax1 = subplot(2, 3, 1); ax2 = subplot(2, 3, 2); ax3 = subplot(2, 3, 3);
        % linkaxes([ax1, ax2, ax3]);
        % ax1 = subplot(2, 3, 4); ax2 = subplot(2, 3, 5); ax3 = subplot(2, 3, 6);
        % linkaxes([ax1, ax2, ax3]);

        % sgtitle([f{1}, ' Controller'], 'Interpreter', 'latex'); 
        set(gcf, 'Position', [0 0 1500 900]);
        savefig(gcf, fullfile(fig_dir, [f{1}, '_fullorder_rob_margins.fig']));
        saveas(gcf, fullfile(fig_dir, [f{1}, '_fullorder_rob_margins.png']));
    end

    %% Plot robustness margins OUTPLOT
    
    figure;
    tcf = tiledlayout(1, 2);
    for w_idx = 1:n_weighting_cases
        nexttile(1);
        outputs = cell(1, length(full_controller_case_basis.WindSpeedIndex));
        [outputs{:}] = mag2db(Controllers_case_list(w_idx).MMio.GainMargin(2));
        scatter(LPV_CONTROLLER_WIND_SPEEDS(full_controller_case_basis.WindSpeedIndex), cell2mat(outputs));
        set(gca,'yscale','log');
        title('Multi-Input-Output Gain Margin');
        ylabel('dB');
        hold on;

        nexttile(2);
        outputs = cell(1, length(full_controller_case_basis.WindSpeedIndex));
        [outputs{:}] = Controllers_case_list(w_idx).MMio.PhaseMargin(2);
        scatter(LPV_CONTROLLER_WIND_SPEEDS(full_controller_case_basis.WindSpeedIndex), cell2mat(outputs));
        title('Multi-Input-Output Phase Margin');
        ylabel('deg');
        hold on;

        % nexttile(3);
        % outputs = cell(1, length(LPV_CONTROLLER_WIND_SPEEDS));
        % [outputs{:}] = Controllers_case_list(w_idx).DMo(1, :).DiskMargin;
        % plot(LPV_CONTROLLER_WIND_SPEEDS, cell2mat(outputs));
        % title('DM_o D Loop');
        % hold on;
        % 
        % nexttile(4);
        % outputs = cell(1, length(LPV_CONTROLLER_WIND_SPEEDS));
        % [outputs{:}] = Controllers_case_list(w_idx).DMo(2, :).DiskMargin;
        % plot(LPV_CONTROLLER_WIND_SPEEDS, cell2mat(outputs));
        % title('DM_o Q Loop');
        % hold on;
    end
    legend([arrayfun(@(n) ['Case ', num2str(n)], ... 
    1:n_weighting_cases, 'UniformOutput', false) {'', '', '', ''}], ...
    'NumColumns', 2);
    xticks(LPV_CONTROLLER_WIND_SPEEDS);
    set(gcf, 'Position', [0 0 1500 900]);
    savefig(gcf, fullfile(fig_dir, 'fullorder_diskmargins.fig'));
    saveas(gcf, fullfile(fig_dir, 'fullorder_diskmargins.png'));

    for w_idx = 1:n_weighting_cases
        close all;

        %% Plot Generalized Plant transfer functions OUTPLOT
        GenPlant_inner = inv(Controllers_case_list(w_idx).Wout(:, :, cc)) ...
            * Controllers_case_list(w_idx).GenPlant(:, :, cc) ...
            * inv(Controllers_case_list(w_idx).Win(:, :, cc));
        GenPlant_inner.OutputName = Controllers_case_list(w_idx).GenPlant(:, :, cc).OutputName;
        GenPlant_inner.InputName = Controllers_case_list(w_idx).GenPlant(:, :, cc).InputName;

        % GenPlant_inner.InputGroup.dist = 1:4;
        GenPlant_inner.InputGroup.dist = find(ismember(...
            GenPlant_inner.InputName, ...
            {'M_d Reference', 'M_q Reference', ...
            '\beta_d Disturbance', '\beta_q Disturbance'}))';
    
        GenPlant_inner.InputGroup.acts = find(ismember(...
            GenPlant_inner.InputName, ...
            {'\beta_d Control Input', '\beta_q Control Input'}))';
            
        GenPlant_inner.OutputGroup.perf = find(ismember(...
            GenPlant_inner.OutputName, ...
            {'Weighted \beta_d Control Input', 'Weighted \beta_q Control Input', ...
            'Weighted M_d Tracking Error', 'Weighted M_q Tracking Error'}))';
            % 'Weighted M_d Output', 'Weighted M_q Output'}))';
    
        GenPlant_inner.OutputGroup.meas = find(ismember(...
            GenPlant_inner.OutputName, ...
            {'Measured M_d Tracking Error', 'Measured M_q Tracking Error'}))';
         
        Win_arr = {};
        Wout_arr = {};
        for i = 1:size(Controllers_case_list(w_idx).Win, 1)
            Win_arr{end+1} = Controllers_case_list(w_idx).Win(i, i, cc);
        end
        for i = 1:size(Controllers_case_list(w_idx).Wout, 1)
            Wout_arr{end+1} = Controllers_case_list(w_idx).Wout(i, i, cc);
        end
        % yellow should touch blue at high freq from
        % bldpitch disturbance to tracking error
        % no active shaping in off-diag elements from bldpitch disturbance to
        % tracking error
        % performance loops are bottom two rows, control effort top two rows
        % (nothing shaped on right col, shaping occuring on left col)
        % So (bottom left corner, d1->ze) and GSi (bottom right corner,
        % d2->ze) have been shifted
        loopData = getLoopData(GenPlant_inner, ...
            Controllers_case_list(w_idx).Controller(:, :, cc), ...
            Win_arr, Wout_arr);
        w = logspace(-2, 4, 400);
        [h, loopData] = plotLoopBode(loopData, [], [], w, bode_plot_opt);
    
        axh = findall(gcf, 'type', 'axes');
        for a = 1:length(axh)
            if loopData.peaks.wGAM > 0
            xline(axh(a), [loopData.peaks.wGAM]);
            axh(a).YLabel.Rotation = 0;
            end
        end
        % legend('W_{out}^{-1}\gamma', 'Open-Loop', 'Closed-Loop', '');
        set(gcf, 'Position', [0 0 1500 900]);
        savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_genplant_bodemag.fig']));
        saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_genplant_bodemag.png']));

        %% Plot transfer functions
        figure;
        Wu_tmp = Controllers_case_list(w_idx).WuGain * Wu;
        We_tmp = Controllers_case_list(w_idx).WeGain * We;
        W1_tmp = Controllers_case_list(w_idx).W1Gain * W1;
        W2_tmp = Controllers_case_list(w_idx).W2Gain * W2;
        SF = loopsens(-Plant(:, :, cc), ...
            -Controllers_case_list(w_idx).Controller_scaled(:, :, cc));
        % sys_KSi = SF.Si * Controllers_case_list(w_idx).Controller_scaled(:, :, cc) * W1_tmp;
        sys_KSo = SF.CSo * W1_tmp;
        sys_Ti = SF.Ti * W2_tmp;
        sys_So = SF.So * W1_tmp;
        % sys_So_ol = Plant(:, :, cc) * W1_tmp;
        % sys_GSi = SF.So * Plant(:, :, cc) * W2_tmp;
        sys_GSi = Sf.PSi * W2_tmp;

        % Plot output sensitivity function and its weighting matrix, dy-y or
        % n->e or r->e (ideally 0)
        % S and T are coupled : S + T = 1, so choosing 1 determines the other
        % need penalty on both bc want both to have small magnitdues at particular
        % frequencies (ie could add 2 complex numbers with large magnitudes at
        % high phases that add to 1, or have one with large mag and the other with very small in alignment)
        % OpenLoop case: So = 1, ClosedLoop case: can shape So, ideally to 0.
        % Real So: hpf with roll-off with high-freq gain of 1.
        % So = e/W1d1, output disturbance -> tracking error
        subplot(2,2,3);
        % sys.InputName = GenPlant.InputName(1:2); % output disturbance/reference
        sys_So_cl.InputName = GenPlant.InputName(1:2); % output disturbance/reference
        % We_tmp.OutputName = {'M_d Reference', 'M_q Reference'};
        sys_So_cl.OutputName = We_tmp.InputName;
        % We_tmp.InputName = {'M_d Tracking Error', 'M_q Tracking Error'};
        h = bodeplot(...
            sys_So_cl, ...
            inv(We_tmp) * Controllers_case_list(w_idx).gamma(cc), w, bode_plot_opt);
        hYLabel = get(gca,'YLabel');
        set(hYLabel,'rotation',0,'VerticalAlignment','middle')
        title('Output Sensitivity Function, S_o');

        axh = findall(gcf, 'type', 'axes');
        % for a = 1:length(axh)
        %     if loopData.peaks.wGAM > 0
        %     xline(axh(a), [loopData.peaks.wGAM]);
        %     end
        % end
        xline(axh(3), omega_1P_rad * [2, 3]);
        xticks(axh(3), [1e-1, 1e0, 1e1, 1e2, 1e3, 1e4]);
        xline(axh(5), omega_1P_rad * [2, 3]);
        xticks(axh(5), [1e-1, 1e0, 1e1, 1e2, 1e3, 1e4]);
        xline(axh(7), omega_1P_rad * [2, 3]);
        xticks(axh(7), [1e-1, 1e0, 1e1, 1e2, 1e3, 1e4]);
        xline(axh(9), omega_1P_rad * [2, 3]);
        xticks(axh(9), [1e-1, 1e0, 1e1, 1e2, 1e3, 1e4])

        % set(gcf, 'Position', [0 0 1500 900]);
        legend('S_o W_1', 'W_e^{-1}\gamma');
        % savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_So_bodemag.fig']));
        % saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_So_bodemag.jpg']));
        
        % Plot complementary output sensitivity function and its weighting matrix
        % To = y/W2d2, input disturbance -> control input
        % figure;
        % % sigma(CL_full_tuned(:, :, 1, cc), ss(gamma_full_tuned(1, cc)));
        % sys = SF.To * W2_tmp; % requirements defined on output side
        % sys.InputName = GenPlant.InputName(1:2); % output disturbance/references
        % sys.OutputName = GenPlant.OutputName(5:6); % output
        % sys.InputName = {'M_d Reference', 'M_q Reference'};
        % Wy.OutputName = {'M_d Reference', 'M_q Reference'};
        % sys.OutputName = {'M_d Output', 'M_q Output'};
        % Wy.InputName = {'M_d Output', 'M_q Output'};
        % bodeplot(...
        %     sys, ...
        %     inv(Wy) * Controllers_case_list(w_idx).gamma(cc), w, bode_plot_opt);
        % title('Complementary Output Sensitivity Function, T_o');
        % axh = findall(gcf, 'type', 'axes');
        % for a = 1:length(axh)
        %     xline(axh(a), [loopData.peaks.wGAM]);
        % end
        % set(gcf, 'Position', [0 0 1500 900]);
        % legend('T_o W_2', 'W_y^{-1}\gamma');
        % savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'To_full_bodemag.fig']));
        % saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'To_full_bodemag.png']));
        
        % Plot complementary input sensitivity function and its weighting matrix
        % Ti = -u/W2d2, input disturbance -> control input
        subplot(2,2,2);
        sys_Ti.InputName = GenPlant.InputName(3:4); % control disturbance
        sys_Ti.OutputName = Wu_tmp.InputName; % control input
        sys_Ti.InputName = {'\beta_d Disturbance', '\beta_q Disturbance'};
        % Wu_tmp.OutputName = {'\beta_d Disturbance', '\beta_q Disturbance'};
        sys_Ti.OutputName = {'\beta_d Control Input', '\beta_q Control Input'};
        % Wu_tmp.InputName = {'\beta_d Control Input', '\beta_q Control Input'};
        bodeplot(...
            sys_Ti, ...
            inv(Wu_tmp) * Controllers_case_list(w_idx).gamma(cc), w, bode_plot_opt);
        title('Complementary Input Sensitivity Function, T_i');
        axh = findall(gcf, 'type', 'axes');
        for a = 1:length(axh)
            xline(axh(a), [loopData.peaks.wGAM]);
        end
        set(gcf, 'Position', [0 0 1500 900]);
        legend('T_i W_2', 'W_u^{-1}\gamma');
        % savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'Ti_full_bodemag.fig']));
        % saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'Ti_full_bodemag.png']));
    
        % Plot GSi with two weighting functions
        % GSi = -e/W2d2, input disturbance -> tracking error
        subplot(2,2,4);
        sys_GSi.InputName = GenPlant.InputName(3:4); % control disturbance
        sys_GSi.OutputName = We_tmp.InputName; % tracking error
        bodeplot(...
            sys_GSi, ...
            inv(We_tmp) * Controllers_case_list(w_idx).gamma(cc), w, bode_plot_opt);
        title('GS_i Function');
        axh = findall(gcf, 'type', 'axes');
        for a = 1:length(axh)
            xline(axh(a), [loopData.peaks.wGAM]);
        end
        set(gcf, 'Position', [0 0 1500 900]);
        legend('GS_i W_2', 'W_e^{-1}\gamma', 'W_y^{-1}\gamma');
        % savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'GSi_full_bodemag.fig']));
        % saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'GSi_full_bodemag.png']));

        % Plot KS_o with two weighting functions
        % KSo = u/W1d1, output disturbance -> control input
        subplot(2,2,1);
        sys_KSo.InputName = GenPlant.InputName(1:2); % output disturbance/reference
        sys_KSo.OutputName = Wu_tmp.InputName; % control input
        bodeplot(...
            sys_KSo, ...
            inv(Wu_tmp) * Controllers_case_list(w_idx).gamma(cc), w, ...
             bode_plot_opt);
        title('KS_o Function');
        axh = findall(gcf, 'type', 'axes');
        for a = 1:length(axh)
            xline(axh(a), [loopData.peaks.wGAM]);
        end
        set(gcf, 'Position', [0 0 1500 900]);
        legend('KS_o W_1', 'W_u^{-1}\gamma');
        
        savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_tfs_bodemag.fig']));
        saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_tfs_bodemag.png']));
    end
    
    %% Plot tuned controller for this weighting case OUTPLOT
    cc = find(full_controller_case_basis.WindSpeedIndex.x == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED));
    % first case = black = highest Wu_gain
    for f = fieldnames(case_basis.W1Gain)'
        ctrl_cond = contains(Controllers_case_table.("Case Desc."), f{1});

        figure;
        Wu_gains = [];

        clear controllers;
        % 
        for w_idx = 1:n_weighting_cases

            wu = ss(Controllers_case_list(w_idx).WuGain.x).D(1, 1);
            Wu_gains = [Wu_gains,  wu];
        
            K_tmp = Controllers_case_list(w_idx).Controller.(f{1})(:, :, cc);
            K_tmp.InputName = {'$M_d$ Tracking Error', '$M_q$ Tracking Error'};
            K_tmp.OutputName = {'$\beta_d$ Control Input', '$\beta_q$ Control Input'};
            controllers(:, :, w_idx) = K_tmp;

        end

        bcol = copper(n_weighting_cases); % Define the color order based on the number of models

        % Plot baseline and tuned controllers
        bodeplot(controllers, 'b', bode_plot_opt);

        % Find handles of all lines in the figure that have the color blue
        axh = findall(gcf, 'type', 'axes');
        
        for ax_idx = [3, 5, 7, 9]
            xline(axh(ax_idx), omega_1P_rad * HARMONICS);
            blineHandle = findobj(axh(ax_idx), 'Type', 'line', '-and', 'Color', 'b');
            for w_idx = 1:n_weighting_cases
                % Change the color to the one you defined
                set(blineHandle(w_idx), 'Color', bcol(w_idx, :));
            end
        end

        set(gcf, 'Position', [0 0 1500 900]);
        savefig(gcf, fullfile(fig_dir, [f{1} '_fullorder_controller_bodemag.fig']));
        saveas(gcf, fullfile(fig_dir, [f{1} '_fullorder_controller_bodemag.png']));
    end

end

%% TODO 1x3 plot for each metric: mulitIO_dm, adc, M1 RMSE (y-axis) vs. A_w for extreme controller sweep
 % one curve corresponds to increasing A_w1, A_w2, A_we or A_wu while
 % holding all others at minimum
 if EXTREME_K_COLLECTION

     figure;
     
     metrics = {'MultiDiskIO_DM', 'ADC', 'RootMycBlade1 RMSE'};
     for m = 1:length(metrics)
         subplot(1, 3, m);
         w1_cond = (Controllers_simulation_case_table.("A_W2") == case_basis.W2Gain(1).Numerator{1});
         Controllers_simulation_case_table();
     end
 end

%% TODO 2x3 Plot for each controller type showing change in mean
 % ADC, M1 RMSE (y-axis) relative to no ipc case for each mean wind speed
 % (x-axis)
 figure(1);
 wind_speeds = sort(unique(Controllers_simulation_case_agg_table.WindMean));

 metrics = {'ADC', 'RootMycBlade1 RMSE'};
 agg_metrics = {'mean_ADC', 'mean_RootMycBlade1 RMSE'};
 label_metrics = {'$ADC$', '$M_1 RMSE$'};
 controller_types = {'rob', 'adc', 'y_mse'};
 label_controller_types = {'$K_{rob}$', '$K_{adc}$', '$K_{err}$'};

 for m = 1:length(metrics)
     for c = 1:length(controller_types)
         plot_idx = (m - 1) * length(controller_types) + c;
         subplot(length(metrics), length(controller_types), plot_idx);
         data.noipc = sortrows(UntunedControllers_simulation_case_table(strcmp(UntunedControllers_simulation_case_table.("Case Desc."), "noipc"), :), 'WindMean');
         data.noipc = data.noipc(:, {'Case Desc.', 'WindMean', metrics{m}});
         data.baseline_controller = sortrows(UntunedControllers_simulation_case_table(strcmp(UntunedControllers_simulation_case_table.("Case Desc."), "baseline_controller"), :), 'WindMean');
         data.baseline_controller = data.baseline_controller(:, {'Case Desc.', 'WindMean', metrics{m}});
        
         y = 100 * (data.baseline_controller.(metrics{m}) - data.noipc.(metrics{m})) ./ data.noipc.(metrics{m});
         plot(wind_speeds, y, 'k--', 'LineWidth', 2); hold on;
         
         data.tuned = sortrows(Controllers_simulation_case_agg_table(contains(Controllers_simulation_case_agg_table.("Case Desc."), controller_types{c}), :), 'WindMean');
         data.tuned = data.tuned(:, {'Case Desc.', 'WindMean', agg_metrics{m}});
         data.tuned = sortrows(data.tuned, "Case Desc.");

         cases = unique(data.tuned.("Case Desc."));
         bcol = flip(copper(length(cases) + 1), 1);
            % TODO why are ADC values same across differetn WindMean values
            % for same controller... check wind field time-series from
            % different simulations...
         for cc = 1:length(cases)
             case_desc = cases{cc};
             y = 100 * (data.tuned(strcmp(data.tuned.("Case Desc."), case_desc), agg_metrics{m}).Variables - data.noipc.(metrics{m}) ./ data.noipc.(metrics{m}));
             plot(wind_speeds, y, 'Color', bcol(cc, :), 'LineWidth', 2); hold on;
         end

         sortrows(Controllers_simulation_case_agg_table, 'WindMean');
         tuned_data(:, agg_metrics{m});

         xticks(wind_speeds); xlim([wind_speeds(1), wind_speeds(end)]);
         
         ylabel('[%]', 'Interpreter', 'latex');
         if m == 2
            xlabel('Mean Wind Speed [m/s]', 'Interpreter', 'latex')
         end
         l = ['Change in ', label_metrics{m}, ' for ' ,label_controller_types{c}];
         subtitle(l, 'Interpreter', 'latex');
     end
 end

%% TODO 1x3 Plot for each controller type showing mean worst case
   % stability margins for range of Wu values (y-axis) for each tuned wind speed (x-axis)

%% TODO 1x3 multibar plot for each controller type showing change in
   % DEL relative to no ipc case for range of Wu values (y-axis) for different loads (x-axis)

% run python DEL analysis
sim_types = 'extreme_k_cases_turbsim noipc_turbsim';
status = system(['/Users/aoifework/miniconda3/envs/weis_dev/bin/python3 ', fullfile(project_dir, 'postprocessing', 'main.py'), ' -st ', sim_types]);

% read python DEL analysis output
del_data = readtable(fullfile(postprocessing_save_dir, 'DELs.csv'));

% plot percentage DEL reduction @ each mean wind speed for blade
% out-of-plane load, nacelle yaw moment, nacelle pitch moment, shaft
% bending moment around y-axis
figure;
loads = cell(length(del_data.Properties.VariableNames) - 1, 1);
[loads{:}] = del_data.Properties.VariableNames{2:end};
noipc_dels = del_data(contains(del_data.("Var1"), 'noipc_turbsim'), loads).Variables;
tuned_dels = del_data(contains(del_data.("Var1"), 'extreme_k_cases_turbsim'), loads).Variables;
delta_del = ((tuned_dels - noipc_dels) / noipc_dels) * 100;
% bar(loads, delta_del);
% find mean wind speed associated with this case
ux_vals = [];
for outdata_fn = del_data.("Var1")
    % values = load(outdata_fn);
    % ux = getData(values, OutList, 'WindVelX1');
    split_str = split(outdata_fn{1}, '_');
    case_idx = str2num(split_str{2});
    ux_vals = [ux_vals, case_list(case_idx).InflowWind.HWindSpeed];
    % TODO need to get noipc/baseline hwindspeeds
end
del_data.("HWindSpeed") = ux_vals;

bar(loads, tuned_dels);

%% TODO Nx3 Plot for each controller type showing PSD (y-axis) for N different loads

% define loads of concern
% Blade root O/P, Shaft My, Yaw bearing My/Mz, Hub, Nacelle, Main Bearing,
% Blade Flapwise/Edgewise, Blade Oop/Ip, Tower
exc_blade_fields = {'Time', 'BldPitch1', 'BldPitch2', 'BldPitch3', ...
    'Azimuth', 'RotSpeed', 'Wind1VelX', 'Wind1VelY', 'Wind1VelZ', ...
    '-React', 'Alpha', 'Rt', 'M2N', 'M8N', 'GenTq'};

% 'Spn2MLxb1', 'Spn2MLyb1', ... % Blade 1 local edgewise/flapwise moment at span station 2
% 'RootFxb1', 'RootFyb1', ... % Blade 1 flapwise/edgewise shear force at the blade root
% 'YawBrFxp', 'YawBrFyp', 'YawBrFzp', ... % Tower-top / yaw bearing fore-aft/side-t0-side shear/axial force
% 'TwrBsFxt', 'TwrBsFyt', 'TwrBsFzt', ... % Tower base fore-aft/side-to-side shear/axial force
                              
blade_op_arr = {'OoPDefl1', 'IPDefl1', 'TwstDefl1', ...
          'RotThrust', ...
          'TTDspFA', 'TTDspSS', 'TTDspTwst', ... % Tower-top / yaw bearing fore-aft/side-to-side/angular torsion deflection
          'RootMxb1', 'RootMyb1', ... % Blade 1 edgewise/flapwise moment 
          'RootMyc1', 'RootMzc1', ... % Blade 1 out-of-plane/pitching moment
          'LSSGagMya', 'LSSGagMza', ... % Rotating low-speed shaft bending moment at the shaft's strain gage (about ya/za axis)
          'YawBrMxp', 'YawBrMyp', 'YawBrMzp', ... % Nonrotating tower-top / yaw bearing roll/pitch/yaw moment
          'TwrBsMxt', 'TwrBsMyt', 'TwrBsMzt' ... % Tower base roll (or side-to-side)/pitching (or fore-aft)/yaw moment 
          };

exc_dq_fields = {'BldPitchD', 'BldPitchQ'};
dq_op_arr = {};
for l = dqOutList'
    if sum(strmatch(l, exc_dq_fields)) == 0 && (strcmp(l{1}(end), 'D') || strcmp(l{1}(end), 'Q'))
        dq_op_arr = [dq_op_arr, l{1}];
    end
end

if COMPUTE_FFT
    % for each simulation
    blade_op_indices = [];
    for i = 1:length(blade_op_arr)
        blade_op_indices = [blade_op_indices strmatch(blade_op_arr{i}, OutList)];
    end
    dq_op_indices = [];
    for i = 1:length(dq_op_arr)
        dq_op_indices = [dq_op_indices strmatch(dq_op_arr{i}, dqOutList)];
    end
    
    values = load(sim_out_list.noipc(1).save_fn);
    values = values.OutData.Data;
    dqValues = mbcTransformOutData(values, OutList);
    op_data_blade_noipc = values(cut_transients/DT + 1:end, blade_op_indices);
    op_data_cdq_noipc = dqValues(cut_transients/DT + 1:end, blade_op_indices);

    % TODO make sure this study is being done for equivalent wind fields
    for c = 1:length(sim_out_list.controller)
        sprintf(['Processing Simulation ' num2str(c)]);
        % fetch time-series of list of loads of concern
        % [Channels, ChanName, ChanUnit, FileID, DescStr] = ReadFASTbinary(strrep(sim_out_list(c).FAST_InputFileName, 'fst', 'outb'), 'n');
        values = load(sim_out_list.controller(c).save_fn);
        values = values.OutData.Data;
        dqValues = mbcTransformOutData(values, OutList); 

        op_data_blade_controller = values(cut_transients/DT + 1:end, blade_op_indices);
        op_data_cdq_controller = dqValues(cut_transients/DT + 1:end, blade_op_indices);
        

        % compute fft for rotating domain corresponding to Blade 1
        fft_blade_controller(:, :, c) = fft(op_data_blade_controller, size(op_data_blade_controller, 1), 1);
        fft_blade_noipc(:, :, 1) = fft(op_data_blade_noipc, size(op_data_blade_noipc, 1), 1);
    
        % compute fft for cdq domain loads
        fft_cdq_controller(:, :, c) = fft(op_data_cdq_controller, size(op_data_cdq_controller, 1), 1);
        fft_cdq_noipc(:, :, 1) = fft(op_data_cdq_noipc, size(op_data_cdq_noipc, 1), 1);
    end
    fft_blade.controller = fft_blade_controller;
    fft_blade.noipc = fft_blade_noipc;
    fft_cdq.controller = fft_cdq_controller;
    fft_cdq.noipc = fft_cdq_noipc;
    
    if strcmp(WIND_TYPE, 'turbsim')
        save(fullfile(project_dir, 'fft_blade_vals_turb.mat'), 'fft_blade', '-v7.3');
        save(fullfile(project_dir, 'fft_cdq_vals_turb.mat'), 'fft_cdq', '-v7.3');
%         save(fullfile(project_dir, 'fft_blade_vals_turb_test.mat'), 'fft_blade', '-v7.3');
%         save(fullfile(project_dir, 'fft_cdq_vals_turb_test.mat'), 'fft_cdq', '-v7.3');
    elseif strcmp(WIND_TYPE, 'steady')
        save(fullfile(project_dir, 'fft_blade_vals_steady.mat'), 'fft_blade', '-v7.3');
        save(fullfile(project_dir, 'fft_cdq_vals_steady.mat'), 'fft_cdq', '-v7.3');
    end
else
    if strcmp(WIND_TYPE, 'turbsim')
        load(fullfile(project_dir, 'fft_blade_vals_turb_test.mat'));
        load(fullfile(project_dir, 'fft_cdq_vals_turb_test.mat'));
    elseif strcmp(WIND_TYPE, 'steady')
        load(fullfile(project_dir, 'fft_blade_vals_steady.mat'));
        load(fullfile(project_dir, 'fft_cdq_vals_steady.mat'));
    end
end

% compute mean and standard deviation over all simulations for each load
% fft_blade_mean = mean(fft_blade, 1);
% fft_blade_std = std(fft_blade, 1);
% fft_cdq_mean = mean(fft_cdq, 1);
% fft_cdq_std = std(fft_cdq, 1);

% figure;
% boxplot(op_arr, );

% plot mean fft for each load
omega_1P_Hz = Parameters.Turbine.wr_rated * (2*pi/60) * (1/(2*pi));

% compute mean and bounds of PSD/FFT of this load over all simulations for
% each load for Blade/CDQ coords
if 0

    figure;
    tcf = tiledlayout('flow');
    plotSpectra(fft_blade.controller, 'psd', DT, omega_1P_Hz, HARMONICS, blade_op_arr); % QUESTION why is frequency range limited to 6p?
    plotSpectra(fft_blade.noipc, 'psd', DT, omega_1P_Hz, HARMONICS, blade_op_arr); 
    % plotSpectra(fft_blade, 'fft', DT, omega_1P_Hz, HARMONICS, blade_op_arr);
    
    figure;
    tcf = tiledlayout('flow');
    plotSpectra(fft_cdq.controller, 'psd', DT, omega_1P_Hz, HARMONICS, dq_op_arr);
    plotSpectra(fft_cdq.noipc, 'psd', DT, omega_1P_Hz, HARMONICS, dq_op_arr);
    % plotSpectra(fft_cdq, 'fft', DT, omega_1P_Hz, HARMONICS, dq_op_arr);

end

[~, ~, Peaks_noipc] = computeFFTPeaks(...
    fft_blade.noipc(:, ismember('RootMyc1', blade_op_arr), :), ...
    DT, omega_1P_Hz, [1,2,3,4]);
[~, ~, Peaks_controllers] = computeFFTPeaks(...
    fft_blade.controller(:, ismember('RootMyc1', blade_op_arr), :), ...
    DT, omega_1P_Hz, [1,2,3,4]);
fft_Peaks.noipc = Peaks_noipc;
fft_Peaks.controller = Peaks_controllers;
fft_Peaks.harmonics = [1,2,3,4];

figure;
plot(fft_Peaks.harmonics, mag2db(squeeze(fft_Peaks.noipc)));
hold on;
for case_idx = 1:length(sim_out_list.controller)
    plot(fft_Peaks.harmonics, mag2db(squeeze(fft_Peaks.controller(:, :, case_idx, :))));
end
xlabel('Harmonic'); ylabel('Magnitude [dB]');
hold off;
legend(['No IPC', arrayfun(@(n) ['Case', num2str(n)], 1:length(sim_out_list.controller), 'UniformOutput', false)], 'NumColumns', 2);
% QUESTION, should these show greater amplitudes at 1p, 2p/4p, 5p/7p?
% (corresponding to 0p, 3p, 6p in non-rotating domain)

% title("Single-Sided Amplitude Spectrum")

% xlim([0, 0.3]);
% ylim([0, 2e4]);

% compute boxplot data over all ffts for each load at rotational HARMONICS
% 1, 2, 3, 4, 5, 6, 7
% plot boxplots at rotational HARMONICS for each load
% boxplot(x,'PlotStyle','compact')

% Make table comparing controllers
if OPTIMAL_K_COLLECTION
    load(fullfile(code_dir, 'matfiles', 'Controllers_case_table.mat'));
    for p = 1:length(fft_Peaks.harmonics)
        Controllers_case_table.(['RootMycBlade1 ', num2str(p), 'th ', 'Harmonic [dB]']) ...
            = [mag2db(squeeze(fft_Peaks.noipc)); mag2db(squeeze(fft_Peaks.controller(:, :, :, p)))];
    end
    sortrows(Controllers_case_table, ['RootMycBlade1 ', num2str(3), 'th ', 'Harmonic [dB]'], 'descend')
elseif EXTREME_K_COLLECTION
    load(fullfile(code_dir, 'matfiles', 'Extreme_Controllers_case_table.mat'));
    for p = 1:length(fft_Peaks.harmonics)
        Controllers_case_table.(['RootMycBlade1 ', num2str(p), 'th ', 'Harmonic [dB]']) ...
            = [mag2db(squeeze(fft_Peaks.noipc)); mag2db(squeeze(fft_Peaks.controller(:, :, :, p)))];
    end
    sortrows(Controllers_case_table, ['RootMycBlade1 ', num2str(3), 'th ', 'Harmonic [dB]'], 'descend')
elseif STRUCT_PARAM_SWEEP
    load(fullfile(code_dir, 'matfiles', 'PI_ParamSweep_redtable.mat'));
end

%% Plot Blade-Pitch Actuation and Loads in Time-Domain OUTPLOT
if 0
    
    figure;
    tiledlayout(3, 2);
    nexttile(1);
    subtitle('\beta_d, \beta_q [deg]');
    plot(sim_out_list.noipc(3).beta_ipc.dq);
    nexttile(2);
    subtitle('Baseline IPC \beta_d, \beta_q [deg]');
    plot(rad2deg(sim_out_list.baseline_controller(3).beta_ipc.dq));
    legend('D', 'Q');

    ax1 = nexttile(3);
    subtitle('No IPC M_d, M_q [kNm?]');
    plot(sim_out_list.noipc(3).RootMyc.dq);
    legend('D', 'Q');
    ax2 = nexttile(4);
    subtitle('Baseline IPC M_d, M_q [kNm?]');
    plot(sim_out_list.baseline_controller(3).RootMyc.dq);
    legend('D', 'Q');
    linkaxes([ax1, ax2]);

    nexttile(5);
    subtitle('No IPC \beta_1 [deg]');
    plot(sim_out_list.noipc(3).beta.blade);
    nexttile(6);
    subtitle('Baseline IPC \beta_1 [deg]');
    plot(sim_out_list.baseline_controller(3).beta.blade);
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
