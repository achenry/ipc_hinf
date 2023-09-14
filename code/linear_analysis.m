%% Initialize
initialize;

%% Plot 2*2 Bodemag structured/full-order K at each wind-speed

%% Compute Disk Margin, (Gain Margin, Phase Margin) for all Transfer Function for each controller at each wind speed

%% Plot Singular Values of Transfer Functions at each wind-speed: OL vs. CL QUESTION how do we compute sensitivity for open loop case, no controller?
% TODO 
%   inverse Weighting Functions penalizing them, 
%   transfer functions for CL Plant with tuned and baseline controllers
if 1
    
    % Plot complementary output sensitivity function and its weighting
    % matrix, r->y (ideally 1 for tracking, at lower freqs) or n->y (ideally 0 for noise attenuation, at higher freqs)
    % ClosedLoop case: Ideal To = 1. Real lpf with bandwidth, low-freq gain of 1. Choose
    % weighting function to look like inverse
    % TODO remove identity controllers
    % figure;
    % bodeplot(To(Plant(:, :, C_WS_IDX), eye(2)), inv(Wy), bode_plot_opt);
    % title('Uncontrolled Complementary Output Sensitivity Function');
    % legend('T_o', 'W_y^{-1}')
    % axh = findall(gcf, 'type', 'axes');
    % 
    % xline(axh(1), omega_1P_rad * HARMONICS);
    % xline(axh(3), omega_1P_rad * HARMONICS);
    % xline(axh(7), omega_1P_rad * HARMONICS);
    % xline(axh(9), omega_1P_rad * HARMONICS);
    
    % Plot complementary input sensitivity function and its weighting matrix
    % Ti = -u/W2d2, control disturbance -> control input
    figure;
    bodeplot(Ti(Plant(:, :, C_WS_IDX), eye(2)), inv(Wu), bode_plot_opt);
    title('Uncontrolled Complementary Input Sensitivity Function');
    legend('T_i', 'W_u^{-1}')
    axh = findall(gcf, 'type', 'axes');
    
    xline(axh(1), omega_1P_rad * HARMONICS);
    xline(axh(3), omega_1P_rad * HARMONICS);
    xline(axh(7), omega_1P_rad * HARMONICS);
    xline(axh(9), omega_1P_rad * HARMONICS);

    % Plot output sensitivity function and its weighting matrix, dy-y or
    % n->e or r->e (ideally 0)
    % S and T are coupled : S + T = 1, so choosing 1 determines the other
    % need penalty on both bc want both to have small magnitdues at particular
    % frequencies (ie could add 2 complex numbers with large magnitudes at
    % high phases that add to 1, or have one with large mag and the other with very small in alignment)
    % OpenLoop case: So = 1, ClosedLoop case: can shape So, ideally to 0.
    % Real So: hpf with roll-off with high-freq gain of 1.
    % So = e/W1d1, output disturbance -> tracking error
    figure;
    bodeplot(So(Plant(:, :, C_WS_IDX), eye(2)), inv(We), bode_plot_opt);
    title('Uncontrolled Output Sensitivity Function');
    legend('S_o', 'W_e^{-1}')
    axh = findall(gcf, 'type', 'axes');
    
    xline(axh(1), omega_1P_rad * HARMONICS);
    xline(axh(3), omega_1P_rad * HARMONICS);
    xline(axh(7), omega_1P_rad * HARMONICS);
    xline(axh(9), omega_1P_rad * HARMONICS);
 
end
%% Compare Baseline vs. Structured vs. Full-Order CT Controllers at each wind-speed
clc; close all;


if 1
    load(fullfile(code_dir, 'matfiles', 'K0.mat'));
    load(fullfile(code_dir, 'matfiles', 'Controllers_case_list.mat'));

    omega = logspace(-2, 4, 300);
    
    for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)

        % Closed-Loop Systems
        CL0 = feedback(series(K0, Plant(:, :, c_ws_idx)), eye(2), -1);
        % CL0 = feedback(GenPlant(tracking_error_idx, ctrl_input_idx, c_ws_idx), ss(K0(:, :, c_ws_idx)), -1);
        CL_struct = feedback(series(K_struct_tuned(:, :, c_ws_idx), Plant(:, :, c_ws_idx)), eye(2), -1); % structured
        % CL_struct = feedback(GenPlant(tracking_error_idx, ctrl_input_idx, c_ws_idx), ss(K_struct_tuned(:, :, c_ws_idx)), -1);
        CL_full = feedback(series(K_full_tuned(:, :, c_ws_idx), Plant(:, :, c_ws_idx)), eye(2), -1); % tuned, alt use lft(GenPlant(C_WS_IDX), K)
        % CL_full = feedback(GenPlant(tracking_error_idx, ctrl_input_idx, c_ws_idx), ss(K_full_tuned(:, :, c_ws_idx)), -1);
        
        CL_struct.InputName = {'RootMycD Reference', 'RootMycQ Reference'};
        CL_full.InputName = {'RootMycD Reference', 'RootMycQ Reference'}; 

        % sigma(CL, ss(gamma))
        
        if  LPV_CONTROLLER_WIND_SPEEDS(c_ws_idx) == NONLPV_CONTROLLER_WIND_SPEED
        % Plot CL transfer functions
        figure;
        bodemag(CL0, CL_struct, CL_full, omega, bode_plot_opt);
        title('Frequency Response of Complementary Output Sensitivity Function with Baseline & Tuned Controllers');
        axh = findall(gcf, 'type', 'axes');
        xline(axh(3), omega_1P_rad * HARMONICS);
        xline(axh(5), omega_1P_rad * HARMONICS);
        xline(axh(7), omega_1P_rad * HARMONICS);
        xline(axh(9), omega_1P_rad * HARMONICS);
        legend('CL Structured Baseline', 'CL Structured Tuned', 'CL Full-Order Tuned', '', '', '', '', 'Location', 'southeast');
        set(gcf, 'Position', get(0, 'Screensize'));
        saveas(gcf, fullfile(fig_dir, 'freq_resp_CL.png'));

        % Plot step and impulse response
        figure;
        ax = subplot(2, 1, 1);
        stepplot(ax, CL_struct, CL_full, 1:0.01:300, time_plot_opt);
        title(ax, 'Step Response of Closed-Loop Plant with Tuned Controllers');
        ax = subplot(2, 1, 2);
        impulseplot(ax, CL_struct, CL_full, 1:0.01:100, time_plot_opt);
        title(ax, 'Impulse Response of Closed-Loop Plant with Tuned Controllers');
        legend('CL Structured Tuned', 'CL Full-Order Tuned', 'Location', 'southeast');
        set(gcf, 'Position', get(0, 'Screensize'));
        saveas(gcf, fullfile(fig_dir, 'time_resp_CL.png'));
        
        % Plot baseline and tuned controllers
        figure;
        bodemag(K0, K_struct_tuned(:, :, c_ws_idx), K_full_tuned(:, :, c_ws_idx), omega, bode_plot_opt);
        title('Frequency Response of Tuned Controllers');
        axh = findall(gcf, 'type', 'axes');
        xline(axh(3), omega_1P_rad * HARMONICS);
        xline(axh(5), omega_1P_rad * HARMONICS);
        xline(axh(7), omega_1P_rad * HARMONICS);
        xline(axh(9), omega_1P_rad * HARMONICS);
        legend('Structured Baseline', 'Structured Tuned', 'Full-Order Tuned', '', '', '', '');
        saveas(gcf, fullfile(fig_dir, 'bodemag_K.png'));
        end
        % Compute disk margins - assumes loop gain with unity feedback w/
        % negative feedback - see influence of variation on open loop gain
        % in Nyquist plot - variation in gain and phase simultaneously (vs
        % independently as 'margin' computes)
        % margin = how much variation (complex gain) open loop system can tolerate until
        % it becomes unstable
        % disk margin = disk that fits within variation region that makes
        % for stable CL system on nyquist plot
        % MIMO systems have multiple loops - can add variation to
        % a) Controller output(s) before they enter Plant or b) to Plant
        % output(s) before they are fed back to the controller
        % Multi-Loop Input Disk Margin - add independent variations to both loops between Controller and Plant and restrict to
        % be within the same disk
        % Multi-Loop Output Disk Margin - add independent variations to both loops after Plant and restrict to
        % be within the same disk
        % Multi-Loop Input/Output Disk Margin - Vary all inputs and outputs independently

        % [DM_Plant, MM_Plant] = diskmargin(Plant(:, :, c_ws_idx))
        % MMIO_Plant = diskmargin(Plant(:, :, c_ws_idx)); % simultaneous
        % variations in inputs and outputs
        % [Gm, Pm, Wcg, Wcp] = margin(Plant(:, :, c_ws_idx))

        [DMin_0_tmp, MMin_0_tmp] = diskmargin(K0 * Plant(:, :, c_ws_idx)); % at Plant inputs, DM = loop-at-a-time input disk margin, MM=multi-loop input disk margin
        DMin_0(:, c_ws_idx) = DMin_0_tmp;
        MMin_0(:, c_ws_idx) = MMin_0_tmp;
        [DMout_0_tmp, MMout_0_tmp] = diskmargin(Plant(:, :, c_ws_idx) * K0); % at Plant outputs,
        DMout_0(:, c_ws_idx) = DMout_0_tmp;
        MMout_0(:, c_ws_idx) = MMout_0_tmp;
        MMIO_0(c_ws_idx) = diskmargin(Plant(:, :, c_ws_idx), K0); % vary gain/phase at both inputs and outputs

        [DMin_full_tmp, MMin_full_tmp] = diskmargin(K_full_tuned(:, :, c_ws_idx) * Plant(:, :, c_ws_idx)); % at Plant inputs, DM = loop-at-a-time input disk margin, MM=multi-loop input disk margin
        DMin_full(:, c_ws_idx) = DMin_full_tmp;
        MMin_full(:, c_ws_idx) = MMin_full_tmp;
        [DMout_full_tmp, MMout_full_tmp] = diskmargin(Plant(:, :, c_ws_idx) * K_full_tuned(:, :, c_ws_idx)); % at Plant outputs,
        DMout_full(:, c_ws_idx) = DMout_full_tmp;
        MMout_full(:, c_ws_idx) = MMout_full_tmp;
        MMIO_full(c_ws_idx) = diskmargin(Plant(:, :, c_ws_idx), K_full_tuned(:, :, c_ws_idx));

        [DMin_struct_tmp, MMin_struct_tmp] = diskmargin(K_struct_tuned(:, :, c_ws_idx) * Plant(:, :, c_ws_idx)); % at Plant inputs, DM = loop-at-a-time input disk margin, MM=multi-loop input disk margin
        DMin_struct(:, c_ws_idx) = DMin_struct_tmp;
        MMin_struct(:, c_ws_idx) = MMin_struct_tmp;
%         [DMout_struct_tmp, MMout_struct_tmp] = diskmargin(Plant(:, :, c_ws_idx) * ss(K_struct_tuned(:, :, c_ws_idx))); % at Plant outputs, QUESTION error?
%         DMout_struct(:, c_ws_idx) = DMout_struct_tmp;
%         MMout_struct(:, c_ws_idx) = MMout_struct_tmp;
%         MMIO_struct(c_ws_idx) = diskmargin(Plant(:, :, c_ws_idx),
%         ss(K_struct_tuned(:, :, c_ws_idx))); % QUESTION error?
        
        % PMTools xloopmargin, ref Brian Douglas
        
    end

end