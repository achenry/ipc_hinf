%% Setup workspace
initialize;

C_WS_IDX = LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED;

%% Load linear models, operating points, hnorms
load(fullfile(mat_save_dir, 'Plant_red'));
Plant = Plant_red;
Plant.InputName = {'\beta_d Control Input', '\beta_q Control Input'};
Plant.OutputName = {'M_d Output', 'M_q Output'};
all(real(pole(Plant)) < 0);

%% Normalization for inputs and outputs

% normalize Plant channels: maximum IPC command/expected Mdq values, multiply input by
% scaling to get actual u before plant; divide output by scaling to get -1,
% 1 after plant; scaling factors are part of controller
load(fullfile(FAST_directory, 'op_absmax.mat'));
% load(fullfile(FAST_directory, 'ss_vals.mat'));
clear ip_scaling op_scaling
for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
    idx = (op_absmax.dq(:, 'Wind1VelX').Variables == LPV_CONTROLLER_WIND_SPEEDS(c_ws_idx));
    ip_scaling(:, :, c_ws_idx) = diag(deg2rad(op_absmax.dq(idx, {'BldPitchC', 'BldPitchC'}).Variables));
    op_scaling(:, :, c_ws_idx) = diag(op_absmax.dq(idx, {'RootMycC', 'RootMycC'}).Variables);
    
    % ip_scaling_2(:, :, c_ws_idx) = diag(deg2rad(ss_vals.dq(idx.Variables, {'BldPitchC', 'BldPitchC'}).Variables) * 0.2);
    % op_scaling_2(:, :, c_ws_idx) = diag(ss_vals.dq(idx.Variables, {'RootMycC', 'RootMycC'}).Variables * 0.2);

    Plant_scaled(:, :, c_ws_idx) = inv(op_scaling(:, :, c_ws_idx)) * Plant(:, :, c_ws_idx) * ip_scaling(:, :, c_ws_idx);
end

%% Compute Weighting Functions
% inputs [d; u], outputs [z; y]
% using Windergarten weighting functions
% how to choose Wu => just above 12P,
% arbitrarily high for roll-off? => detune gain with higher low frequency
% gain of Wu, choose breakpoint frequency just above 3P

% Disturbances to consider: on input (output of Controller, K), on output (output of Plant)
% Peformance Measures to consider: zy (output of Plant), ze (y-r), zu
% (output of Controller, K)

% Define weighting matrices
s = tf('s');

% Wu = Weighting on Ti KSi
% apply high-pass filter to penalize control action, w/ roll-off after 8P
% st it is realizable
% omega_bk = 6 * omega_1P_rad;
% omega_sat = 1e3 * omega_bk;
% hpf = (s + omega_bk) / (s + omega_sat);
% Wu = 500 * eye(2) * hpf;
eps_u = 1e-2; % desired disturbance attenuation at really high frequencies
M_u = 10; % desired bound on Hinf norm of input sensitivity function at really low frequencies
w1_u = 2 * omega_3P_rad; % first breakpoint frequency
omega_u = M_u * w1_u; % 2.5e-3; % controller bw in the high-freq range
hpf = (1 / eps_u) * (s + omega_u / M_u) / (s + (omega_u / eps_u));
Wu = 1 * hpf * eye(2); % bodemag(Wu);
% Wu = 500 * tf([1 3.464], [1 3464]) * eye(2); % Ossman's values
Wu.InputName = {'\beta_d Control Input', '\beta_q Control Input'};
Wu.OutputName = {'Weighted \beta_d Control Input', 'Weighted \beta_q Control Input'};
% bodemag(Wu)
% Wy=Weighting on To, GSo 0dB at low frequency, low at
% high-frequency => hpf with low-frequency gain below 0dB 
% apply integrator + inverse notch penalty (zeta in num > zeta in
% denom) to output and error
eps_y = 1e-1; % desired disturbance attenuation at really high frequencies
M_y = 2; % desired bound on Hinf norm of input sensitivity function at really low frequencies, ideally 1
w1_y = omega_1P_rad * 0.1; % first breakpoint frequency
omega_y = M_y * w1_y; % 2.5e-3; % controller bw in the high-freq range
hpf_y = (1 / eps_y) * (s + omega_y/M_y) / (s + (omega_y / eps_y));
Wy = 1 * eye(2) * hpf_y * 0;
% Wy = 0.1 * tf([1 3], [1 0.03]) * tf([1 (0.49/sqrt(24))*omega_3P_rad omega_3P_rad^2], [1 (6.93/sqrt(24))*omega_3P_rad omega_3P_rad^2]) * eye(2); % Ossman's values
Wy.InputName = {'M_d Output', 'M_q Output'}; 
Wy.OutputName = {'Weighted M_d Output', 'Weighted M_q Output'};

% We = Weighting on So, GSo. 0dB at high frequency, low at
% low-frequency => lpf with high-frequency gain below 0dB that saturates
eps_e = 1e-1; % desired disturbance attenuation at really low frequencies
w1_e = omega_1P_rad * 1e-1; % first breakpoint frequency
omega_e = (1 / eps_e) * w1_e; % 0.07*pi; % desired CL bw
M_e = 10; % desired bound on Hinf norm of output sensitivity function at really high frequencies, ideally 1
lpf_e = (1 / M_e) * (s + M_e*omega_e) / (s + eps_e*omega_e);

% design notch weighting filter as per Ossman to limit singular
    % value of disturbance sensitivity on the diagonal bc we don't need to
    % penalize weighted sum of Md and Mq
zeta = 0.001; % damping, greater value leads to greater width
gbar = 1e2; % gain at notch frequency
notch_3P_diag = tf([1 2*gbar*zeta*omega_3P_rad omega_3P_rad^2], ...
    [1 2*zeta*omega_3P_rad omega_3P_rad^2]);
notch = notch_3P_diag * eye(2);

We = 1 * eye(2) * lpf_e; % * notch;
% We = 0.1 * tf([1 3], [1 0.03]) * tf([1 (0.49/sqrt(24))*omega_3P_rad omega_3P_rad^2], [1 (6.93/sqrt(24))*omega_3P_rad omega_3P_rad^2]) * eye(2); % Ossman's values
We.InputName = {'Measured M_d Tracking Error', 'Measured M_q Tracking Error'}; 
We.OutputName = {'Weighted M_d Tracking Error', 'Weighted M_q Tracking Error'}; 

% Weighting on reference
W1 = notch; %tf(1 * eye(2));

% Weighting on control input disturbance
% apply proportional filter to penalize disturbances on inputs, outputs
W2 = tf(1 * eye(2));
% Wdu.InputName = {'Input Disturbance, $d_u$'};
% Wdu.OutputName = {'Weighted Input Disturbance, $W_{d_u} d_u$'};


%% Create Case Basis for different controllers

if EXTREME_K_COLLECTION
    case_basis.Scheduling.x = {'No'}; %, 'No'};
    case_basis.Structure.x = {'Full-Order'}; %, 'Structured'};
    case_basis.Reference.d = [0];
    case_basis.Reference.q = [0];
    case_basis.Saturation.d = [Inf];
    case_basis.Saturation.q = [Inf];

    IP_HIGH_GAIN = 1;
    WU_HIGH_GAIN = 10;
    WE_HIGH_GAIN = 10;
    IP_LOW_GAIN = 10;
    OP_LOW_GAIN = 10;
    % case_basis.WuGain = {tf(OP_LOW_GAIN * eye(2)), tf(WU_HIGH_GAIN * eye(2))}; %{100, 200, 400, 800, 1600}; % for detuning blade-pitch actuation
    case_basis.WuGain.x = ...
        {tf(WU_HIGH_GAIN * 0.05 * eye(2)), ...
        tf(WU_HIGH_GAIN * 0.1 * eye(2)), ...
        tf(WU_HIGH_GAIN * 0.2 * eye(2)), ...
        tf(WU_HIGH_GAIN * 0.4 * eye(2)), ...
        tf(WU_HIGH_GAIN * 0.8 * eye(2)), ...
        tf(WU_HIGH_GAIN * eye(2))};
    case_basis.WeGain.x = ...
        {tf(WE_HIGH_GAIN * 0.05 * eye(2)), ...
        tf(WE_HIGH_GAIN * 0.1 * eye(2)), ...
        tf(WE_HIGH_GAIN * 0.2 * eye(2)), ...
        tf(WE_HIGH_GAIN * 0.4 * eye(2)), ...
        tf(WE_HIGH_GAIN * 0.8 * eye(2)), ...
        tf(WE_HIGH_GAIN * eye(2))};
    case_basis.W1Gain.x = ...
        {tf(IP_HIGH_GAIN * 0.05 * eye(2)), ...
        tf(IP_HIGH_GAIN * 0.1 * eye(2)), ...
        tf(IP_HIGH_GAIN * 0.2 * eye(2)), ...
        tf(IP_HIGH_GAIN * 0.4 * eye(2)), ...
        tf(IP_HIGH_GAIN * 0.8 * eye(2)), ...
        tf(IP_HIGH_GAIN * eye(2))};
    case_basis.W2Gain.x = ...
        {tf(IP_HIGH_GAIN * 0.05 * eye(2)) , ...
        tf(IP_HIGH_GAIN * 0.1 * eye(2)), ...
        tf(IP_HIGH_GAIN * 0.2 * eye(2)), ...
        tf(IP_HIGH_GAIN * 0.4 * eye(2)), ...
        tf(IP_HIGH_GAIN * 0.8 * eye(2)), ...
        tf(IP_HIGH_GAIN * eye(2))};

    % case_basis.WuGain = {tf(1 * eye(2)), tf(10 * eye(2)), tf(100 * eye(2)), tf(250 * eye(2)), tf(500 * eye(2))}; %{100, 200, 400, 800, 1600}; % for detuning blade-pitch actuation
    % case_basis.WeGain = {tf(1 * eye(2)), tf(10 * eye(2)), tf(100 * eye(2))};
    % case_basis.W1Gain = {0.1 * notch, 1 * notch, 10 * notch};
    % case_basis.W2Gain = {tf(0.1 * eye(2)), tf(1 * eye(2)), tf(10 * eye(2))};
    
    [Controllers_case_list, Controllers_case_name_list, Controllers_n_cases] ...
        = generateCases(case_basis, 'tuned_controllers', true);
    
    % remove cases
    if 0
        exc_idx = [];
        for c_idx = 1:Controllers_n_cases
            % want exactly 2 weighting matrices to be 'on'
            % and only one weighting filter to have a notch
            if (~((length(Controllers_case_list(c_idx).WeGain.Numerator{1,1}) > 1) ...
                    && (length(Controllers_case_list(c_idx).W1Gain.Numerator{1,1}) > 1))) ...
                    && (ss(Controllers_case_list(c_idx).W1Gain).D(1, 1) ...
                    + ss(Controllers_case_list(c_idx).W2Gain).D(1, 1) == IP_HIGH_GAIN + IP_LOW_GAIN) ... % want one (but not both) to have high gain
                    && ((ss(Controllers_case_list(c_idx).WuGain).D(1, 1) ...
                    + ss(Controllers_case_list(c_idx).WeGain).D(1, 1) == WU_HIGH_GAIN + OP_LOW_GAIN) ... 
                    || (ss(Controllers_case_list(c_idx).WuGain).D(1, 1) ...
                    + ss(Controllers_case_list(c_idx).WeGain).D(1, 1) == WE_HIGH_GAIN + OP_LOW_GAIN))
                continue;
            end
            exc_idx = [exc_idx, c_idx];
        end
        Controllers_case_list = Controllers_case_list(~ismember(1:Controllers_n_cases, exc_idx));
        Controllers_n_cases = Controllers_n_cases - length(exc_idx);
        Controllers_case_name_list = arrayfun(@(n) ['tuned_controllers_', num2str(n)], 1:Controllers_n_cases, 'UniformOutput', false);
    end
elseif OPTIMAL_K_COLLECTION
    case_basis.Scheduling.x = {'No'}; %, 'No'};
    case_basis.Structure.x = {'Full-Order'}; %, 'Structured'};

    if VARY_REFERENCE
        case_basis.Reference.d = [];
        case_basis.Reference.q = [];
        for ref = VARY_REFERENCE_BASIS
            % TODO choose reference to be percentage of linearization value
            case_basis.Reference.d(end + 1) = ref * op_absmax.dq(idx.Variables, 'RootMycD').Variables;
            case_basis.Reference.q(end + 1) = ref * op_absmax.dq(idx.Variables, 'RootMycQ').Variables;
            % mbcTransformOutData(values, OutList)
            case_basis.Reference.d(end + 1) = ref * ss_vals.dq(idx.Variables, 'RootMycD').Variables;
            case_basis.Reference.q(end + 1) = ref * ss_vals.dq(idx.Variables, 'RootMycQ').Variables;
        end
    else
        case_basis.Reference = [0];
    end

    if VARY_SATURATION
        case_basis.Saturation.d = [];
        case_basis.Saturation.q = [];
        for sat = VARY_SATURATION_BASIS
            % TODO choose saturation to be percentage of linearization value
            case_basis.Saturation.d(end + 1) = sat * op_absmax.dq(idx.Variables, 'BldPitch1D').Variables;
            case_basis.Saturation.q(end + 1) = sat * op_absmax.dq(idx.Variables, 'BldPitch1Q').Variables;
        end
    else
        case_basis.Saturation.d = [Inf];
        case_basis.Saturation.q = [Inf];
    end

    % case_basis.WuGain = {tf(10 * eye(2)), tf(100 * eye(2))}; %{100, 200, 400, 800, 1600}; % for detuning blade-pitch actuation
    % case_basis.WeGain = {tf(1 * eye(2)), tf(10 * eye(2))};
    % case_basis.W1Gain = {0.1 * notch, 1 * notch};
    % case_basis.W2Gain = {tf(0.1 * eye(2)), tf(1 * eye(2))};

    case_basis.WuGain = {tf(50 * eye(2)), tf(100 * eye(2)), tf(200 * eye(2)), tf(400 * eye(2))}; %{100, 200, 400, 800, 1600}; % for detuning blade-pitch actuation
    case_basis.WeGain = {tf(10 * eye(2))};
    case_basis.W1Gain = {1 * notch};
    case_basis.W2Gain = {tf(0.1 * eye(2))};
    
    [Controllers_case_list, Controllers_case_name_list, Controllers_n_cases] ...
        = generateCases(case_basis, 'tuned_controllers', false);
else
    case_basis.Reference.d = [0];
    case_basis.Reference.q = [0];
    case_basis.Saturation.d = [Inf];
    case_basis.Saturation.q = [Inf];
end

%% Synthesize Continuous-Time Generalized Plant P from plant Plant and weighting filters W
% effectively an output disturbance
% [zu; ze; zy; e] = GenPlant(C_WS_IDX) [dr du dy u]

for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
    [GenPlant_tmp, Win_tmp, Wout_tmp] ...
        = generateGenPlant(Plant_scaled(:, :, c_ws_idx), Wu, We, W1, W2);
    GenPlant(:, :, c_ws_idx) = GenPlant_tmp;
    Win(:, :, c_ws_idx) = Win_tmp;
    Wout(:, :, c_ws_idx) = Wout_tmp;
end

% Define LPV GenPlant
% lpv_domain = rgrid({'u'}, {LPV_CONTROLLER_WIND_SPEEDS});
% GenPlant_lpv = pss(GenPlant, lpv_domain);
% GenPlant_lpv.Parameter.u.RateBounds = [-2, 2]; % QUESTION what is realistic here ?

ref_idx = [strmatch('M_d Reference', GenPlant(:, :, C_WS_IDX).InputName), strmatch('M_q Reference', GenPlant(:, :, C_WS_IDX).InputName)];
ctrl_input_idx = [strmatch('\beta_d Control Input', GenPlant(:, :, C_WS_IDX).InputName), strmatch('\beta_q Control Input', GenPlant(:, :, C_WS_IDX).InputName)];
tracking_error_idx = [strmatch('Measured M_d Tracking Error', GenPlant(:, :, C_WS_IDX).OutputName), strmatch('Measured M_q Tracking Error', GenPlant(:, :, C_WS_IDX).OutputName)];

if 0
    % Plot all OL transfer functions for generalized plant
    figure;
    omega = logspace(-2, 2, 200);
    bodemag(GenPlant(:, :, C_WS_IDX), omega);
    title('Open Loop Generalized Plant');
end


%% Formulate Baseline Controller as SISO PI with Notch Filters for 3P and 6P loads
% also including moving notch (function of omega) and LPF in
% baseline? => we are operating at fixed rotor speed, so for controller
% design use notch filter at fixed rotor speed, but for nonlinear
% simulations use LPV notch-filter
% For nonlinear simulations: implement in SL can implement directly OR
% deconstruct into LPF, LPV Notch, PI but not a consistent storyline
% To include anti-windup => require different strategies
% for MIMO, not for LPV, choosing very high Wu weights. Deconstruct
% controller QUESTION Manuel

omega_6P_rad = omega_1P_rad * 6;

baseline_PI = d2c(pid(Parameters.cIPC.DQ_Kp_1P, Parameters.cIPC.DQ_Ki_1P, 0, 0, DT));
PI0 = parallel(baseline_PI.Kp * eye(2), baseline_PI.Ki * (1 / tf('s')) * eye(2));
% Notch_3P_0 = tf(d2c(PSM0_NF_3P(:,:, 5, 1))) * eye(2);
% Notch_6P_0 = tf(d2c(PSM0_NF_6P(:,:, 5, 1))) * eye(2);
zeta = 1;
drop = 10;
Notch_3P_0 = tf([1,2*omega_3P_rad*zeta/drop,(omega_3P_rad)^2],[1,2*omega_3P_rad*zeta,(omega_3P_rad)^2]);
Notch_6P_0 = tf([1,2*omega_6P_rad*zeta/drop,(omega_6P_rad)^2],[1,2*omega_6P_rad*zeta,(omega_6P_rad)^2]);

K0 = PI0 * Notch_6P_0 * Notch_3P_0;
save(fullfile(mat_save_dir, 'K0.mat'), 'K0');


%% Plotting
PLOTTING = 0;
plotting_idx = find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED);

 if PLOTTING

    % Plot scaled Plant OUTPLOT
    figure;
    bcol = copper(length(LPV_CONTROLLER_WIND_SPEEDS)); % Define the color order based on the number of models
    % rcol = jet(length(LPV_CONTROLLER_WIND_SPEEDS));
    % ycol = hot(length(LPV_CONTROLLER_WIND_SPEEDS)); % https://www.mathworks.com/help/matlab/colors-1.html?s_tid=CRUX_lftnav

    omega = logspace(-2, 4, 300);
    
    for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
        if false || LPV_CONTROLLER_WIND_SPEEDS(c_ws_idx) ~= NONLPV_CONTROLLER_WIND_SPEED
            continue;
        end
        % Plot baseline and tuned controllers
        bodeplot(Plant(:, :, c_ws_idx), omega, bode_plot_opt);

        % Find handles of all lines in the figure that have the color blue
        blineHandle = findobj(gcf,'Type','line','-and','Color','b');
        % rlineHandle = findobj(gcf,'Type','line','-and','Color','r');
        % ylineHandle = findobj(gcf,'Type','line','-and','Color','y');

        % Change the color to the one you defined
        set(blineHandle,'Color',bcol(c_ws_idx,:));
        % set(rlineHandle,'Color',rcol(c_ws_idx,:));
        % set(ylineHandle,'Color',ycol(c_ws_idx,:));
        
        hold on
    end

    axh = findall(gcf, 'type', 'axes');
    xline(axh(3), omega_1P_rad * HARMONICS);
    xline(axh(5), omega_1P_rad * HARMONICS);
    xline(axh(7), omega_1P_rad * HARMONICS);
    xline(axh(9), omega_1P_rad * HARMONICS);
    set(gcf, 'Position', [0 0 1500 900]);
    legend('Structured Baseline', 'Structured Tuned', 'Full-Order Tuned', '', '', '', '');
    %         title('Frequency Response of Tuned Controllers');
    hold off;

    set(gcf, 'Position', [0 0 1500 900]);
    savefig(gcf, fullfile(fig_dir, 'Plant_scaled.fig'));
    saveas(gcf, fullfile(fig_dir, 'Plant_scaled.png'));

    
    % Plot Weighting matrices OUTPLOT TODO plot range of Wu under
    % examination
    figure;
    omega = logspace(-2, 4, 300);
    bodemag(W1, W2, Wu, We, omega, bode_plot_opt);
    % bode_plot_opt.XLim = [10^(-1), 11];
    % setoptions(h, 'XLim', )
    % title('Weighting Functions');
    axh = findall(gcf, 'type', 'axes');
    xline(axh(3), omega_1P_rad * HARMONICS);
    xline(axh(5), omega_1P_rad * HARMONICS);
    xline(axh(7), omega_1P_rad * HARMONICS);
    xline(axh(9), omega_1P_rad * HARMONICS);
    legend("W_1", "W_2", "W_u", "W_e", '', '', '', '');
    set(gcf, 'Position', [0 0 1500 900]);%get(0, 'Screensize'));
    savefig(gcf, fullfile(fig_dir, 'weighting_funcs.fig'));
    saveas(gcf, fullfile(fig_dir, 'weighting_funcs.png'));
    
    % Plot baseline structured controller
    % figure;
    % bodeplot(K0, bode_plot_opt);
    % axh = findall(gcf, 'type', 'axes');
    % xline(axh(5), omega_1P_rad * HARMONICS);
    % xline(axh(3), omega_1P_rad * HARMONICS);
    % xline(axh(7), omega_1P_rad * HARMONICS);
    % xline(axh(9), omega_1P_rad * HARMONICS);
    % title('Frequency Response of Baseline Controller');

end