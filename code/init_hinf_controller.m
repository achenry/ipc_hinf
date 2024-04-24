%% Setup workspace
initialize;

% TODO set up gain-scheduled controller and compare performance for
% turbulent wind fields vs single-tuned controller

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
% TODO Use same scaling over every wind speed for blade-pitch as ipc does
% not vary over wind speeds as collective pitch control does
cc = find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED);
for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
    idx = (op_absmax.dq(:, 'Wind1VelX').Variables == LPV_CONTROLLER_WIND_SPEEDS(cc));
    ip_scaling(:, :, c_ws_idx) = diag(deg2rad(op_absmax.dq(idx, {'BldPitchC', 'BldPitchC'}).Variables));
    op_scaling(:, :, c_ws_idx) = diag(op_absmax.dq(idx, {'RootMycC', 'RootMycC'}).Variables);
    
    % ip_scaling_2(:, :, c_ws_idx) = diag(deg2rad(ss_vals.dq(idx.Variables, {'BldPitchC', 'BldPitchC'}).Variables) * 0.2);
    % op_scaling_2(:, :, c_ws_idx) = diag(ss_vals.dq(idx.Variables, {'RootMycC', 'RootMycC'}).Variables * 0.2);

    Plant_scaled(:, :, c_ws_idx) = inv(op_scaling(:, :, c_ws_idx)) * Plant(:, :, c_ws_idx) * ip_scaling(:, :, c_ws_idx);
end
% bodeplot(Plant(:,:, 3), Plant_scaled(:,:, 3), bode_plot_opt);
% legend('Original', 'Scaled')
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
M_u = 2; % desired bound on Hinf norm of input sensitivity function at really low frequencies
eps_u = M_u / 1e3; % desired disturbance attenuation at really high frequencies
w1_u = 0.75 * omega_3P_rad; % first breakpoint frequency
omega_u = M_u * w1_u; % 2.5e-3; % controller bw in the high-freq range
hpf = (1 / eps_u) * (s + omega_u / M_u) / (s + (omega_u / eps_u));
% hpf = (s + omega_u / M_u) / (s + omega_u);
Wu = 1 * hpf * eye(2);
if 0
    % x = 1;
    % bodemag(inv(x*hpf), 1-inv(x*hpf), x*hpf);
    bodemag(inv(1*hpf), inv(10*hpf));
    xline(omega_1P_rad); xline(omega_3P_rad); xline(2*omega_3P_rad)
    xlim([1e-1, 1e2]);
    % eps_u  = 1e-2;
    % M_u = 1 / eps_u;
    % y1 = (1 / eps_u) * (s + omega_u / M_u) / (s + (omega_u / eps_u));
    % eps_u  = 1e-3;
    % M_u = 1 / eps_u;
    % y2 = (1 / eps_u) * (s + omega_u / M_u) / (s + (omega_u / eps_u));
    %  bodemag(inv(y1), inv(y2));
    % legend('Bound on Ti', 'Bound on Si', 'Bound on Li')
    % bodemag(lpf_e, hpf)
end

% Wu = 500 * tf([1 3.464], [1 3464]) * eye(2); % Ossman's values
Wu.InputName = {'\beta_d Control Input', '\beta_q Control Input'};
Wu.OutputName = {'Weighted \beta_d Control Input', 'Weighted \beta_q Control Input'};

% We = Weighting on So, GSo. 0dB at high frequency, low at
% low-frequency => lpf with high-frequency gain below 0dB that saturates
% eps_e = 1e-1;
w1_e = omega_1P_rad * 0.005; %* 0.02; % first breakpoint frequency
% M_e = 1e-1; 
% M_e = eps_e * 100;
M_e = 5; eps_e = M_e / 100; % rmse=2.7614e+04, adc=0.1933, desired bound on Hinf norm of output sensitivity function at really high frequencies, rmse=9.0619e+04, adc=9.0619e+04
% M_e = 1; eps_e = M_e / 100; %  BAD
% M_e = 1; eps_e = M_e / 10; % rmse=9.8120e+04, adc=1.26, desired disturbance attenuation at really low frequencies, decreasing this allows lower So values at low freq => To values closer to 1
% M_e = 10; eps_e = M_e / 1000; % rmse=6.9324e+04, adc=7.1871 BETTER
% M_e = 2; eps_e = M_e / 100; % ? nicht gut
omega_e = (1 / eps_e) * w1_e; % 0.07*pi = 0.2*omega_1P_rad; % desired CL bw
omega_e = omega_1P_rad * 0.1;
lpf_e = (1 / M_e) * (s + M_e*omega_e) / (s + eps_e*omega_e);
% lpf_e = (s + omega_e) / (s + eps_e*omega_e);


% design notch weighting filter as per Ossman to limit singular
    % value of disturbance sensitivity on the diagonal bc we don't need to
    % penalize weighted sum of Md and Mq
% zeta = 0.01; % damping, greater value leads to greater width
% gbar = 15; % gain at notch frequency
zeta = 1.; % damping, greater value leads to greater width
gbar = 10; % gain at notch frequency
notch_3P_diag = tf([1 2*gbar*zeta*omega_3P_rad omega_3P_rad^2], ...
    [1 2*zeta*omega_3P_rad omega_3P_rad^2]);
notch = notch_3P_diag * eye(2);
% bodemag(lpf_e*notch, hpf, logspace(-2,2,200))

We = 1 * eye(2) * lpf_e * notch;
% We = 0.1 * tf([1 3], [1 0.03]) * tf([1 (0.49/sqrt(24))*omega_3P_rad omega_3P_rad^2], [1 (6.93/sqrt(24))*omega_3P_rad omega_3P_rad^2]) * eye(2); % Ossman's values
We.InputName = {'M_d Tracking Error', 'Measured M_q Tracking Error'}; 
We.OutputName = {'M_d Tracking Error', 'Weighted M_q Tracking Error'}; 

% Weighting on reference
W1 = tf(eye(2)); %notch;

% Weighting on control input disturbance
% apply proportional filter to penalize disturbances on inputs, outputs
W2 = tf(eye(2));
% Wdu.InputName = {'Input Disturbance, $d_u$'};
% Wdu.OutputName = {'Weighted Input Disturbance, $W_{d_u} d_u$'};

if 0
    % x = 0.1;
    % bodemag(inv(x*lpf_e * notch), 1-inv(x*lpf_e* notch), x*lpf_e* notch - 1);
    % legend('Bound on So', 'Bound on To', 'Bound on Lo')
    bodeplot(lpf_e * notch_3P_diag, hpf, logspace(-2,2, 200), bode_plot_opt)
end



%% Create Case Basis for different controllers
% for debugging
if DEBUG
    n_seeds = 1;

    case_basis.WindSpeedIndex.x = find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED);
    % case_basis.Scheduling.x = [0]; %, 'No'};
    case_basis.Structure.x = {'Full-Order'}; %, 'Structured'};
    case_basis.Reference.x = [0];
    case_basis.Saturation.x = [0];
    
    % W1 = 0.01 W2 = 1 ->  Wu = 75 We = 10
    case_basis.W1Gain.x = {1 * tf(eye(2))};
    case_basis.W2Gain.x = {0.01 * tf(eye(2))};
    case_basis.WeGain.x = {tf(eye(2))};
    % i = 1;
    % for M_e_tmp = [1, 2, 5, 10]
    %     eps_e_tmp = M_e_tmp / 100;
    %     for omega_e_tmp = omega_1P_rad * [0.01, 0.025, 0.05, 0.1, 0.5, 1]
    %         % omega_e_tmp = (1 / eps_e_tmp) * w1_e;
    %         case_basis.WeGain.x{end + 1} = ...
    %             inv(lpf_e) * (1 / M_e_tmp) * (s + M_e_tmp*omega_e_tmp) / (s + eps_e_tmp*omega_e_tmp) * tf(eye(2));
    %         if (i == 16)
    %         % if (i == 15) || (i == 16)
    %             M_e_tmp
    %             omega_e_tmp / omega_1P_rad
    %         end
    %         i = i + 1;
    %         % figure(1);
    %         % bodeplot(case_basis.WeGain.x{end} * We, logspace(-4, 2, 200), bode_plot_opt); hold on;
    %     end
    % end
    % case_list 5, 6, 7, 8, 9, 10 corresponds to case_basis 19, 19, 13, 13, 13, 13
    
    case_basis.WuGain.x = {0.5 * tf(eye(2)),1 * tf(eye(2)), 2 * tf(eye(2))};
    
    [Controllers_case_list, Controllers_case_name_list, Controllers_n_cases] ...
        = generateCases(case_basis, 'tuned_controllers', true);
    
    % Controllers_case_list 5, 6, 7, 8, 9, 10 corresponds to case_basis 20, 19, 18, 17, 16, 15
    % for jdx = 5:10
    %     for idx = 1:length(case_basis.WeGain.x)
    %         % x = (case_basis.WeGain.x{idx}) - case_list(jdx).WeGain;
    %         x = (case_basis.WeGain.x{idx}) - Controllers_case_list(jdx).WeGain.x;
    %         if sum(x(1,1).Numerator{1,1}) == 0
    %             idx
    %             jdx
    %             x(1,1)
    %             break;
    %         end
    %     end
    % end

elseif EXTREME_K_COLLECTION
    case_basis.WindSpeedIndex.x = find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED);
    n_seeds = 1;
    % case_basis.Scheduling.x = [0]; %, 'No'};
    case_basis.Structure.x = {'Full-Order'}; %, 'Structured'};
    case_basis.Reference.x = [0];
    case_basis.Saturation.x = [0];

    IP_HIGH_GAIN = 10;
    WU_HIGH_GAIN = 10; % multiplied by initial gain of (1 / eps_u)=100
    WE_HIGH_GAIN = 10; % multiplied by initial gain of (1 / M_e)=1
    IP_LOW_GAIN = 10;
    OP_LOW_GAIN = 10;
    % [0.01, 0.05, 0.1, 0.5, 0.75, 1] * WU_HIGH_GAIN
    case_basis.WuGain.x = ...
        {tf(WU_HIGH_GAIN * 0.01 * eye(2)), ...
        tf(WU_HIGH_GAIN * 0.05 * eye(2)), ...
        tf(WU_HIGH_GAIN * 0.1 * eye(2)), ...
        tf(WU_HIGH_GAIN * 0.5 * eye(2)), ...
        tf(WU_HIGH_GAIN * 0.75 * eye(2)), ...
        tf(WU_HIGH_GAIN * eye(2))};
    % case_basis.WuGain.x = {tf(eye(2))};
    case_basis.WeGain.x = ...
        {tf(WE_HIGH_GAIN * 0.01 * eye(2)), ...
        tf(WE_HIGH_GAIN * 0.05 * eye(2)), ...
        tf(WE_HIGH_GAIN * 0.1 * eye(2)), ...
        tf(WE_HIGH_GAIN * 0.5 * eye(2)), ...
        tf(WE_HIGH_GAIN * 0.75 * eye(2)), ...
        tf(WE_HIGH_GAIN * eye(2))};
    % case_basis.W1Gain.x = ...
    %     {tf(IP_HIGH_GAIN * 0.01 * eye(2)), ...
    %     tf(IP_HIGH_GAIN * 0.05 * eye(2)), ...
    %     tf(IP_HIGH_GAIN * 0.1 * eye(2)), ...
    %     tf(IP_HIGH_GAIN * 0.5 * eye(2)), ...
    %     tf(IP_HIGH_GAIN * 0.75 * eye(2)), ...
    %     tf(IP_HIGH_GAIN * eye(2))};
    case_basis.W1Gain.x = {tf(eye(2))};

    % [0.01, 0.1, 1, 5, 7.5, 10]
    case_basis.W2Gain.x = ...
        {tf(IP_HIGH_GAIN * 0.001 * eye(2)), ...
        tf(IP_HIGH_GAIN * 0.01 * eye(2)) , ...
        tf(IP_HIGH_GAIN * 0.1 * eye(2)), ...
        tf(IP_HIGH_GAIN * 0.5 * eye(2)), ...
        tf(IP_HIGH_GAIN * 0.75 * eye(2)), ...
        tf(IP_HIGH_GAIN * eye(2))};
    

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
    case_basis.WindSpeedIndex.x = find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED);
    case_basis.WindSpeedIndex.x = 1:length(LPV_CONTROLLER_WIND_SPEEDS);
    % case_basis.Scheduling.x = [0]; %, 'No'};
    case_basis.Structure.x = {'Full-Order'}; %, 'Structured'};

    if VARY_REFERENCE
        % TODO Choose % of Open Loop mean value. Only use d reference
        load(fullfile(mat_save_dir, 'M_dq_reference.mat'));
        M_dq_reference(2) = 0; % set q reference to zeros
        case_basis.Reference.x = 1:length(VARY_REFERENCE_BASIS);
        % case_basis.Reference.d = [];
        % case_basis.Reference.q = [];
        % load(fullfile(mat_save_dir, 'Mdq_Reference.mat'));
        % for ref = VARY_REFERENCE_BASIS
        %     case_basis.Reference.d(end + 1) = ref * M_dq_reference(1);
        %     case_basis.Reference.q(end + 1) = ref * M_dq_reference(2);
        % end
    else
        case_basis.Reference.x = [0];
    end

    if VARY_SATURATION
        % load(fullfile(mat_save_dir, 'Beta_dq_saturation.mat'));
        load(fullfile(mat_save_dir, 'Beta_ipc_blade_saturation.mat'));

        case_basis.Saturation.x = 1:length(VARY_SATURATION_BASIS);

    else
        case_basis.Saturation.x = [0];
    end

    if VARY_WU
        case_basis.WuGain.x = {};
        for wu = VARY_WU_BASIS
            case_basis.WuGain.x{end+1} = tf(wu * eye(2));
        end
    else
        case_basis.WuGain.x = {tf(VARY_WU_BASIS(4) * eye(2))};
    end

    % most_robust_table(1, 'Case Desc.') % W1 = 1 W2 = 0.01 ->  Wu = 10 We = 0.1
    % for high robustness controller
    % case_basis.W1Gain.rob = ...
    %     {tf(1 * eye(2))};
    % case_basis.W2Gain.rob = ...
    %     {tf(0.01 * eye(2))};
    % case_basis.WeGain.rob = ...
    %     {tf(0.1 * eye(2))};


    % lowest_adc_table(1, 'Case Desc.') % W1 = 1 W2 = 0.01 ->  Wu = 10 We = 0.1
    % for low adc controller
    case_basis.W1Gain.adc = ...
        {tf(1 * eye(2))};
    case_basis.W2Gain.adc = ...
        {tf(0.01 * eye(2))};
    case_basis.WeGain.adc = ...
        {tf(0.1 * eye(2))};
    
    % lowest_mse_table(1, 'Case Desc.') % W1 = 1 W2 = 0.1 ->  Wu = 7.5 We = 10
    % for low output mse controller
    case_basis.W1Gain.y_mse = ...
        {tf(1 * eye(2))};
    case_basis.W2Gain.y_mse = ...
        {tf(0.1 * eye(2))};
    case_basis.WeGain.y_mse = ...
        {tf(10 * eye(2))};
    
    [Controllers_case_list, Controllers_case_name_list, Controllers_n_cases] ...
        = generateCases(case_basis, 'tuned_controllers', true);

    controller_types = fieldnames(case_basis.W1Gain)';
elseif BASELINE_K
    % case_basis.Scheduling.x = [0];
    case_basis.Reference.x = [0];
    case_basis.Saturation.x = [Inf];
    % case_basis.Scheduling.x = [Inf];
    
elseif STRUCT_PARAM_SWEEP
    case_basis.Reference.x = [0];
    case_basis.Saturation.x = [Inf];

   
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
% GenPlant_lpv.Parameter.u.RateBounds = [-2, 2];

ref_idx = [strmatch('M_d Reference', GenPlant(:, :, C_WS_IDX).InputName), strmatch('M_q Reference', GenPlant(:, :, C_WS_IDX).InputName)];
ctrl_input_idx = [strmatch('\beta_d Control Input', GenPlant(:, :, C_WS_IDX).InputName), strmatch('\beta_q Control Input', GenPlant(:, :, C_WS_IDX).InputName)];
tracking_error_idx = [strmatch('M_d Tracking Error', GenPlant(:, :, C_WS_IDX).OutputName), strmatch('M_q Tracking Error', GenPlant(:, :, C_WS_IDX).OutputName)];

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
% controller

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
