%% Setup workspace
initialize;

C_WS_IDX = LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED;

if EXTREME_K_COLLECTION
    fig_dir = fullfile(fig_dir, 'extreme_controllers');
elseif OPTIMAL_K_COLLECTION
    fig_dir = fullfile(fig_dir, 'optimal_controllers');
elseif BASELINE_K
    fig_dir = fullfile(fig_dir, 'baseline_controller');
end

if ~exist(fig_dir, 'dir')
    mkdir(fig_dir)
end

% Define transfer functions
% What are GSo and KSi? What transfer functions do we care about?
% => Shaping all in GenPlant. Eliminate d3, it is redundant, d3 column
% matrices appear in others
% => Ti<->Wu, To<->Wy, So<->We?
% => Want to shape To. Check Julien's thesis for GSo, KSi.
% Note: these functions will not work for structured controller, for which
% G*K are element-wise multiplied
% So = @(Plant, K) inv(eye(size(Plant)) + Plant*K) * eye(size(Plant)); % weighted by We, Wy
% Si = @(Plant, K) inv(eye(size(Plant)) + K*Plant) * eye(size(Plant)); % weighted by Wu
% GSo = @(Plant, K) inv(eye(size(Plant)) + Plant*K) * Plant; % weighted by We, Wy
% KSi = @(Plant, K) inv(eye(size(Plant)) + K*Plant) * eye(size(Plant)); % weighted by Wu
% To = @(Plant, K) inv(eye(size(Plant)) + Plant*K) * (Plant*K); % weighted by Wy
% Ti = @(Plant, K) inv(eye(size(Plant)) + K*Plant) * (K*Plant); % weighted by Wu

%% Load linear models, operating points, hnorms
load(fullfile(code_dir, 'matfiles', 'Plant_red'));
Plant = Plant_red;
Plant.InputName = {'BldPitchD Control Input', 'BldPitchQ Control Input'};
Plant.OutputName = {'RootMycD Output', 'RootMycQ Output'};
all(real(pole(Plant)) < 0);

%% Normalization for inputs and outputs

% normalize Plant channels: maximum IPC command/expected Mdq values, multiply input by
% scaling to get actual u before plant; divide output by scaling to get -1,
% 1 after plant; scaling factors are part of controller
load(fullfile(FAST_directory, 'op_absmax'));
clear ip_scaling op_scaling
for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
    idx = (op_absmax.dq(:, 'Wind1VelX') == LPV_CONTROLLER_WIND_SPEEDS(c_ws_idx));
    ip_scaling(:, :, c_ws_idx) = diag(deg2rad(op_absmax.dq(idx.Variables, {'BldPitchC', 'BldPitchC'}).Variables) * 0.2);
    op_scaling(:, :, c_ws_idx) = diag(op_absmax.dq(idx.Variables, {'RootMycC', 'RootMycC'}).Variables * 0.2);

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
eps_u = 1e-1; % desired disturbance attenuation at really high frequencies
M_u = 10; % desired bound on Hinf norm of input sensitivity function at really low frequencies
w1_u = omega_1P_rad * 4; % first breakpoint frequency
omega_u = M_u * w1_u; % 2.5e-3; % controller bw in the high-freq range
hpf = (1 / eps_u) * (s + omega_u/M_u) / (s + (omega_u / eps_u));
Wu = 1 * hpf * eye(2);
% Wu = 500 * tf([1 3.464], [1 3464]) * eye(2); % Ossman's values
Wu.InputName = {'BldPitchD Control Input', 'BldPitchQ Control Input'};
Wu.OutputName = {'Weighted BldPitchD Control Input', 'Weighted BldPitchQ Control Input'};
% bodemag(Wu)
% Wy=Weighting on To, GSo 0dB at low frequency, low at
% high-frequency => hpf with low-frequency gain below 0dB 
% apply integrator + inverse notch penalty (zeta in num > zeta in
% denom) to output and error
eps_y = 1e-1; % desired disturbance attenuation at really high frequencies
M_y = 2; % desired bound on Hinf norm of input sensitivity function at really low frequencies, ideally 1
w1_y = omega_1P_rad * 0.1; % first breakpoint frequency
omega_y = M_y * w1_y; % 2.5e-3; % controller bw in the high-freq range
hpf = (1 / eps_y) * (s + omega_y/M_y) / (s + (omega_y / eps_y));
Wy = 1 * eye(2) * hpf * 0;
% Wy = 0.1 * tf([1 3], [1 0.03]) * tf([1 (0.49/sqrt(24))*omega_3P_rad omega_3P_rad^2], [1 (6.93/sqrt(24))*omega_3P_rad omega_3P_rad^2]) * eye(2); % Ossman's values
Wy.InputName = {'RootMycD Output', 'RootMycQ Output'}; 
Wy.OutputName = {'Weighted RootMycD Output', 'Weighted RootMycQ Output'};

% We = Weighting on So, GSo. 0dB at high frequency, low at
% low-frequency => lpf with high-frequency gain below 0dB that saturates
eps_e = 1e-1; % desired disturbance attenuation at really low frequencies
w1_e = omega_3P_rad; % * 1e-1; % first breakpoint frequency
omega_e = (1 / eps_e) * w1_e; % 0.07*pi; % desired CL bw
M_e = 2; % desired bound on Hinf norm of output sensitivity function at really high frequencies, ideally 1
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
We.InputName = {'Measured RootMycD Tracking Error', 'Measured RootMycQ Tracking Error'}; 
We.OutputName = {'Weighted RootMycD Tracking Error', 'Weighted RootMycQ Tracking Error'}; 

% Weighting on reference
W1 = tf(1 * eye(2));

% Weighting on control input disturbance
% apply proportional filter to penalize disturbances on inputs, outputs
W2 = tf(1 * eye(2));
% Wdu.InputName = {'Input Disturbance, $d_u$'};
% Wdu.OutputName = {'Weighted Input Disturbance, $W_{d_u} d_u$'};

if 0
    % Plot Weighting matrices
    figure;
    omega = logspace(-2, 4, 300);
    bodemag(Wu, We, Wy, omega, bode_plot_opt);
    % bode_plot_opt.XLim = [10^(-1), 11];
    % setoptions(h, 'XLim', )
    % title('Weighting Functions');
    axh = findall(gcf, 'type', 'axes');
    xline(axh(3), omega_1P_rad * HARMONICS);
    xline(axh(5), omega_1P_rad * HARMONICS);
    xline(axh(7), omega_1P_rad * HARMONICS);
    xline(axh(9), omega_1P_rad * HARMONICS);
    legend("W_u", "W_e", "W_y", '', '', '', '');
    set(gcf, 'Position', [0 0 1500 900]);%get(0, 'Screensize'));
    savefig(gcf, fullfile(fig_dir, 'weighting_funcs.fig'));
    saveas(gcf, fullfile(fig_dir, 'weighting_funcs.png'));
end

%% Create Case Basis for different controllers

if EXTREME_K_COLLECTION
    case_basis.Scheduling = {'No'}; %, 'No'};
    case_basis.Structure = {'Full-Order'}; %, 'Structured'};

    IP_HIGH_GAIN = 1;
    WU_HIGH_GAIN = 100;
    WE_HIGH_GAIN = 10;
    IP_LOW_GAIN = 0.1;
    OP_LOW_GAIN = 1;
    % case_basis.WuGain = {tf(OP_LOW_GAIN * eye(2)), tf(WU_HIGH_GAIN * eye(2))}; %{100, 200, 400, 800, 1600}; % for detuning blade-pitch actuation
    case_basis.WuGain = {tf(IP_LOW_GAIN * eye(2))};
    case_basis.WeGain = {...
        tf(WE_HIGH_GAIN * 0.1 * eye(2)), ...
        tf(WE_HIGH_GAIN * 0.2 * eye(2)), ...
        tf(WE_HIGH_GAIN * 0.4 * eye(2)), ...
        tf(WE_HIGH_GAIN * 0.8 * eye(2)), ...
        tf(WE_HIGH_GAIN * eye(2))};
    case_basis.W1Gain = {...
        tf(IP_HIGH_GAIN * 0.1 * eye(2) * notch), ...
        tf(IP_HIGH_GAIN * 0.2 * eye(2) * notch), ...
        tf(IP_HIGH_GAIN * 0.4 * eye(2) * notch), ...
        tf(IP_HIGH_GAIN * 0.8 * eye(2) * notch), ...
        tf(IP_HIGH_GAIN * eye(2) * notch)};
    case_basis.W2Gain = {...
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
        = generateCases(case_basis, 'tuned_controllers', false);
    
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
    case_basis.Scheduling = {'No'}; %, 'No'};
    case_basis.Structure = {'Full-Order'}; %, 'Structured'};

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

ref_idx = [strmatch('RootMycD Reference', GenPlant(:, :, C_WS_IDX).InputName), strmatch('RootMycQ Reference', GenPlant(:, :, C_WS_IDX).InputName)];
ctrl_input_idx = [strmatch('BldPitchD Control Input', GenPlant(:, :, C_WS_IDX).InputName), strmatch('BldPitchQ Control Input', GenPlant(:, :, C_WS_IDX).InputName)];
tracking_error_idx = [strmatch('Measured RootMycD Tracking Error', GenPlant(:, :, C_WS_IDX).OutputName), strmatch('Measured RootMycQ Tracking Error', GenPlant(:, :, C_WS_IDX).OutputName)];

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
save(fullfile(code_dir, 'matfiles', 'K0.mat'), 'K0');

if 0
    figure;
    bodeplot(K0, bode_plot_opt);
    axh = findall(gcf, 'type', 'axes');
    xline(axh(5), omega_1P_rad * HARMONICS);
    xline(axh(3), omega_1P_rad * HARMONICS);
    xline(axh(7), omega_1P_rad * HARMONICS);
    xline(axh(9), omega_1P_rad * HARMONICS);
    title('Frequency Response of Baseline Controller');
end