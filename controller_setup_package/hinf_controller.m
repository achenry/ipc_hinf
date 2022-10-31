clc; clear all; close all;

username = char(java.lang.System.getProperty('user.name'));
% char(java.net.InetAddress.getLocalHost.getHostName)
if strcmp(username, 'aoifework')
    home_dir = '/Users/aoifework/Documents';
    
    project_dir = fullfile(home_dir, 'Research', 'ipc_tuning');

    controller_setup_dir = fullfile(project_dir, 'controller_setup_package');
    chdir(controller_setup_dir);
    
    % contains setup_linear_plant.m, xop_arr, xop_red_arr, sys_arr,
    % sys_red_arr
    plant_setup_dir = fullfile(project_dir, 'plant_setup_package');
    addpath(plant_setup_dir);
    
    toolbox_dir = fullfile(home_dir, 'toolboxes');
    addpath(fullfile(toolbox_dir, 'PMtools'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox/A_Functions'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox/Utilities'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox', 'MBC', 'Source'));
   
    fig_dir = fullfile(project_dir, 'paper', 'figs');

elseif strmp(username, 'manuel')
    
    % directory containing hinf_controller.m (this file)
    controller_setup_dir = fullfile(project_dir, 'controller_setup_package');
    chdir(controller_setup_dir);

    % contains setup_linear_plant.m, xop_arr, xop_red_arr, sys_arr,
    % sys_red_arr
    plant_setup_dir = fullfile(project_dir, 'plant_setup_package');
    addpath(plant_setup_dir);

    addpath(fullfile('CHANGE ME', 'PMtools')); % MANUEL: add your filepaths here
    addpath(fullfile('CHANGE ME', 'matlab-toolbox/A_Functions'));
    addpath(fullfile('CHANGE ME', 'matlab-toolbox/Utilities'));
    addpath(fullfile('CHANGE ME', 'matlab-toolbox', 'MBC', 'Source')); % Need fx_mbc3 from this toolbox
    
    fig_dir = fullfile(project_dir, 'paper', 'figs');
end     

autrun;

addpath(fullfile(project_dir)); % sl model

CpCtCqFile = 'weis_job_00_Cp_Ct_Cq.txt';
DT = 0.0125;
C_BL_SOAR25_V2f_c73_Clean;

%% Other useful commands
% examine singular values of the CL results
% sigma(CL, ss(gamma))

% generalized feedbaack interconnection of two models
% CL = lft(P, K)

%% Define Continuous-Time Plant G
load('plant_setup_package/xop_arr.mat')
load('plant_setup_package/xop_red_arr.mat')
load('plant_setup_package/sys_arr.mat')
load('plant_setup_package/sys_red_arr.mat')

ws = 14;
WIND_SPEEDS = 10:0.5:22;
ws_idx = WIND_SPEEDS == ws;

Plant = sys_red_arr(:, :, ws_idx); % plant at 14m/s
xop = xop_red_arr(:, ws_idx);
% additional state added for each output due to low-pass filter
omega = xop(contains(Plant.StateName, 'First time derivative of Variable speed generator DOF')); % rad/s

% Fetch reference for Mdq
% load(fullfile(project_dir, 'xop_arr'));
% r_Md = xop_arr(6, ws_idx);
% r_Mq = xop_arr(7, ws_idx);

%% Formulate Tunable Continuous-Time Controller Structure for hinfstruct

% Q: how to include filters? 
% => replace notch filters with LPFs (w/ notch
% filtering on top if roll-off is not steep enough?)
% could introduce perforance goal to be unimportant at high-freq,
% could weight control input with HPF => suppress high-freq action
% ? which approach would be better: LP characteristic via Weighting Filters
% or explicit tunable filter ?
% => First try with explicit first/second-order LPF

% Q: how to include penalty on dBeta/dt
% =>


% Define LPF structure with no zeros and one pole. \
% QUESTION is there a LPF for D and Q?
% LPF_D = tunableTF('LPF_D', 0, 1);
% LPF_Q = tunableTF('LPF_Q', 0, 1);

% TODO Manuel - Notch filters are very involved - LPV and this omega_ss is out of range - is it necessary to define here?
% Define Notch Filter structure
% LPF_param = tunableSS('LPF_param', 2, 1, 1, DT);
% LPF_param.A.Value = dLPF_slow.a;
% LPF_param.B.Value = dLPF_slow.b;
% LPF_param.C.Value = dLPF_slow.c;
% LPF_param.D.Value = dLPF_slow.d;
% Gain_param = tunableGain('Gain_param', 1, 1);
% Gain_param.Value = 1/Parameters.Turbine.G * 2;


Notch_3P = tunableSS('LPV_3P', 2, 1, 1, DT);
Notch_3P.A.Value = PSM0_NF_3P(:,:,10,1);

% Define PI controller structure
PI_D = tunablePID('PI_D', 'pi'); % tunable PI for d components w/ initial values
PI_Q = tunablePID('PI_Q', 'pi'); % tunable PI for q components

% Set initial, minimum, maximum values
LPF_D.Denominator.Value(2) = 3 * max(om_1P);
LPF_D.Denominator.Minimum(2) = max(om_1P);
LPF_Q.Denominator.Value(2) = 3 * max(om_1P);
LPF_Q.Denominator.Minimum(2) = max(om_1P);

PI_D.Kp.Value = Parameters.cIPC.DQ_Kp_1P;
PI_D.Ki.Value = Parameters.cIPC.DQ_Ki_1P;

PI_Q.Kp.Value = Parameters.cIPC.DQ_Kp_1P;
PI_Q.Ki.Value = Parameters.cIPC.DQ_Ki_1P;

% Formulate SISO blocks
PI_DQ = blkdiag(PI_D, PI_Q);
LPF_DQ = blkdiag(LPF_D, LPF_Q);

% Formulate full controller
K_DQ = series(LPF_DQ, PI_DQ);


%% Compute Continuous-Time Generalized Plant P from plant G and weighting filters W
% inputs [w; u], outputs [z; y]

% define exogenous inputs w = disturbance on Mdq ? how to define ?

% define performance outputs z = Wu * beta_dq, Wt * M_dq

% define measured outputs y

% Define weighting matrices
s = tf('s');
omega_1 = 2 * max(om_1P);
omega_2 = 8 * max(om_1P);
hpf = (s + omega_1) / (s + omega_2);

% add high-pass filter w/ roll-off after 2P,
% need pole (s+2P)/(s+8P) s.t. it is implementable
W2 = 1e-6 * eye(2) * hpf; % Wu, control input u weighting to performance outputs z2 

W3 = 1e-6 * ones(2); % Wt, output y weighting to performance outputs z3
% can reduce to 0 w/ enough control authority, not near limit of fine-pitch/rated bc risk of stall
% cannot command asymmetrical deflections at fine-pitch ie near or below ie
% bc (mean) collective pitch is at minimum and cannot oscillate below this
% rated, so instead set 'fine-pitch' to higher values

P = augw(G_red, [], W2, W3);
P.InputName(1) = {'RootMycD Disturbance'};
P.InputName(2) = {'RootMycQ Disturbance'};
P.OutputName(1) = {'Weighted Blade D pitch command'};
P.OutputName(2) = {'Weighted Blade Q pitch command'};
P.OutputName(3) = {'Weighted RootMycD'};
P.OutputName(4) = {'Weighted RootMycQ'};
P.OutputName(5) = {'Weighted Perturbed RootMycD'};
P.OutputName(6) = {'Weighted Perturbed RootMycQ'};

% TODO bodemag(P), compare to G_min,
figure;
bodemag(G, P)

nmeas = 2; % number of measurement outputs y, K has nmeas inputs
ncont = 2; % number of control inputs u, K has ncont outputs

%% Synthesize Continuous-Time Controller w/ systune
if 0
    % define controller structure
    PI_D = tunablePID('PI_D', 'pi'); % tunable PI for d components
    PI_D.Kp.Value = Parameters.cIPC.DQ_Kp_1P;
    PI_D.Ki.Value = Parameters.cIPC.DQ_Ki_1P;
    PI_Q = tunablePID('PI_Q', 'pi'); % tunable PI for q components
    PI_Q.Kp.Value = Parameters.cIPC.DQ_Kp_1P;
    PI_Q.Ki.Value = Parameters.cIPC.DQ_Ki_1P;
    PI_DQ = blkdiag(PI_D, PI_Q);
    
    % define analysis points
    M_ap = blkdiag(AnalysisPoint('RootMycD'), AnalysisPoint('RootMycQ'));
    Beta_ap = blkdiag(AnalysisPoint('BldPitchD'), AnalysisPoint('BldPitchQ'));
    FB_ap = blkdiag(AnalysisPoint('FeedbackD'), AnalysisPoint('FeedbackQ'));
    
    % build closed-loop transfer function
    T0 = feedback(M_ap * G_min * Beta_ap * PI_DQ, FB_ap);
    T0.InputName = {'rd', 'rq'};
    T0.OutputName = {'RootMycD', 'RootMycQ'};
    
    % Define Tuning Goals
    Req1 = TuningGoal.Margins({'FeedbackD', 'FeedbackQ'}, 10, 40); % 6dB of Gain Margin and 45deg of PM
    Req1.Focus = [max(om_1P), Inf];
    % Req1.Focus = [1, 10];
    % Req2 = TuningGoal.Margins({'BldPitchD', 'BldPitchQ'}, 18, 60);
    Req3 = TuningGoal.Tracking({'rd', 'rq'}, {'RootMycD', 'RootMycQ'}, 5, 0.01);
    % Req3.Focus = [max(om_1P), Inf];
    
    SoftReqs = [Req1, Req3];
    [CL,fSoft,gHard] = systune(T0,[],SoftReqs);
    
    getBlockValue(PI_DQ)
    
    PI_DQ_final = blkdiag(pid(PI_DQ.Blocks.PI_D), pid(PI_DQ.Blocks.PI_Q));
    save(fullfile(project_dir, 'PI_DQ_final'), 'PI_DQ_final');
    
    % pid(CL.Blocks.PI_D)
    % pid(CL.Blocks.PI_Q)
    
    figure;
    viewGoal(SoftReqs,CL)
    
    figure;
    stepplot(CL);
    
    figure;
    impulseplot(CL);
end


%% Synthesize Continuous-Time Controller w/ hinfsyn for gain controller, full-order controller

if 0
    % gamma = 0.7; % controller performance, Hinf norm of CL
    [K, CL, gamma] = hinfsyn(P, nmeas, ncont);
    L = series(K, G_min);
end


%% Analyze System w/ Initial Continuous-Time Controller
L0 = G_red * K_DQ;
CL0 = feedback(L, 1);
step(CL0)
impulse(CL0)
diskmargin(L0)
bode(L0)

%% Synthesize Continuous-Time Controller w/ hinfstruct

% QUESTION how to format full system as block diagram??
% T0 = feedback(series(PI_DQ, P), eye(2));
% rng('default')
opt = hinfstructOptions;
T = hinfstruct(P, K_DQ, opt); % lft(P, PI_DQ)

%% Analyze Continuous-Time Controller
if 0
    % figure(1)
    % step(CL)
    % set(gcf, 'Position', get(0, 'Screensize'));
    % saveas(gcf, './presentation/figs/stepplot.png')
    % figure(2)
    % bode(G_min, L)
    % legend('Uncompensated', 'Compensated')
    % set(gcf, 'Position', get(0, 'Screensize'));
    % saveas(gcf, './presentation/figs/bodeplot.png')
    margins_G = diskmargin(G_min)
    margins_L = diskmargin(L)
    
    margins_G.GainMargin
    margins_L.GainMargin
    
    margins_G.PhaseMargin
    margins_L.PhaseMargin
    
    margins_G.DiskMargin
    margins_L.DiskMargin
    
    margins_G.WorstPerturbation
    margins_L.WorstPerturbation
    % legend('Uncompensated', 'Compensated')
    % set(gcf, 'Position', get(0, 'Screensize'));
    % saveas(gcf, './presentation/figs/marginplot.png')
end

%% Analyze Discrete-Time Controller

if 0
    Ts = logspace(-3, -1, 3);
    
    for ts = Ts
        G_min_dt = c2d(G_min, ts);
        L_dt = c2d(L, ts);
        CL_dt = feedback(L_dt, eye(size(L_dt)));
        
        figure(1)
        step(CL_dt, 100)
        set(gcf, 'Position', get(0, 'Screensize'));
        saveas(gcf, ['./presentation/figs/stepplot_', num2str(ts), '.png'])
        
        figure(2)
        bode(G_min_dt, L_dt)
        legend('Uncompensated', 'Compensated')
        set(gcf, 'Position', get(0, 'Screensize'));
        saveas(gcf, ['./presentation/figs/bodeplot_', num2str(ts), '.png'])
        
        [DM_G, MM_G] = diskmargin(G_min_dt)
        [DM_L, MM_L] = diskmargin(L_dt)
        
        margins_G_dt.GainMargin
        margins_L_dt.GainMargin
        
        margins_G_dt.PhaseMargin
        margins_L_dt.PhaseMargin
        
        margins_G_dt.DiskMargin
        margins_L_dt.DiskMargin
    end
    
    % margins_G_dt.WorstPerturbation
    % margins_L_dt.WorstPerturbation
    % legend('Uncompensated', 'Compensated')
    % set(gcf, 'Position', get(0, 'Screensize'));
    % saveas(gcf, './presentation/figs/marginplot.png')
end

%% Run Simulink simulation with new controller

%% Questions
% how to enforce structure of controller ie PI
% => Ossman did full-order controller design (same order as plant more complex), 
% we could look at MIMO structured, need to use hinfstruct (contained by
% systune)

% how to implement low vs high order controller?
% => structured (ie PI) => convex problem vs. non-structured (plant/full)
% => nonlinear, nonsmooth, starting points matter, need to minimize number
% of variables bc solutions will not converge
% how to consider tf for deviations of Mdq and Betadq AND absolute values ?
% => define additional z output and put weights on output
% => define artificial output y_beta = 0x + 1u
% => dy_beta/dt = Cdx/dt + Ddu/dt = C(Ax + Bu) + D du/dt
% => Check D matrix. At infinite gain, for real systems, there is zero
% gain. Can make zero by adding low-pass filter bc required for many
% algorithms w/ cutoff frequency just above system dynamics.
% Getting drvt of gain is equiv to multi;lying by s, equiv to rotation in
% Bode plot => can shape frequency response of gain to shape rate of gain
% Can make weighting filter that shapes gain

% how to implement LPV vs non-LPV?
% non-LPV = one LTI, non-varying controller for all wind speeds (const
% gains for all wind speeds)
% Better performance with gain-scheduling, LPV is one approach
% First, establish control design at one wind-speed for as long as
% possible,
% then extend to multiple wind speeds
% (i) test that controller for full range of wind speeds
% (ii) design controller at different wind speed and test for full range
% (iii) tune LTI controllers for different wind speeds, interpolate
% inbetween. Investigate how they are different. Best performance at partic
% wind speed is one from controller tuned at that wind speed
% (iv) Interpolation step. Assume basis function for gain 
% e.g. linear model k_p = kp0 + v kp1 and tune kp0 and kp1 
%  => use regression to approximate kp with minimum number of coefficients
% increase order of basis function incrementally and compare scheduling
% impact

% how to implement MIMO vs SISO?
% SISO: eye block controller, (i) individual blocks by just consider d or q
% components, then together tuning by consider dq system

% how to structure discrete-time generalized plant P, are disturbances
% considered ?
% what margins to use for MIMO system

% Including filters in tuning process, weighting filters are tunable

% TODO
% Structured SISO PI with hinfstruct
% => keep Overleaf with Problem Formulation & Notes
% Read Tutorial
% Read IPC Ossman papers
% Read Multivariable Feedback Hinf control

%% Synthesize Discrete-Time Controller
if 0
    % ? what about disturbances ? am I formulating this correctly ?
    nmeas = 2; % number of measurement outputs y, K has nmeas inputs
    ncont = 2; % number of control inputs u, K has ncont outputs
    Ts = 0.000001;
    
    P_dt = ss(P.A, P.B, P.C, zeros(6, 4));
    
    % gamma = 0.7; % controller performance, Hinf norm of CL
    [K_dt, gamma_dt] = sdhinfsyn(P_dt, nmeas, ncont, 'Ts', Ts);
end

%% Test Discrete-Time Controller
