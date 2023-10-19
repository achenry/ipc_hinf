% SUMR-D Baseline Control Script
% 6/8/17 - Including Precone angle in gain calculation
% This script sets up the variables for the CP simulink models

% addpath('/Users/aoifework/Documents/toolboxes/matlab-toolbox/A_Functions/')
% addpath('./TurbSim/A_Functions/')

% Load Wind File
% velocity, twrVelocity, y, z, zTwr, nz, ny, dz, dy, dt, zHub, z1,mffws] = readTSgrid('./TurbSim/TurbSim_IF/TurbSim.bts');
% time = 0:dt:(size(velocity, 1) - 1) * dt;
% ws_ts = timeseries(velocity(:, 1), time);

% load('/Users/aoifework/Documents/Research/learning_actuation/SOAR-25-V2f_DLC12_PitchActuatorAnalysis/B_14_2.mat')
% rs_dt = Chan.tt(2) - Chan.tt(1);
% rs_ts = timeseries(Chan.RotSpeed, Chan.tt);

%% LOAD TURBINE PARAMETERS
if exist('TurbineParameters', 'file')
    ptemp                   = load(fullfile('TurbineParameters', Parameters.Turbine.String));
    Parameters.Turbine      = catstruct(ptemp.Parameters.Turbine, Parameters.Turbine);
    
    if isfield(Parameters,'Control')
        Parameters.Control      = catstruct(ptemp.Parameters.Control,Parameters.Control);
    else
        Parameters.Control = ptemp.Parameters.Control;
    end
    if isfield(Parameters,'Tower')
        Parameters.Tower      = catstruct(ptemp.Parameters.Tower,Parameters.Tower);
    else
        Parameters.Tower = ptemp.Parameters.Tower;
    end
else
    load 'SOAR-25-V2f'
    ptemp.Parameters = Parameters;
end
% ============================================================= %


%%
Upratelimiter.Disable = 51922763.3;%51922763.3;5192
Downratelimiter.Disable = -99922763;%-99922763;
% Setup simulation variables ONLY USED FOR NREL BASELINE CONTROLLER!!!
% S_ConstTq=1;    % Switch: 1 will enable constant torque in Region 3
S_LowBPLim=1;   % Switch: 1 will hold Region 3 for BP>1, 0 will have Region 3 dependent on speed alone

%% Parking Brake
Parameters.Turbine.Brake = 1;

%% Set IC's of turbine
Parameters.Turbine.IC.Bp = 10; % Blade pitch IC, [degrees]
Parameters.Turbine.IC.BpSH = 0;
Parameters.Turbine.IC.Wr = 4.118; % Rotor speed IC [rpm]
Parameters.Turbine.IC.Wg = 4.118;
Parameters.Turbine.IC.Tg = 35e3;
Parameters.Turbine.max_pitch = 90; % deg

%% Sampling Rate
Control.DT = 0.0125;

%% TEMP FOR WSE TEST
% range of rotor speeds
RotorSpeedDomain  = linspace(Parameters.Turbine.wr_min, 1.15*Parameters.Turbine.wr_rated, 10);
Parameters.Control.PowerRating = 1.0;

%% Low pass filter for High Speed Shaft speed
Parameters.LPF.f_cut = 0.125; % new value after SUMR-D issue is 1 HZ;0.5 old  %Low pass filter cut-off frequency [Hz]
Parameters.LPF.w_cut = 2*pi*Parameters.LPF.f_cut;         %Low pass filter cut-off frequency [rad/s]

Parameters.LPF.LPnum = [(1-exp(-Parameters.LPF.w_cut*Control.DT)) 0];
Parameters.LPF.LPden = [1 -exp(-Parameters.LPF.w_cut*Control.DT)];
% ============================================================= %

%% Torque Controller Variables (Defined in Simulink Matlab Function)
Turbine = Parameters.Turbine;

%% Optimal Region 2
rho = Parameters.Turbine.rho;

R_proj   = Turbine.R*cosd(Turbine.ConeAngle);
Cp_opt = Turbine.Cp_max/(cosd(Turbine.ConeAngle))^2;
la_opt  = Turbine.la_max*cosd(Turbine.ConeAngle);
G   = Turbine.G;

k_factor = Parameters.Control.k_factor;
k_opt_actual = k_factor * ( 0.5*rho*pi*R_proj^5*Cp_opt * (cosd(Turbine.ConeAngle))^5 ) / ((la_opt)^3 * G^3);
k_opt = k_opt_actual / (radps2rpm(1))^2;

%% Linear Region 1.5
wg_min  = Turbine.wr_min * G;
wg_1d5  = wg_min*1.2;

T_1d5_high = k_opt*wg_1d5^2;
T_1d5_low = 0;

m1_5 = (T_1d5_high - T_1d5_low)/(wg_1d5 - wg_min);
c1_5 = T_1d5_high - m1_5*wg_1d5;

%% Linear Region 2.5
T_rated     = Turbine.T_rated;
wg_rated    = Parameters.Turbine.wg_rated; % rpm
R25_start   = 0.95;
w2_high     = R25_start*wg_rated;
T_2_high    = k_opt*w2_high^2;
R25_start   = 0.95;

% Slope/intercept
m2_5 = (T_rated-T_2_high)/(wg_rated-w2_high);
c2_5 = T_rated - m2_5*wg_rated;     %Region 2.5 offset (Nm)

%% PI Torque Control

TorqueControlParams.PIControl = 0;          %use PI Control?

if TorqueControlParams.PIControl
    R25_start = 0.96;
    w2_high     = R25_start*wg_rated;
end

% hard coded from Bossanyi, with gain factor
Tq_Kp = (rpm2radps(4200)/1)*990.9534067;
Tq_Ki = (rpm2radps(2100)/1)/2;
Tq_Kd = 0;
Kaw = 5;

% Margins for transition
R1d5_Thresh = 0.05;
R2d5_Thresh = 0.05;

%% Setpoint Smoothing Control
Parameters.Control.SetSmooth.Enable = 0;
% g1, g2 tuned for R2:10.55 and R3:10.575 wind speeds
Parameters.Control.SetSmooth.g_1         = 7.5;
Parameters.Control.SetSmooth.g_2         = 5;

LPF_SetSmooth                            = Af_LPF(2*pi/10,1,Control.DT,1);

%% Assign Values to Struct
TorqueControlParams.wg_rated    = wg_rated; % rpm
TorqueControlParams.T_rated     = T_rated; % Nm
TorqueControlParams.w2_high     = w2_high; 
TorqueControlParams.w1_high     = wg_1d5;
TorqueControlParams.T1_high     = T_1d5_high;
TorqueControlParams.wg_min      = wg_min;
TorqueControlParams.T1_min      = T_1d5_low;
TorqueControlParams.m2_5        = m2_5;
TorqueControlParams.c2_5        = c2_5;
TorqueControlParams.m1_5        = m1_5;
TorqueControlParams.c1_5        = c1_5;
TorqueControlParams.k_opt       = k_opt;
TorqueControlParams.Ki          = Tq_Ki;
TorqueControlParams.Kp          = Tq_Kp;
TorqueControlParams.Kd          = Tq_Kd;
TorqueControlParams.T           = Control.DT;
TorqueControlParams.w2_low      = wg_1d5;
TorqueControlParams.T2_low      = k_opt*(wg_min*1.05)^2;
TorqueControlParams.fine_pitch  = Parameters.Turbine.fine_pitch; % deg
TorqueControlParams.Kaw         = Kaw;
TorqueControlParams.StartupTime = 0;
TorqueControlParams.R1d5_Thresh = R1d5_Thresh;
TorqueControlParams.R2d5_Thresh = R2d5_Thresh;
TorqueControlParams.R25_start   = R25_start;
% ============================================================= %



%% BLADE PITCH CONTROL
%% Blade Pitch Actuator
Parameters.PitchActuator.f_cutoff   = ptemp.Parameters.PitchActuator.f_cutoff;%1 Hz
Parameters.PitchActuator.damping    = 0.7;%0.7; %0.7 
Parameters.PitchActuator.TF_num     = ((2*pi*Parameters.PitchActuator.f_cutoff)^2)*1;
Parameters.PitchActuator.TF_den     = [1,2*Parameters.PitchActuator.damping*(2*pi*Parameters.PitchActuator.f_cutoff),(2*pi*Parameters.PitchActuator.f_cutoff)^2];


%% Blade Pitch Controller gains
PitchControlParams.zeta     = Parameters.Control.pitch_zeta;
PitchControlParams.omega_n  = Parameters.Control.pitch_omega;
PitchControlParams.theta_k  = Parameters.Turbine.theta_kP*[Parameters.Turbine.ConeAngle^2;Parameters.Turbine.ConeAngle;1];
PitchControlParams.Sens_0   = Parameters.Turbine.Sens_0;

%First principles zero-pitch gains
GainFactor = Parameters.Control.p_factor;% 1;
PitchControlParams.Kp_0 =( GainFactor*2*Parameters.Turbine.Jtot*rpm2radps(Parameters.Turbine.wr_rated)*PitchControlParams.zeta*PitchControlParams.omega_n/(Parameters.Turbine.G*-PitchControlParams.Sens_0))*5;
PitchControlParams.Ki_0 = GainFactor*Parameters.Turbine.Jtot*rpm2radps(Parameters.Turbine.wr_rated)*PitchControlParams.omega_n^2/(Parameters.Turbine.G*-PitchControlParams.Sens_0);
% ============================================================= %

%% INDIVIDUAL PITCH CONTROL
%% Cyclic Pitch Controller
Parameters.Control.IPCDQ.Enable     = 0;
Parameters.Control.IPC3DQ.Enable     = 0;
Parameters.Control.IPCDQ.Excite3P     = 0;
Parameters.Control.IPCDQ.Mode       = 1;  % 1 no IPC below rated (no power loss), 2 IPC always on, 3 M_0 dependent

% IPC DQ GAINS
kmul = 0.025;
Parameters.cIPC.DQ_Kp_1P = kmul*40e-8;%0e-8;
Parameters.cIPC.DQ_Ki_1P = kmul*30e-8;%16e-8;
Parameters.cIPC.DQ_Kd_1P = kmul*5e-8;%0e-7;

Parameters.cIPC.D_Kp_1P = 2.8066e-06; %kmul*40e-8;%0e-8;
Parameters.cIPC.Q_Kp_1P = 2.6919e-06; %kmul*40e-8;%0e-8;
Parameters.cIPC.D_Ki_1P = 2.8066e-06; %kmul*30e-8;%16e-8;
Parameters.cIPC.Q_Ki_1P = 2.6919e-06; %kmul*30e-8;%16e-8;
Parameters.cIPC.D_Kd_1P = kmul*5e-8;%0e-7;
Parameters.cIPC.Q_Kd_1P = kmul*5e-8;%0e-7;

Parameters.cIPC.D_Kp_3P = 6.1751e-06; %kmul*40e-8;%0e-8;
Parameters.cIPC.Q_Kp_3P = 6.1617e-06; %kmul*40e-8;%0e-8;
Parameters.cIPC.D_Ki_3P = 6.1751e-06; % kmul*30e-8;%16e-8;
Parameters.cIPC.Q_Ki_3P = 6.1617e-06; %kmul*30e-8;%16e-8;
Parameters.cIPC.D_Kd_3P = kmul*5e-8; %0e-7;
Parameters.cIPC.Q_Kd_3P = kmul*5e-8; %0e-7;

Parameters.cIPC.DQ_Kp_2P = 2e-8;
Parameters.cIPC.DQ_Ki_2P = 4e-8;

% RotorSpeedDomain = linspace(Parameters.Turbine.wr_min, 1.15*Parameters.Turbine.wr_rated, 10)
om_1P = 2*pi*RotorSpeedDomain/60; %rad/s

% Cyclic Pitch Controller Filters, scheduled by omega [rad/s] - parameter
% input is 2/3/4/6 * Omega
[~,IPCDQ_NF_2P] = Af_MovingNotch(2*om_1P,0.8,0.02,Control.DT);
[~,IPCDQ_NF_4P] = Af_MovingNotch(4*om_1P,0.5,0.01,Control.DT);

[~,PSM0_NF_1P]  = Af_MovingNotch(om_1P,0.8,0.08,Control.DT);
[~,PSM0_NF_2P]  = Af_MovingNotch(2*om_1P,0.8,0.008,Control.DT);

zeta = 1;
drop = 10;
[~,PSM0_NF_3P]  = Af_MovingNotch(3*om_1P,zeta,zeta/drop,Control.DT);

[~,PSM0_NF_6P]  = Af_MovingNotch(6*om_1P,0.8,0.008,Control.DT);
PSM0_LPF    = Af_LPF(2*pi/1,.707,Control.DT);

IPCDQ_LPF    = Af_LPF(3*om_1P(8),.707,Control.DT);

RotorSpeedDomain_bp = linspace(0.98*Parameters.Turbine.wr_rated, 1.02*Parameters.Turbine.wr_rated, 10);
om_1P_bp = 2*pi*RotorSpeedDomain_bp/60; %rad/s
[~, bandpass_1P]  = Af_MovingNotch_bandpass(om_1P_bp,0.8,0.08,Control.DT);

% om_1P_bp / (2*pi) % Hz
[~, bandpass_3P]  = Af_MovingNotch_bandpass(3*om_1P_bp,0.8,0.08,Control.DT); 
% 3*om_1P_bp / (2*pi) % Hz
% QUESTION MANUEL this seems to result in selection of 2P loads?, do we need matrices for full range of omega?
% [~, bandpass_1P]  = Af_MovingNotch_bandpass((2*pi/60) * Parameters.Turbine.wr_rated,0.8,0.08,Control.DT);
% [~, bandpass_3P]  = Af_MovingNotch_bandpass(3*(2*pi/60)*Parameters.Turbine.wr_rated,0.8,0.08,Control.DT);
% bode(bandpass_1P, bandpass_3P)

% Cyclic Pitch Phase Lead
% 0 deg yaw
Parameters.cIPC.phaseLead_1P = 1.8;      %deg
Parameters.cIPC.phaseLead_2P = 69.4;      %deg

% mode 3 gains
Parameters.Control.IPCDQ.M0_set     = 30e3;
Parameters.Control.IPCDQ.k_sat      = 1/20e3;


% Set Params
PitchControlParams.IPCDQ_MODE   = Parameters.Control.IPCDQ.Mode;
PitchControlParams.IPCDQ_M0_set = Parameters.Control.IPCDQ.M0_set;
PitchControlParams.IPCDQ_k_sat  = Parameters.Control.IPCDQ.k_sat;


%% CPC Filters
% Slow LPF for LPV parameter MANUEL QUESTION should this differ ie
% omega_sloe * 3 for 3P filter? => No, just for high-freq
% MANUEL QUESTION should phase in IPC controller differ for 3P load
% mitigation? => 3*Phase for 3P
% Should IPCDQ_LPF differ for 3P?
omega_slow = 0.05;
omega_slow2 = (2*pi*Parameters.Turbine.wr_rated/60) * 6;
zeta_slow = 0.7;
LPF_slow = tf([omega_slow^2],[1,2*zeta_slow*omega_slow,omega_slow^2]);
dLPF_slow = c2d(ss(LPF_slow),Control.DT);
LPF_slow2 = tf([omega_slow2^2],[1,2*zeta_slow*omega_slow2,omega_slow2^2]);
dLPF_slow2 = c2d(ss(LPF_slow2),Control.DT);
% bodemag(LPF_slow2)
% hold on;
% plot([2*pi*Parameters.Turbine.wr_rated/60, 2*pi*Parameters.Turbine.wr_rated/60], [-100, 20])
% hold off;
% 2*pi*Parameters.Turbine.wr_rated/60

% MISC CYCLIC PITCH CONTROL DEPENDENCIES
LSM0_LPF    = Af_LPF(2*pi,.707,Control.DT);
PitchControlParams.theta_fine   = Parameters.Turbine.fine_pitch;
% ============================================================= %



%% DTU Peak Shaver
% Anemometer LPF
% Increase minimum pitch angle using low pass filtered wind speed
Parameters.Control.DTU_PS.Enable    = 0;

Parameters.Control.DTU_PS.TB_LPF = Af_LPF(2*pi*0.25,.707,Control.DT);
[dLPF_windspeed, ~] = Af_LPF(2*pi*0.01,.707,Control.DT);
dLPF_windspeed = ss(dLPF_windspeed);
if false
    bodemag(dLPF_windspeed)
    end

% init
iWS = 1; Parameters.Control.DTU_PS.UU = 0;      Parameters.Control.DTU_PS.PP = 0;  %init
x_arr = 9.5:0.25:10.75;
y_arr = 2.90912.*x_arr - 26.273;
% BreakPoints
Parameters.Control.DTU_PS.UU(iWS) = 0;      Parameters.Control.DTU_PS.PP(iWS) = Parameters.Turbine.fine_pitch;  iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = 6;      Parameters.Control.DTU_PS.PP(iWS) = Parameters.Turbine.fine_pitch;  iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = 7;      Parameters.Control.DTU_PS.PP(iWS) = Parameters.Turbine.fine_pitch;  iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = 8;      Parameters.Control.DTU_PS.PP(iWS) = Parameters.Turbine.fine_pitch;  iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = 9;      Parameters.Control.DTU_PS.PP(iWS) = Parameters.Turbine.fine_pitch;  iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = x_arr(1);     Parameters.Control.DTU_PS.PP(iWS) = y_arr(1);  iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = x_arr(2);     Parameters.Control.DTU_PS.PP(iWS) = y_arr(2);  iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = x_arr(3);     Parameters.Control.DTU_PS.PP(iWS) = y_arr(3);  iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = x_arr(4);     Parameters.Control.DTU_PS.PP(iWS) = y_arr(4);  iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = x_arr(5);     Parameters.Control.DTU_PS.PP(iWS) = y_arr(5);  iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = x_arr(6);     Parameters.Control.DTU_PS.PP(iWS) = y_arr(6);  iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = 11;     Parameters.Control.DTU_PS.PP(iWS) = y_arr(6);  iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = 12;     Parameters.Control.DTU_PS.PP(iWS) = y_arr(6);  iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = 13;     Parameters.Control.DTU_PS.PP(iWS) = y_arr(6); iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = 14;     Parameters.Control.DTU_PS.PP(iWS) = y_arr(6);   iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = 16;     Parameters.Control.DTU_PS.PP(iWS) = y_arr(6);   iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = 18;     Parameters.Control.DTU_PS.PP(iWS) = y_arr(6);   iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = 20;     Parameters.Control.DTU_PS.PP(iWS) = y_arr(6);   iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = 22;     Parameters.Control.DTU_PS.PP(iWS) = y_arr(6);   iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = 24;     Parameters.Control.DTU_PS.PP(iWS) = y_arr(6);   iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = 30;     Parameters.Control.DTU_PS.PP(iWS) = y_arr(6);   iWS=iWS+1;
Parameters.Control.DTU_PS.UU(iWS) = 50;     Parameters.Control.DTU_PS.PP(iWS) = y_arr(6);   iWS=iWS+1;

Parameters.Control.DTU_PS.uu = linspace(0,50,1000);
Parameters.Control.DTU_PS.pp = interp1(Parameters.Control.DTU_PS.UU,...
    Parameters.Control.DTU_PS.PP, Parameters.Control.DTU_PS.uu,'cubic');

% ============================================================= %


%% Yaw 
OLControlParams.YawError = 360;
OLControlParams.Disable = 1;
% ============================================================= %



%% ESTIMATION %%

%% EKF WIND SPEED ESTIMATOR
% Parameters
Parameters.Control.WSE.enable       = 1;

if Parameters.Control.WSE.enable
    Parameters.Control.WSE.time_enable = 30;
else
    Parameters.Control.WSE.time_enable = 1e10;
end

% Initial state and covariance
if ~exist('Lidar','var')
    Lidar.MWS = 10;
end
% Initial condition

% ROSCO
Parameters.Control.WSE.v0  = 12;
Parameters.Control.WSE.w0  = Parameters.Turbine.IC.Wr * Parameters.Turbine.G;
Cx                         = Pre_LoadRotPerf(CpCtCqFile);
% SOAR
Parameters.Control.WSE.x_0          = [rpm2radps(Parameters.Turbine.IC.Wr),Lidar.MWS,0]'; %inital state
Parameters.Control.WSE.P_0          = diag([0.1,0.1,1])^2; %inital cov.

% Turbulence Parameters
Parameters.Control.WSE.L            = 320; %turb. length scale (m)

% Turbine Parameters
Parameters.Control.WSE.A            = pi * (cosd(Parameters.Turbine.ConeAngle) * Parameters.Turbine.R)^2;       %rotor area m^2 (SHAFT TILT IGNORED)
Parameters.Control.WSE.rho          = Parameters.Turbine.rho;
Parameters.Control.WSE.Rrot         = Parameters.Turbine.R;
Parameters.Control.WSE.ConeAngle    = Parameters.Turbine.ConeAngle; % deg
Parameters.Control.WSE.G            = Parameters.Turbine.G;
Parameters.Control.WSE.J_tot        = Parameters.Turbine.Jtot;

% covariance matrices - constant for now
Parameters.Control.WSE.Q_0 = diag([8e-7, Control.DT * 2^2 /600, ...
    10 * Control.DT * pi * Parameters.Control.WSE.x_0(2)^3 * .24^2 / Parameters.Control.WSE.L]);

Parameters.Control.WSE.R = 0.02;


Parameters.Control.WSE.H = [Parameters.Turbine.G, 0 , 0];

% Cp Parameterization TODO where is this .mat file
if exist('Cp_Param.mat','file')
    load('Cp_Param')
else
    disp(['CONTROL Warning: ',Parameters.Turbine.String,' Cp Params. not found: EKF may give unexpected results']);
    disp('Loading default Cp Params., Disabling WSE');
    load(fullfile(project_dir,'SUMR-13_v1_c5_a0333_v113_P169_B3','Cp_Param'),'a_hat_opt','a_hat')
    Parameters.Control.WSE.enable       = 0;
end

Parameters.Control.WSE.a_Cp = a_hat;

Parameters.Control.WSE.useTurbModel = 1;
Parameters.Control.WSE.dt           = Control.DT;

% Assign to single struct
WSE_Parameters  = Parameters.Control.WSE;

% EKF_Data        = Simulink.Bus;
Parameters.Control.EKF_GenSpeed_tau          = 1;
Parameters.Control.EKF_GenSpeed_LPF          = Af_LPF(2*pi/Parameters.Control.EKF_GenSpeed_tau,...
    .707,Control.DT);

% Generic Inputs for Soft Start
Parameters.Control.WSE.GenericInp = ...
                      [Parameters.Control.PowerRating * rpm2radps(Parameters.Turbine.wg_rated);...
                       Parameters.Turbine.T_rated; 10];
% ============================================================= %