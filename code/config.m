clear all;
close all;
clc;

username = char(java.lang.System.getProperty('user.name'));
if strcmp(username, 'aoifework')
    n_seeds = 5;
elseif strcmp(username, 'aohe7145')
    n_seeds = 5;
end

TMax = 600;
WIND_TYPE = 'turbsim';
WIND_SPEEDS = 0.5:0.5:22;
LPV_CONTROLLER_WIND_SPEEDS = 12:2:22;
NONLPV_CONTROLLER_WIND_SPEED = 16;
RUN_CL = 1;
RUN_OL_DQ = 0;
RUN_OL_BLADE = 0;

uxs = LPV_CONTROLLER_WIND_SPEEDS;


SCHEDULING = 0;

VARY_WU = 1;
VARY_WU_BASIS = [1, 2.5, 5, 7.5, 10];

VARY_REFERENCE = 0;
VARY_REFERENCE_BASIS = [0.025, 0.05, 0.1, 0.2, 0.4]; % 0.01 is only one which results in reduced DEL, set reference for Mdq to be % of minimum value found in optimal simulations

VARY_SATURATION = 0;
VARY_SATURATION_BASIS = [0.2, 0.4, 0.8, 0.9, 0.95]; % TODO farther apart set saturation for Betadq to be % of maximum value found in optimal simulations
COMPUTE_FFT = 0;
HARMONICS = [2, 3, 6];

RUN_SIMS_PAR = 1;
RUN_SIMS_SINGLE = 1;
RUN_TURBSIM = 0;
GENERATE_CASES = 1;
GENERATE_FASTINPUT_FILES = 1;

USE_IPC = 1;

DEBUG = 0;
STRUCT_PARAM_SWEEP = 0; % conduct nonlinear simulations for parameter sweep over MIMO PI Gains
OPTIMAL_K_COLLECTION = 1; % conduct nonlinear simulations for collection of Hinf-synthesized controllers
EXTREME_K_COLLECTION = 0;
BASELINE_K = 0; % conduct nonlinear simulations for baseline structured controller AND no ipc casd

% if OPTIMAL_K_COLLECTION
%     if VARY_WU + VARY_SATURATION + VARY_REFERENCE + SCHEDULING ~= 1
%         error('One of VARY_WU, VARY_SATURATION or VARY_REFERENCE or SCHEDULING must be true if OPTIMAL_K_COLLECTION is true.')
%     end
% end