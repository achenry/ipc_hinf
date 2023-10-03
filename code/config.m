clear all;
close all;
clc;

username = char(java.lang.System.getProperty('user.name'));
if strcmp(username, 'aoifework')
    n_seeds = 1;
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

VARY_WU = 1;
VARY_WU_BASIS = 100 * [0.01, 0.05, 0.1, 0.5, 0.75, 1.0];

VARY_REFERENCE = 0;
VARY_REFERENCE_BASIS = [0.05, 0.1, 0.15, 0.2, 0.25];

VARY_SATURATION = 0;
VARY_SATURATION_BASIS = deg2rad(15) * [0.05, 0.1, 0.15, 0.2, 0.25];

COMPUTE_FFT = 0;
HARMONICS = [2, 3, 6];

RUN_SIMS_PAR = 1;
RUN_SIMS_SINGLE = 0;
RUN_TURSIM = 0;
GENERATE_CASES = 1;
GENERATE_FASTINPUT_FILES = 1;

USE_IPC = 1;

STRUCT_PARAM_SWEEP = 0; % conduct nonlinear simulations for parameter sweep over MIMO PI Gains
OPTIMAL_K_COLLECTION = 0; % conduct nonlinear simulations for collection of Hinf-synthesized controllers
EXTREME_K_COLLECTION = 1;
BASELINE_K = 0; % conduct nonlinear simulations for baseline structured controller