%% Initialize workspace and add paths
close all;
clc;
clear all;

TMax = 600;
n_seeds = 100;
WIND_TYPE = 'turbsim';
WIND_SPEEDS = 0.5:0.5:22;
LPV_CONTROLLER_WIND_SPEEDS = 12:2:22;
NONLPV_CONTROLLER_WIND_SPEED = 16;
SIMS_INDICES = 1:9;
RUN_CL = 1;
RUN_OL_DQ = 0;
RUN_OL_BLADE = 0;

COMPUTE_FFT = 0;
HARMONICS = 1:6;

USE_IPC = 1;

STRUCT_PARAM_SWEEP = 0; % conduct nonlinear simulations for parameter sweep over MIMO PI Gains
OPTIMAL_K_COLLECTION = 0; % conduct nonlinear simulations for collection of Hinf-synthesized controllers
EXTREME_K_COLLECTION = 1;
BASELINE_K = 0; % conduct nonlinear simulations for baseline structured controller

if STRUCT_PARAM_SWEEP || OPTIMAL_K_COLLECTION || EXTREME_K_COLLECTION || BASELINE_K
    USE_IPC = 1;
end

if STRUCT_PARAM_SWEEP + OPTIMAL_K_COLLECTION + EXTREME_K_COLLECTION + BASELINE_K ~= 1
    print('Warning: choose only one of STRUCT_PARAM_SWEEP, OPTIMAL_K_COLLECTION, EXTREME_K_COLLECTION, BASELINE_K');
end

% model_names = {'excGenDOF_incSecOrd', 'excGenDOF_excSecOrd'};

username = char(java.lang.System.getProperty('user.name'));
% char(java.net.InetAddress.getLocalHost.getHostName)
if strcmp(username, 'aoifework')
    home_dir = '/Users/aoifework/Documents';
    toolbox_dir = fullfile(home_dir, 'toolboxes');
    project_dir = fullfile(home_dir, 'Research', 'ipc_tuning');
    plant_setup_dir = fullfile(project_dir, 'plant_setup_package');
    simulink_model_dir = fullfile(project_dir, 'simulink_models');
    fig_dir = fullfile(project_dir, 'paper', 'figs');
    
    addpath(plant_setup_dir);
    addpath(simulink_model_dir);
    addpath(fullfile(toolbox_dir, 'turbsim-toolbox'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox/A_Functions'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox/Utilities'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox', 'MBC', 'Source'));
    addpath(fullfile(toolbox_dir, 'turbsim-toolbox/A_Functions/'));
    addpath(fullfile(toolbox_dir, 'PMtools/'));


    % chdir(fullfile(home_dir, 'MATLAB/LPVToolsV1.0'));
    % addlpv;
%     addpath(fullfile(home_dir, 'MATLAB/LPVToolsV1.0'));

    code_dir = fullfile(project_dir, 'code');
    chdir(code_dir);

    addpath(fullfile(code_dir, 'helper_functions/'));

    libext = '.dylib';
    
    % fast_install_dir = fullfile(home_dir, 'dev/WEIS/OpenFAST/install');
    fast_install_dir = fullfile(home_dir, 'usflowt_src/openfast/sl_install');
    FAST_runDirectory = fullfile(project_dir, 'simulations');

    windfiles_dir = fullfile(project_dir, 'WindFiles');
    
    addpath(fullfile(project_dir)); % sl model
    
    % FAST_SFunc location
    addpath(fullfile(fast_install_dir, 'lib'));

    save_dir = fullfile(code_dir, 'matfiles');

elseif strmp(username, 'aohe7145')
    % scp -r /Users/aoifework/Documents/Research/ipc_tuning aohe7145@login.rc.colorado.edu:/projects/aohe7145/projects/ipc_tuning
    % scp -r /Users/aoifework/Documents/toolboxes aohe7145@login.rc.colorado.edu:/projects/aohe7145/toolboxes
    % scp -r /Users/aoifework/Documents/usflowt_src/ aohe7145@login.rc.colorado.edu:/projects/aohe7145/usflowt_src/
    home_dir = '/projects/aohe7145/';
    project_dir = fullfile(home_dir, 'projects', 'ipc_tuning');
    fast_install_dir = fullfile(home_dir, 'usflowt_src/openfast/sl_install');
    simulink_model_dir = fullfile(project_dir, 'simulink_models');
    fig_dir = fullfile(project_dir, 'paper', 'figs');
    toolbox_dir = fullfile(home_dir, 'toolboxes');
    code_dir = fullfile(project_dir, 'code');
    save_dir = fullfile('/scratch/alpine/aohe7145/ipc_tuning', 'matfiles');
    % fast_install_dir = fullfile(home_dir, 'dev/WEIS/OpenFAST/install');
    
    FAST_runDirectory = fullfile(project_dir, 'simulations');

    addpath(plant_setup_dir);
    addpath(simulink_model_dir);
    addpath(fullfile(toolbox_dir, 'turbsim-toolbox'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox/A_Functions'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox/Utilities'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox', 'MBC', 'Source'));
    addpath(fullfile(toolbox_dir, 'turbsim-toolbox/A_Functions/'));
    addpath(fullfile(toolbox_dir, 'PMtools/'));

    chdir(code_dir);

    libext = '.dylib';

    windfiles_dir = fullfile(project_dir, 'WindFiles');
    
    addpath(fullfile(project_dir)); % sl model
    
    % FAST_SFunc location
    addpath(fullfile(fast_install_dir, 'lib'));


elseif strmp(username, 'manuel')
    
    code_dir = fullfile(project_dir, 'plant_setup_package');
    chdir(code_dir);

    addpath(fullfile('CHANGE ME', 'PMtools')); % MANUEL: add your filepaths here
    addpath(fullfile('CHANGE ME', 'matlab-toolbox/A_Functions'));
    addpath(fullfile('CHANGE ME', 'matlab-toolbox/Utilities'));
    addpath(fullfile('CHANGE ME', 'matlab-toolbox', 'MBC', 'Source')); % Need fx_mbc3 from this toolbox
    lin_models_dir = fullfile('CHANGE ME', 'linfiles');

    save_dir = fullfile(code_dir, 'matfiles');
end

FAST_SimulinkModel_dir = simulink_model_dir;
if RUN_OL_DQ
    FAST_SimulinkModel = 'AD_SOAR_c7_V2f_c73_Clean_OL_DQ';
elseif RUN_OL_BLADE
    FAST_SimulinkModel = 'AD_SOAR_c7_V2f_c73_Clean_OL_BLADE';
else
    FAST_SimulinkModel = 'AD_SOAR_c7_V2f_c73_ControllerTest';
end

fastRunner.FAST_exe = fullfile(fast_install_dir, 'bin/openfast');
fastRunner.FAST_lib = fullfile(fast_install_dir, ['lib/libopenfastlib', libext]);
fastRunner.FAST_InputFile = 'weis_job_00';
fastRunner.run_name = 'excGenDOF_incSecOrd';

if ~exist(FAST_runDirectory, 'dir')
    mkdir(FAST_runDirectory);
end

autrun;



if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end
if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end

%% Setup simulation
Simulation.OpenFAST         = 1;
Simulation.OpenFAST_Date    = '042821';
Simulation.AeroDynVer       = '15';
Simulation.DT = 0.0125;
DT = Simulation.DT;
cut_transients              = 100;

Name_Model                  = 'AD_SOAR_c7_V2f_c73_ControllerTest';
% Name_Control                = 'C_BL_SOAR25_V2f_c73_Clean';
Parameters.Turbine.String   = 'SOAR-25-V2f';
Parameters.Turbine.fine_pitch = 0.303;  % deg

Parameters.Turbine.ConeAngle = 3.6;%2.5;% deg
Parameters.Turbine.ShaftTilt = 8.4;%7.0; % deg
Parameters.Tower.Height  = 193.287;  % meters


FAST_directory = fullfile(project_dir, [Parameters.Turbine.String '_IF']);
FAST_InputFileName = fullfile(FAST_directory, [fastRunner.FAST_InputFile '.fst']);
CpCtCqFile = fullfile(FAST_directory, 'weis_job_00_Cp_Ct_Cq.txt');
C_BL_SOAR25_V2f_c73_Clean;

model_names = {'excGenDOF_incSecOrd', 'excGenDOF_excSecOrd'};
run_name = model_names{1};
lin_models_dir = fullfile(FAST_directory, 'linearization', 'steady_wind-CL', 'linfiles', run_name);

omega_1P_rad = Parameters.Turbine.wr_rated * 2*pi/60;
omega_2P_rad = Parameters.Turbine.wr_rated * (2*pi/60) * 2;
omega_3P_rad = Parameters.Turbine.wr_rated * (2*pi/60) * 3;

%% OutLists
OutList = manualOutList([fullfile(FAST_runDirectory, ...
            ['AD_SOAR_c7_V2f_c73_Clean', '_1']), '.SFunc.sum']);
save(fullfile(project_dir, 'OutList.mat'), 'OutList');

dqOutList = OutList;
for op_label = OutList'
    % if rotating quantity
    if strcmp(op_label{1}(end), '1')
        % get all corresponding quantities
        blade_op_labels = cellfun(@(b) [op_label{1}(1:end-1) b], {'1', '2', '3'}, 'UniformOutput', false);
        cdq_op_labels = cellfun(@(b) [op_label{1}(1:end-1) b], {'C', 'D', 'Q'}, 'UniformOutput', false);
        
        % replace in transformed signal matrix
        dqOutList(ismember(dqOutList, blade_op_labels)) = cdq_op_labels;
    end
end
save(fullfile(project_dir, 'dqOutList.mat'), 'dqOutList');

bode_plot_opt = bodeoptions;
bode_plot_opt.Title.FontSize = 25;
bode_plot_opt.Title.String = '';
bode_plot_opt.InputLabels.FontSize = 25;
bode_plot_opt.OutputLabels.FontSize = 25;
bode_plot_opt.XLabel.FontSize = 25;
bode_plot_opt.YLabel.FontSize = 25;
bode_plot_opt.TickLabel.FontSize = 22;
bode_plot_opt.PhaseMatching = 'on';
bode_plot_opt.Grid = 'on';
bode_plot_opt.PhaseMatchingFreq = 0;
bode_plot_opt.PhaseMatchingValue = 0;
bode_plot_opt.PhaseVisible = 'off';
%bode_plot_opt.XLim = [];
% bode_plot_opt.XLimMode = 'manual';

time_plot_opt = timeoptions;
time_plot_opt.Title.FontSize = 25;
time_plot_opt.Title.String = '';
time_plot_opt.InputLabels.FontSize = 25;
time_plot_opt.OutputLabels.FontSize = 25;
time_plot_opt.XLabel.FontSize = 25;
time_plot_opt.YLabel.FontSize = 25;
time_plot_opt.TickLabel.FontSize = 22;
time_plot_opt.Grid = 'on';
% plot_opt.Xlim = [0, 600];

pz_plot_opt = pzoptions;
% pz_plot_opt.Grid = 'on';
pz_plot_opt.Title.FontSize = 25;
pz_plot_opt.Title.String = '';
pz_plot_opt.InputLabels.FontSize = 25;
pz_plot_opt.OutputLabels.FontSize = 25;
pz_plot_opt.XLabel.FontSize = 25;
pz_plot_opt.YLabel.FontSize = 25;
pz_plot_opt.TickLabel.FontSize = 22;