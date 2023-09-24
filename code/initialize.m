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

VARY_REFERENCE = 0;
VARY_REFERENCE_BASIS = [0.05, 0.1, 0.15, 0.2, 0.25]; % QUESTION MANUEL of peak, ss value?

VARY_SATURATION = 0;
VARY_SATURATION_BASIS = deg2rad(15) * [0.05, 0.1, 0.15, 0.2, 0.25]; % QUESTION MANUEL of peak, ss value?

COMPUTE_FFT = 0;
HARMONICS = [2, 3, 6];

USE_IPC = 1;

STRUCT_PARAM_SWEEP = 0; % conduct nonlinear simulations for parameter sweep over MIMO PI Gains
OPTIMAL_K_COLLECTION = 0; % conduct nonlinear simulations for collection of Hinf-synthesized controllers
EXTREME_K_COLLECTION = 1;
BASELINE_K = 0; % conduct nonlinear simulations for baseline structured controller

if STRUCT_PARAM_SWEEP || OPTIMAL_K_COLLECTION || EXTREME_K_COLLECTION || BASELINE_K
    USE_IPC = 1;
end

if ~(STRUCT_PARAM_SWEEP || OPTIMAL_K_COLLECTION || EXTREME_K_COLLECTION || BASELINE_K || ~USE_IPC)
    sprintf('Warning: choose only one of STRUCT_PARAM_SWEEP, OPTIMAL_K_COLLECTION, EXTREME_K_COLLECTION, BASELINE_K');
end

% model_names = {'excGenDOF_incSecOrd', 'excGenDOF_excSecOrd'};

username = char(java.lang.System.getProperty('user.name'));
% char(java.net.InetAddress.getLocalHost.getHostName)
if strcmp(username, 'aoifework')
    MAX_SIMULATIONS = 10;
    home_dir = '/Users/aoifework/Documents';
    toolbox_dir = fullfile(home_dir, 'toolboxes');
    project_dir = fullfile(home_dir, 'Research', 'ipc_tuning');
    simulink_model_dir = fullfile(project_dir, 'simulink_models');
    fig_dir = fullfile(project_dir, 'paper', 'figs');
    
    addpath(simulink_model_dir);
    addpath(fullfile(toolbox_dir, 'turbsim-toolbox'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox/A_Functions'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox/Utilities'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox', 'MBC', 'Source'));
    addpath(fullfile(toolbox_dir, 'turbsim-toolbox/A_Functions/'));
    addpath(fullfile(toolbox_dir, 'PMtools/'));

    code_dir = fullfile(project_dir, 'code');
    chdir(code_dir);

    addpath(fullfile(code_dir, 'helper_functions/'));

    libext = '.dylib';
    
    % fast_install_dir = fullfile(home_dir, 'dev/WEIS/OpenFAST/install');
    fast_install_dir = fullfile(toolbox_dir, 'openfast/install/lib');
    FAST_runDirectory = fullfile(project_dir, 'simulations');

    windfiles_dir = fullfile(project_dir, 'WindFiles');
    
    addpath(fullfile(project_dir)); % sl model
    
    % FAST_SFunc location
    addpath(fast_install_dir);

    mat_save_dir = fullfile(code_dir, 'matfiles');
    sl_save_dir = fullfile(project_dir, 'sl_outputs');
    sl_metadata_save_dir = fullfile(project_dir, 'sl_metadata');
    postprocessing_save_dir = fullfile(project_dir, 'postprocessing_results');

elseif strcmp(username, 'aohe7145')
    MAX_SIMULATIONS = -1;
    % scp -r /Users/aoifework/Documents/Research/ipc_tuning aohe7145@login.rc.colorado.edu:/projects/aohe7145/projects/ipc_tuning
    % scp -r /Users/aoifework/Documents/toolboxes aohe7145@login.rc.colorado.edu:/projects/aohe7145/toolboxes
    % scp -r /Users/aoifework/Documents/usflowt_src/ aohe7145@login.rc.colorado.edu:/projects/aohe7145/usflowt_src/
    home_dir = '/projects/aohe7145/';
    project_dir = fullfile(home_dir, 'projects', 'ipc_tuning');
    fast_install_dir = fullfile(home_dir, 'toolboxes/openfast_dev/glue-codes/simulink/src');
    simulink_model_dir = fullfile(project_dir, 'simulink_models');
    fig_dir = fullfile(project_dir, 'paper', 'figs');
    toolbox_dir = fullfile(home_dir, 'toolboxes');
    code_dir = fullfile(project_dir, 'code');
    mat_save_dir = fullfile('/scratch/alpine/aohe7145/ipc_tuning', 'matfiles');
    sl_save_dir = fullfile('/scratch/alpine/aohe7145/ipc_tuning', 'sl_outputs');
    sl_metadata_save_dir = fullfile('/scratch/alpine/aohe7145/ipc_tuning', 'sl_metadata');
    postprocessing_save_dir = fullfile('/scratch/alpine/aohe7145/ipc_tuning', 'postprocessing_results');
    % fast_install_dir = fullfile(home_dir, 'dev/WEIS/OpenFAST/install');
    
    FAST_runDirectory = fullfile(project_dir, 'simulations');

    addpath(simulink_model_dir);
    addpath(fullfile(toolbox_dir, 'turbsim-toolbox'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox/A_Functions'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox/Utilities'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox', 'MBC', 'Source'));
    addpath(fullfile(toolbox_dir, 'turbsim-toolbox/A_Functions/'));
    addpath(fullfile(toolbox_dir, 'PMtools/'));
    addpath(fullfile(code_dir, 'helper_functions/'));

    chdir(code_dir);

    libext = '.dylib';

    windfiles_dir = fullfile('/scratch/alpine/aohe7145/ipc_tuning', 'WindFiles');
    
    addpath(fullfile(project_dir)); % sl model
    
    % FAST_SFunc location
    addpath(fast_install_dir);

end

if ~exist(mat_save_dir, 'dir')
    mkdir(mat_save_dir);
end
if ~exist(sl_save_dir, 'dir')
    mkdir(sl_save_dir);
end
if ~exist(sl_metadata_save_dir, 'dir')
    mkdir(sl_metadata_save_dir);
end
if ~exist(postprocessing_save_dir, 'dir')
    mkdir(postprocessing_save_dir);
end
if ~exist(FAST_runDirectory, 'dir')
    mkdir(FAST_runDirectory);
end


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

FAST_SimulinkModel_dir = simulink_model_dir;
FAST_SimulinkModel = 'AD_SOAR_c7_V2f_c73_Clean';

fastRunner.FAST_exe = fullfile(fast_install_dir, 'bin/openfast');
fastRunner.FAST_lib = fullfile(fast_install_dir, ['lib/libopenfastlib', libext]);
fastRunner.FAST_InputFile = 'weis_job_00';
fastRunner.run_name = 'excGenDOF_incSecOrd';

autrun;

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
addpath(FAST_directory);
C_BL_SOAR25_V2f_c73_Clean;

model_names = {'excGenDOF_incSecOrd', 'excGenDOF_excSecOrd'};
run_name = model_names{1};
lin_models_dir = fullfile(FAST_directory, 'linearization', 'steady_wind-CL', 'linfiles', run_name);

omega_1P_rad = Parameters.Turbine.wr_rated * 2*pi/60;
omega_2P_rad = Parameters.Turbine.wr_rated * (2*pi/60) * 2;
omega_3P_rad = Parameters.Turbine.wr_rated * (2*pi/60) * 3;

%% OutLists
if false || ~exist(fullfile(project_dir, 'OutList.mat'))
	% OutList = manualOutList([fullfile(FAST_runDirectory, ...
    %         ['AD_SOAR_c7_V2f_c73_Clean', '_1']), '.SFunc.sum']);
    OutList = manualOutList(fullfile(FAST_directory, 'weis_job_00_cmdline.sum'));
	save(fullfile(project_dir, 'OutList.mat'), 'OutList');
else
	load(fullfile(project_dir, 'OutList.mat'));
end

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

sigma_plot_opt = sigmaoptions;
sigma_plot_opt.Title.FontSize = 25;
sigma_plot_opt.Title.String = '';
sigma_plot_opt.InputLabels.FontSize = 25;
sigma_plot_opt.OutputLabels.FontSize = 25;
sigma_plot_opt.XLabel.FontSize = 25;
sigma_plot_opt.YLabel.FontSize = 25;
sigma_plot_opt.TickLabel.FontSize = 22;
sigma_plot_opt.Grid = 'on';

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
