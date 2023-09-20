%% Run nonlinear simulations for different controllers in Simulink
%   compute power spectra of different loads
%   compute ADC of blade-pitch

% compute power spectra of different rotating and non-rotating loads for
% different controllers
% compute ADC for different controllers
% plot Beta, BetaDot, BetaDDdot for different controllers

% run a parfor loop to simulate 10 minutes of a turbsim wind field for each
% controller type with IPC on
clear all;
initialize;

RUN_SIMS_PAR = 0;
RUN_SIMS_SINGLE = 1;
COMPUTE_FFT = 0;
RUN_TURSIM = 0;

RootMyc_ref = [0, 0];

%% Generate Turbsim Files
if RUN_TURSIM
    run_turbsim_multi;
end

%% Generate simulation cases with 100 seeds of turbulent wind field
if RUN_OL_DQ
    % generate disturbances on Blade-Pitch D/Q 
    DistAmp = deg2rad(15);
    RampRate = DistAmp; %1e-3;
    SettleTime = 100;

    BaselineSteadyState = SettleTime:(2 * SettleTime);

    dRampStart = 2 * SettleTime + 1; % leave 100 seconds to settle, then another 100 to get mean and max baseline
    dRampStop = dRampStart + (DistAmp / RampRate); % constant by 110
    dSteadyState = dRampStop + SettleTime:dRampStop + (2 * SettleTime);
    
    qRampStart = dRampStop + (2 * SettleTime) + 1; % leave 100 seconds to settle, then another 100 to get mean and max
    qRampStop = qRampStart + (DistAmp / RampRate); % constant by 220
    qSteadyState = qRampStop + SettleTime:qRampStop + (2 * SettleTime);
    
    TMax = qSteadyState(end); % leave 100 seconds to settle, then another 100 to get mean and max
elseif RUN_OL_BLADE
    % generate disturbances on Blade-Pitch D/Q 
    DistAmp = deg2rad(15); %1e-3; % rad
    RampRate = DistAmp; %1e-3;
    SettleTime = 100;

    BaselineSteadyState = SettleTime:(2 * SettleTime);

    b1RampStart = 2 * SettleTime + 1; % leave 100 seconds to settle, then another 100 to get mean and max baseline
    b1RampStop = b1RampStart + (DistAmp / RampRate); % constant by 110
    b1SteadyState = b1RampStop + SettleTime:b1RampStop + (2 * SettleTime);
    
    TMax = b1SteadyState(end); % leave 100 seconds to settle, then another 100 to get mean and max
end

Parameters.Control.IPCDQ.Enable = USE_IPC;

%% Generate Simulation Cases

% Generate Turbsim Cases

% ux_mean = 11.4;
class = 'A'; % class A
n_seeds = 1;
uxs = [16];

if sum([RUN_OL_DQ, RUN_OL_BLADE, RUN_CL]) == 1
    CaseGen.dir_matrix = FAST_runDirectory;
    CaseGen.namebase = FAST_SimulinkModel;
    [~, CaseGen.model_name, ~] = fileparts(fastRunner.FAST_InputFile);
    clear case_basis;
    case_basis.InflowWind.HWindSpeed = [];
    if strcmp(WIND_TYPE, 'turbsim')
        case_basis.InflowWind.WindType = {'3'};
        case_basis.InflowWind.FileName_BTS = {};
        for ux = uxs
            for bts_idx = 1:n_seeds
                case_basis.InflowWind.FileName_BTS{bts_idx} = ['"' fullfile(windfiles_dir, ...
                    [class, '_', replace(num2str(ux), '.', '-'), '_', num2str(bts_idx), '.bts']) '"'];
                case_basis.InflowWind.HWindSpeed = [case_basis.InflowWind.HWindSpeed, ux];
          
            end
        end
        
    %     case_basis.InflowWind.HWindSpeed = num2str(ux_mean);
    elseif strcmp(WIND_TYPE, 'steady')
        case_basis.InflowWind.WindType = {'1'};
        case_basis.InflowWind.HWindSpeed = split(num2str(WIND_SPEEDS));
    end

    case_basis.Fst.TMax = {num2str(TMax)};

    [Wind_case_list, Wind_case_name_list, Wind_n_cases] = generateCases(case_basis, CaseGen.namebase, true);
 
end

if STRUCT_PARAM_SWEEP || BASELINE_K
    % Load MIMO PI Parameter Sweep Gains and Controllers (in tunable Block form)
    if STRUCT_PARAM_SWEEP
        load(fullfile(mat_save_dir, 'PI_ParameterSweep_case_list.mat'));
    elseif BASELINE_K
        load(fullfile(mat_save_dir, 'PI_BaselineParameters_case_list.mat'));
    end

    % Merge Wind and Controller Cases
    case_idx = 1;
    for c_idx = 1:length(PI_ParameterSweep_case_list)
        for w_idx = 1:Wind_n_cases
            for fn1 = fieldnames(Wind_case_list(w_idx))'
                for fn2 = fieldnames(Wind_case_list(w_idx).(fn1{1}))'
                    case_list(case_idx).(fn1{1}).(fn2{1}) = Wind_case_list(w_idx).(fn1{1}).(fn2{1});
                end
            end
            for fn1 = fieldnames(PI_ParameterSweep_case_list(c_idx))'
                case_list(case_idx).(fn1{1}) = PI_ParameterSweep_case_list(c_idx).(fn1{1});
            end
            case_idx = case_idx + 1;
        end
    end
    n_cases = case_idx - 1;
    case_name_list = arrayfun(@(n) ['case_', num2str(n)], 1:n_cases, 'UniformOutput', false);
elseif OPTIMAL_K_COLLECTION || EXTREME_K_COLLECTION
    if OPTIMAL_K_COLLECTION
        % Load Controller Cases corresponding to scheduled full-order controllers
        load(fullfile(mat_save_dir, 'Optimal_Controllers_case_list.mat'));
    elseif EXTREME_K_COLLECTION
        load(fullfile(mat_save_dir, 'Extreme_Controllers_case_list.mat'));
    end
    % Merge Wind and Controller Cases
    case_idx = 1;
    for c_idx = 1:length(Controllers_case_list)
        for w_idx = 1:Wind_n_cases
            for fn1 = fieldnames(Wind_case_list(w_idx))'
                for fn2 = fieldnames(Wind_case_list(w_idx).(fn1{1}))'
                    case_list(case_idx).(fn1{1}).(fn2{1}) = Wind_case_list(w_idx).(fn1{1}).(fn2{1});
                end
            end
            for fn1 = fieldnames(Controllers_case_list(c_idx))'
                case_list(case_idx).(fn1{1}) = Controllers_case_list(c_idx).(fn1{1});
            end
            case_idx = case_idx + 1;
        end
    end
    n_cases = case_idx - 1;
    case_name_list = arrayfun(@(n) ['case_', num2str(n)], 1:n_cases, 'UniformOutput', false);
    if OPTIMAL_K_COLLECTION
        save(fullfile(mat_save_dir, 'Optimal_Controllers_nonlinear_simulation_case_list.mat'), "case_list", '-v7.3');
    elseif EXTREME_K_COLLECTION
        save(fullfile(mat_save_dir, 'Extreme_Controllers_nonlinear_simulation_case_list.mat'), "case_list", '-v7.3');
    elseif BASELINE_K
        save(fullfile(mat_save_dir, 'Baseline_Controller_nonlinear_simulation_case_list.mat'), "case_list", '-v7.3');
    end
elseif ~USE_IPC % no ipc
    case_list = Wind_case_list;
    n_cases = Wind_n_cases;
    case_name_list = Wind_case_name_list;
    save(fullfile(mat_save_dir, 'noIPC_nonlinear_simulation_case_list.mat'), "case_list", '-v7.3');

end

%% Load files 

% OutList for Blade & DQ coordinates
% load(fullfile(project_dir, 'OutList.mat'));
% load(fullfile(project_dir, 'dqOutList.mat'));

%% Generate OpenFAST input files for each case

input_mode = 2;
def_fst_file = fullfile(FAST_directory, fastRunner.FAST_InputFile);
def_infw_file = fullfile(FAST_directory, [fastRunner.FAST_InputFile, '_InflowFile']);
templates_dir = fullfile(FAST_directory, 'templates');
template_fst_dir = fullfile(templates_dir, 'Fst');
template_infw_dir = fullfile(templates_dir, 'InflowWind');

copyfile(fullfile(FAST_directory, 'Airfoils'), fullfile(FAST_runDirectory, 'Airfoils'));
copyfile(fullfile(FAST_directory, '*.txt'), FAST_runDirectory);
copyfile(fullfile(FAST_directory, '*.dat'), FAST_runDirectory);
copyfile(fullfile(FAST_directory, '*.fst'), FAST_runDirectory);

if RUN_SIMS_PAR
    % load(fullfile(fastRunner.FAST_directory, 'ss_vals'));
    parfor case_idx=1:n_cases

        new_fst_name = fullfile(FAST_runDirectory, ...
            case_name_list{case_idx});
        new_infw_name = fullfile(FAST_runDirectory, ...
            [case_name_list{case_idx}, '_InflowWind']);
    
        fst_lines = fields(case_list(case_idx).Fst);
        fst_edits = {};
    
        for l = 1:length(fst_lines)
            fst_edits{l} = case_list(case_idx).Fst.(fst_lines{l});
        end
    
        if isfield(case_list, 'InflowWind')
            fst_lines{l + 1} = 'InflowFile';
            fst_edits{l + 1} = ['"' new_infw_name '.dat' '"'];
        end
        
        infw_lines = fields(case_list(case_idx).InflowWind);
        infw_edits = {};
    
        for l = 1:length(infw_lines)
            infw_edits{l} = case_list(case_idx).InflowWind.(infw_lines{l});
        end
        
        Af_EditFast(fst_lines, fst_edits, new_fst_name, def_fst_file, template_fst_dir, input_mode);
        Af_EditInflow(infw_lines, infw_edits, new_infw_name, def_infw_file, template_infw_dir, input_mode);
    end
elseif RUN_SIMS_SINGLE
    case_idx = 1;
    new_fst_name = fullfile(FAST_runDirectory, ...
            case_name_list{case_idx});
    new_infw_name = fullfile(FAST_runDirectory, ...
        [case_name_list{case_idx}, '_InflowWind']);

    fst_lines = fields(case_list(case_idx).Fst);
    fst_edits = {};

    for l = 1:length(fst_lines)
        fst_edits{l} = case_list(case_idx).Fst.(fst_lines{l});
    end

    if isfield(case_list, 'InflowWind')
        fst_lines{l + 1} = 'InflowFile';
        fst_edits{l + 1} = ['"' new_infw_name '.dat' '"'];
    end
    
    infw_lines = fields(case_list(case_idx).InflowWind);
    infw_edits = {};

    for l = 1:length(infw_lines)
        infw_edits{l} = case_list(case_idx).InflowWind.(infw_lines{l});
    end
    
    Af_EditFast(fst_lines, fst_edits, new_fst_name, def_fst_file, template_fst_dir, input_mode);
    Af_EditInflow(infw_lines, infw_edits, new_infw_name, def_infw_file, template_infw_dir, input_mode);
end
% Af_EditSub;
% Af_EditServo;
% Af_EditLin;
% Af_EditHydro;
% Af_EditElast;
% Af_EditDriver;
% Af_EditBeam;
% Af_EditAero15;
% Af_EditADriver;

if RUN_SIMS_SINGLE || RUN_SIMS_PAR
    if RUN_OL_DQ
        sim_type = 'ol_dq';
    elseif RUN_OL_BLADE
        sim_type = 'ol_blade';
    elseif RUN_CL && strcmp(WIND_TYPE, 'turbsim')
        if STRUCT_PARAM_SWEEP
            sim_type = 'pi_param_sweep_turbsim';
        elseif OPTIMAL_K_COLLECTION
            sim_type = 'optimal_k_cases_turbsim';
        elseif EXTREME_K_COLLECTION
            sim_type = 'extreme_k_cases_turbsim';
        elseif BASELINE_K
            sim_type = 'baseline_k_turbsim';
        elseif ~USE_IPC
            sim_type = 'noipc_turbsim';
        end
    end
end

if RUN_SIMS_PAR
    cd(project_dir);
    % sim_inputs = repmat(struct(), n_cases, 1 );
    for case_idx = 1:n_cases
    
        case_list(case_idx).FAST_InputFileName = fullfile(FAST_runDirectory, ...
            [case_name_list{case_idx}, '.fst']);
    
        % Populate thread parameters
        if STRUCT_PARAM_SWEEP
            % SL_model_name = 'AD_SOAR_c7_V2f_c73_MIMOPIControllerTest';
            % K_IPC = c2d(tf(case_list(case_idx).Controller_scaled(:, :, ...
            % LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED)), DT);
            
            SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean_FullOrderControllerTest';
            K_IPC = c2d(case_list(case_idx).Controller, DT);
            sim_inputs(case_idx) = Simulink.SimulationInput(SL_model_name);
            sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('K_IPC', K_IPC);
        elseif OPTIMAL_K_COLLECTION || EXTREME_K_COLLECTION
            
            if strcmp(case_list(case_idx).Structure, 'Full-Order')
                SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean_FullOrderControllerTest';
            elseif strcmp(case_list(case_idx).Structure, 'Structured')
                SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean_StructuredControllerTest';
            end
            % QUESTION MANUEL shouldn't be need to negate controller, isn't
            % SL model a positive feedback system? But negated controller
            % results in poorer performance. Does it need to be converted
            % to DT?
            sim_inputs(case_idx) = Simulink.SimulationInput(SL_model_name);
            K_IPC = c2d(case_list(case_idx).Controller_scaled, DT); % Note, this is the scaled controller
            sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('K_IPC', K_IPC);
        elseif BASELINE_K
            % sim_inputs(case_idx) = Simulink.SimulationInput('AD_SOAR_c7_V2f_c73_Clean_StructuredControllerTest');
            % K_IPC = case_list(case_idx).Controller; % Note, this is NOT the scaled controller
            % sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('K_IPC', K_IPC);
            SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean';
            sim_inputs(case_idx) = Simulink.SimulationInput(SL_model_name);
        elseif ~USE_IPC
            SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean';
            sim_inputs(case_idx) = Simulink.SimulationInput(SL_model_name);
        end
        
        [filepath, name, ext] = fileparts(strrep(case_list(case_idx).FAST_InputFileName, '.fst', ''));
        save_fn = fullfile(sl_save_dir, [name, '_', sim_type]);
        case_list(case_idx).save_fn = save_fn;
        sim_inputs(case_idx) = setBlockParameter(sim_inputs(case_idx), ...
            [SL_model_name, '/To File'], 'Filename', save_fn);

        sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('TMax', TMax);
        sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('DT', Simulation.DT);
        sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('HWindSpeed', case_list(case_idx).InflowWind.HWindSpeed);
        sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('RootMyc_ref', case_list(case_idx).Reference);
        
    end

    %% Run simulations in multiple parallel threads
    
    sim_out_list = parsim(sim_inputs, ...
                   'TransferBaseWorkspaceVariables', true, ... % Run simulation
                   'RunInBackground', 'off', ...
                   'ShowProgress',  'on', ...
                   'ShowSimulationManager', 'off');
    for case_idx = 1:n_cases
        sim_out_list(case_idx).InflowWind = case_list(case_idx).InflowWind;
        sim_out_list(case_idx).save_fn = case_list(case_idx).save_fn;
        sim_out_list(case_idx).FAST_InputFileName = case_list(case_idx).FAST_InputFileName;
    end

    
elseif RUN_SIMS_SINGLE
    % run single case
    for case_idx = 1:1
        FAST_InputFileName = fullfile(FAST_runDirectory, ...
            [case_name_list{case_idx}, '.fst']);
        DT = Simulation.DT;
        HWindSpeed = case_list(case_idx).InflowWind.HWindSpeed;
        RootMyc_ref = case_list(case_idx).Reference;

        % save_fn = strrep(FAST_InputFileName, '.fst', '');
        
        if STRUCT_PARAM_SWEEP
            % SL_model_name = 'AD_SOAR_c7_V2f_c73_MIMOPIControllerTest';
             SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean_FullOrderControllerTest';
            % K_IPC = c2d(tf(case_list(case_idx).Controller_scaled(:, :, ...
            %     LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED)), DT);
            K_IPC = c2d(case_list(case_idx).Controller, DT);
            % K_IPC(2, 2).Numerator(2) / K_IPC(2, 2).Denominator(1) 
        elseif OPTIMAL_K_COLLECTION || EXTREME_K_COLLECTION
            if strcmp(case_list(case_idx).Structure, 'Full-Order')
                SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean_FullOrderControllerTest';
            elseif strcmp(case_list(case_idx).Structure, 'Structured')
                SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean_StructuredControllerTest';
            end
            % QUESTION MANUEL does it make sense to implement negative of
            % tuned controller which assumed negative feedback since SL
            % simulation assumes positive feedback?
            K_IPC = c2d(case_list(case_idx).Controller_scaled, DT); % Note, this is the scaled controller
        else
            SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean';
        end
        open_system(fullfile(FAST_SimulinkModel_dir, SL_model_name))
        
        [filepath, name, ext] = fileparts(strrep(FAST_InputFileName, '.fst', ''));
        save_fn = fullfile(sl_save_dir, [name, '_', sim_type]);
        set_param([SL_model_name, '/To File'], 'Filename', save_fn);

        sim(fullfile(FAST_SimulinkModel_dir, ...
                SL_model_name), [0, TMax]);

        sim_out_list(case_idx).save_fn = save_fn;
        sim_out_list(case_idx).InflowWind = case_list(case_idx).InflowWind;
        sim_out_list(case_idx).FAST_InputFileName = FAST_InputFileName;
    end
end

%% Perform Transformations on OutData
% if RUN_SIMS_PAR || RUN_SIMS_SINGLE
% % QUESTION use linear of nearest lpv?
% %     load(fullfile(project_dir, 'sim_out_list_ol_dq.mat'));
%     if RUN_OL_BLADE || RUN_OL_DQ
%         load(fullfile(fastRunner.FAST_directory, 'op_absmax'));
%     end
%     for c = 1:length(sim_out_list)
%         [Channels, ~, ~, ~, ~] = ReadFASTbinary(strrep(sim_out_list(c).FAST_InputFileName, 'fst', 'outb'), 'n');
%         sim_out_list(c).values = Channels;
%         sim_out_list(c).dqValues = mbcTransformOutData(sim_out_list(c).values, OutList);
%         if RUN_OL_BLADE || RUN_OL_DQ
%             sim_out_list(c).OutData.signals.normalizedValues = normalizeOutData(sim_out_list(c).OutData.signals.values, table2array(op_absmax.blade(c, :)));
%             sim_out_list(c).OutData.signals.dqNormalizedValues = normalizeOutData(sim_out_list(c).OutData.signals.dqValues, table2array(op_absmax.dq(c, :)));
% 
%         end
%     end
% end

%% Save/Load Simulation Data

if RUN_SIMS_PAR || RUN_SIMS_SINGLE
  
    save(fullfile(sl_metadata_save_dir, ['sim_out_list_', sim_type, '.mat']), 'sim_out_list', '-v7.3');
    % for c = 1:length(sim_out_list.controller)
    %     [filepath,name,ext] = fileparts(strrep(sim_out_list.controller(c).FAST_InputFileName, 'fst', 'outb'));
    %     new_name = [name, '_', sim_type];
    %     % movefile(fullfile(filepath, [name, ext]), fullfile(project_dir, 'sl_outputs', [new_name, ext]));
    % end
elseif RUN_OL_DQ
    % get open-loop sensitivity to IPC simulations from memory
    load(fullfile(sl_metadata_save_dir, 'sim_out_list_ol_dq.mat'));
elseif RUN_OL_BLADE
    % get open-loop sensitivity to IPC simulations from memory
    load(fullfile(sl_metadata_save_dir, 'sim_out_list_ol_blade.mat'));
end

if RUN_CL && strcmp(WIND_TYPE, 'turbsim')
    if STRUCT_PARAM_SWEEP
        clear sim_out_list;
        sim_out_list.controller = load(fullfile(sl_metadata_save_dir, 'sim_out_list_pi_param_sweep_turbsim.mat')); % load ipc case
        sim_out_list.controller = sim_out_list.controller.sim_out_list;
        sim_out_list.noipc = load(fullfile(sl_metadata_save_dir, 'sim_out_list_noipc_turbsim.mat'));
        sim_out_list.noipc = sim_out_list.noipc.sim_out_list;
        sim_out_list.baseline_controller = load(fullfile(sl_metadata_save_dir, 'sim_out_list_baseline_k_turbsim.mat')); % load baseline ipc case
        sim_out_list.baseline_controller = sim_out_list.baseline_controller.sim_out_list;
    elseif OPTIMAL_K_COLLECTION
        clear sim_out_list;
        sim_out_list.controller = load(fullfile(sl_metadata_save_dir, 'sim_out_list_optimal_k_cases_turbsim.mat'));
        sim_out_list.controller = sim_out_list.controller.sim_out_list;
        sim_out_list.noipc = load(fullfile(sl_metadata_save_dir, 'sim_out_list_noipc_turbsim.mat')); % load noipc case
        sim_out_list.noipc = sim_out_list.noipc.sim_out_list;
        sim_out_list.baseline_controller = load(fullfile(sl_metadata_save_dir, 'sim_out_list_baseline_k_turbsim.mat')); % load baseline ipc case
        sim_out_list.baseline_controller = sim_out_list.baseline_controller.sim_out_list;
    elseif EXTREME_K_COLLECTION
        clear sim_out_list;
        sim_out_list.controller = load(fullfile(sl_metadata_save_dir, 'sim_out_list_extreme_k_cases_turbsim.mat')); % load ipc case
        sim_out_list.controller = sim_out_list.controller.sim_out_list;
        sim_out_list.noipc = load(fullfile(sl_metadata_save_dir, 'sim_out_list_noipc_turbsim.mat')); % load noipc case
        sim_out_list.noipc = sim_out_list.noipc.sim_out_list;
        sim_out_list.baseline_controller = load(fullfile(sl_metadata_save_dir, 'sim_out_list_baseline_k_turbsim.mat')); % load baseline ipc case
        sim_out_list.baseline_controller = sim_out_list.baseline_controller.sim_out_list;
    elseif ~USE_IPC
        load(fullfile(sl_metadata_save_dir, 'sim_out_list_noipc_turbsim.mat'));
    end
end

%% Analysis of Blade-Pitch Actuation and Loads in Time-Domain
if EXTREME_K_COLLECTION || OPTIMAL_K_COLLECTION
    beta_dot_norm = @(beta_dot) ((beta_dot >= 0) * 5) + ((beta_dot < 0) * (-4)); % TODO get correct values

    bts_filename = case_basis.InflowWind.FileName_BTS{1};
    ux = case_basis.InflowWind.HWindSpeed(1);
    
    % dq values of blade-pitch and blade root bending moment for
    % particular wind field
    values = load([sim_out_list.noipc(1).save_fn '_1']);
    values = values.OutData.Data;
    dqValues = mbcTransformOutData(values, OutList);

    beta(1).noipc.dq = [getData(dqValues, dqOutList, 'BldPitchD'),...
            getData(dqValues, dqOutList, 'BldPitchQ')]; % in degrees
    RootMyc(1).noipc.dq = [getData(dqValues, dqOutList, 'RootMycD'), ...
            getData(dqValues, dqOutList, 'RootMycQ')];
    sim_out_list.noipc(1).RootMycMSE = struct;
    sim_out_list.noipc(1).RootMycMSE.dq = (1 / size(RootMyc(1).noipc.dq, 1)) * sum(RootMyc(1).noipc.dq.^2, 1);

    beta(1).noipc.blade = getData(values, OutList, 'BldPitch1'); % in degrees
    beta_dot(1).noipc.blade = diff(beta(1).noipc.blade) / DT;
    sim_out_list.noipc(1).ADC = (1 / length(beta_dot(1).noipc.blade)) * sum((beta_dot(1).noipc.blade ./ beta_dot_norm(beta_dot(1).noipc.blade)));
    RootMyc(1).noipc.blade = getData(values, OutList, 'RootMyc1');
    sim_out_list.noipc(1).RootMycMSE.blade = (1 / length(RootMyc(1).noipc.blade)) * sum(RootMyc(1).noipc.blade.^2);

    cc = 0;
    for c = 1:length(sim_out_list.controller)
        % if length(sim_out_list.controller(c).ErrorMessage)
        %     continue;
        % end
        if ~strcmp(sim_out_list.controller(c).InflowWind.FileName_BTS, bts_filename) ...
                || (sim_out_list.controller(c).InflowWind.HWindSpeed ~= ux) ...
                || ~strcmp(sim_out_list.noipc(1).InflowWind.FileName_BTS, bts_filename) ...
                || (sim_out_list.noipc(1).InflowWind.HWindSpeed ~= ux)
            continue;
        end
        cc = cc + 1;
        % [values, ~, ~, ~, ~] = ReadFASTbinary(strrep(sim_out_list.controller(c).FAST_InputFileName, 'fst', 'outb'), 'n');
        values = load([sim_out_list.controller(c).save_fn '_' num2str(c)]);
        values = values.OutData.Data;
        dqValues = mbcTransformOutData(values, OutList);

        beta(cc).controller.dq = [getData(dqValues, dqOutList, 'BldPitchD'), ...
            getData(dqValues, dqOutList, 'BldPitchQ')]; % in degrees
        
        t = getData(values, OutList, 'Time');
        
        RootMyc(cc).controller.dq = [getData(dqValues, dqOutList, 'RootMycD'), ...
            getData(dqValues, dqOutList, 'RootMycQ')];
        
        sim_out_list.controller(c).RootMycMSE = struct;
        sim_out_list.controller(c).RootMycMSE.dq = (1 / size(RootMyc(cc).controller.dq, 1)) * sum(RootMyc(cc).controller.dq.^2, 1);
    end
    
    % blade values of blade-pitch and blade root bending moment for
    % particular wind field
    cc = 0;

    for c = 1:length(sim_out_list.controller)
        if ~strcmp(sim_out_list.controller(c).InflowWind.FileName_BTS, bts_filename) ...
                || (sim_out_list.controller(c).InflowWind.HWindSpeed ~= ux) ...
                || ~strcmp(sim_out_list.noipc(1).InflowWind.FileName_BTS, bts_filename) ...
                || (sim_out_list.noipc(1).InflowWind.HWindSpeed ~= ux)
            continue;
        end
        cc = cc + 1;
        
        % [values, ~, ~, ~, ~] = ReadFASTbinary(strrep(sim_out_list.controller(c).FAST_InputFileName, 'fst', 'outb'), 'n');
        values = load([sim_out_list.controller(c).save_fn '_' num2str(c)]);
        values = values.OutData.Data;
        dqValues = mbcTransformOutData(values, OutList); 
        
        beta(cc).controller.blade = getData(values, OutList, 'BldPitch1'); % in degrees
        beta_dot(cc).controller.blade = diff(beta(cc).controller.blade) / DT;
        
        t = getData(values, OutList, 'Time');
        sim_out_list.controller(c).ADC = (1 / length(beta_dot(cc).controller.blade)) * sum((beta_dot(cc).controller.blade ./ beta_dot_norm(beta_dot(cc).controller.blade)));
        
        RootMyc(cc).controller.blade = getData(values, OutList, 'RootMyc1');
        
        sim_out_list.controller(c).RootMycMSE.blade = (1 / length(RootMyc(cc).controller.blade)) * sum(RootMyc(cc).controller.blade.^2);
        % sim_out_list.noipc(1).RootMycMean = (1 / length(RootMyc(1).noipc.blade)) * sum(RootMyc(1).noipc.blade);
        % sim_out_list.controller(c).RootMycMean = (1 / length(RootMyc(cc).controller.blade)) * sum(RootMyc(cc).controller.blade);
    end
    
    % Make table comparing controllers
    if OPTIMAL_K_COLLECTION || EXTREME_K_COLLECTION
        if OPTIMAL_K_COLLECTION
            load(fullfile(mat_save_dir, 'Optimal_Controllers_case_table.mat'));
        elseif EXTREME_K_COLLECTION
            load(fullfile(mat_save_dir, 'Extreme_Controllers_case_table.mat'));
        end
    
        op = cell(length(sim_out_list.controller), 1);
        [op{:}] = sim_out_list.controller.ADC;
        Controllers_case_table.("ADC") = [sim_out_list.noipc(1).ADC; cell2mat(op)];
        op = cell(length(sim_out_list.controller), 1);
        [op{:}] = sim_out_list.controller.RootMycMSE.blade;
        Controllers_case_table.("RootMycBlade1 MSE") = [sim_out_list.noipc(1).RootMycMSE.blade; cell2mat(op)];
        op = cell(length(sim_out_list.controller), 1);
        [op{:}] = sim_out_list.controller.RootMycMSE.dq(:, 1);
        Controllers_case_table.("RootMycD MSE") = [sim_out_list.noipc(1).RootMycMSE.dq(:, 1); cell2mat(op)];
        op = cell(length(sim_out_list.controller), 1);
        [op{:}] = sim_out_list.controller.RootMycMSE.dq(:, 2);
        Controllers_case_table.("RootMycQ MSE") = [sim_out_list.noipc(1).RootMycMSE.dq(:, 2); cell2mat(op)];
        % sortrows(Controllers_case_table, 'Mean DMi', 'descend')
        W1Gains = [0]; W2Gains = [0]; WeGains = [0]; WuGains = [0];
        for c = 1:length(case_list)
            x = cell2mat(case_list(c).W1Gain.Numerator(1,1));
            W1Gains = [W1Gains x(1)];
            x = cell2mat(case_list(c).W2Gain.Numerator(1,1));
            W2Gains = [W2Gains x(1)];
            x = cell2mat(case_list(c).WuGain.Numerator(1,1));
            WuGains = [WuGains x(1)];
            x = cell2mat(case_list(c).WeGain.Numerator(1,1));
            WeGains = [WeGains x(1)];
        end
        Controllers_case_table.("$W_1$ Gain") = W1Gains';
        Controllers_case_table.("$W_2$ Gain") = W2Gains';
        Controllers_case_table.("$W_u$ Gain") = WuGains';
        Controllers_case_table.("$W_e$ Gain") = WeGains';

        sortrows(Controllers_case_table, 'ADC', 'ascend')
        sortrows(Controllers_case_table, 'RootMycBlade1 MSE', 'ascend')

        table_tmp = Controllers_case_table(:, ["Case No.", "$W_1$ Gain", "$W_2$ Gain", "$W_u$ Gain", "$W_e$ Gain", "wc", "ADC", "RootMycBlade1 MSE", "RootMycD MSE", "RootMycQ MSE"]);
        table_tmp.Properties.VariableNames = {'Controller', '$W_1$ Gain', '$W_2$ Gain', '$W_u$ Gain', '$W_e$ Gain','$\omega_c$', 'ADC', '$M_{b}$ MSE', '$\tilde{M}_{d}$ MSE', '$\tilde{M}_{q}$ MSE'};
        table_tmp.("Controller") = [{"None"}; num2cell(1:size(table_tmp, 1) - 1)'];
        
        if OPTIMAL_K_COLLECTION
            save(fullfile(mat_save_dir, 'Optimal_Controllers_case_table.mat'), "Controllers_case_table");
            table2latex(table_tmp, fullfile(fig_dir, "optimal_controller_table.tex"));
        elseif EXTREME_K_COLLECTION
            save(fullfile(mat_save_dir, 'Extreme_Controllers_case_table.mat'), "Controllers_case_table");
            table2latex(table_tmp, char(fullfile(fig_dir, 'extreme_controller_table.tex')));
        end
        
    
    elseif STRUCT_PARAM_SWEEP
        load(fullfile(mat_save_dir, 'PI_ParameterSweep_redtable.mat'));
        op = cell(length(sim_out_list.controller), 1);
        [op{:}] = sim_out_list.controller.ADC;
        PI_ParameterSweep_redtable.("ADC") = [sim_out_list.noipc(1).ADC; cell2mat(op)];
        op = cell(length(sim_out_list.controller), 1);
        [op{:}] = sim_out_list.controller.RootMycMSE.blade;
        PI_ParameterSweep_redtable.("RootMycBlade1 MSE") = [sim_out_list.noipc(1).RootMycMSE.blade; cell2mat(op)];
        op = cell(length(sim_out_list.controller), 1);
        [op{:}] = sim_out_list.controller.RootMycMSE.dq(:, 1);
        PI_ParameterSweep_redtable.("RootMycD MSE") = [sim_out_list.noipc(1).RootMycMSE.dq(:, 1); cell2mat(op)];
        op = cell(length(sim_out_list.controller), 1);
        [op{:}] = sim_out_list.controller.RootMycMSE.dq(:, 2);
        PI_ParameterSweep_redtable.("RootMycQ MSE") = [sim_out_list.noipc(1).RootMycMSE.dq(:, 2); cell2mat(op)];
        % sortrows(Controllers_case_table, 'Mean DMi', 'descend')
        sortrows(PI_ParameterSweep_redtable, 'ADC', 'ascend')
        sortrows(PI_ParameterSweep_redtable, 'RootMycBlade1 MSE', 'ascend')
        % sortrows(PI_ParameterSweep_redtable, 'RootMycBlade1 Mean', 'ascend')
        save(fullfile(code_dir, 'matfiles', 'PI_ParameterSweep_redtable.mat'), "PI_ParameterSweep_redtable");
        table_tmp = PI_ParameterSweep_redtable(:, ["Case No.", "Kp_diag", "Kp_offdiag", "Ki_diag", "Ki_offdiag", "ADC", "RootMycBlade1 MSE", "RootMycD MSE", "RootMycQ MSE"]);
        table_tmp.Properties.VariableNames = {'Controller', '$k^p_{d}$', '$k^p_{od}$', '$k^i_{d}$', '$k^i_{od}$', 'ADC', '$M_{b}$ MSE', '$\tilde{M}_{d}$ MSE', '$\tilde{M}_{q}$ MSE'};
        table_tmp.("Controller") = [{"None"}; num2cell(1:size(table_tmp, 1) - 1)'];
        table2latex(table_tmp, fullfile(fig_dir, "param_sweep_table.tex"));
    end
end

%% Plot Blade-Pitch Actuation and Loads in Time-Domain OUTPLOT
if 0
% plot(sim_out_list.controller.K_IPC_op.Data)
% plot(sim_out_list.controller.Cyc_BldPitch2
% Plot DQ values of blade-pitch and blade root bending moment for
% particular wind field
% if (strcmp(case_basis.InflowWind.WindType, '3')) && (strcmp(case_basis.InflowWind.FileName_BTS{i}, ))
bts_filename = case_basis.InflowWind.FileName_BTS{1};
ux = case_basis.InflowWind.HWindSpeed(1);
figure;
tcf = tiledlayout(2, 2);
cc = 0;
for c = 1:length(sim_out_list.controller)
    if length(sim_out_list.controller(c).ErrorMessage)
        continue;
    end
    if ~strcmp(sim_out_list.controller(c).InflowWind.FileName_BTS, bts_filename) ...
            || (sim_out_list.controller(c).InflowWind.HWindSpeed ~= ux) ...
            || ~strcmp(sim_out_list.noipc(1).InflowWind.FileName_BTS, bts_filename) ...
            || (sim_out_list.noipc(1).InflowWind.HWindSpeed ~= ux)
        continue;
    end
    t = sim_out_list.controller(c).OutData.time;
    cc = cc + 1;
    nexttile(1);
    plot(t, beta(cc).controller.dq(:, 1));
    hold on;
    nexttile(2);
    plot(t, beta(cc).controller.dq(:, 2));
    hold on;
    nexttile(3);
    plot(t, RootMyc(cc).controller.dq(:, 1));
    hold on;
    nexttile(4);
    plot(t, RootMyc(cc).controller.dq(:, 2));
    hold on;
end
nexttile(1); plot(t, beta(1).noipc.dq(:, 1)); hold off;
nexttile(2); plot(t, beta(1).noipc.dq(:, 2)); hold off;
nexttile(3); plot(t, RootMyc(1).noipc.dq(:, 1)); hold off;
nexttile(4); plot(t, RootMyc(1).noipc.dq(:, 2)); hold off;

nexttile(1);
xlabel('Time [s]'); ylabel('Blade-Pitch DQ Actuation [deg]', 'Rotation', 0);
nexttile(3);
xlabel('Time [s]'); ylabel('Root Blade DQ Bending Moment [kNm]', 'Rotation', 0);
nexttile(4);

legend([arrayfun(@(n) ['', num2str(n)], 1:length(sim_out_list.controller), 'UniformOutput', false), 'None']);


% Plot blade values of blade-pitch and blade root bending moment for
% particular wind field
figure;
tcf = tiledlayout(2, 1);
cc = 0;
for c = 1:length(sim_out_list.controller)
    if length(sim_out_list.controller(c).ErrorMessage)
        continue;
    end
    if ~strcmp(sim_out_list.controller(c).InflowWind.FileName_BTS, bts_filename) ...
            || (sim_out_list.controller(c).InflowWind.HWindSpeed ~= ux) ...
            || ~strcmp(sim_out_list.noipc(1).InflowWind.FileName_BTS, bts_filename) ...
            || (sim_out_list.noipc(1).InflowWind.HWindSpeed ~= ux)
        continue;
    end
    t = sim_out_list.controller(c).OutData.time;
    cc = cc + 1;
    nexttile(1);
    plot(t, beta(cc).controller.blade);
    hold on;
    nexttile(2);
    plot(t, RootMyc(cc).controller.blade);
    hold on;
end
nexttile(1); plot(t, beta(1).noipc.blade); hold off;
nexttile(2); plot(t, RootMyc(1).noipc.blade); hold off;
legend([arrayfun(@(n) ['', num2str(n)], 1:length(sim_out_list.controller), 'UniformOutput', false), 'None']);
nexttile(1);
xlabel('Time [s]'); ylabel('Blade-Pitch Actuation [deg]', 'Rotation', 0);
nexttile(2);
xlabel('Time [s]'); ylabel('Root Blade 1 Bending Moment [kNm]', 'Rotation', 0);
set(gcf, 'Position', [0 0 1500 900]);%get(0, 'Screensize'));
savefig(gcf, fullfile(fig_dir, 'nonlin_ts.fig'));
saveas(gcf, fullfile(fig_dir, 'nonlin_ts.png'));
end


%% PSD Analysis of loads OUTPLOT
% define loads of concern
% Blade root O/P, Shaft My, Yaw bearing My/Mz, Hub, Nacelle, Main Bearing,
% Blade Flapwise/Edgewise, Blade Oop/Ip, Tower
exc_blade_fields = {'Time', 'BldPitch1', 'BldPitch2', 'BldPitch3', ...
    'Azimuth', 'RotSpeed', 'Wind1VelX', 'Wind1VelY', 'Wind1VelZ', ...
    '-React', 'Alpha', 'Rt', 'M2N', 'M8N', 'GenTq'};

% 'Spn2MLxb1', 'Spn2MLyb1', ... % Blade 1 local edgewise/flapwise moment at span station 2
% 'RootFxb1', 'RootFyb1', ... % Blade 1 flapwise/edgewise shear force at the blade root
% 'YawBrFxp', 'YawBrFyp', 'YawBrFzp', ... % Tower-top / yaw bearing fore-aft/side-t0-side shear/axial force
% 'TwrBsFxt', 'TwrBsFyt', 'TwrBsFzt', ... % Tower base fore-aft/side-to-side shear/axial force
                              
blade_op_arr = {'OoPDefl1', 'IPDefl1', 'TwstDefl1', ...
          'RotThrust', ...
          'TTDspFA', 'TTDspSS', 'TTDspTwst', ... % Tower-top / yaw bearing fore-aft/side-to-side/angular torsion deflection
          'RootMxb1', 'RootMyb1', ... % Blade 1 edgewise/flapwise moment 
          'RootMyc1', 'RootMzc1', ... % Blade 1 out-of-plane/pitching moment
          'LSSGagMya', 'LSSGagMza', ... % Rotating low-speed shaft bending moment at the shaft's strain gage (about ya/za axis)
          'YawBrMxp', 'YawBrMyp', 'YawBrMzp', ... % Nonrotating tower-top / yaw bearing roll/pitch/yaw moment
          'TwrBsMxt', 'TwrBsMyt', 'TwrBsMzt' ... % Tower base roll (or side-to-side)/pitching (or fore-aft)/yaw moment 
          };

exc_dq_fields = {'BldPitchD', 'BldPitchQ'};
dq_op_arr = {};
for l = dqOutList'
    if sum(strmatch(l, exc_dq_fields)) == 0 && (strcmp(l{1}(end), 'D') || strcmp(l{1}(end), 'Q'))
        dq_op_arr = [dq_op_arr, l{1}];
    end
end
COMPUTE_FFT = 1;
if COMPUTE_FFT
    % for each simulation
    blade_op_indices = [];
    for i = 1:length(blade_op_arr)
        blade_op_indices = [blade_op_indices strmatch(blade_op_arr{i}, OutList)];
    end
    dq_op_indices = [];
    for i = 1:length(dq_op_arr)
        dq_op_indices = [dq_op_indices strmatch(dq_op_arr{i}, dqOutList)];
    end
    % TODO make sure this study is being done for equivalent wind fields
    for c = 1:length(sim_out_list.controller)
        sprintf(['Processing Simulation ' num2str(c)]);
        % fetch time-series of list of loads of concern
        % [Channels, ChanName, ChanUnit, FileID, DescStr] = ReadFASTbinary(strrep(sim_out_list(c).FAST_InputFileName, 'fst', 'outb'), 'n');
        values = load(sim_out_list.controller(c).save_fn);
        values = values.OutData.Data;
        dqValues = mbcTransformOutData(values, OutList); 

        op_data_blade_controller = values(cut_transients/DT + 1:end, blade_op_indices);
        op_data_cdq_controller = dqValues(cut_transients/DT + 1:end, blade_op_indices);
        
        op_data_blade_noipc = values(cut_transients/DT + 1:end, blade_op_indices);
        op_data_cdq_noipc = dqValues(cut_transients/DT + 1:end, blade_op_indices);

        % compute fft for rotating domain corresponding to Blade 1
        fft_blade_controller(:, :, c) = fft(op_data_blade_controller, size(op_data_blade_controller, 1), 1);
        fft_blade_noipc(:, :, 1) = fft(op_data_blade_noipc, size(op_data_blade_noipc, 1), 1);
    
        % compute fft for cdq domain loads
        fft_cdq_controller(:, :, c) = fft(op_data_cdq_controller, size(op_data_cdq_controller, 1), 1);
        fft_cdq_noipc(:, :, 1) = fft(op_data_cdq_noipc, size(op_data_cdq_noipc, 1), 1);
    end
    fft_blade.controller = fft_blade_controller;
    fft_blade.noipc = fft_blade_noipc;
    fft_cdq.controller = fft_cdq_controller;
    fft_cdq.noipc = fft_cdq_noipc;
    
    if strcmp(WIND_TYPE, 'turbsim')
        save(fullfile(project_dir, 'fft_blade_vals_turb.mat'), 'fft_blade', '-v7.3');
        save(fullfile(project_dir, 'fft_cdq_vals_turb.mat'), 'fft_cdq', '-v7.3');
%         save(fullfile(project_dir, 'fft_blade_vals_turb_test.mat'), 'fft_blade', '-v7.3');
%         save(fullfile(project_dir, 'fft_cdq_vals_turb_test.mat'), 'fft_cdq', '-v7.3');
    elseif strcmp(WIND_TYPE, 'steady')
        save(fullfile(project_dir, 'fft_blade_vals_steady.mat'), 'fft_blade', '-v7.3');
        save(fullfile(project_dir, 'fft_cdq_vals_steady.mat'), 'fft_cdq', '-v7.3');
    end
else
    if strcmp(WIND_TYPE, 'turbsim')
        load(fullfile(project_dir, 'fft_blade_vals_turb_test.mat'));
        load(fullfile(project_dir, 'fft_cdq_vals_turb_test.mat'));
    elseif strcmp(WIND_TYPE, 'steady')
        load(fullfile(project_dir, 'fft_blade_vals_steady.mat'));
        load(fullfile(project_dir, 'fft_cdq_vals_steady.mat'));
    end
end

% compute mean and standard deviation over all simulations for each load
% fft_blade_mean = mean(fft_blade, 1);
% fft_blade_std = std(fft_blade, 1);
% fft_cdq_mean = mean(fft_cdq, 1);
% fft_cdq_std = std(fft_cdq, 1);

% figure;
% boxplot(op_arr, );

% plot mean fft for each load
omega_1P_Hz = Parameters.Turbine.wr_rated * (2*pi/60) * (1/(2*pi));

% compute mean and bounds of PSD/FFT of this load over all simulations for
% each load for Blade/CDQ coords
if 0

    figure;
    tcf = tiledlayout('flow');
    plotSpectra(fft_blade.controller, 'psd', DT, omega_1P_Hz, HARMONICS, blade_op_arr); % QUESTION why is frequency range limited to 6p?
    plotSpectra(fft_blade.noipc, 'psd', DT, omega_1P_Hz, HARMONICS, blade_op_arr); 
    % plotSpectra(fft_blade, 'fft', DT, omega_1P_Hz, HARMONICS, blade_op_arr);
    
    figure;
    tcf = tiledlayout('flow');
    plotSpectra(fft_cdq.controller, 'psd', DT, omega_1P_Hz, HARMONICS, dq_op_arr);
    plotSpectra(fft_cdq.noipc, 'psd', DT, omega_1P_Hz, HARMONICS, dq_op_arr);
    % plotSpectra(fft_cdq, 'fft', DT, omega_1P_Hz, HARMONICS, dq_op_arr);

end

[~, ~, Peaks_noipc] = computeFFTPeaks(...
    fft_blade.noipc(:, ismember('RootMyc1', blade_op_arr), :), ...
    DT, omega_1P_Hz, [1,2,3,4]);
[~, ~, Peaks_controllers] = computeFFTPeaks(...
    fft_blade.controller(:, ismember('RootMyc1', blade_op_arr), :), ...
    DT, omega_1P_Hz, [1,2,3,4]);
fft_Peaks.noipc = Peaks_noipc;
fft_Peaks.controller = Peaks_controllers;
fft_Peaks.harmonics = [1,2,3,4];

figure;
plot(fft_Peaks.harmonics, mag2db(squeeze(fft_Peaks.noipc)));
hold on;
for case_idx = 1:length(sim_out_list.controller)
    plot(fft_Peaks.harmonics, mag2db(squeeze(fft_Peaks.controller(:, :, case_idx, :))));
end
xlabel('Harmonic'); ylabel('Magnitude [dB]');
hold off;
legend(['No IPC', arrayfun(@(n) ['Case', num2str(n)], 1:length(sim_out_list.controller), 'UniformOutput', false)], 'NumColumns', 2);
% QUESTION, should these show greater amplitudes at 1p, 2p/4p, 5p/7p?
% (corresponding to 0p, 3p, 6p in non-rotating domain)

% title("Single-Sided Amplitude Spectrum")

% xlim([0, 0.3]);
% ylim([0, 2e4]);

% compute boxplot data over all ffts for each load at rotational HARMONICS
% 1, 2, 3, 4, 5, 6, 7
% plot boxplots at rotational HARMONICS for each load
% boxplot(x,'PlotStyle','compact')

% Make table comparing controllers
if OPTIMAL_K_COLLECTION
    load(fullfile(code_dir, 'matfiles', 'Controllers_case_table.mat'));
    for p = 1:length(fft_Peaks.harmonics)
        Controllers_case_table.(['RootMycBlade1 ', num2str(p), 'th ', 'Harmonic [dB]']) ...
            = [mag2db(squeeze(fft_Peaks.noipc)); mag2db(squeeze(fft_Peaks.controller(:, :, :, p)))];
    end
    sortrows(Controllers_case_table, ['RootMycBlade1 ', num2str(3), 'th ', 'Harmonic [dB]'], 'descend')
elseif EXTREME_K_COLLECTION
    load(fullfile(code_dir, 'matfiles', 'Extreme_Controllers_case_table.mat'));
    for p = 1:length(fft_Peaks.harmonics)
        Controllers_case_table.(['RootMycBlade1 ', num2str(p), 'th ', 'Harmonic [dB]']) ...
            = [mag2db(squeeze(fft_Peaks.noipc)); mag2db(squeeze(fft_Peaks.controller(:, :, :, p)))];
    end
    sortrows(Controllers_case_table, ['RootMycBlade1 ', num2str(3), 'th ', 'Harmonic [dB]'], 'descend')
elseif STRUCT_PARAM_SWEEP
    load(fullfile(code_dir, 'matfiles', 'PI_ParamSweep_redtable.mat'));
end

%% DEL analysis of loads
% run python DEL analysis

% read python DEL analysis output

% plot percentage DEL reduction @ each mean wind speed for blade
% out-of-plane load, nacelle yaw moment, nacelle pitch moment, shaft
% bending moment around y-axis
figure;


%% Analysis of loads from nonlinear simulations

Delta_op_nonlin = [];
% for each case, compute output mean and absmax before and after
% RampStart
OutList_op = OutList;
exc_outdata_fields = {'Time', 'BldPitch1', 'BldPitch2', 'BldPitch3', ...
    'Azimuth', 'RotSpeed', 'Wind1VelX', 'Wind1VelY', 'Wind1VelZ', ...
    '-React', 'Alpha', 'Rt', 'M2N', 'M8N', 'GenTq'};
inc_outdata_indices = 1:length(OutList);
for field = exc_outdata_fields
    field_idx = cellfun(@(a) ~isempty(a) && a > 0, strfind(OutList_op, field));
    OutList_op(field_idx) = [];
    inc_outdata_indices(field_idx) = [];
end

absmax_varnames = {};
mean_varnames = {};
abs_absmax_varnames = {};
abs_mean_varnames = {};

load(fullfile(fastRunner.FAST_directory, 'op_absmax'));

for c = 1:length(sim_out_list)
    % [Channels, ChanName, ChanUnit, FileID, DescStr] = ReadFASTbinary(strrep(sim_out_list(c).FAST_InputFileName, 'fst', 'outb'), 'n');
    values = load(sim_out_list.controller(c).save_fn);
    values = values.OutData.Data;
    dqValues = mbcTransformOutData(values, OutList); 
    
    ws = mean(getData(values, OutList, 'Wind1VelX'));

    nanvals = zeros(length(OutList_op), 1); % ismember(OutList_op, {'GenPwr'});%all(pre_dist.data == 0);

    if RUN_OL_DQ
        pre_dist.data = dqNormalizedValues(...
            BaselineSteadyState / DT, inc_outdata_indices);

        post_dist.data.d = dqNormalizedValues(...
        dSteadyState / DT, inc_outdata_indices);
        post_dist.data.q = dqNormalizedValues(...
            qSteadyState / DT, inc_outdata_indices);

        % compute mean over all rows (time) for each column (output) before/after RampStart    
        pre_dist.mean = mean(pre_dist.data, 1);
        post_dist.mean.d = mean(post_dist.data.d, 1);
        post_dist.mean.q = mean(post_dist.data.q, 1);

        % compute absmax over all rows (time) for each column (output) before/after RampStart
        pre_dist.absmax = abs(max(pre_dist.data, [], 1, 'ComparisonMethod', 'abs'));

        post_dist.absmax.d = abs(max(post_dist.data.d, [], 1, 'ComparisonMethod', 'abs'));
        post_dist.absmax.q = abs(max(post_dist.data.q, [], 1, 'ComparisonMethod', 'abs'));
        
        rel_mean.d = 100 * (post_dist.mean.d - pre_dist.mean) ./ pre_dist.mean;
        rel_mean.q = 100 * (post_dist.mean.q - pre_dist.mean) ./ pre_dist.mean;

        rel_absmax.d = 100 * (post_dist.absmax.d - pre_dist.absmax) ./ pre_dist.absmax;
        rel_absmax.q = 100 * (post_dist.absmax.q - pre_dist.absmax) ./ pre_dist.absmax;

        abs_rel_mean.d = abs(rel_mean.d);
        abs_rel_mean.q = abs(rel_mean.q);

        abs_rel_absmax.d = abs(rel_absmax.d);
        abs_rel_absmax.q = abs(rel_absmax.q);

        absmax_varnames.d{c} = ['Relative Change in AbsMax WS = ' num2str(ws) 'm/s' ' D'];
        mean_varnames.d{c} = ['Relative Change in Mean WS = ' num2str(ws) 'm/s' ' D'];
        absmax_varnames.q{c} = ['Relative Change in AbsMax WS = ' num2str(ws) 'm/s' ' Q'];
        mean_varnames.q{c} = ['Relative Change in Mean WS = ' num2str(ws) 'm/s' ' Q'];

        abs_absmax_varnames.d{c} = ['Absolute ' absmax_varnames.d{c}];
        abs_mean_varnames.d{c} = ['Absolute ' mean_varnames.d{c}];
        abs_absmax_varnames.q{c} = ['Absolute ' absmax_varnames.q{c}];
        abs_mean_varnames.q{c} = ['Absolute ' mean_varnames.q{c}];

        base_varnames = {mean_varnames.d{c}, absmax_varnames.d{c}, ...
        abs_mean_varnames.d{c}, abs_absmax_varnames.d{c}, ...
        mean_varnames.q{c}, absmax_varnames.q{c}, ...
        abs_mean_varnames.q{c}, abs_absmax_varnames.q{c}};

        Delta_op_nonlin = [Delta_op_nonlin, ...
        table(...
        rel_mean.d(:, ~nanvals)', rel_absmax.d(:, ~nanvals)', ...
        abs_rel_mean.d(:, ~nanvals)', abs_rel_absmax.d(:, ~nanvals)', ...
        rel_mean.q(:, ~nanvals)', rel_absmax.q(:, ~nanvals)', ...
        abs_rel_mean.q(:, ~nanvals)', abs_rel_absmax.q(:, ~nanvals)', ...
        'VariableNames', base_varnames, ...
        'RowNames', OutList_op(~nanvals))];

    elseif RUN_OL_BLADE
        % [Channels, ChanName, ChanUnit, FileID, DescStr] = ReadFASTbinary(strrep(sim_out_list(c).FAST_InputFileName, 'fst', 'outb'), 'n');
        values = load(sim_out_list.controller(c).save_fn);
        values = values.OutData.Data;
        dqValues = mbcTransformOutData(values, OutList); 
        
        pre_dist.data = normalizedValues(...
            BaselineSteadyState / DT, inc_outdata_indices);

        post_dist.data = normalizedValues(...
        b1SteadyState / DT, inc_outdata_indices);

        % compute mean over all rows (time) for each column (output) before/after RampStart    
        pre_dist.mean = mean(pre_dist.data, 1);
        post_dist.mean = mean(post_dist.data, 1);

        % compute absmax over all rows (time) for each column (output) before/after RampStart
        pre_dist.absmax = abs(max(pre_dist.data, [], 1, 'ComparisonMethod', 'abs'));

        post_dist.absmax = abs(max(post_dist.data, [], 1, 'ComparisonMethod', 'abs'));
        
        rel_mean = 100 * (post_dist.mean - pre_dist.mean) ./ pre_dist.mean;

        rel_absmax = 100 * (post_dist.absmax - pre_dist.absmax) ./ pre_dist.absmax;

        abs_rel_mean = abs(rel_mean);

        abs_rel_absmax = abs(rel_absmax);

        absmax_varnames{c} = ['Relative Change in AbsMax WS = ' num2str(ws) 'm/s'];
        mean_varnames{c} = ['Relative Change in Mean WS = ' num2str(ws) 'm/s'];

        abs_absmax_varnames{c} = ['Absolute ' absmax_varnames{c}];
        abs_mean_varnames{c} = ['Absolute ' mean_varnames{c}];

        base_varnames = {mean_varnames{c}, absmax_varnames{c}, ...
        abs_mean_varnames{c}, abs_absmax_varnames{c}};

        Delta_op_nonlin = [Delta_op_nonlin, ...
        table(...
        rel_mean(:, ~nanvals)', rel_absmax(:, ~nanvals)', ...
        abs_rel_mean(:, ~nanvals)', abs_rel_absmax(:, ~nanvals)', ...
        'VariableNames', base_varnames, ...
        'RowNames', OutList_op(~nanvals))];
    else
        % not open-loop simulation run, but with turbulent wind fields
        % TODO compute the mean (over 100 turbulent simulations) FFT of different loads
    end
    

end

if RUN_OL_DQ
    for comp = {'d', 'q'}

        Delta_op_nonlin = [Delta_op_nonlin, ...
            table(mean(table2array(Delta_op_nonlin(:, ...
            abs_absmax_varnames.(comp{1}))), 2), ...
            'VariableNames', {['Mean Relative Change in AbsMax ' comp{1}]})];
        Delta_op_nonlin = [Delta_op_nonlin, ...
            table(median(table2array(Delta_op_nonlin(:, ...
            abs_absmax_varnames.(comp{1}))), 2), ...
            'VariableNames', {['Median Relative Change in AbsMax ' comp{1}]})];
        Delta_op_nonlin = [Delta_op_nonlin, ...
            table(max(table2array(Delta_op_nonlin(:, ...
            abs_absmax_varnames.(comp{1}))), [], 2), ...
            'VariableNames', {['Max Relative Change in AbsMax ' comp{1}]})];

        Delta_op_nonlin = [Delta_op_nonlin, ...
            table(mean(table2array(Delta_op_nonlin(:, ...
            abs_mean_varnames.(comp{1}))), 2), ...
            'VariableNames', {['Mean Relative Change in Mean ' comp{1}]})];
        Delta_op_nonlin = [Delta_op_nonlin, ...
            table(median(table2array(Delta_op_nonlin(:, ...
            abs_mean_varnames.(comp{1}))), 2), ...
            'VariableNames', {['Median Relative Change in Mean ' comp{1}]})];
        Delta_op_nonlin = [Delta_op_nonlin, ...
            table(max(table2array(Delta_op_nonlin(:, ...
            abs_mean_varnames.(comp{1}))), [], 2), ...
            'VariableNames', {['Max Relative Change in Mean ' comp{1}]})];
    end
    Delta_op_nonlin = sortrows(Delta_op_nonlin, 'Max Relative Change in Mean d', 'descend');
    writetable(Delta_op_nonlin, fullfile(fig_dir, 'ol_dq_rel_change.csv'));
elseif RUN_OL_BLADE
    Delta_op_nonlin = [Delta_op_nonlin, ...
        table(mean(table2array(Delta_op_nonlin(:, ...
        abs_absmax_varnames)), 2), ...
        'VariableNames', {['Mean Relative Change in AbsMax']})];
    Delta_op_nonlin = [Delta_op_nonlin, ...
        table(median(table2array(Delta_op_nonlin(:, ...
        abs_absmax_varnames)), 2), ...
        'VariableNames', {['Median Relative Change in AbsMax']})];
    Delta_op_nonlin = [Delta_op_nonlin, ...
        table(max(table2array(Delta_op_nonlin(:, ...
        abs_absmax_varnames)), [], 2), ...
        'VariableNames', {['Max Relative Change in AbsMax']})];

    Delta_op_nonlin = [Delta_op_nonlin, ...
        table(mean(table2array(Delta_op_nonlin(:, ...
        abs_mean_varnames)), 2), ...
        'VariableNames', {['Mean Relative Change in Mean']})];
    Delta_op_nonlin = [Delta_op_nonlin, ...
        table(median(table2array(Delta_op_nonlin(:, ...
        abs_mean_varnames)), 2), ...
        'VariableNames', {['Median Relative Change in Mean']})];
    Delta_op_nonlin = [Delta_op_nonlin, ...
        table(max(table2array(Delta_op_nonlin(:, ...
        abs_mean_varnames)), [], 2), ...
        'VariableNames', {['Max Relative Change in Mean']})];

    Delta_op_nonlin = sortrows(Delta_op_nonlin, 'Max Relative Change in Mean', 'descend');
    writetable(Delta_op_nonlin, fullfile(fig_dir, 'ol_blade_rel_change.csv'));
end

% plot barchart of relative change in mean/max from before RampStart to
% after for 'big movers'
figure(1);
tiledlayout(2, 1);
title('Relative Change in Mean/Absolute Maximum of Outputs over Disturbance[%]')
test_case_idx = find(WIND_SPEEDS == 14);
ws_suffix = num2str(WIND_SPEEDS(test_case_idx));
axes = [];

if RUN_OL_DQ
    % barchart data on rel change of DQ outputs
    mean_data = table2array(Delta_op_nonlin(:, ...
        {mean_varnames.d{test_case_idx} mean_varnames.q{test_case_idx}}));
    absmax_data = table2array(Delta_op_nonlin(:, ...
        {absmax_varnames.d{test_case_idx} absmax_varnames.q{test_case_idx}}));

    % remove outputs which have nan values in mean or absmax, d or q
    nan_idx = all((~isnan(mean_data)) & (~isnan(absmax_data)), 2);
    OutList_op = OutList_op(nan_idx);
    mean_data = mean_data(nan_idx, :);
    absmax_data = absmax_data(nan_idx, :);

    % only plot outliers
    n_stds = 2;
    outlier_idx = ...
        (any(mean_data > mean(mean_data, 1) + (n_stds * std(mean_data, 1)), 2)) ...
        | (any(mean_data < mean(mean_data, 1) - (n_stds * std(mean_data, 1)), 2)) ...
        | (any(absmax_data > mean(absmax_data, 1) + (n_stds * std(absmax_data, 1)), 2)) ...
        | (any(absmax_data < mean(absmax_data, 1) - (n_stds * std(absmax_data, 1)), 2));
    OutList_op = OutList_op(outlier_idx);
    mean_data = mean_data(outlier_idx, :);
    absmax_data = absmax_data(outlier_idx, :);
    
    % plot mean before/after RampStart
    axes = [axes, nexttile(1)];
    bar(mean_data);
    title('Mean');
    xticks(1:length(OutList_op));
    xticklabels(OutList_op);
    legend('d', 'q');

    % plot max before/after RampStart
    axes = [axes, nexttile(2)];
    bar(absmax_data);
    title('Absolute Maximum');
    xticks(1:length(OutList_op));
    xticklabels(OutList_op);
    legend('d', 'q');
    linkaxes(axes, 'x');

    savefig(fullfile(fig_dir, ['ol_dq_rel_change_' ws_suffix '.fig']));
    saveas(gcf, fullfile(fig_dir, ['ol_dq_rel_change_' ws_suffix '.png']));
elseif RUN_OL_BLADE
    
    % barchart data on rel change of BLADE outputs
    mean_data = table2array(Delta_op_nonlin(:, ...
        {mean_varnames{test_case_idx}}));
    absmax_data = table2array(Delta_op_nonlin(:, ...
        {absmax_varnames{test_case_idx}}));

    % remove outputs which have nan values in mean or absmax, d or q
    nan_idx = all((~isnan(mean_data)) & (~isnan(absmax_data)), 2);
    OutList_op = OutList_op(nan_idx);
    mean_data = mean_data(nan_idx, :);
    absmax_data = absmax_data(nan_idx, :);

    % only plot outliers
    n_stds = 0.75;
    outlier_idx = ...
        (any(mean_data > mean(mean_data, 1) + (n_stds * std(mean_data, 1)), 2)) ...
        | (any(mean_data < mean(mean_data, 1) - (n_stds * std(mean_data, 1)), 2)) ...
        | (any(absmax_data > mean(absmax_data, 1) + (n_stds * std(absmax_data, 1)), 2)) ...
        | (any(absmax_data < mean(absmax_data, 1) - (n_stds * std(absmax_data, 1)), 2));
    OutList_op = OutList_op(outlier_idx);
    mean_data = mean_data(outlier_idx, :);
    absmax_data = absmax_data(outlier_idx, :);

    % plot mean before/after RampStart
    axes = [axes, nexttile(1)];
    bar(mean_data);
    title('Mean');
    xticks(1:length(OutList_op));
    xticklabels(OutList_op);

    % plot max before/after RampStart
    axes = [axes, nexttile(2)];
    bar(absmax_data);
    title('Absolute Maximum');
    xticks(1:length(OutList_op));
    xticklabels(OutList_op);
    linkaxes(axes, 'x');

    savefig(fullfile(fig_dir, ['ol_blade_rel_change_' ws_suffix '.fig']));
    saveas(gcf, fullfile(fig_dir, ['ol_blade_rel_change_' ws_suffix '.png']))
end


%     n_output_plots = 0;
%     for o = 1:length(OutList)
%         op = OutList{o};
%         if ~strcmp(op, 'Time') && ~strcmp(op(end), '2') && ~strcmp(op(end), '3')
%             n_output_plots = n_output_plots + 1;
%         end
%     end

% Blade root bending moment: out of plane, RootMyc1,2,3
% Shaft bending moment (My), LSSTipMya. LSSTipMys
% Yaw bearing yaw moment (Mz&My), YawBrMyp, YawBrMzp, YawBrMyn, YawBrMzn
% Deflection: 
% Tower Base
if RUN_OL_DQ
    OutList_plot = {
        'BldPitchC', 'BldPitchD', 'BldPitchQ', ...
        'RootMycC', 'RootMycD', 'RootMycqQ', ... % Blade 3 out-of-plane moment (i.e., the moment caused by out-of-plane forces) at the blade root
        'OoPDeflC', 'OoPDeflD', 'OoPDeflQ'};
%             'GenSpeed', ...
%             'TwrBsMyt', ... % Tower base pitching (or fore-aft) moment (i.e., the moment caused by fore-aft forces)
%             'YawBrMyp', ... % Nonrotating tower-top / yaw bearing pitch moment
%             'YawBrMzp', ... % Tower-top / yaw bearing yaw moment
%             'LSSGagMya' % Nonrotating low-speed shaft bending moment at the shaft tip (teeter pin for 2-blader, apex of rotation for 3-blader)
%         };
%             'IPDeflC', 'IPDeflD', 'IPDeflQ'};
    n_output_plots = 3;

    time_data = getData(sim_out_list(1).OutData.signals.values, OutList, 'Time');
    
    figure(2);
    tiledlayout(n_output_plots, 1);
    axs = [];
   
    
    for op_label = OutList_plot
        if strcmp(op_label{1}(end), 'C')
            axs = [axs, nexttile];
            
            cdq_op_labels = cellfun(@(q) [op_label{1}(1:end-1) q], ...
                {'C', 'D', 'Q'}, 'UniformOutput', false);
            for cdq_label = cdq_op_labels
                plot(time_data, getData(sim_out_list(test_case_idx).OutData.signals.dqValues, dqOutList, cdq_label{1})); hold on;
            end
 
            plot([dRampStart, dRampStart], ylim, 'k--');
            plot([dRampStop, dRampStop], ylim, 'k--');
            plot([qRampStart, qRampStart], ylim, 'k--');
            plot([qRampStop, qRampStop], ylim, 'k--');
            hold off;
            ylabel(op_label{1}(1:end-1));
            legend('c', 'd', 'q');
        elseif ~strcmp(op_label{1}(end), 'D') && ~strcmp(op_label{1}(end), 'Q')
            axs = [axs, nexttile];
            plot(time_data, getData(sim_out_list(test_case_idx).OutData.signals.dqValues, dqOutList, op_label{1}));
            hold on;
            plot([dRampStart, dRampStart], ylim, 'k--');
            plot([dRampStop, dRampStop], ylim, 'k--');
            plot([qRampStart, qRampStart], ylim, 'k--');
            plot([qRampStop, qRampStop], ylim, 'k--');
            hold off;
            ylabel(op_label{1});
        end
    end
    linkaxes(axs, 'x');
    xlabel('Time [s]');
    xlim([0, max(time_data, [], 1)]);

    savefig(fullfile(fig_dir, ['ol_dq_ts_' ws_suffix '.fig']));
    saveas(gcf, fullfile(fig_dir, ['ol_dq_ts_' ws_suffix '.png']));
elseif RUN_OL_BLADE
    OutList_plot = {
        'BldPitch1', 'BldPitch2', 'BldPitch3', ...
        'RootMyc1', 'RootMyc2', 'RootMycq3', ... % Blade 3 out-of-plane moment (i.e., the moment caused by out-of-plane forces) at the blade root
        'OoPDefl1', 'OoPDefl2', 'OoPDefl3'};
        %'GenSpeed', ...
        %'TwrBsMyt', ... % Tower base pitching (or fore-aft) moment (i.e., the moment caused by fore-aft forces)
        %'YawBrMyp', ... % Nonrotating tower-top / yaw bearing pitch moment
        %'YawBrMzp', ... % Tower-top / yaw bearing yaw moment
        %'LSSGagMya' % Nonrotating low-speed shaft bending moment at the shaft tip (teeter pin for 2-blader, apex of rotation for 3-blader)
%             };
    %'TwrBsFxt', TwrBsMxt, 
%         'OoPDefl1', 'OoPDefl2', 'OoPDefl3', ...
%         'IPDefl1', 'IPDefl2', 'IPDefl3'};
    n_output_plots = 3;

    time_data = getData(sim_out_list(1).OutData.signals.values, OutList, 'Time');
    
    figure(2);
    tiledlayout(n_output_plots, 1);
    axs = [];
    
    for op_label = OutList_plot
        if strcmp(op_label{1}(end), '1')
            axs = [axs, nexttile];
            
            blade_op_labels = cellfun(@(q) [op_label{1}(1:end-1) q], ...
                {'1', '2', '3'}, 'UniformOutput', false);
            for blade_label = blade_op_labels
                plot(time_data, getData(sim_out_list(test_case_idx).OutData.signals.values, OutList, blade_label{1})); hold on;
            end
            plot([b1RampStart, b1RampStart], ylim, 'k--');
            plot([b1RampStop, b1RampStop], ylim, 'k--');
            hold off;
            ylabel(op_label{1}(1:end-1));
            legend('1', '2', '3');
        elseif ~strcmp(op_label{1}(end), '2') && ~strcmp(op_label{1}(end), '3')
            axs = [axs, nexttile];
            plot(time_data, getData(sim_out_list(test_case_idx).OutData.signals.values, OutList, op_label{1}));
            hold on;
            plot([b1RampStart, b1RampStart], ylim, 'k--');
            plot([b1RampStop, b1RampStop], ylim, 'k--');
            hold off;
            ylabel(op_label{1});
        end
    end
    linkaxes(axs, 'x');
    xlabel('Time [s]')
    xlim([0, max(time_data, [], 1)]);

    savefig(fullfile(fig_dir, ['ol_blade_ts_' ws_suffix '.fig']));
    saveas(gcf, fullfile(fig_dir, ['ol_blade_ts_' ws_suffix '.png']));
end
