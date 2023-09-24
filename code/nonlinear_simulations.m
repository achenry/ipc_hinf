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
init_hinf_controller;

RUN_SIMS_PAR = 1;
RUN_SIMS_SINGLE = 0;
RUN_TURSIM = 0;
GENERATE_CASES = 0;
GENERATE_FASTINPUT_FILES = 0;

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

if GENERATE_CASES
    generate_simulation_cases;
else
    if MAX_SIMULATIONS > -1
        save_prefix = 'debug_';
    else
        save_prefix = '';
    end

    if OPTIMAL_K_COLLECTION
        load(fullfile(mat_save_dir, [save_prefix, 'Optimal_Controllers_nonlinear_simulation_case_list.mat']));
    elseif EXTREME_K_COLLECTION
        load(fullfile(mat_save_dir, [save_prefix, 'Extreme_Controllers_nonlinear_simulation_case_list.mat']));
    elseif BASELINE_K
        load(fullfile(mat_save_dir, [save_prefix, 'Baseline_Controller_nonlinear_simulation_case_list.mat']));
    elseif STRUCT_PARAM_SWEEP
        load(fullfile(mat_save_dir, [save_prefix, 'Structured_Controllers_nonlinear_simulation_case_list.mat']));
    end
    n_cases = length(case_list);
end

%% Generate OpenFAST input files for each case
if GENERATE_FASTINPUT_FILES
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
    for case_idx=1:n_cases

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
end

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
            
            SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean_FullOrderControllerTest_old';
            K_IPC = c2d(case_list(case_idx).Controller, DT);
            sim_inputs(case_idx) = Simulink.SimulationInput(SL_model_name);
            sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('K_IPC', K_IPC);
        elseif OPTIMAL_K_COLLECTION || EXTREME_K_COLLECTION
            
            if strcmp(case_list(case_idx).Structure.x, 'Full-Order')
                SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean_FullOrderControllerTest_old';
            elseif strcmp(case_list(case_idx).Structure.x, 'Structured')
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
            % SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean';
            SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean_FullOrderControllerTest_old';
            K_IPC = case_list(case_idx).Controller;
            sim_inputs(case_idx) = Simulink.SimulationInput(SL_model_name);
            sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('K_IPC', K_IPC);
        elseif ~USE_IPC
            SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean';
            sim_inputs(case_idx) = Simulink.SimulationInput(SL_model_name);
        end
        
        [filepath, name, ext] = fileparts(strrep(case_list(case_idx).FAST_InputFileName, '.fst', ''));
        outdata_save_fn = fullfile(sl_save_dir, [name, '_', sim_type, '_outdata']);
        blpitch_save_fn = fullfile(sl_save_dir, [name, '_', sim_type, '_blpitch']);

        case_list(case_idx).outdata_save_fn = outdata_save_fn;
        case_list(case_idx).blpitch_save_fn = blpitch_save_fn;

        sim_inputs(case_idx) = setBlockParameter(sim_inputs(case_idx), ...
            [SL_model_name, ...
            '/To File'], 'Filename', outdata_save_fn);
        sim_inputs(case_idx) = setBlockParameter(sim_inputs(case_idx), ...
            [SL_model_name, ...
            '/Baseline Controller/Cyclic Pitch controller/1P Cyclic Pitch Controller1', ...
            '/To File'], 'Filename', blpitch_save_fn);

        sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('TMax', TMax);
        sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('DT', Simulation.DT);
        sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('HWindSpeed', case_list(case_idx).InflowWind.HWindSpeed);
        sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('RootMyc_ref', [case_list(case_idx).Reference.d case_list(case_idx).Reference.q]);
        sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('BldPitch_sat', [case_list(case_idx).Saturation.d case_list(case_idx).Saturation.q]);
        
    end

    %% Run simulations in multiple parallel threads
    
    sim_out_list = parsim(sim_inputs, ...
                   'TransferBaseWorkspaceVariables', true, ... % Run simulation
                   'RunInBackground', 'off', ...
                   'ShowProgress',  'on', ...
                   'ShowSimulationManager', 'off');
    for case_idx = 1:n_cases
        sim_out_list(case_idx).InflowWind = case_list(case_idx).InflowWind;
        sim_out_list(case_idx).outdata_save_fn = case_list(case_idx).outdata_save_fn;
        sim_out_list(case_idx).blpitch_save_fn = case_list(case_idx).blpitch_save_fn;
        sim_out_list(case_idx).FAST_InputFileName = case_list(case_idx).FAST_InputFileName;
    end

    
elseif RUN_SIMS_SINGLE
    % run single case
    for case_idx = 1:1
        FAST_InputFileName = fullfile(FAST_runDirectory, ...
            [case_name_list{case_idx}, '.fst']);
        DT = Simulation.DT;
        HWindSpeed = case_list(case_idx).InflowWind.HWindSpeed;
        RootMyc_ref = [case_list(case_idx).Reference.d case_list(case_idx).Reference.q];
        BldPitch_sat = [case_list(case_idx).Saturation.d case_list(case_idx).Saturation.q];

        % save_fn = strrep(FAST_InputFileName, '.fst', '');
        
        if STRUCT_PARAM_SWEEP
            % SL_model_name = 'AD_SOAR_c7_V2f_c73_MIMOPIControllerTest';
             SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean_FullOrderControllerTest_old';
            % K_IPC = c2d(tf(case_list(case_idx).Controller_scaled(:, :, ...
            %     LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED)), DT);
            K_IPC = c2d(case_list(case_idx).Controller, DT);
            % K_IPC(2, 2).Numerator(2) / K_IPC(2, 2).Denominator(1) 
        elseif OPTIMAL_K_COLLECTION || EXTREME_K_COLLECTION
            if strcmp(case_list(case_idx).Structure.x, 'Full-Order')
                SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean_FullOrderControllerTest_old';
            elseif strcmp(case_list(case_idx).Structure.x, 'Structured')
                SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean_StructuredControllerTest';
            end
            % QUESTION MANUEL does it make sense to implement negative of
            % tuned controller which assumed negative feedback since SL
            % simulation assumes positive feedback?
            K_IPC = c2d(case_list(case_idx).Controller_scaled, DT); % Note, this is the scaled controller
        elseif BASELINE_K
            SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean_FullOrderControllerTest_old';
            K_IPC = case_list(case_idx).Controller;
        else
            SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean';
        end
        open_system(fullfile(FAST_SimulinkModel_dir, SL_model_name))
        
        [filepath, name, ext] = fileparts(strrep(FAST_InputFileName, '.fst', ''));
        outdata_save_fn = fullfile(sl_save_dir, [name, '_', sim_type, '_outdata']);
        blpitch_save_fn = fullfile(sl_save_dir, [name, '_', sim_type, '_blpitch']);
        set_param([SL_model_name, '/To File'], 'Filename', outdata_save_fn);
        set_param([SL_model_name, ...
            '/Baseline Controller/Cyclic Pitch controller/1P Cyclic Pitch Controller1', ...
            '/To File'], 'Filename', blpitch_save_fn);

        sim(fullfile(FAST_SimulinkModel_dir, ...
                SL_model_name), [0, TMax]);

        sim_out_list(case_idx).outdata_save_fn = outdata_save_fn;
        sim_out_list(case_idx).blpitch_save_fn = blpitch_save_fn;
        sim_out_list(case_idx).InflowWind = case_list(case_idx).InflowWind;
        sim_out_list(case_idx).FAST_InputFileName = FAST_InputFileName;
    end
end

%% Save Simulation Data

if RUN_SIMS_PAR || RUN_SIMS_SINGLE
    save(fullfile(sl_metadata_save_dir, ['sim_out_list_', sim_type, '.mat']), 'sim_out_list', '-v7.3');
end
