%% Run nonlinear simulations for different controllers in Simulink

% TODO, compute blade-pitch saturation as percentage of peak values in IPC
% from first rotation of wind turbine after transients have settled OR
% compute reference beta_sat values based on singel AWu=7,5
% TODO compute reference Md_ref values based on singel AWu=7.5
% TODO run varying_wu case for higher range of AWu
% TODO when plotting change in ADC, make sure to choose case that also
% reduces loads
% TODO compare results of controllers tuned for different wind speeds
% TODO test results over Region 2.5 where greatest loads occur
% TODO run all of this on desktop with 96 cores
% TODO plot Beta_ipc signals for baseline and different tuning methods with
% ADC, tracking error attenuation values

init_hinf_controller;

single_run_case_idx = 45;

%% Generate Turbsim Files
if RUN_TURBSIM
    run_turbsim_multi;
    % [velocity, twrVelocity, y, z, zTwr, nz, ny, dz, dy, dt, zHub, z1,mffws] = readTSgrid(fullfile(windfiles_dir, 'A_16_1.bts'));
    % plot(1:21, squeeze(mean(velocity(:, 1, 11, :), 1))); hold on;
    plot(1:21, 16*exp(0.2 * (z/zHub)))
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
    
    load(fullfile(mat_save_dir, [sim_type, '_Controllers_nonlinear_simulation_case_list.mat']));
    n_cases = length(case_list);
end

% for jdx = 1:length(case_list)
%     for idx = 1:length(case_basis.WeGain.x)
%         % x = (case_basis.WeGain.x{idx}) - case_list(jdx).WeGain;
%         x = (case_basis.WeGain.x{idx}) - case_list(jdx).WeGain;
%         if (sum(x(1,1).Numerator{1,1}) == 0) && (case_list(jdx).WuGain(1,1).Numerator{1,1} == 1)
%             idx
%             jdx
%             x(1,1)
%             break;
%         end
%     end
% end

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
    for case_idx = single_run_case_idx:single_run_case_idx
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

if RUN_SIMS_PAR
    cd(project_dir);
    % sim_inputs = repmat(struct(), n_cases, 1 );
    for case_idx = 1:n_cases
        % for case_idx = [46, 48, 136, 138]
        case_list(case_idx).FAST_InputFileName = fullfile(FAST_runDirectory, ...
            [case_name_list{case_idx}, '.fst']);
        
        % Populate thread parameters
        SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean_FullOrderControllerTest_old';

        if BASELINE_K || STRUCT_PARAM_SWEEP
            K_IPC = c2d(case_list(case_idx).Controller, DT);
        else
            K_IPC = c2d(case_list(case_idx).Controller_scaled, DT);
        end

        sim_inputs(case_idx) = Simulink.SimulationInput(SL_model_name);
        sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('K_IPC', K_IPC);

        [~, x, ~] = fileparts(case_list(case_idx).InflowWind.FileName_BTS);
        x = split(x, '_');
        sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('WindMean', str2double(x{2}));
        
        [filepath, name, ext] = fileparts(strrep(case_list(case_idx).FAST_InputFileName, '.fst', ''));
        outdata_save_fn = fullfile(sl_save_dir, [name, '_', sim_type, '_outdata']);
        blpitch_save_fn = fullfile(sl_save_dir, [name, '_', sim_type, '_blpitch']);
        rootmyc_save_fn = fullfile(sl_save_dir, [name, '_', sim_type, '_rootmyc']);

        case_list(case_idx).outdata_save_fn = outdata_save_fn;
        case_list(case_idx).blpitch_save_fn = blpitch_save_fn;
        case_list(case_idx).rootmyc_save_fn = rootmyc_save_fn;

        sim_inputs(case_idx) = setBlockParameter(sim_inputs(case_idx), ...
            [SL_model_name, ...
            '/To File'], 'Filename', outdata_save_fn);
        % sim_inputs(case_idx) = setBlockParameter(sim_inputs(case_idx), ...
        %     [SL_model_name, ...
        %     '/Baseline Controller/Cyclic Pitch controller/1P Cyclic Pitch Controller1', ...
        %     '/To File'], 'Filename', blpitch_save_fn);
        sim_inputs(case_idx) = setBlockParameter(sim_inputs(case_idx), ...
            [SL_model_name, ...
            '/Baseline Controller/To File'], 'Filename', blpitch_save_fn);
        sim_inputs(case_idx) = setBlockParameter(sim_inputs(case_idx), ...
            [SL_model_name, ...
            '/Baseline Controller/Cyclic Pitch controller/1P Cyclic Pitch Controller1', ...
            '/To File1'], 'Filename', rootmyc_save_fn);
        
        sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('FAST_InputFileName', case_list(case_idx).FAST_InputFileName);
        sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('TMax', TMax);
        sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('DT', Simulation.DT);
        sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('HWindSpeed', case_list(case_idx).InflowWind.HWindSpeed);
        % sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('Scheduling', case_list(case_idx).Scheduling);
        sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('Scheduling', SCHEDULING);

        % sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('RootMyc_ref', [case_list(case_idx).Reference.d case_list(case_idx).Reference.q]);
        % sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('BldPitch_sat', [case_list(case_idx).Saturation.d case_list(case_idx).Saturation.q]);
        if VARY_REFERENCE
            sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('RootMyc_ref', case_list(case_idx).Reference * M_dq_reference);
        else
            sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('RootMyc_ref', [0, 0]);
        end

        if VARY_SATURATION
            % sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('BldPitch_sat', case_list(case_idx).Saturation * Beta_dq_saturation);
            sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('BldPitch_sat', case_list(case_idx).Saturation * Beta_ipc_blade_saturation);
        else
            sim_inputs(case_idx) = sim_inputs(case_idx).setVariable('BldPitch_sat', Inf);
        end
        
    end

    %% Run simulations in multiple parallel threads
    
    sim_out_list = parsim(sim_inputs, ...
                   'TransferBaseWorkspaceVariables', true, ... % Run simulation
                   'RunInBackground', 'off', ...
                   'ShowProgress',  'on', ...
                   'ShowSimulationManager', 'off');

    for case_idx = 1:n_cases
    % for case_idx = [46, 48, 136, 138]
        % if case_list(case_idx).Kp_diag == 0
        %     case_list(case_idx).CaseDesc = "noipc";
        % else
        %     case_list(case_idx).CaseDesc = "baseline_controller";
        % end
        sim_out_list(case_idx).CaseDesc = case_list(case_idx).CaseDesc;
        sim_out_list(case_idx).InflowWind = case_list(case_idx).InflowWind;
        sim_out_list(case_idx).outdata_save_fn = case_list(case_idx).outdata_save_fn;
        sim_out_list(case_idx).blpitch_save_fn = case_list(case_idx).blpitch_save_fn;
        sim_out_list(case_idx).rootmyc_save_fn = case_list(case_idx).rootmyc_save_fn;
        sim_out_list(case_idx).FAST_InputFileName = case_list(case_idx).FAST_InputFileName;
    end
    
elseif RUN_SIMS_SINGLE
    % run single case
    
    for case_idx = single_run_case_idx:single_run_case_idx
    % for case_idx = [46, 48, 136, 138]
        FAST_InputFileName = fullfile(FAST_runDirectory, ...
            [case_name_list{case_idx}, '.fst']);
        [~, x, ~] = fileparts(case_list(case_idx).InflowWind.FileName_BTS);
        x = split(x, '_');
        WindMean = str2double(x{2});

        DT = Simulation.DT;
        HWindSpeed = case_list(case_idx).InflowWind.HWindSpeed;
        % Scheduling = case_list(case_idx).Scheduling;
        Scheduling = SCHEDULING;

        if VARY_REFERENCE
            RootMyc_ref = case_list(case_idx).Reference * M_dq_reference;
        else
            RootMyc_ref = [0, 0];
        end
        
        if VARY_SATURATION
            % BldPitch_sat = case_list(case_idx).Saturation * Beta_dq_saturation;
            BldPitch_sat = case_list(case_idx).Saturation * Beta_ipc_blade_saturation;
        else
            BldPitch_sat = Inf;
        end

        SL_model_name = 'AD_SOAR_c7_V2f_c73_Clean_FullOrderControllerTest_old';
        if BASELINE_K || STRUCT_PARAM_SWEEP
            K_IPC = c2d(case_list(case_idx).Controller, DT);
        else
            K_IPC = c2d(case_list(case_idx).Controller_scaled, DT);
        end

        % figure(1); bode(K_IPC(:, :, 3), bode_plot_opt); hold on;

        open_system(fullfile(FAST_SimulinkModel_dir, SL_model_name))
        
        [filepath, name, ext] = fileparts(strrep(FAST_InputFileName, '.fst', ''));
        outdata_save_fn = fullfile(sl_save_dir, [name, '_', sim_type, '_outdata']);
        blpitch_save_fn = fullfile(sl_save_dir, [name, '_', sim_type, '_blpitch']);
        rootmyc_save_fn = fullfile(sl_save_dir, [name, '_', sim_type, '_rootmyc']);
        
        set_param([SL_model_name, '/To File'], 'Filename', outdata_save_fn);
        set_param([SL_model_name, '/Baseline Controller/To File'], ...
            'Filename', blpitch_save_fn);
        % set_param([SL_model_name, ...
        %     '/Baseline Controller/Cyclic Pitch controller/1P Cyclic Pitch Controller1', ...
        %     '/To File'], 'Filename', blpitch_save_fn);
        set_param([SL_model_name, ...
            '/Baseline Controller/Cyclic Pitch controller/1P Cyclic Pitch Controller1', ...
            '/To File1'], 'Filename', rootmyc_save_fn);

        sim(fullfile(FAST_SimulinkModel_dir, ...
                SL_model_name), [0, TMax]);

        % if case_list(case_idx).Kp_diag == 0
        %     case_list(case_idx).CaseDesc = "noipc";
        % else
        %     case_list(case_idx).CaseDesc = "baseline_controller";
        % end
        
        sim_out_list(case_idx).CaseDesc = case_list(case_idx).CaseDesc;
        sim_out_list(case_idx).outdata_save_fn = outdata_save_fn;
        sim_out_list(case_idx).blpitch_save_fn = blpitch_save_fn;
        sim_out_list(case_idx).rootmyc_save_fn = rootmyc_save_fn;
        sim_out_list(case_idx).InflowWind = case_list(case_idx).InflowWind;
        sim_out_list(case_idx).FAST_InputFileName = FAST_InputFileName;
    end
end

if 0
    
    idx = 11;
    noipc_vals = load(fullfile(sl_save_dir, ['case_', num2str(idx), '_baseline_k_turbsim_outdata_', num2str(idx)]));
    noipc_vals = noipc_vals.OutData';
    noipc_vals = noipc_vals(floor(cut_transients / DT):end, 2:end);
    RootMyc1 = getData(noipc_vals, OutList, 'RootMyc1');
    rmse = sqrt((1 / length(RootMyc1)) * sum(RootMyc1.^2))
    beta_dot_norm = @(beta_dot) ((beta_dot >= 0) * 5) + ((beta_dot < 0) * (-4));
    BldPitch1 = getData(noipc_vals, OutList, 'BldPitch1');
    dBldPitch1dt = diff(BldPitch1) / DT;
    adc = (1 / length(dBldPitch1dt)) * sum((dBldPitch1dt ./ beta_dot_norm(dBldPitch1dt)))

    vals = load(fullfile(sl_save_dir, ['case_', num2str(idx), '_baseline_k_turbsim_rootmyc_', num2str(idx)]));
    vals = vals.RootMyc';
    vals = vals(floor(cut_transients / DT):end, 2:end);
    t = getData(vals, OutList, 'Time');
    RootMycDQ = vals(:, 1:2);

    figure(3); subplot(2, 1, 1); plot(t, RootMycDQ(:, 1)); hold on;
    subplot(2, 1, 2); plot(t, RootMycDQ(:, 2)); hold on;

    idx = 50;
    infw_fn = 'A_16_1';
    baseline_sim_out_list = load(fullfile(sl_metadata_save_dir, ['sim_out_list_', 'baseline_k_turbsim', '.mat'])).sim_out_list;
    infw_filenames = cell(length(baseline_sim_out_list), 1);
    [infw_filenames{:}] = baseline_sim_out_list.InflowWind.FileName_BTS;
    baseline_types = cell(length(baseline_sim_out_list), 1);
    [baseline_types{:}] = baseline_sim_out_list.CaseDesc;
    % find(contains(infw_filenames, infw_fn) & strcmp(baseline_types{:}, "baseline_controller"))
    %   2.8785e+04, 0.1485
    baseline_vals = load(fullfile(sl_save_dir, ['case_', num2str(idx), '_baseline_k_turbsim_outdata_', num2str(idx)]));
    baseline_vals = baseline_vals.OutData';
    baseline_vals = baseline_vals(floor(cut_transients / DT):end, 2:end);
    t = getData(baseline_vals, OutList, 'Time');
    RootMyc1 = getData(baseline_vals, OutList, 'RootMyc1');
    rmse.baseline = sqrt((1 / length(RootMyc1)) * sum(RootMyc1.^2))
    beta_dot_norm = @(beta_dot) ((beta_dot >= 0) * 5) + ((beta_dot < 0) * (-4));
    BldPitch1 = getData(baseline_vals, OutList, 'BldPitch1');
    dBldPitch1dt = diff(BldPitch1) / DT;
    adc.baseline = (1 / length(dBldPitch1dt)) * sum((dBldPitch1dt ./ beta_dot_norm(dBldPitch1dt)))
    figure(2);
    baseline_case_list = load(fullfile(mat_save_dir, ['baseline_k_turbsim', '_Controllers_nonlinear_simulation_case_list.mat']));
    baseline_case_list = baseline_case_list.case_list;

    vals = load(fullfile(sl_save_dir, ['case_', num2str(idx), '_baseline_k_turbsim_rootmyc_', num2str(idx)]));
    vals = vals.RootMyc';
    vals = vals(floor(cut_transients / DT):end, 2:end);
    RootMycDQ = vals(:, 1:2);

    figure(3); subplot(2, 1, 1); plot(t, RootMycDQ(:, 1)); hold on;
    subplot(2, 1, 2); plot(t, RootMycDQ(:, 2)); hold on;

    figure(2);
    bodeplot(baseline_case_list(idx).Controller(:,:,3), bode_plot_opt); hold on;

    for idx = [3, 6, 9, 12, 15]
        case_list(idx).CaseDesc
    end
    % sim_out_list = sim_out_list.controller;
    for idx = 1:length(sim_out_list)
        % if strcmp(sim_out_list(idx).CaseDesc, 'adc W1 = 1 W2 = 0.01 ->  Wu = 7.5 We = 0.1 Beta_dq_sat = [0.2, 0.2]') || ...
        %         strcmp(sim_out_list(idx).InflowWind.FileName_BTS, 'A_16_1')
        %     continue;
        % end

    % for idx  = [46, 48, 136, 138]
        % idx = 45;
        vals = load([sim_out_list(idx).outdata_save_fn, '_', num2str(idx)]);
        % vals = load(sim_out_list(idx).outdata_save_fn);
        vals = vals.OutData';
        vals = vals(floor(cut_transients / DT):end, 2:end);

        t = getData(vals, OutList, 'Time');
        RootMyc1 = getData(vals, OutList, 'RootMyc1');


        beta_dot_norm = @(beta_dot) ((beta_dot >= 0) * 5) + ((beta_dot < 0) * (-4));
        BldPitch1 = getData(vals, OutList, 'BldPitch1');
        dBldPitch1dt = diff(BldPitch1) / DT;
        

        vals = load([sim_out_list(idx).blpitch_save_fn, '_', num2str(idx)]);
        %vals = load([sim_out_list(idx).blpitch_save_fn]);
        vals = vals.BlPitch';
        vals = vals(floor(cut_transients / DT):end, 2:end);
        BldPitch_ipc = vals(:, 1:3);
        BldPitch_ipc_sat = vals(:, 4:6);


        if sum(BldPitch_ipc) == 0
            continue;
        end
        sim_out_list(idx).CaseDesc
        % if any(sim_out_list)
        %       sim_out_list(idx).CaseDesc
        %     sim_out_list(idx).InflowWind.FileName_BTS
        % end
        % if any(vals(:, 3:4))

        vals = load([sim_out_list(idx).rootmyc_save_fn, '_', num2str(idx)]);
        % vals = load([sim_out_list(idx).rootmyc_save_fn]);
        vals = vals.RootMyc';
        vals = vals(floor(cut_transients / DT):end, 2:end);
        RootMycDQ = vals(:, 1:2);
        
        rmse = sqrt((1 / length(RootMyc1)) * sum(RootMyc1.^2));
        if contains(sim_out_list(case_idx).InflowWind.FileName_BTS, 'A_16_1') % && (rmse < 3*1e4)
            idx
            rmse
            adc = (1 / length(dBldPitch1dt)) * sum((dBldPitch1dt ./ beta_dot_norm(dBldPitch1dt)))
        end
        figure(3); 
        % subplot(2, 1, 1); plot(t, RootMycDQ(:, 1)); hold on;
        % subplot(2, 1, 2); plot(t, RootMycDQ(:, 2)); hold on;
        % subplot(2, 1, 1); subtitle('D'); xlim([100, TMax])
        % subplot(2, 1, 2); subtitle('Q'); xlim([100, TMax])
        % legend('Open-Loop', 'Baseline PI', 'Tuned Full-Order');
        plot(t, BldPitch_ipc);
        
        
        % figure(1);
        % subplot(2, 2, 1); plot(t, RootMyc1); hold on;
        % subplot(2, 2, 2); plot(t, RootMycDQ); hold on;
        % subplot(2, 2, 3); plot(t, BldPitch1); hold on;
        % subplot(2, 2, 4); plot(t, BldPitchDQ); hold on;
        % % 2.2733e+04 0.1362 for single
        % figure(2);
        % bodeplot(case_list(idx).Controller_scaled(:,:,3), bode_plot_opt); hold on;
    end
    % esp 9 and 10, or 16 and 15 in case_basis.WeGain.x
    % for jdx = [3,4,5,6,9,10,11,12,15,16,17,18]
    %     for idx = 1:length(case_basis.WeGain.x)
    %         % x = (case_basis.WeGain.x{idx}) - case_list(jdx).WeGain;
    %         x = (case_basis.WeGain.x{idx}) - case_list(jdx).WeGain;
    %         if (sum(x(1,1).Numerator{1,1}) == 0) && (case_list(jdx).WuGain(1,1).Numerator{1,1} == 1)
    %             idx
    %             jdx
    %             x(1,1)
    %             break;
    %         end
    %     end
    % end
end

%% Compute DELs
if n_seeds == 5 && (BASELINE_K || OPTIMAL_K_COLLECTION)
    % sim_type = 'optimal_k_cases_turbsim_wu';
    status = system(['/Users/aoifework/miniconda3/envs/weis_dev/bin/python3 ', fullfile(project_dir, 'postprocessing', 'main.py'), ' -st ', sim_type]);
    % sim_type = 'optimal_k_cases_turbsim_ref';
    % status = system(['/Users/aoifework/miniconda3/envs/weis_dev/bin/python3 ', fullfile(project_dir, 'postprocessing', 'main.py'), ' -st ', sim_type]);
    % sim_type = 'optimal_k_cases_turbsim_sat';
    % status = system(['/Users/aoifework/miniconda3/envs/weis_dev/bin/python3 ', fullfile(project_dir, 'postprocessing', 'main.py'), ' -st ', sim_type]);
end

%% Save Simulation Data

if RUN_SIMS_PAR || RUN_SIMS_SINGLE
    save(fullfile(sl_metadata_save_dir, ['sim_out_list_', sim_type, '.mat']), 'sim_out_list', '-v7.3');
end

chdir(code_dir);
