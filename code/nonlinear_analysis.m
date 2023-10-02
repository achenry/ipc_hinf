%% Run nonlinear simulations for different controllers in Simulink
%   compute power spectra of different loads
%   compute ADC of blade-pitch

% compute power spectra of different rotating and non-rotating loads for
% different controllers
% compute ADC for different controllers
% plot Beta, BetaDot, BetaDDdot for different controllers

% run a parfor loop to simulate 10 minutes of a turbsim wind field for each
% controller type with IPC on

initialize;
single_run_case_idx = 100;

COMPUTE_FFT = 0;

if OPTIMAL_K_COLLECTION
    load(fullfile(mat_save_dir, 'Optimal_Controllers_nonlinear_simulation_case_list.mat')); % case list
elseif EXTREME_K_COLLECTION
    load(fullfile(mat_save_dir, 'Extreme_Controllers_nonlinear_simulation_case_list.mat'));
elseif BASELINE_K
    load(fullfile(mat_save_dir, 'Baseline_Controller_nonlinear_simulation_case_list.mat'));
elseif STRUCT_PARAM_SWEEP
    load(fullfile(mat_save_dir, 'Structured_Controllers_nonlinear_simulation_case_list.mat'));
end

if 0
A = permute(ipc_ss.A.Data, [3, 1, 2]);
B = permute(ipc_ss.B.Data, [3, 1, 2]);
C = permute(ipc_ss.C.Data, [3, 1, 2]);
D = permute(ipc_ss.D.Data, [3, 1, 2]);
figure;
tiledlayout(2, 2);
nexttile(1);
for row = size(A, 2)
    plot(A(:, row, :));
    hold on;
end
hold off;
nexttile(2);
for row = size(B, 2)
    plot(B(:, row, :));
    hold on;
end
nexttile(3);
for row = size(C, 2)
    plot(C(:, row, :));
    hold on;
end
hold off;
nexttile(4);
for row = size(D, 2)
    plot(D(:, row, :));
    hold on;
end
hold off;
end

%% Load Simulation Data

if RUN_CL && strcmp(WIND_TYPE, 'turbsim')
    if STRUCT_PARAM_SWEEP
        sim_out_list.controller = load(fullfile(sl_metadata_save_dir, 'sim_out_list_pi_param_sweep_turbsim.mat')); % load ipc case
        sim_out_list.controller = sim_out_list.controller.sim_out_list;
    elseif OPTIMAL_K_COLLECTION
        sim_out_list.controller = load(fullfile(sl_metadata_save_dir, 'sim_out_list_optimal_k_cases_turbsim.mat'));
        sim_out_list.controller = sim_out_list.controller.sim_out_list;
    elseif EXTREME_K_COLLECTION
        sim_out_list.controller = load(fullfile(sl_metadata_save_dir, 'sim_out_list_extreme_k_cases_turbsim.mat')); % load ipc case
        sim_out_list.controller = sim_out_list.controller.sim_out_list;
    end
    
    untuned_sim_out_list = load(fullfile(sl_metadata_save_dir, 'sim_out_list_baseline_k_turbsim.mat')).sim_out_list; % load noipc case
    untuned_case_list = load(fullfile(mat_save_dir, "Baseline_Controller_nonlinear_simulation_case_list.mat")).case_list;
    noipc_idx = 1;
    baseline_idx = 1;
    for case_idx = 1:length(untuned_sim_out_list)
        if sum(untuned_case_list(case_idx).Controller.C, 'all') + sum(untuned_case_list(case_idx).Controller.D, 'all') == 0
            sim_out_list.noipc(noipc_idx) = untuned_sim_out_list(case_idx);
            noipc_idx = noipc_idx + 1;
        else
            sim_out_list.baseline_controller(baseline_idx) = untuned_sim_out_list(case_idx);
            baseline_idx = baseline_idx + 1;
        end
    end
    clear untuned_sim_out_list untuned_case_list
else
    if RUN_OL_DQ
        % get open-loop sensitivity to IPC simulations from memory
        load(fullfile(sl_metadata_save_dir, 'sim_out_list_ol_dq.mat'));
    elseif RUN_OL_BLADE
        % get open-loop sensitivity to IPC simulations from memory
        load(fullfile(sl_metadata_save_dir, 'sim_out_list_ol_blade.mat'));
    end
end

%% Analysis of Blade-Pitch Actuation and Loads in Time-Domain
if 1
% if EXTREME_K_COLLECTION || OPTIMAL_K_COLLECTION || STRUCT_PARAM_SWEEP
    beta_dot_norm = @(beta_dot) ((beta_dot >= 0) * 5) + ((beta_dot < 0) * (-4)); % TODO get correct values
    
    % want to compare all cases for the same wind field mean speed and
    % seed, choose first in case_basis for comparison
    
    % OPTIMAL_K_COLLECTION
    % if EXTREME_K_COLLECTION
    %     bts_filename = sim_out_list.controller(1).InflowWind.FileName_BTS{1};
    %     ux = sim_out_list.controller(1).InflowWind.HWindSpeed(1);
    % elseif OPTIMAL_K_COLLECTION
    % end

    % dq and blade values of blade-pitch and blade root bending moment for
    % particular wind field

    % Make table comparing tuned controllers (no nonlinear metrics yet)
    if OPTIMAL_K_COLLECTION
        load(fullfile(mat_save_dir, 'Optimal_Controllers_case_table.mat')); % Controllers_case_table: tuned controllers, no data from nonlinear simulations yet
    elseif EXTREME_K_COLLECTION
        load(fullfile(mat_save_dir, 'Extreme_Controllers_case_table.mat'));
    elseif STRUCT_PARAM_SWEEP
        load(fullfile(mat_save_dir, 'PI_ParameterSweep_redtable.mat'));
        Controllers_case_table = PI_ParameterSweep_redtable;
    end
    lin_cols = Controllers_case_table.Properties.VariableNames;
    lin_cols(ismember(lin_cols, {'ADC', 'RootMycBlade1 RMSE', 'Case No.', 'Case Desc.', 'TunedWindSpeed'})) = [];
    
     % add rows corresponding to ipc and baseline cases
    load(fullfile(mat_save_dir, 'PI_BaselineParameters_table.mat'));
    UntunedControllers_case_table = PI_ParameterSweep_table;
    cols = UntunedControllers_case_table.Properties.VariableNames;
    cols(ismember(cols, 'RootMycBlade1 RMSE')) = []; % RootMycBlade1 RMSE column 
    cols(ismember(cols, 'ADC')) = []; % remove ADC column
    cols(ismember(cols, {'Case No.', 'Kp_diag', 'Kp_offdiag', 'Ki_diag', 'Ki_offdiag'})) = []; % remove Case No. and gains column
    UntunedControllers_case_table.("Case Desc.") = cell2mat({"noipc"; "baseline"});

    n_untuned_cases = length(sim_out_list.noipc) + length(sim_out_list.baseline_controller);
    % Untuned_case_table = array2table(zeros(n_untuned_cases, width(Controllers_case_table)));
    % Untuned_case_table.Properties.VariableNames = Controllers_case_table.Properties.VariableNames;
    % Untuned_case_table.("Case No.") = [(1:length(sim_out_list.noipc))'; (1:length(sim_out_list.baseline_controller))'];
    % untuned_case_types = cell(n_untuned_cases, 1);
    % [untuned_case_types{1:length(sim_out_list.noipc)}] = deal('noipc');
    % [untuned_case_types{length(sim_out_list.noipc)+1:end}] = deal('baseline');
    % Untuned_case_table.("Case Desc.") = untuned_case_types;


    for untuned_type = {'noipc', 'baseline_controller'}

        % Untuned_case_table(strcmp(Untuned_case_table.("Case Desc."), untuned_type{1}) & ...
        % (Untuned_case_table.("Case No.") == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED)), cols) ... 
        % = UntunedControllers_case_table(strcmp(UntunedControllers_case_table.("Case Desc."), untuned_type{1}), cols);

        for untuned_case_idx = 1:length(sim_out_list.(untuned_type{1}))
            % if strcmp(sim_out_list.controller(ctlr_case_idx).InflowWind.FileName_BTS, ...
            %         sim_out_list.(untuned_type{1})(untuned_case_idx).InflowWind.FileName_BTS)
                % && ... 
                    % (sim_out_list.controller(ctlr_case_idx).InflowWind.HWindSpeed == ...
                    % sim_out_list.(untuned_type{1})(untuned_case_idx).InflowWind.HWindSpeed)
                
                if length(sim_out_list.(untuned_type{1})) < untuned_case_idx
                    break;
                end

                if ~length(sim_out_list.(untuned_type{1})(untuned_case_idx).outdata_save_fn)
                    continue;
                end
                % dq values of blade-pitch and blade root bending moment for
                % particular wind field
                [~, x, ~] = fileparts(sim_out_list.(untuned_type{1})(untuned_case_idx).outdata_save_fn);
                x = split(x, '_');
                case_no = num2str(x{2});
                values = load([sim_out_list.(untuned_type{1})(untuned_case_idx).outdata_save_fn '_' case_no]);
                % values = load([sim_out_list.(untuned_type{1})(untuned_case_idx).outdata_save_fn]);
                values = values.OutData';
                values = values(:, 2:end); % remove extra time column
                
                ipc_values = load([sim_out_list.(untuned_type{1})(untuned_case_idx).blpitch_save_fn '_' case_no]);
                ipc_values = ipc_values.BlPitch';
                ipc_values = ipc_values(floor(cut_transients / DT):end, 2:end);
                dqValues = mbcTransformOutData(values, OutList);
                
                sim_out_list.(untuned_type{1})(untuned_case_idx).beta = struct;
                
                sim_out_list.(untuned_type{1})(untuned_case_idx).beta.blade = ...
                    getData(values, OutList, 'BldPitch1'); % in degrees
                sim_out_list.(untuned_type{1})(untuned_case_idx).beta.dq = ...
                    [getData(dqValues, dqOutList, 'BldPitchD'),...
                        getData(dqValues, dqOutList, 'BldPitchQ')]; % in degrees

                sim_out_list.(untuned_type{1})(untuned_case_idx).beta_ipc = struct;
                sim_out_list.(untuned_type{1})(untuned_case_idx).beta_ipc.dq = ...
                    ipc_values(:, 1:2); % in degrees

                sim_out_list.(untuned_type{1})(untuned_case_idx).beta_ipc_sat = struct;
                sim_out_list.(untuned_type{1})(untuned_case_idx).beta_ipc_sat.dq = ...
                    ipc_values(:, 3:4);

                sim_out_list.(untuned_type{1})(untuned_case_idx).beta_dot = struct;
                sim_out_list.(untuned_type{1})(untuned_case_idx).beta_dot.blade = ...
                    diff(sim_out_list.(untuned_type{1})(untuned_case_idx).beta.blade) / DT;
                
                sim_out_list.(untuned_type{1})(untuned_case_idx).RootMyc = struct;
                sim_out_list.(untuned_type{1})(untuned_case_idx).RootMyc.blade = ...
                    getData(values, OutList, 'RootMyc1');
                sim_out_list.(untuned_type{1})(untuned_case_idx).RootMyc.dq = ...
                    [getData(dqValues, dqOutList, 'RootMycD'), ...
                        getData(dqValues, dqOutList, 'RootMycQ')];
                
                sim_out_list.(untuned_type{1})(untuned_case_idx).RootMycRMSE = struct;
                
                x = sim_out_list.(untuned_type{1})(untuned_case_idx).RootRMyc.blade;
                sim_out_list.(untuned_type{1})(untuned_case_idx).RootMycRMSE.blade = ...
                    sqrt((1 / length(x)) * sum(x.^2));
                
                x = sim_out_list.(untuned_type{1})(untuned_case_idx).RootMyc.dq;
                sim_out_list.(untuned_type{1})(untuned_case_idx).RootMycMSE.dq = ...
                    sqrt((1 / size(x, 1)) * sum(x.^2, 1));
                
                x = sim_out_list.(untuned_type{1})(untuned_case_idx).beta_dot.blade;
                sim_out_list.(untuned_type{1})(untuned_case_idx).ADC = ...
                    (1 / length(x)) * sum((x ./ beta_dot_norm(x)));

                sim_out_list.(untuned_type{1})(untuned_case_idx).CaseDesc = untuned_type{1};
                

        end
    end
    
    % make a new table with a row for every nonlinear simulation conducted
    % with untuned controllers, and add variables from linear analysis
    % (UntunedControllers_case_table) and nonlinear analysis
    % (sim_out_list.noipc, sim_out_list.baseline)
    UntunedControllers_simulation_case_table = table();
    case_idx = 1;
    for untuned_type = {'noipc', 'baseline_controller'}
        for untuned_case_idx = 1:length(sim_out_list.(untuned_type{1}))
            % untuned_type = UntunedControllers_case_table(untuned_case_idx, "Case Desc.").Variables;
    
            cond = strcmp(UntunedControllers_case_table.("Case Desc."), ...
                            untuned_type{1});
            
            for f = lin_cols
                UntunedControllers_simulation_case_table(case_idx, f{1}) = ...
                    table(UntunedControllers_case_table(cond, f{1}).Variables, 'VariableNames', f);
            end
    
            [~, x, ~] = fileparts(sim_out_list.(untuned_type{1})(untuned_case_idx).InflowWind.FileName_BTS);
            UntunedControllers_simulation_case_table(case_idx, 'WindField') = {x};
            x = split(x, '_');
            UntunedControllers_simulation_case_table(case_idx, 'WindTurbulence') = x(1);
            UntunedControllers_simulation_case_table(case_idx, 'WindMean') = {str2double(x(2))};
            UntunedControllers_simulation_case_table(case_idx, 'WindSeed') = {str2double(x{3})};
    
            UntunedControllers_simulation_case_table(case_idx, ...
                {'ADC', 'RootMycBlade1 RMSE', 'RootMycD RMSE', 'RootMycQ RMSE'}) = ...
                    table(...
                    sim_out_list.(untuned_type)(untuned_case_idx).ADC, ...
                    sim_out_list.(untuned_type)(untuned_case_idx).RootMycMSE.blade, ...
                    sim_out_list.(untuned_type)(untuned_case_idx).RootMycMSE.dq(1), ...
                    sim_out_list.(untuned_type)(untuned_case_idx).RootMycMSE.dq(2), ...
                    'VariableNames', {'ADC', 'RootMycBlade1 RMSE', 'RootMycD RMSE', 'RootMycQ RMSE'});
    
            case_idx = case_idx + 1;
        end
    end

    if 0
        
        figure;
        tiledlayout(3, 2);
        nexttile(1);
        subtitle('\beta_d, \beta_q [deg]');
        plot(sim_out_list.noipc(3).beta_ipc.dq);
        nexttile(2);
        subtitle('Baseline IPC \beta_d, \beta_q [deg]');
        plot(rad2deg(sim_out_list.baseline_controller(3).beta_ipc.dq));
        legend('D', 'Q');

        ax1 = nexttile(3);
        subtitle('No IPC M_d, M_q [kNm?]');
        plot(sim_out_list.noipc(3).RootMyc.dq);
        legend('D', 'Q');
        ax2 = nexttile(4);
        subtitle('Baseline IPC M_d, M_q [kNm?]');
        plot(sim_out_list.baseline_controller(3).RootMyc.dq);
        legend('D', 'Q');
        linkaxes([ax1, ax2]);

        nexttile(5);
        subtitle('No IPC \beta_1 [deg]');
        plot(sim_out_list.noipc(3).beta.blade);
        nexttile(6);
        subtitle('Baseline IPC \beta_1 [deg]');
        plot(sim_out_list.baseline_controller(3).beta.blade);
    end
    
    if EXTREME_K_COLLECTION || OPTIMAL_K_COLLECTION || STRUCT_PARAM_SWEEP
        
        

        % TODO 1x3 Plot for each controller type showing mean worst case
        % stability margins for range of Wu values (y-axis) for each tuned wind speed (x-axis)

        % TODO 1x3 multibar plot for each controller type showing change in
        % DEL relative to no ipc case for range of Wu values (y-axis) for different loads (x-axis)

        % TODO Nx3 Plot for each controller type showing Power frequency
        % spectra (y-axis) for N different loads


        % TODO run EXTREME_K_COLLECTION for Wu=10 and generate pareto front
        % of adc, rmse, robusteness for varying weights

        % TODO run OPTIMAL_K_COLLECTION for VARY_WU = true and 5 seeds

        % TODO run OPTIMAL_K_COLLECTION for VARY_REF = true and 5 seeds

        % TODO run OPTIMAL_K_COLLECTION for VARY_SAT = true and 5 seeds
        
        % ? 1x2 Plot for each controller type showing beta_1, M1
        % (y-axis) relative to no ipc case vs. time (x-axis)
        
        n_ctlr_cases = length(sim_out_list.controller);
        sim_out_list_tmp = repmat(struct(), n_ctlr_cases, 1);
        outdata_filenames = cell(n_ctlr_cases, 1);
        [outdata_filenames{:}] = sim_out_list.controller.outdata_save_fn;
        blpitch_filenames = cell(n_ctlr_cases, 1);
        [blpitch_filenames{:}] = sim_out_list.controller.blpitch_save_fn;
        
        parfor ctlr_case_idx = 1:n_ctlr_cases
            
            % cc = cc + 1;
            % [values, ~, ~, ~, ~] = ReadFASTbinary(strrep(sim_out_list.controller(ctlr_case_idx).FAST_InputFileName, 'fst', 'outb'), 'n');
            [~, x, ~] = fileparts(outdata_filenames{ctlr_case_idx});
            x = split(x, '_');
            case_no = num2str(x{2});
            values = load([outdata_filenames{ctlr_case_idx} '_' case_no]);
            values = values.OutData';
            values = values(floor(cut_transients / DT):end, 2:end); % remove extra time column
            ipc_values = load([blpitch_filenames{ctlr_case_idx} '_' case_no]);
            ipc_values = ipc_values.BlPitch'; % time, beta_dq_sat, is_sat_dq
            ipc_values = ipc_values(floor(cut_transients / DT):end, 2:end);
            t = getData(values, OutList, 'Time');
            dqValues = mbcTransformOutData(values, OutList);
            
            sim_out_list_tmp(ctlr_case_idx).beta = struct;
            sim_out_list_tmp(ctlr_case_idx).beta.blade = ...
                getData(values, OutList, 'BldPitch1'); % in degrees
            sim_out_list_tmp(ctlr_case_idx).beta.dq = ...
                [getData(dqValues, dqOutList, 'BldPitchD'), ...
                getData(dqValues, dqOutList, 'BldPitchQ')]; % in degrees
    
            sim_out_list_tmp(ctlr_case_idx).beta_ipc = struct;
            sim_out_list_tmp(ctlr_case_idx).beta_ipc.dq = ...
                        ipc_values(:, 1:2); % in degrees
    
            sim_out_list_tmp(ctlr_case_idx).beta_ipc_sat = struct;
            sim_out_list_tmp(ctlr_case_idx).beta_ipc_sat.dq = ...
                        ipc_values(:, 3:4);
    
            sim_out_list_tmp(ctlr_case_idx).beta_dot = struct;
            sim_out_list_tmp(ctlr_case_idx).beta_dot.blade = ...
                diff(sim_out_list_tmp(ctlr_case_idx).beta.blade) / DT;
            
            sim_out_list_tmp(ctlr_case_idx).RootMyc = struct;
            sim_out_list_tmp(ctlr_case_idx).RootMyc.blade = ...
                getData(values, OutList, 'RootMyc1');
            sim_out_list_tmp(ctlr_case_idx).RootMyc.dq = ...
                [getData(dqValues, dqOutList, 'RootMycD'), ...
                getData(dqValues, dqOutList, 'RootMycQ')];
            
            sim_out_list_tmp(ctlr_case_idx).RootMycRMSE = struct;
    
            x = sim_out_list_tmp(ctlr_case_idx).RootMyc.blade;
            sim_out_list_tmp(ctlr_case_idx).RootMycRMSE.blade = ...
                sqrt((1 / length(x)) * sum(x.^2));
            
            x = sim_out_list_tmp(ctlr_case_idx).RootMyc.dq;
            sim_out_list_tmp(ctlr_case_idx).RootMycRMSE.dq = ...
                sqrt((1 / size(x, 1)) * sum(x.^2, 1));
            
            x = sim_out_list_tmp(ctlr_case_idx).beta_dot.blade;
            sim_out_list_tmp(ctlr_case_idx).ADC = ...
                (1 / length(x)) * sum((x ./ beta_dot_norm(x)));
            
            
        end
        
        if OPTIMAL_K_COLLECTION
            % find minimum values of
            % sim_out_list.controller(ctlr_case_idx).(f{1}).RootMyc.dq over all 
            % simulataions to compute varying reference values from
            % min_M_dq = struct;
            % find maximum values of
            % sim_out_list.controller(ctlr_case_idx).(f{1}).beta_ipc.dq over all 
            % simulataions to compute varying saturation values from
            % max_beta_dq = struct;
            min_M_dq = zeros(0, 2);
            max_beta_dq = zeros(0, 2);

            
            Controllers_simulation_case_table = table();

            for ctlr_case_idx = 1:n_ctlr_cases
                if case_list(ctlr_case_idx).Scheduling.x
                    [~, x, ~] = fileparts(case_list(ctlr_case_idx).InflowWind.Filename_BTS);
                    x = split(x, '_');
                    cond = strcmp(Controllers_case_table.("Case Desc."), ...
                           sim_out_list_tmp(ctlr_case_idx).CaseDesc) ...
                           & (Controllers_case_table.("TunedWindSpeed") == str2double(x{2}));
                else
                    cond = strcmp(Controllers_case_table.("Case Desc."), ...
                        case_list(ctlr_case_idx).CaseDesc) ...
                            & (Controllers_case_table.("TunedWindSpeed") == NONLPV_CONTROLLER_WIND_SPEED);
                end
                for f = lin_cols
                    Controllers_simulation_case_table(ctlr_case_idx, f{1}) = ...
                        table(Controllers_case_table(cond, f{1}).Variables, 'VariableNames', f);
                end

                [~, x, ~] = fileparts(case_list(ctlr_case_idx).InflowWind.FileName_BTS);
                Controllers_simulation_case_table(ctlr_case_idx, 'WindField') = x;
                x = split(x, '_');
                Controllers_simulation_case_table(ctlr_case_idx, 'WindTurbulence') = x(1);
                Controllers_simulation_case_table(ctlr_case_idx, 'WindMean') = {str2double(x(2))};
                Controllers_simulation_case_table(ctlr_case_idx, 'WindSeed') = {str2double(x(3))};

                Controllers_simulation_case_table(ctlr_case_idx, ...
                    {'ADC', 'RootMycBlade1 RMSE', 'RootMycD RMSE', 'RootMycQ RMSE'}) = ...
                        table(sim_out_list_tmp(ctlr_case_idx).ADC, ...
                        sim_out_list_tmp(ctlr_case_idx).RootMycMSE.blade, ...
                        sim_out_list_tmp(ctlr_case_idx).RootMycMSE.dq(1), ...
                        sim_out_list_tmp(ctlr_case_idx).RootMycMSE.dq(2), ...
                        'VariableNames', {'ADC', 'RootMycBlade1 RMSE', 'RootMycD RMSE', 'RootMycQ RMSE'});
                
                for f = fieldnames(sim_out_list_tmp(ctlr_case_idx))'
                    sim_out_list.controller(ctlr_case_idx).(f{1}) = sim_out_list_tmp(ctlr_case_idx).(f{1});
                end
                if case_list(ctlr_case_idx).WuGain.x.Numerator{1, 1} == min(VARY_WU_BASIS)
                    min_M_dq = [min_M_dq; min(abs(sim_out_list.controller(ctlr_case_idx).RootMyc.dq), [], 1)];
                    max_beta_dq = [max_beta_dq; max(abs(sim_out_list.controller(ctlr_case_idx).beta_ipc.dq), [], 1)];
                end
            end
            clear sim_out_list_tmp;
        end
        Controllers_simulation_case_table = sortrows(Controllers_simulation_case_table, 'Case Desc.');
        M_dq_reference = round(mean(min_M_dq, 1), 1); 
        save(fullfile(mat_save_dir, 'M_dq_Reference.mat'), "M_dq_reference");
        Beta_dq_saturation = round(mean(max_beta_dq, 1), 1);
        save(fullfile(mat_save_dir, 'Beta_dq_saturation.mat'), "Beta_dq_saturation");

        
        % TODO remove sigsOut, tout, BldPitch_c from SL model
        % 
        % case_list(1).CaseDesc
        % case_list(1).Scheduling
        % sim_out_list.controller(1).CaseDesc
        % sim_out_list.controller(1).InflowWind.FileName_BTS
        % Controllers_case_table
        sim_out_list.controller

        % TODO merge sim_out_list and case_list
        
        % tuned_case_types = cell(length(sim_out_list.controller), 1);
        % [tuned_case_types{:}] = deal('tuned_extreme');
        % Controllers_case_table.("Case Type") = tuned_case_types;
        Controllers_case_table = ...
            [Untuned_case_table; ...
            Controllers_case_table];

        % add wind field column
        noipc_op = cell(length(sim_out_list.noipc), 1);
        [noipc_op{:}] = sim_out_list.noipc.InflowWind.FileName_BTS;
        for i = 1:length(noipc_op)
            [~, x, ~] = fileparts(noipc_op{i});
            noipc_op{i} = x;
            x = split(x, '_');
            noipc_wind_op.turb(i) = x{1};
            noipc_wind_op.mean(i) = str2num(x{2});
            noipc_wind_op.seed(i) = str2num(x{3});
        end
        bl_op = cell(length(sim_out_list.baseline_controller), 1);
        [bl_op{:}] = sim_out_list.baseline_controller.InflowWind.FileName_BTS;
        for i = 1:length(bl_op)
            [~, x, ~] = fileparts(bl_op{i});
            bl_op{i} = x;
            x = split(x, '_');
            bl_wind_op.turb(i) = x{1};
            bl_wind_op.mean(i) = str2num(x{2});
            bl_wind_op.seed(i) = str2num(x{3});
        end

        ctrl_op = cell(length(sim_out_list.controller), 1);
        [ctrl_op{:}] = sim_out_list.controller.InflowWind.FileName_BTS;
        for i = 1:length(ctrl_op)
            [~, x, ~] = fileparts(ctrl_op{i});
            ctrl_op{i} = x;
            x = split(x, '_');
            ctrl_wind_op.turb(i) = x{1};
            ctrl_wind_op.mean(i) = str2num(x{2});
            ctrl_wind_op.seed(i) = str2num(x{3});
        end
        
        Untuned_case_table.("WindField") = ...
            [noipc_op; ...
             bl_op];
        Untuned_case_table.("WindMean") = ...
            [noipc_wind_op.mean'; ...
            bl_wind_op.mean'];
        Untuned_case_table.("WindTurbulence") = ...
            [noipc_wind_op.turb'; ...
            bl_wind_op.turb'];
        Untuned_case_table.("WindSeed") = ...
            [noipc_wind_op.seed'; ...
            bl_wind_op.seed'];

        Controllers_case_table.("WindField") = ...
            [noipc_op; ...
             bl_op; ...
            ctrl_op];
        Controllers_case_table.("WindMean") = ...
            [noipc_wind_op.mean'; ...
            bl_wind_op.mean'; ...
            ctrl_wind_op.mean'];
        Controllers_case_table.("WindTurbulence") = ...
            [noipc_wind_op.turb'; ...
            bl_wind_op.turb'; ...
            ctrl_wind_op.turb'];
        Controllers_case_table.("WindSeed") = ...
            [noipc_wind_op.seed'; ...
            bl_wind_op.seed'; ...
            ctrl_wind_op.seed'];

        noipc_op = cell(length(sim_out_list.noipc), 1);
        [noipc_op{:}] = sim_out_list.noipc.ADC;
        bl_op = cell(length(sim_out_list.baseline_controller), 1);
        [bl_op{:}] = sim_out_list.baseline_controller.ADC;
        ctrl_op = cell(length(sim_out_list.controller), 1);
        [ctrl_op{:}] = sim_out_list.controller.ADC;
        Untuned_case_table.("ADC") = ...
            [cell2mat(noipc_op); ...
            cell2mat(bl_op)];
        Controllers_case_table.("ADC") = ...
            [cell2mat(noipc_op); ...
            cell2mat(bl_op); ...
            cell2mat(ctrl_op)];
        
        noipc_op = cell(length(sim_out_list.noipc), 1);
        [noipc_op{:}] = sim_out_list.noipc.RootMycMSE.blade;
        bl_op = cell(length(sim_out_list.baseline_controller), 1);
        [bl_op{:}] = sim_out_list.baseline_controller.RootMycMSE.blade;
        ctrl_op = cell(length(sim_out_list.controller), 1);
        [ctrl_op{:}] = sim_out_list.controller.RootMycMSE.blade;
        Untuned_case_table.("RootMycBlade1 RMSE") = ...
            [cell2mat(noipc_op); ...
            cell2mat(bl_op)];
        Controllers_case_table.("RootMycBlade1 RMSE") = ...
            [cell2mat(noipc_op); ...
            cell2mat(bl_op); ...
            cell2mat(ctrl_op)];

        noipc_op = cell(length(sim_out_list.noipc), 1);
        [noipc_op{:}] = sim_out_list.noipc.RootMycMSE.dq(:, 1);
        bl_op = cell(length(sim_out_list.baseline_controller), 1);
        [bl_op{:}] = sim_out_list.baseline_controller.RootMycMSE.dq(:, 1);
        ctrl_op = cell(length(sim_out_list.controller), 1);
        [ctrl_op{:}] = sim_out_list.controller.RootMycMSE.dq(:, 1);
        Untuned_case_table.("RootMycD RMSE") = ...
            [cell2mat(noipc_op); ...
            cell2mat(bl_op)];
        Controllers_case_table.("RootMycD RMSE") = ...
            [cell2mat(noipc_op); ...
            cell2mat(bl_op); ...
            cell2mat(ctrl_op)];

        noipc_op = cell(length(sim_out_list.noipc), 1);
        [noipc_op{:}] = sim_out_list.noipc.RootMycMSE.dq(:, 2);
        bl_op = cell(length(sim_out_list.baseline_controller), 1);
        [bl_op{:}] = sim_out_list.baseline_controller.RootMycMSE.dq(:, 2);
        ctrl_op = cell(length(sim_out_list.controller), 1);
        [ctrl_op{:}] = sim_out_list.controller.RootMycMSE.dq(:, 2);
        Untuned_case_table.("RootMycQ RMSE") = ...
            [cell2mat(noipc_op); ...
            cell2mat(bl_op)];
        Controllers_case_table.("RootMycQ RMSE") = ...
            [cell2mat(noipc_op); ...
            cell2mat(bl_op); ...
            cell2mat(ctrl_op)];
        
        % average Untuned_case_table and Controllers_case_table over
        % wind field seeds for each mean wind speed and case no.
        % Controllers_case_table = renamevars(Controllers_case_table, {'Wind Speed'}, {'TunedWindSpeed'})
        agg_cols = Controllers_case_table.Properties.VariableNames;
        agg_cols(ismember(agg_cols, 'Case No.')) = [];
        agg_cols(ismember(agg_cols, 'Case Desc.')) = [];
        agg_cols(ismember(agg_cols, 'Stable')) = [];
        agg_cols(ismember(agg_cols, 'TunedWindSpeed')) = [];
        agg_cols(ismember(agg_cols, 'WindField')) = [];
        agg_cols(ismember(agg_cols, 'WindMean')) = [];
        agg_cols(ismember(agg_cols, 'WindTurbulence')) = [];
        agg_cols(ismember(agg_cols, 'WindSeed')) = [];
        % groupsummary(Untuned_case_table, ["Case Desc.", "WindMean", "WindSeed", "WindTurbulence"], "mean", agg_cols)
        Controllers_case_agg_table = groupsummary(Controllers_case_table, ["Case Desc.", "TunedWindSpeed", "WindMean", "WindTurbulence"], {"mean", "min", "max", "median", "std"}, agg_cols)
        Controllers_case_agg_table((Controllers_case_agg_table.("WindMean") == NONLPV_CONTROLLER_WIND_SPEED) & (Controllers_case_agg_table.("TunedWindSpeed") == NONLPV_CONTROLLER_WIND_SPEED), :)
        % groupsummary(Untuned_cas(e_table, ["Case Desc.", "WindMean", "WindSeed", "WindTurbulence"], "min", agg_cols)
        % groupsummary(Untuned_case_table, ["Case Desc.", "WindMean", "WindSeed", "WindTurbulence"], "max", agg_cols)
        % groupsummary(Untuned_case_table, ["Case Desc.", "WindMean", "WindSeed", "WindTurbulence"], "median", agg_cols)
        % groupsummary(Untuned_case_table, ["Case Desc.", "WindMean", "WindSeed", "WindTurbulence"], "var", agg_cols)
        
        % W1Gains = untuned_case_types;
        % W2Gains = untuned_case_types;
        % WeGains = untuned_case_types;
        % WuGains = untuned_case_types;
        % W1Gains = nan * ones(n_untuned_cases, 1);
        % W2Gains = nan * ones(n_untuned_cases, 1);
        % WeGains = nan * ones(n_untuned_cases, 1);
        % WuGains = nan * ones(n_untuned_cases, 1);
        % for c = 1:length(case_list)
        %     x = cell2mat(case_list(c).W1Gain.x.Numerator(1,1));
        %     W1Gains = [W1Gains; x];
        %     x = cell2mat(case_list(c).W2Gain.x.Numerator(1,1));
        %     W2Gains = [W2Gains; x];
        %     x = cell2mat(case_list(c).WuGain.x.Numerator(1,1));
        %     WuGains = [WuGains; x];
        %     x = cell2mat(case_list(c).WeGain.x.Numerator(1,1));
        %     WeGains = [WeGains; x];
        % end
        % Controllers_case_table.("$W_1$ Gain") = W1Gains;
        % Controllers_case_table.("$W_2$ Gain") = W2Gains;
        % Controllers_case_table.("$W_u$ Gain") = WuGains;
        % Controllers_case_table.("$W_e$ Gain") = WeGains;
        
        % ctrl_cond = ~strcmp(Controllers_case_table.("$W_1$ Gain"), 'noipc') ...
        %     & ~strcmp(Controllers_case_table.("$W_1$ Gain"), 'baseline');
        % op_cols = ["Case No.", "WindField", "$W_1$ Gain", "$W_2$ Gain", "$W_u$ Gain", "$W_e$ Gain", "MultiDiskIO_DM", "wc", "ADC", "RootMycBlade1 RMSE", "RootMycD RMSE", "RootMycQ RMSE"];
        ctrl_cond = ~strcmp(Controllers_case_agg_table.("Case Desc."), 'noipc') ...
            & ~strcmp(Controllers_case_agg_table.("Case Desc."), 'baseline');

        % if EXTREME_K_COLLECTION || OPTIMAL_K_COLLECTION
            op_cols = ["Case No.", "Case Desc.", "WindField", "MultiDiskIO_DM", "wc", "ADC", "RootMycBlade1 RMSE", "RootMycD RMSE", "RootMycQ RMSE"];
        % else
            % op_cols = ["Case No.", "Kp_diag", "Kp_offdiag", "Ki_diag", "Ki_offdiag", "MultiDiskIO_DM", "wc", "ADC", "RootMycBlade1 RMSE", "RootMycD RMSE", "RootMycQ RMSE"];
        % end
        
        n_top_cases_per_wind_case = 5;
        % group by WindMean
        n_top_cases = n_top_cases_per_wind_case * length(unique(Controllers_case_agg_table(ctrl_cond, "WindField")).Variables);
        most_robust_table = array2table(zeros(0, length(op_cols)), 'VariableNames', op_cols);
        lowest_adc_table = array2table(zeros(0, length(op_cols)), 'VariableNames', op_cols);
        lowest_mse_table = array2table(zeros(0, length(op_cols)), 'VariableNames', op_cols);
        wind_idx = 1;

        % group by mean wind speed and seed (ie. .bts file used in simulation)
        for wind_case = unique(Controllers_case_table(ctrl_cond, "WindField")).Variables
            table_start_idx = (wind_idx - 1) * n_top_cases_per_wind_case + 1;
            table_end_idx = (wind_idx - 1) * n_top_cases_per_wind_case + n_top_cases_per_wind_case;
            x = sortrows(Controllers_case_table(ctrl_cond & wind_case, :), 'MultiDiskIO_DM', 'descend');
            most_robust_table = [most_robust_table; x(1:n_top_cases_per_wind_case, op_cols)];
            
            x = sortrows(Controllers_case_table(ctrl_cond * wind_case, :), 'ADC', 'ascend');
            lowest_adc_table = [lowest_adc_table; x(1:n_top_cases_per_wind_case, op_cols)];
            
            x = sortrows(Controllers_case_table(ctrl_cond & wind_case, :), 'RootMycBlade1 RMSE', 'ascend');
            lowest_mse_table = [lowest_mse_table; x(1:n_top_cases_per_wind_case, op_cols)];
            wind_idx = wind_idx + 1;
        end
        
        % TODO compute wc and MultiDiskIO_DM for baseline cases, wc for
        % extreme cases
        % add noipc and baseline cases from tuned wind speed
        for wf = unique(most_robust_table.("WindField"))
            most_robust_table = ...
                [Untuned_case_table(strcmp(Untuned_case_table.("WindField"), wf{1}), op_cols); ...
                 most_robust_table];

            lowest_adc_table = ...
                [Untuned_case_table(strcmp(Untuned_case_table.("WindField"), wf{1}), op_cols); ...
                 lowest_adc_table];

            lowest_mse_table = ...
                [Untuned_case_table(strcmp(Untuned_case_table.("WindField"), wf{1}), op_cols); ...
                 lowest_mse_table];
        end

        % TODO choose best of three types of tuned controllers
        most_robust_table(3, :).("Case Desc.")
        lowest_adc_table(3, :).("Case Desc.")
        lowest_mse_table(3, :).("Case Desc.")

        % TODO rename variable names for Latex, specify scientific notation
        % for numbers to table2latex


        Untuned_case_table(:, op_cols)

        % add back in noipc and baseline cases corresponding to wind fields
        % included in controller testing
        unique(lowest_cumerror_table.("WindField"))

        table_tmp = Controllers_case_table(:, op_cols);
        table_tmp.Properties.VariableNames = {'Controller', '$W_1$ Gain', '$W_2$ Gain', '$W_u$ Gain', '$W_e$ Gain', 'MLIO Disk Margin', '$\omega_c$', 'ADC', '$M_{b}$ RMSE', '$\tilde{M}_{d}$ RMSE', '$\tilde{M}_{q}$ RMSE'};
        table_tmp.("Controller") = [{"None"}; num2cell(1:size(table_tmp, 1) - 1)'];
        
        save(fullfile(mat_save_dir, 'Untuned_Controllers_case_table.mat'), "Untuned_case_table");
        
        op_cols = ["Case Desc.", "MultiDiskIO_DM", "wc", "ADC", "RootMycBlade1 RMSE", "RootMycD RMSE", "RootMycQ RMSE"];

        % TODO average results over seeds

        if OPTIMAL_K_COLLECTION
            % TODO select one weighting combination for each best metric,
            % than run for sweep over Wu
            table2latex(most_robust_table(:, op_cols), fullfile(fig_dir, 'optimal_robust_controller_table.tex'));
            table2latex(lowest_adc_table(:, op_cols), fullfile(fig_dir, 'optimal_lowadc_controller_table.tex'));
            table2latex(lowest_mse_table(:, op_cols), fullfile(fig_dir, 'optimal_lowmse_controller_table.tex'));


            save(fullfile(mat_save_dir, 'Optimal_Controllers_case_table.mat'), 'Controllers_case_table');
            table2latex(table_tmp, fullfile(fig_dir, 'optimal_controller_table.tex'));
        elseif EXTREME_K_COLLECTION
            table2latex(most_robust_table(:, op_cols), fullfile(fig_dir, 'extreme_robust_controller_table.tex'));
            table2latex(lowest_adc_table(:, op_cols), fullfile(fig_dir, 'extreme_lowadc_controller_table.tex'));
            table2latex(lowest_mse_table(:, op_cols), fullfile(fig_dir, 'extreme_lowmse_controller_table.tex'));

            save(fullfile(mat_save_dir, 'Extreme_Controllers_case_table.mat'), 'Controllers_case_table');
            table2latex(table_tmp, char(fullfile(fig_dir, 'extreme_controller_table.tex')));
        elseif STRUCT_PARAM_SWEEP

            table2latex(most_robust_table(:, op_cols), fullfile(fig_dir, 'structsweep_robust_controller_table.tex'));
            table2latex(lowest_adc_table(:, op_cols), fullfile(fig_dir, 'structsweep_lowadc_controller_table.tex'));
            table2latex(lowest_mse_table(:, op_cols), fullfile(fig_dir, 'structsweep_lowmse_controller_table.tex'));

            save(fullfile(mat_save_dir, 'PI_ParameterSweep_redtable.mat'), 'Controllers_case_table');
            table_tmp = Controllers_case_table(:, op_cols);
            table_tmp.Properties.VariableNames = {'Controller', '$k^p_{d}$', '$k^p_{od}$', '$k^i_{d}$', '$k^i_{od}$','MLIO Disk Margin', 'ADC', '$M_{b}$ RMSE', '$\tilde{M}_{d}$ RMSE', '$\tilde{M}_{q}$ RMSE'};
            table_tmp.("Controller") = [{"None"}; num2cell(1:size(table_tmp, 1) - 1)'];
            table2latex(table_tmp, fullfile(fig_dir, "param_sweep_table.tex"));
        end
    elseif BASELINE_K

        load(fullfile(mat_save_dir, 'PI_BaselineParameters_table.mat'));
        Controllers_case_table = PI_ParameterSweep_table;
        cols = Controllers_case_table.Properties.VariableNames;
        cols(1) = []; % remove Case No. column
        cols(37) = []; % RootMycBlade1 RMSE column
        cols(36) = []; % remove ADC column
        Controllers_case_table.("Case Desc.") = cell2mat({"noipc"; "baseline"});

        n_untuned_cases = length(sim_out_list.noipc) + length(sim_out_list.baseline_controller);
        Untuned_case_table = array2table(zeros(n_untuned_cases, width(Controllers_case_table)));
        Untuned_case_table.Properties.VariableNames = Controllers_case_table.Properties.VariableNames;
        Untuned_case_table.("Case No.") = [(1:length(sim_out_list.noipc))'; (1:length(sim_out_list.baseline_controller))'];
        untuned_case_types = cell(n_untuned_cases, 1);
        [untuned_case_types{1:length(sim_out_list.noipc)}] = deal('noipc');
        [untuned_case_types{length(sim_out_list.noipc)+1:end}] = deal('baseline');
        Untuned_case_table.("Case Desc.") = untuned_case_types;

        noipc_op = cell(length(sim_out_list.noipc), 1);
        [noipc_op{:}] = sim_out_list.noipc.InflowWind.FileName_BTS;
        for i = 1:length(noipc_op)
            [~, x, ~] = fileparts(noipc_op{i});
            noipc_op{i} = x;
        end
        bl_op = cell(length(sim_out_list.baseline_controller), 1);
        [bl_op{:}] = sim_out_list.baseline_controller.InflowWind.FileName_BTS;
        for i = 1:length(bl_op)
            [~, x, ~] = fileparts(bl_op{i});
            bl_op{i} = x;
        end
        Untuned_case_table.("WindField") = ...
            [noipc_op; ...
             bl_op];

        noipc_op = cell(length(sim_out_list.noipc), 1);
        [noipc_op{:}] = sim_out_list.noipc.ADC;
        bl_op = cell(length(sim_out_list.baseline_controller), 1);
        [bl_op{:}] = sim_out_list.baseline_controller.ADC;
        Untuned_case_table.("ADC") = ...
            [cell2mat(noipc_op); ...
            cell2mat(bl_op)];
        
        noipc_op = cell(length(sim_out_list.noipc), 1);
        [noipc_op{:}] = sim_out_list.noipc.RootMycMSE.blade;
        bl_op = cell(length(sim_out_list.baseline_controller), 1);
        [bl_op{:}] = sim_out_list.baseline_controller.RootMycMSE.blade;
        Untuned_case_table.("RootMycBlade1 RMSE") = ...
            [cell2mat(noipc_op); ...
            cell2mat(bl_op)];

        noipc_op = cell(length(sim_out_list.noipc), 1);
        [noipc_op{:}] = sim_out_list.noipc.RootMycMSE.dq(:, 1);
        bl_op = cell(length(sim_out_list.baseline_controller), 1);
        [bl_op{:}] = sim_out_list.baseline_controller.RootMycMSE.dq(:, 1);
        Untuned_case_table.("RootMycD RMSE") = ...
            [cell2mat(noipc_op); ...
            cell2mat(bl_op)];

        noipc_op = cell(length(sim_out_list.noipc), 1);
        [noipc_op{:}] = sim_out_list.noipc.RootMycMSE.dq(:, 2);
        bl_op = cell(length(sim_out_list.baseline_controller), 1);
        [bl_op{:}] = sim_out_list.baseline_controller.RootMycMSE.dq(:, 2);
        Untuned_case_table.("RootMycQ RMSE") = ...
            [cell2mat(noipc_op); ...
            cell2mat(bl_op)];
        
        % set stability metrics for wind speed they were computed at TODO
        % make this more generalazable if metrics were computed at many
        % wind speeds
        
        for case_desc = {"noipc", "baseline"}
            Untuned_case_table(strcmp(Untuned_case_table.("Case Desc."), case_desc{1}) & ...
            (Untuned_case_table.("Case No.") == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED)), cols) ... 
            = Controllers_case_table(strcmp(Controllers_case_table.("Case Desc."), case_desc{1}), cols);
        end

    elseif STRUCT_PARAM_SWEEP
        % TODO add no ipc and baseline cases from correct wind speed

        load(fullfile(mat_save_dir, 'PI_ParameterSweep_redtable.mat'));
        op = cell(length(sim_out_list.controller), 1);
        [op{:}] = sim_out_list.controller.ADC;
        PI_ParameterSweep_redtable.("ADC") = [sim_out_list.noipc(1).ADC; cell2mat(op)];
        op = cell(length(sim_out_list.controller), 1);
        [op{:}] = sim_out_list.controller.RootMycMSE.blade;
        PI_ParameterSweep_redtable.("RootMycBlade1 RMSE") = [sim_out_list.noipc(1).RootMycMSE.blade; cell2mat(op)];
        op = cell(length(sim_out_list.controller), 1);
        [op{:}] = sim_out_list.controller.RootMycMSE.dq(:, 1);
        PI_ParameterSweep_redtable.("RootMycD RMSE") = [sim_out_list.noipc(1).RootMycMSE.dq(:, 1); cell2mat(op)];
        op = cell(length(sim_out_list.controller), 1);
        [op{:}] = sim_out_list.controller.RootMycMSE.dq(:, 2);
        PI_ParameterSweep_redtable.("RootMycQ RMSE") = [sim_out_list.noipc(1).RootMycMSE.dq(:, 2); cell2mat(op)];

        op_cols = ["Case No.", "Kp_diag", "Kp_offdiag", "Ki_diag", "Ki_offdiag", "MultiDiskIO_DM", "wc", "ADC", "RootMycBlade1 RMSE", "RootMycD RMSE", "RootMycQ RMSE"];
        most_robust_table = sortrows(PI_ParameterSweep_redtable, 'MultiDiskIO_DM', 'descend');
        most_robust_table = most_robust_table(1:5, op_cols);
        lowest_adc_table = sortrows(PI_ParameterSweep_redtable, 'ADC', 'ascend');
        lowest_adc_table = lowest_adc_table(1:5, op_cols);
        lowest_cumerror_table = sortrows(PI_ParameterSweep_redtable, 'RootMycBlade1 RMSE', 'ascend');
        lowest_cumerror_table = lowest_cumerror_table(1:5, op_cols);
        

        save(fullfile(code_dir, 'matfiles', 'PI_ParameterSweep_redtable.mat'), "PI_ParameterSweep_redtable");
        table_tmp = PI_ParameterSweep_redtable(:, op_cols);
        table_tmp.Properties.VariableNames = {'Controller', '$k^p_{d}$', '$k^p_{od}$', '$k^i_{d}$', '$k^i_{od}$','MLIO Disk Margin', 'ADC', '$M_{b}$ RMSE', '$\tilde{M}_{d}$ RMSE', '$\tilde{M}_{q}$ RMSE'};
        table_tmp.("Controller") = [{"None"}; num2cell(1:size(table_tmp, 1) - 1)'];
        table2latex(table_tmp, fullfile(fig_dir, "param_sweep_table.tex"));
    end
% end
end

%% TODO 2x3 Plot for each controller type showing change in mean
 % ADC, M1 RMSE (y-axis) relative to no ipc case for each mean wind speed
 % (x-axis)

%% DEL analysis of loads
% run python DEL analysis
sim_types = 'extreme_k_cases_turbsim noipc_turbsim';
status = system(['/Users/aoifework/miniconda3/envs/weis_dev/bin/python3 ', fullfile(project_dir, 'postprocessing', 'main.py'), ' -st ', sim_types]);

% read python DEL analysis output
del_data = readtable(fullfile(postprocessing_save_dir, 'DELs.csv'));

% plot percentage DEL reduction @ each mean wind speed for blade
% out-of-plane load, nacelle yaw moment, nacelle pitch moment, shaft
% bending moment around y-axis
figure;
loads = cell(length(del_data.Properties.VariableNames) - 1, 1);
[loads{:}] = del_data.Properties.VariableNames{2:end};
noipc_dels = del_data(contains(del_data.("Var1"), 'noipc_turbsim'), loads).Variables;
tuned_dels = del_data(contains(del_data.("Var1"), 'extreme_k_cases_turbsim'), loads).Variables;
delta_del = ((tuned_dels - noipc_dels) / noipc_dels) * 100;
% bar(loads, delta_del);
% find mean wind speed associated with this case
ux_vals = [];
for outdata_fn = del_data.("Var1")
    % values = load(outdata_fn);
    % ux = getData(values, OutList, 'WindVelX1');
    split_str = split(outdata_fn{1}, '_');
    case_idx = str2num(split_str{2});
    ux_vals = [ux_vals, case_list(case_idx).InflowWind.HWindSpeed];
    % TODO need to get noipc/baseline hwindspeeds
end
del_data.("HWindSpeed") = ux_vals;

bar(loads, tuned_dels);


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
    
    values = load(sim_out_list.noipc(1).save_fn);
    values = values.OutData.Data;
    dqValues = mbcTransformOutData(values, OutList);
    op_data_blade_noipc = values(cut_transients/DT + 1:end, blade_op_indices);
    op_data_cdq_noipc = dqValues(cut_transients/DT + 1:end, blade_op_indices);

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


%% Analysis of loads from Open-Loop nonlinear simulations

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
