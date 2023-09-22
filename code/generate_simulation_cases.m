%% Generate Simulation Cases

% Generate Turbsim Cases

% ux_mean = 11.4;
class = 'A'; % class A
n_seeds = 1;
uxs = LPV_CONTROLLER_WIND_SPEEDS;

if sum([RUN_OL_DQ, RUN_OL_BLADE, RUN_CL]) == 1
    CaseGen.dir_matrix = FAST_runDirectory;
    CaseGen.namebase = FAST_SimulinkModel;
    [~, CaseGen.model_name, ~] = fileparts(fastRunner.FAST_InputFile);
    % clear case_basis;
    case_basis.InflowWind.HWindSpeed = [0];
    i = 1;
    if strcmp(WIND_TYPE, 'turbsim')
        case_basis.InflowWind.WindType = {'3'};
        case_basis.InflowWind.FileName_BTS = {};
        for ux = uxs
            for bts_idx = 1:n_seeds
                case_basis.InflowWind.FileName_BTS{i} = ['"' fullfile(windfiles_dir, ...
                    [class, '_', replace(num2str(ux), '.', '-'), '_', num2str(bts_idx), '.bts']) '"'];
                % case_basis.InflowWind.HWindSpeed(i) = ux;
                i = i + 1;
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
            if strcmp(WIND_TYPE, 'turbsim')
                x = split(case_list(case_idx).InflowWind.FileName_BTS, '_');
                case_list(case_idx).InflowWind.HWindSpeed = x;
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
