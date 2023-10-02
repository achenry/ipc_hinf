%% Generate Simulation Cases

% Generate Turbsim Cases

% ux_mean = 11.4;
class = 'A'; % class A

% uxs = NONLPV_CONTROLLER_WIND_SPEEDS;

if sum([RUN_OL_DQ, RUN_OL_BLADE, RUN_CL]) == 1
    CaseGen.dir_matrix = FAST_runDirectory;
    CaseGen.namebase = FAST_SimulinkModel;
    [~, CaseGen.model_name, ~] = fileparts(fastRunner.FAST_InputFile);
    ctrl_types = fieldnames(case_basis.W1Gain);
    clear case_basis;
    case_basis.InflowWind.HWindSpeed = [0];
    i = 1;
    if strcmp(WIND_TYPE, 'turbsim')
        case_basis.InflowWind.WindType = {'3'};
        case_basis.InflowWind.FileName_BTS = {};
        for ux = uxs
            for bts_idx = 1:n_seeds
                case_basis.InflowWind.FileName_BTS{i} = ['"' fullfile(windfiles_dir, ...
                    [class, '_', replace(num2str(ux), '.', '-'), '_', num2str(bts_idx), '.bts']) '"'];
                i = i + 1;
            end
        end
        
    elseif strcmp(WIND_TYPE, 'steady')
        case_basis.InflowWind.WindType = {'1'};
        case_basis.InflowWind.HWindSpeed = split(num2str(WIND_SPEEDS));
    end

    case_basis.Fst.TMax = {num2str(TMax)};

    [Wind_case_list, Wind_case_name_list, n_wind_cases] = generateCases(case_basis, CaseGen.namebase, true);
 
end

if STRUCT_PARAM_SWEEP || BASELINE_K
    % Load MIMO PI Parameter Sweep Gains and Controllers (in tunable Block form)
    if STRUCT_PARAM_SWEEP
        load(fullfile(mat_save_dir, 'PI_ParameterSweep_case_list.mat'));
    elseif BASELINE_K
        load(fullfile(mat_save_dir, 'PI_BaselineParameters_case_list.mat'));
    end

    % Merge Wind and Controller Cases
    % case_idx = 1;
    n_controller_cases = length(PI_ParameterSweep_case_list);
    if MAX_SIMULATIONS > -1
        n_total_cases = min(n_controller_cases * n_wind_cases, MAX_SIMULATIONS);
    else
        n_total_cases = n_controller_cases * n_wind_cases;
    end
    case_list = repmat(struct(), n_total_cases, 1 );
    case_idx = 1;
    for c_idx = 1:n_controller_cases
        for w_idx = 1:n_wind_cases
            % case_idx = (c_idx - 1) * n_controller_cases + w_idx;
            sprintf(['Generating case ', num2str(case_idx)]);
            for fn1 = fieldnames(Wind_case_list(w_idx))'
                for fn2 = fieldnames(Wind_case_list(w_idx).(fn1{1}))'
                    case_list(case_idx).(fn1{1}).(fn2{1}) = Wind_case_list(w_idx).(fn1{1}).(fn2{1});
                end
            end
            for fn1 = fieldnames(PI_ParameterSweep_case_list(c_idx))'
                case_list(case_idx).(fn1{1}) = PI_ParameterSweep_case_list(c_idx).(fn1{1});
            end
            if strcmp(WIND_TYPE, 'turbsim')
                [~,x,~] = fileparts(case_list(case_idx).InflowWind.FileName_BTS);
                x = split(x, '_');
                x = str2num(x{2});
                case_list(case_idx).InflowWind.HWindSpeed = x;
            end
            case_idx = case_idx + 1;
            if MAX_SIMULATIONS > -1 && case_idx > MAX_SIMULATIONS
                break;
            end
        end
        if MAX_SIMULATIONS > -1 && case_idx > MAX_SIMULATIONS
            break;
        end
    end
    % n_cases = case_idx - 1;
    n_cases = length(case_list);
    case_name_list = arrayfun(@(n) ['case_', num2str(n)], 1:n_cases, 'UniformOutput', false);

    if MAX_SIMULATIONS > -1
        save_prefix = 'debug_';
    else
        save_prefix = '';
    end
    
    if STRUCT_PARAM_SWEEP
        save(fullfile(mat_save_dir, [save_prefix, 'Structured_Controllers_nonlinear_simulation_case_list.mat']), "case_list", "case_name_list", '-v7.3');
    elseif BASELINE_K
        save(fullfile(mat_save_dir, [save_prefix, 'Baseline_Controller_nonlinear_simulation_case_list.mat']), "case_list", "case_name_list", '-v7.3');
    end
elseif OPTIMAL_K_COLLECTION
    % Load Controller Cases corresponding to scheduled full-order controllers
    load(fullfile(mat_save_dir, 'Optimal_Controllers_case_list.mat'));

    % Merge Wind, controller types and Controller weighting Cases
    n_controller_cases = length(Controllers_case_list);
    
    n_ctrl_types = length(ctrl_types);
    
    if MAX_SIMULATIONS > -1
        n_total_cases = min(n_controller_cases * n_wind_cases * n_ctrl_types, MAX_SIMULATIONS);
    else
        n_total_cases = n_controller_cases * n_wind_cases * n_ctrl_types;
    end
    case_list = repmat(struct(), n_total_cases, 1 );
    case_idx = 1;
    for c_idx = 1:n_controller_cases
        for w_idx = 1:n_wind_cases
            for t_idx = 1:length(ctrl_types)
                ctrl_type = ctrl_types{t_idx};
                % case_idx = (c_idx - 1) * n_controller_cases + w_idx;
                sprintf(['Generating case ', num2str(case_idx)]);
                for fn1 = fieldnames(Wind_case_list(w_idx))'
                    for fn2 = fieldnames(Wind_case_list(w_idx).(fn1{1}))'
                        case_list(case_idx).(fn1{1}).(fn2{1}) = Wind_case_list(w_idx).(fn1{1}).(fn2{1});
                    end
                end
                for fn1 = fieldnames(Controllers_case_list(c_idx))'
                    if isfield(Controllers_case_list(c_idx).(fn1{1}), ctrl_type)
                        case_list(case_idx).(fn1{1}) = Controllers_case_list(c_idx).(fn1{1}).(ctrl_type);
                    else
                        case_list(case_idx).(fn1{1}) = Controllers_case_list(c_idx).(fn1{1});
                    end
                end

                if strcmp(WIND_TYPE, 'turbsim')
                    [~,x,~] = fileparts(case_list(case_idx).InflowWind.FileName_BTS);
                    x = split(x, '_');
                    x = str2num(x{2});
                    case_list(case_idx).InflowWind.HWindSpeed = x;
                end
                case_idx = case_idx + 1;
                if MAX_SIMULATIONS > -1 && case_idx > MAX_SIMULATIONS
                    break;
                end

            end
        end
        if MAX_SIMULATIONS > -1 && case_idx > MAX_SIMULATIONS
            break;
        end
    end
    % n_cases = case_idx - 1;
    n_cases = length(case_list);
    case_name_list = arrayfun(@(n) ['case_', num2str(n)], 1:n_cases, 'UniformOutput', false);
    
    if MAX_SIMULATIONS > -1
        save_prefix = 'debug_';
    else
        save_prefix = '';
    end
    save(fullfile(mat_save_dir, [save_prefix, 'Optimal_Controllers_nonlinear_simulation_case_list.mat']), "case_list", "case_name_list", '-v7.3');

elseif EXTREME_K_COLLECTION
    % Load Controller Cases corresponding to scheduled full-order controllers
    load(fullfile(mat_save_dir, 'Extreme_Controllers_case_list.mat'));

    % Merge Wind and Controller Cases
    % case_idx = 1;
    n_controller_cases = length(Controllers_case_list);
    if MAX_SIMULATIONS > -1
        n_total_cases = min(n_controller_cases * n_wind_cases, MAX_SIMULATIONS);
    else
        n_total_cases = n_controller_cases * n_wind_cases;
    end
    case_list = repmat(struct(), n_total_cases, 1 );
    case_idx = 1;
    for c_idx = 1:n_controller_cases
        for w_idx = 1:n_wind_cases
            % case_idx = (c_idx - 1) * n_controller_cases + w_idx;
            sprintf(['Generating case ', num2str(case_idx)]);
            for fn1 = fieldnames(Wind_case_list(w_idx))'
                for fn2 = fieldnames(Wind_case_list(w_idx).(fn1{1}))'
                    case_list(case_idx).(fn1{1}).(fn2{1}) = Wind_case_list(w_idx).(fn1{1}).(fn2{1});
                end
            end
            for fn1 = fieldnames(Controllers_case_list(c_idx))'
                case_list(case_idx).(fn1{1}) = Controllers_case_list(c_idx).(fn1{1});
            end
            if strcmp(WIND_TYPE, 'turbsim')
                [~,x,~] = fileparts(case_list(case_idx).InflowWind.FileName_BTS);
                x = split(x, '_');
                x = str2num(x{2});
                case_list(case_idx).InflowWind.HWindSpeed = x;
            end
            case_idx = case_idx + 1;
            if MAX_SIMULATIONS > -1 && case_idx > MAX_SIMULATIONS
                break;
            end
        end
        if MAX_SIMULATIONS > -1 && case_idx > MAX_SIMULATIONS
            break;
        end
    end
    % n_cases = case_idx - 1;
    n_cases = length(case_list);
    case_name_list = arrayfun(@(n) ['case_', num2str(n)], 1:n_cases, 'UniformOutput', false);
    
    if MAX_SIMULATIONS > -1
        save_prefix = 'debug_';
    else
        save_prefix = '';
    end

    save(fullfile(mat_save_dir, [save_prefix, 'Extreme_Controllers_nonlinear_simulation_case_list.mat']), "case_list", "case_name_list", '-v7.3');
    
elseif ~USE_IPC % no ipc
    case_list = Wind_case_list;
    n_cases = n_wind_cases;
    case_name_list = Wind_case_name_list;
    save(fullfile(mat_save_dir, 'noIPC_nonlinear_simulation_case_list.mat'), "case_list", "case_name_list", '-v7.3');

end
