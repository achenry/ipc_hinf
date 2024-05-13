%% Generate Simulation Cases

% Generate Turbsim Cases
class = 'A'; % class A

if sum([RUN_OL_DQ, RUN_OL_BLADE, RUN_CL]) == 1
    CaseGen.dir_matrix = FAST_runDirectory;
    CaseGen.namebase = FAST_SimulinkModel;
    [~, CaseGen.model_name, ~] = fileparts(fastRunner.FAST_InputFile);
    if OPTIMAL_K_COLLECTION || EXTREME_K_COLLECTION || DEBUG
        ctrl_types = fieldnames(case_basis.W1Gain);
    end
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
    load(fullfile(mat_save_dir, [sim_type, '_Controllers_case_list.mat']));
    Controllers_case_list = PI_ParameterSweep_case_list;
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
            % for fn1 = fieldnames(PI_ParameterSweep_case_list(c_idx))'
            %     case_list(case_idx).(fn1{1}) = PI_ParameterSweep_case_list(c_idx).(fn1{1});
            % end
            for fn1 = fieldnames(Controllers_case_list(c_idx))'
                if isfield(Controllers_case_list(c_idx).(fn1{1}), 'x')
                    case_list(case_idx).(fn1{1}) = Controllers_case_list(c_idx).(fn1{1}).x;
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

    save(fullfile(mat_save_dir, [sim_type, '_Controllers_nonlinear_simulation_case_list.mat']), "case_list", "case_name_list", '-v7.3');
    
elseif OPTIMAL_K_COLLECTION || EXTREME_K_COLLECTION || DEBUG
    % Load Controller Cases corresponding to scheduled full-order controllers
    load(fullfile(mat_save_dir, [sim_type, '_Controllers_case_list.mat']));

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
                    elseif isfield(Controllers_case_list(c_idx).(fn1{1}), 'x')
                        case_list(case_idx).(fn1{1}) = Controllers_case_list(c_idx).(fn1{1}).x;
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
    % for c = 1:n_cases
    %     if strcmp(case_list(c).CaseDesc, 'adc W1 = 1 W2 = 0.01 ->  Wu = 1 We = 0.1') || strcmp(case_list(c).CaseDesc, 'y_mse W1 = 1 W2 = 0.1 ->  Wu = 10 We = 10')
    %         disp([case_list(c).CaseDesc, num2str(c)])
    %     end
    % end
    case_name_list = arrayfun(@(n) ['case_', num2str(n)], 1:n_cases, 'UniformOutput', false);
    
    if MAX_SIMULATIONS > -1
        save_prefix = 'debug_';
    else
        save_prefix = '';
    end
    save(fullfile(mat_save_dir, [sim_type, '_Controllers_nonlinear_simulation_case_list.mat']), "case_list", "case_name_list", '-v7.3');

end
