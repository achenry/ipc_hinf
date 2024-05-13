%% README- Run nonlinear simulations for different controllers in Simulink, make sure that sim_type is set to something other than BASELINE_K
%   compute power spectra of different loads
%   compute ADC of blade-pitch

% compute power spectra of different rotating and non-rotating loads for
% different controllers
% compute ADC for different controllers
% plot Beta, BetaDot, BetaDDdot for different controllers

% run a parfor loop to simulate 10 minutes of a turbsim wind field for each
% controller type with IPC on
% config;
% VARY_WU = 1;
initialize;
single_run_case_idx = 100;

% blade_op_arr = {'OoPDefl1', 'IPDefl1', ..., 'TwstDefl1', ...
%           'RotThrust', ...
%           'TTDspFA', 'TTDspSS', 'TTDspTwst', ... % Tower-top / yaw bearing fore-aft/side-to-side/angular torsion deflection
%           'RootMxb1', 'RootMyb1', ... % Blade 1 edgewise/flapwise moment 
%           'RootMyc1', 'RootMzc1', ... % Blade 1 out-of-plane/pitching moment
%           'LSSGagMya', 'LSSGagMza', ... % Rotating low-speed shaft bending moment at the shaft's strain gage (about ya/za axis)
%           'YawBrMxp', 'YawBrMyp', 'YawBrMzp', ... % Nonrotating tower-top / yaw bearing roll/pitch/yaw moment
%           'TwrBsMxt', 'TwrBsMyt', 'TwrBsMzt' ... % Tower base roll (or side-to-side)/pitching (or fore-aft)/yaw moment 
%           };
blade_op_arr = {'BldPitch1', 'RootMyc1', 'RootMxb1', 'YawBrMyp', 'YawBrMzp'};
% blade_op_arr = {'BldPitch1', 'RootMyc1'};
% 'LSSTipMys', 'LSSTipMzs'

% exc_dq_fields = {'BldPitchD', 'BldPitchQ'};
exc_dq_fields = {};
dq_op_arr = {};
for l = dqOutList'
    if sum(strmatch(l, exc_dq_fields)) == 0 && (strcmp(l{1}(end), 'D') || strcmp(l{1}(end), 'Q'))
        dq_op_arr = [dq_op_arr, l{1}];
    end
end

blade_op_indices = [];
for i = 1:length(blade_op_arr)
    blade_op_indices = [blade_op_indices strmatch(blade_op_arr{i}, OutList)];
end
dq_op_indices = [];
for i = 1:length(dq_op_arr)
    dq_op_indices = [dq_op_indices strmatch(dq_op_arr{i}, dqOutList)];
end

if EXTREME_K_COLLECTION
    gain_col = 'A_Wu';
elseif OPTIMAL_K_COLLECTION
    if contains(sim_type, "wu")
        gain_col = 'A_Wu';
    elseif contains(sim_type, "ref")
        gain_col = 'Reference';
    elseif contains(sim_type, "sat")
        gain_col = 'Saturation';
    elseif contains(sim_type, "sched")
        gain_col = 'A_Wu';
    end
end

%% Load Controller Tuning & Simulation Data
load(fullfile(mat_save_dir, [sim_type, '_Controllers_nonlinear_simulation_case_list.mat'])); % case list
load(fullfile(mat_save_dir, [sim_type, '_Controllers_case_table.mat'])); % case list

if STRUCT_PARAM_SWEEP
    Controllers_case_table = PI_ParameterSweep_redtable;
elseif OPTIMAL_K_COLLECTION || EXTREME_K_COLLECTION || DEBUG
    sim_out_list.controller = load(fullfile(sl_metadata_save_dir, ['sim_out_list_', sim_type, '.mat'])).sim_out_list;
end

untuned_sim_out_list = load(fullfile(sl_metadata_save_dir, 'sim_out_list_baseline_k_turbsim.mat')).sim_out_list; % load noipc case and baseline cases
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

%% Analysis of Untuned Blade-Pitch Actuation and Loads in Time-Domain

beta_dot_norm = @(beta_dot) ((beta_dot >= 0) * 5) + ((beta_dot < 0) * (-4));
if 0
    % want to compare all cases for the same wind field mean speed and
    % seed, choose first in case_basis for comparison
    
    % dq and blade values of blade-pitch and blade root bending moment for
    % particular wind field


    % Controllers_case_table = renamevars(Controllers_case_table, 'RootMycBlade1 MSE', 'RootMycBlade1 RMSE')
    % save(fullfile(mat_save_dir, 'Optimal_Controllers_case_table.mat'), 'Controllers_case_table')
    
     % add rows corresponding to ipc and baseline cases
    load(fullfile(mat_save_dir, 'PI_BaselineParameters_table.mat'));
    UntunedControllers_case_table = PI_ParameterSweep_table;
    UntunedControllers_case_table.("Case Desc.") = cell2mat({"noipc"; "baseline_controller"});
    UntunedControllers_case_table.("TunedWindSpeed") = nan * ones(2, 1);

    lin_cols = UntunedControllers_case_table.Properties.VariableNames;
    lin_cols(ismember(lin_cols, {'ADC', 'RootMycBlade1 RMSE'})) = [];

    for untuned_type = {'noipc', 'baseline_controller'}
        for untuned_case_idx = 1:length(sim_out_list.(untuned_type{1}))
                
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
            
            if exist([sim_out_list.(untuned_type{1})(untuned_case_idx).outdata_save_fn '_' case_no, '.mat'])
                values = load([sim_out_list.(untuned_type{1})(untuned_case_idx).outdata_save_fn '_' case_no]);
            else
                values = load([sim_out_list.(untuned_type{1})(untuned_case_idx).outdata_save_fn]);
            end
            % values = load([sim_out_list.(untuned_type{1})(untuned_case_idx).outdata_save_fn]);
            values = values.OutData';
            values = values(floor(cut_transients / DT):end, 2:end); % remove extra time column
            
            if exist([sim_out_list.(untuned_type{1})(untuned_case_idx).blpitch_save_fn '_' case_no, '.mat'])
                ipc_values = load([sim_out_list.(untuned_type{1})(untuned_case_idx).blpitch_save_fn '_' case_no]);
            else
                ipc_values = load([sim_out_list.(untuned_type{1})(untuned_case_idx).blpitch_save_fn]);
            end

            ipc_values = ipc_values.BlPitch';
            ipc_values = ipc_values(floor(cut_transients / DT):end, 2:end);
            dqValues = mbcTransformOutData(values, OutList);
            % dqValues(:, ismember(dqOutList, 'BldPitchD') | ismember(dqOutList, 'BldPitchQ'))

            x = fft(values(:, blade_op_indices), size(values, 1), 1);
            % x = fft(dqValues(:, dq_op_indices), size(dqValues, 1), 1);
            sim_out_list.(untuned_type{1})(untuned_case_idx).fft_peaks ...
                = computeFFTPeaks(x, 'blade', DT, omega_1P_rad / (2 * pi), [0,1,2,3,4]);
            
            time = values(:, ismember(OutList, 'Time'));
            az = values(:, ismember(OutList, 'Azimuth'));
            relValues = [values(:, ismember(OutList, 'RootMyc1')), values(:, ismember(OutList, 'BldPitch1')), values(:, ismember(OutList, 'GenPwr'))];
            relDqValues = [dqValues(:, ismember(dqOutList, 'RootMycD')), dqValues(:, ismember(dqOutList, 'RootMycQ'))];

            if 0

                figure;
                L = size(relDqValues, 1);
                f = (1 / (DT * L)) * (0:L/2) * 2 * pi;
                x = (1 / L) * abs(x);
                x = x(1:L/2 + 1, :);
                x(2:end-1, :) = 2 * x(2:end-1, :);
                plot(f, P1,"LineWidth", 2);
                title("DQ Single-Sided Amplitude Spectrum", 'Interpreter', 'latex')
        
                xline(harmonics);
                xlim([0, harmonics(end) * 1.25])
                xlabel(['$f$', ' [Hz]'], 'Interpreter', 'latex')
            
                linkaxes([ax1, ax2], 'xy')

                plotSpectra(...
                    x, ...
                    {'RootMyc1'}, {'RootMycD', 'RootMycQ'}, ...
                    'psd', DT, omega_1P_rad * [0, 3] * (1 / (2*pi)));
            end
        
            % sim_out_list.(untuned_type{1})(untuned_case_idx).ux = ...
            %     getData(values, OutList, 'Wind1VelX');
            
            % sim_out_list.(untuned_type{1})(untuned_case_idx).beta = struct;
            
            % sim_out_list.(untuned_type{1})(untuned_case_idx).beta.blade = ...
            %     getData(values, OutList, 'BldPitch1'); % in degrees
            % sim_out_list.(untuned_type{1})(untuned_case_idx).beta.dq = ...
            %     [getData(dqValues, dqOutList, 'BldPitchD'),...
            %         getData(dqValues, dqOutList, 'BldPitchQ')]; % in degrees

            sim_out_list.(untuned_type{1})(untuned_case_idx).beta_ipc = struct;
            sim_out_list.(untuned_type{1})(untuned_case_idx).beta_ipc.dq = ...
                ipc_values(:, 1:3); % in degrees

            sim_out_list.(untuned_type{1})(untuned_case_idx).beta_ipc_sat = struct;
            sim_out_list.(untuned_type{1})(untuned_case_idx).beta_ipc_sat.dq = ...
                ipc_values(:, 4:6);

            % sim_out_list.(untuned_type{1})(untuned_case_idx).beta_dot = struct;
            % sim_out_list.(untuned_type{1})(untuned_case_idx).beta_dot.blade = ...
            %     diff(sim_out_list.(untuned_type{1})(untuned_case_idx).beta.blade) / DT;
            
            sim_out_list.(untuned_type{1})(untuned_case_idx).RootMyc = struct;
            sim_out_list.(untuned_type{1})(untuned_case_idx).RootMyc.blade = relValues(:, 1);
                % getData(values, OutList, 'RootMyc1');
            sim_out_list.(untuned_type{1})(untuned_case_idx).RootMyc.dq = relDqValues(:, 1:2);
                % [getData(dqValues, dqOutList, 'RootMycD'), ...
                %     getData(dqValues, dqOutList, 'RootMycQ')];
            
            sim_out_list.(untuned_type{1})(untuned_case_idx).RootMycRMSE = struct;
            
            % x = sim_out_list.(untuned_type{1})(untuned_case_idx).RootMyc.blade;
            % x = getData(values, OutList, 'RootMyc1');
            x = relValues(:, 1);
            sim_out_list.(untuned_type{1})(untuned_case_idx).RootMycRMSE.blade = ...
                sqrt((1 / length(x)) * sum(x.^2));
            
            % x = sim_out_list.(untuned_type{1})(untuned_case_idx).RootMyc.dq;
            % x = [getData(dqValues, dqOutList, 'RootMycD'), ...
            %      getData(dqValues, dqOutList, 'RootMycQ')];x
            x = relDqValues(:, 1:2);
            sim_out_list.(untuned_type{1})(untuned_case_idx).RootMycRMSE.dq = ...
                sqrt((1 / size(x, 1)) * sum(x.^2, 1));
            
            % x = sim_out_list.(untuned_type{1})(untuned_case_idx).beta_dot.blade;
            % x = diff(getData(values, OutList, 'BldPitch1')) / DT;
            % x = diff(values(:, 2)) / DT;
            x = filter(drvt_filt, relValues(:, 2)) / DT;
            delay = mean(grpdelay(drvt_filt));
            x(1:delay) = [];
            x(1:delay) = [];

            % x2 = gradient(values(:, 2), DT);
            sim_out_list.(untuned_type{1})(untuned_case_idx).ADC = ...
                (1 / length(x)) * sum((x ./ beta_dot_norm(x)));

            if 1 && untuned_case_idx == 1
                % n_rotation_steps = ceil(((az * pi / 180) / omega_1P_rad) / DT);
                
                figure(find(ismember({'noipc', 'baseline_controller'}, untuned_type)));
                ax1 = subplot(3, 1, 1);
                plot(time, az); ylabel('Azimuth [deg]')
                ax2 = subplot(3, 1, 2);
                plot(time, relValues(:, 2)); ylabel('BldPitch1 [deg]')
                ax3 = subplot(3, 1, 3);
                plot(time(delay+1:end-delay), x); hold on; plot(time(2:end), diff(relValues(:, 2)) / DT); ylabel('dBldPitch1/dt [deg/s]')
                legend("smoothed diff", "forward difference")
                linkaxes([ax1, ax2, ax3], 'x')
                periods =  find(abs(az - az(1)) < 0.2);
                last_step = periods(2);
                xlim([time(delay+1), time(delay+1+last_step)]);
                savefig(gcf, fullfile(fig_dir, [untuned_type{1} '_vals' '.fig']));
                saveas(gcf, fullfile(fig_dir, [untuned_type{1} '_vals' '.png']));


                % ax4 = subplot(4, 1, 4);
                % ipc_blade_vals = invMbcTransformOutData(ipc_values(:, 1:3), {'BldPitchC', 'BldPitchD', 'BldPitchQ'});
                % x2 = filter(drvt_filt, ipc_blade_vals) / DT;
                % delay = mean(grpdelay(drvt_filt));
                % x2(1:delay) = [];
                % x2(1:delay) = [];
                % plot(time(2*delay+1:end), x2); hold on; plot(time(2:end), diff(ipc_blade_vals) / DT);
                % linkaxes([ax1, ax2, ax3, ax4], 'x')
            %     % MANUEL - is it okay to filter beta_dot like this... 
            %     pwelch(values(:, 2), [], [], [], 1 / DT)
            %     x2 = (values(3:end, 2) - values(1:(end - 2), 2)) / (2 * DT);
            %     figure(1)
            %     plot(x); hold on; plot(x2);
            %     % 1;
            % 
            %     x2 = filter(drvt_filt, values(:, 2)) / DT;
            %     delay = mean(grpdelay(drvt_filt));
            %     tt = 1:(length(x2)-delay) * DT;
            %     x2(1:delay) = [];
            % 
            %     % x2(1:delay) = [];
            %     tt(1:delay) = [];
            %     figure(1)
            %     plot(x); hold on; plot(x2) % check beta_dot
            end

            sim_out_list.(untuned_type{1})(untuned_case_idx).GenPwr = relValues(:, 3);
            sim_out_list.(untuned_type{1})(untuned_case_idx).GenPwrMean = mean(relValues(:, 3));

            sim_out_list.(untuned_type{1})(untuned_case_idx).CaseDesc = untuned_type{1};

        end
    end
    
    if 0
        figure;
        for c = 1:length(sim_out_list.("baseline_controller"))
            subplot(2, 1, 1);
            plot(sim_out_list.("baseline_controller")(c).ux); hold on;
            subplot(2, 1, 2);
            plot(sim_out_list.("baseline_controller")(c).beta_ipc.dq); hold on;
        end
         idx = 7; plot(values(:, idx)); title(OutList{idx});
    end
    
    % make a new table with a row for every nonlinear simulation conducted
    % with untuned controllers, and add variables from linear analysis
    % (UntunedControllers_case_table) and nonlinear analysis
    % (sim_out_list.noipc, sim_out_list.baseline)
    UntunedControllers_simulation_case_table = table();
    UntunedControllers_fft_case_table = table();
    case_idx = 1;
    mean_M_dq = [];
    for untuned_type = {'noipc', 'baseline_controller'}
        for untuned_case_idx = 1:length(sim_out_list.(untuned_type{1}))
            
            for l = 1:length(blade_op_arr)
                load_type = blade_op_arr(l);
                for h = 1:length(sim_out_list.(untuned_type{1})(untuned_case_idx).fft_peaks.harmonics)
                    harmonic = sim_out_list.(untuned_type{1})(untuned_case_idx).fft_peaks.harmonics(h);
                    UntunedControllers_fft_case_table(case_idx, [load_type{1}, '_', num2str(harmonic), 'P']) ...
                        = {sim_out_list.(untuned_type{1})(untuned_case_idx).fft_peaks.blade(h, l)};
                end
            end
            UntunedControllers_fft_case_table(case_idx, 'Case Desc.') = untuned_type;
            cond = strcmp(UntunedControllers_case_table.("Case Desc."), ...
                            untuned_type{1});
            
            for f = lin_cols
                if sum(ismember(UntunedControllers_case_table.Properties.VariableNames, f{1}))
                    UntunedControllers_simulation_case_table(case_idx, f{1}) = {UntunedControllers_case_table(cond, f{1}).Variables};
                end
            end

            [~, x, ~] = fileparts(sim_out_list.(untuned_type{1})(untuned_case_idx).InflowWind.FileName_BTS);
            UntunedControllers_simulation_case_table(case_idx, 'WindField') = {x};
            UntunedControllers_fft_case_table(case_idx, 'WindField') = {x};
            x = split(x, '_');
            UntunedControllers_simulation_case_table(case_idx, 'WindTurbulence') = x(1);
            UntunedControllers_simulation_case_table(case_idx, 'WindMean') = {str2double(x(2))};
            UntunedControllers_simulation_case_table(case_idx, 'WindSeed') = {str2double(x{3})};
            UntunedControllers_fft_case_table(case_idx, 'WindTurbulence') = x(1);
            UntunedControllers_fft_case_table(case_idx, 'WindMean') = {str2double(x(2))};
            UntunedControllers_fft_case_table(case_idx, 'WindSeed') = {str2double(x{3})};
    
            UntunedControllers_simulation_case_table(case_idx, ...
                {'ADC', 'RootMycBlade1 RMSE', 'RootMycD RMSE', 'RootMycQ RMSE', 'GenPwr Mean'}) = ...
                    table(...
                    sim_out_list.(untuned_type{1})(untuned_case_idx).ADC, ...
                    sim_out_list.(untuned_type{1})(untuned_case_idx).RootMycRMSE.blade, ...
                    sim_out_list.(untuned_type{1})(untuned_case_idx).RootMycRMSE.dq(1), ...
                    sim_out_list.(untuned_type{1})(untuned_case_idx).RootMycRMSE.dq(2), ...
                    sim_out_list.(untuned_type{1})(untuned_case_idx).GenPwrMean, ...
                    'VariableNames', {'ADC', 'RootMycBlade1 RMSE', 'RootMycD RMSE', 'RootMycQ RMSE', 'GenPwr Mean'});

            if strcmp(untuned_type, 'noipc')
                mean_M_dq = [mean_M_dq; mean((sim_out_list.(untuned_type{1})(untuned_case_idx).RootMyc.dq), 1)];
            end
    
            case_idx = case_idx + 1;
        end
    end

    M_dq_reference = round(mean(mean_M_dq, 1), 1); 
    save(fullfile(mat_save_dir, 'M_dq_reference.mat'), "M_dq_reference");

    %% Aggregate untuned and no ipc cases
    agg_cols = UntunedControllers_fft_case_table.Properties.VariableNames;
    agg_cols(ismember(agg_cols, 'Case Desc.')) = [];
    agg_cols(ismember(agg_cols, 'WindField')) = [];
    agg_cols(ismember(agg_cols, 'WindMean')) = [];
    agg_cols(ismember(agg_cols, 'WindTurbulence')) = [];
    agg_cols(ismember(agg_cols, 'WindSeed')) = [];

    UntunedControllers_fft_case_agg_table = ...
        groupsummary(UntunedControllers_fft_case_table, ...
        ["Case Desc.", "WindMean", "WindTurbulence"], "mean", agg_cols);
        
    % clear UntunedControllers_fft_case_table;
    save(fullfile(mat_save_dir, 'untuned_k_turbsim_fft_case_agg_table.mat'), 'UntunedControllers_fft_case_agg_table');

    agg_cols = UntunedControllers_simulation_case_table.Properties.VariableNames;
    agg_cols(ismember(agg_cols, 'Case No.')) = [];
    agg_cols(ismember(agg_cols, 'Case Desc.')) = [];
    agg_cols(ismember(agg_cols, 'Stable')) = [];
    agg_cols(ismember(agg_cols, 'TunedWindSpeed')) = [];
    agg_cols(ismember(agg_cols, 'WindField')) = [];
    agg_cols(ismember(agg_cols, 'WindMean')) = [];
    agg_cols(ismember(agg_cols, 'WindTurbulence')) = [];
    agg_cols(ismember(agg_cols, 'WindSeed')) = [];
    UntunedControllers_simulation_case_agg_table = ...
        groupsummary(UntunedControllers_simulation_case_table, ...
        ["Case Desc.", "TunedWindSpeed", "WindMean", "WindTurbulence"], ...
        "mean", agg_cols)
    % clear UntunedControllers_simulation_case_table;
    save(fullfile(mat_save_dir, 'untuned_k_turbsim_simulation_case_table.mat'), 'UntunedControllers_simulation_case_table');
    save(fullfile(mat_save_dir, 'untuned_k_turbsim_simulation_case_agg_table.mat'), 'UntunedControllers_simulation_case_agg_table');

else
    load(fullfile(mat_save_dir, 'untuned_k_turbsim_simulation_case_table.mat'));
    load(fullfile(mat_save_dir, 'untuned_k_turbsim_simulation_case_agg_table.mat'));
    load(fullfile(mat_save_dir, 'untuned_k_turbsim_fft_case_agg_table.mat'));
end

%% Analysis of Tuned Blade-Pitch Actuation and Loads in Time-Domain
if 1
        
    n_ctrl_cases = length(sim_out_list.controller);
    sim_out_list_tmp = repmat(struct(), n_ctrl_cases, 1);
    outdata_filenames = cell(n_ctrl_cases, 1);
    [outdata_filenames{:}] = sim_out_list.controller.outdata_save_fn;
    blpitch_filenames = cell(n_ctrl_cases, 1);
    [blpitch_filenames{:}] = sim_out_list.controller.blpitch_save_fn;
    single_run_case_idx = [2:2:60, 241:2:299]; % TODO generalize this across nonlinear_simulations.m and here
    single_run_case_idx = 1:n_ctrl_cases;
    % parfor ctrl_case_idx = single_run_case_idx
    % for ctrl_case_idx = single_run_case_idx
    % sim_out_list_tmp = struct.empty(n_ctrl_cases, 0);
    parfor case_no = 1:length(single_run_case_idx)
    % for ctrl_case_idx = 1:n_ctrl_cases
        ctrl_case_idx = single_run_case_idx(case_no);
        [~, x, ~] = fileparts(outdata_filenames{ctrl_case_idx});
        x = split(x, '_');
        % case_no = num2str(x{2});
        % case_no = num2str(find(ctrl_case_idx == single_run_case_idx));
        

        if exist([outdata_filenames{ctrl_case_idx} '_' num2str(case_no) '.mat'], 'file')
            values = load([outdata_filenames{ctrl_case_idx} '_' num2str(case_no)]);
        else
            values = load(outdata_filenames{ctrl_case_idx});
        end

        values = values.OutData';
        values = values(floor(cut_transients / DT):end, 2:end); % remove extra time column

        if exist([blpitch_filenames{ctrl_case_idx} '_' num2str(case_no), '.mat'], 'file')
            ipc_values = load([blpitch_filenames{ctrl_case_idx} '_' num2str(case_no)]);
        else
            ipc_values = load([blpitch_filenames{ctrl_case_idx}]);
        end

        ipc_values = ipc_values.BlPitch'; % time, beta_dq_sat, is_sat_dq
        ipc_values = ipc_values(floor(cut_transients / DT):end, 2:end);
        % t = getData(values, OutList, 'Time');
        dqValues = mbcTransformOutData(values, OutList);
        
        az = values(:, ismember(OutList, 'Azimuth'));
        time = values(:, ismember(OutList, 'Time'));
        relValues = [values(:, ismember(OutList, 'RootMyc1')), values(:, ismember(OutList, 'BldPitch1')), values(:, ismember(OutList, 'GenPwr'))];
        relDqValues = [dqValues(:, ismember(dqOutList, 'RootMycD')), dqValues(:, ismember(dqOutList, 'RootMycQ'))];
        
        % sim_out_list_tmp(ctrl_case_idx).ux = ...
        %     getData(values, OutList, 'Wind1VelX');
        
        % sim_out_list_tmp(ctrl_case_idx).beta = struct;
        % sim_out_list_tmp(ctrl_case_idx).beta.blade = ...
        %     getData(values, OutList, 'BldPitch1'); % in degrees
        % sim_out_list_tmp(ctrl_case_idx).beta.dq = ...
        %     [getData(dqValues, dqOutList, 'BldPitchD'), ...
        %     getData(dqValues, dqOutList, 'BldPitchQ')]; % in degrees

        sim_out_list_tmp(case_no).beta_ipc = struct;
        % sim_out_list_tmp(ctrl_case_idx).beta_ipc.dq = ...
        %             ipc_values(:, 1:2); % in degrees
        sim_out_list_tmp(case_no).beta_ipc.blade = ...
                    ipc_values(:, 1:3);

        sim_out_list_tmp(case_no).beta_ipc_sat = struct;
        % sim_out_list_tmp(ctrl_case_idx).beta_ipc_sat.dq = ...
        %             ipc_values(:, 3:4);
        sim_out_list_tmp(case_no).beta_ipc_sat.blade = ...
                    ipc_values(:, 4:6);

        % sim_out_list_tmp(ctrl_case_idx).beta_dot = struct;
        % sim_out_list_tmp(ctrl_case_idx).beta_dot.blade = ...
        %     diff(sim_out_list_tmp(ctrl_case_idx).beta.blade) / DT;
        
        sim_out_list_tmp(case_no).RootMyc = struct;
        sim_out_list_tmp(case_no).RootMyc.blade = relValues(:, 1);
            % getData(values, OutList, 'RootMyc1');
        sim_out_list_tmp(case_no).RootMyc.dq = relDqValues(:, 1:2);
            % [getData(dqValues, dqOutList, 'RootMycD'), ...
            % getData(dqValues, dqOutList, 'RootMycQ')];
        
        sim_out_list_tmp(case_no).RootMycRMSE = struct;

        % x = sim_out_list_tmp(ctrl_case_idx).RootMyc.blade;
        % x = getData(values, OutList, 'RootMyc1');
        x = relValues(:, 1);
        sim_out_list_tmp(case_no).RootMycRMSE.blade = ...
            sqrt((1 / length(x)) * sum(x.^2));
        
        % x = sim_out_list_tmp(ctrl_case_idx).RootMyc.dq;
        % x = [getData(dqValues, dqOutList, 'RootMycD'), ...
        %     getData(dqValues, dqOutList, 'RootMycQ')];
        x = relDqValues(:, 1:2);
        sim_out_list_tmp(case_no).RootMycRMSE.dq = ...
            sqrt((1 / size(x, 1)) * sum(x.^2, 1));
        
        % x = sim_out_list_tmp(ctrl_case_idx).beta_dot.blade;
        % x = diff(getData(values, OutList, 'BldPitch1')) / DT;
        % QUESTION MANUEL is this okay
        % x = diff(values(:, 2)) / DT;
        x = filter(drvt_filt, relValues(:, 2)) / DT;
        delay = mean(grpdelay(drvt_filt));
        x(1:delay) = [];
        x(1:delay) = [];
        sim_out_list_tmp(case_no).ADC = ...
            (1 / length(x)) * sum((x ./ beta_dot_norm(x)));

        if 0 && ctrl_case_idx == 1
                figure(1);
                ax1 = subplot(3, 1, 1);
                plot(time, az);
                ax2 = subplot(3, 1, 2);
                plot(time, relValues(:, 2));
                ax3 = subplot(3, 1, 3);
                plot(time(delay+1:end-delay), x); hold on; plot(time(2:end), diff(relValues(:, 2)) / DT);
                linkaxes([ax1, ax2, ax3], 'x')
        end

        sim_out_list_tmp(case_no).GenPwr = relValues(:, 3);
        sim_out_list_tmp(case_no).GenPwrMean = mean(relValues(:, 3));
        
    end

    % for ctrl_case_idx = 1:n_ctrl_cases
    for ctrl_case_idx = single_run_case_idx
        [~, x, ~] = fileparts(outdata_filenames{ctrl_case_idx});
        x = split(x, '_');
        % case_no = num2str(x{2});
        case_no = find(ctrl_case_idx == single_run_case_idx);

        if exist([outdata_filenames{ctrl_case_idx} '_' num2str(case_no), '.mat'], 'file')
            values = load([outdata_filenames{ctrl_case_idx} '_' num2str(case_no)]);
        else
            values = load(outdata_filenames{ctrl_case_idx});
        end

        values = values.OutData';
        values = values(floor(cut_transients / DT):end, 2:end); % remove extra time column

        dqValues = mbcTransformOutData(values, OutList);

        % x = fft(dqValues(:, dq_op_indices), size(dqValues, 1), 1);
        x = fft(values(:, blade_op_indices), size(values, 1), 1);
        sim_out_list_tmp(case_no).fft_peaks = computeFFTPeaks(...
            x, 'blade', DT, omega_1P_rad / (2 * pi), [0,1,2,3,4]);
        
        if 0
            plotSpectra(...
                sim_out_list.(untuned_type{1})(untuned_case_idx).fft.blade(:, ismember(blade_op_arr, {'RootMyc1'})), ...
                sim_out_list.(untuned_type{1})(untuned_case_idx).fft.dq(:, ismember(dq_outputs, {'RootMycD', 'RootMycQ'})), ...
                {'RootMyc1'}, {'RootMycD', 'RootMycQ'}, ...
                'psd', DT, omega_1P_rad * [0, 3] * (1 / (2*pi)));
        end
        
    end
    clear values dqValues relValues relDqValues
    
    % find minimum values of
    % sim_out_list.controller(ctrl_case_idx).(f{1}).RootMyc.dq over all 
    % simulataions to compute varying reference values from
    % min_M_dq = struct;
    % find maximum values of
    % sim_out_list.controller(ctrl_case_idx).(f{1}).beta_ipc.dq over all 
    % simulataions to compute varying saturation values from
    % max_beta_dq = struct;
    % mean_beta_dq = [];
    absmax_beta_ipc = [];
    load(fullfile(mat_save_dir, [sim_type, '_full_controller_cases']));

    lin_cols = Controllers_case_table.Properties.VariableNames;
    lin_cols(ismember(lin_cols, {'ADC', 'RootMycBlade1 RMSE', 'GenPwr Mean'})) = [];

    Controllers_simulation_case_table = table();
    Controllers_fft_case_table = table();

    % for ctrl_case_idx = 1:n_ctrl_cases
    for ctrl_case_idx = single_run_case_idx
        case_no = find(ctrl_case_idx == single_run_case_idx);
        % if case_list(ctrl_case_idx).Scheduling
        if SCHEDULING
            [~, x, ~] = fileparts(case_list(ctrl_case_idx).InflowWind.FileName_BTS);
            x = split(x, '_');
            cond = strcmp(Controllers_case_table.("Case Desc."), ...
                case_list(ctrl_case_idx).CaseDesc{1}) ...
                   & (Controllers_case_table.("TunedWindSpeed") == str2double(x{2}));
        else
            cond = strcmp(Controllers_case_table.("Case Desc."), ...
                case_list(ctrl_case_idx).CaseDesc{1}) ...
                    & (Controllers_case_table.("TunedWindSpeed") == NONLPV_CONTROLLER_WIND_SPEED);
        end
        for f = lin_cols
            if sum(ismember(Controllers_case_table.Properties.VariableNames, f{1}))
                Controllers_simulation_case_table(ctrl_case_idx, f{1}) = {Controllers_case_table(cond, f{1}).Variables};
            end
        end

        Controllers_fft_case_table(ctrl_case_idx, "Case Desc.") = Controllers_case_table(cond, "Case Desc.").Variables; % case_list(ctrl_case_idx).CaseDesc;
        
        Controllers_fft_case_table(ctrl_case_idx, ["A_W1", "A_W2", "A_We", "A_Wu", gain_col]) = Controllers_case_table(cond, ["A_W1", "A_W2", "A_We", "A_Wu", gain_col]);
        for l = 1:length(blade_op_arr)
            load_type = blade_op_arr(l);
            for h = 1:length(sim_out_list_tmp(case_no).fft_peaks.harmonics)
                harmonic = sim_out_list_tmp(case_no).fft_peaks.harmonics(h);
                Controllers_fft_case_table(ctrl_case_idx, [load_type{1}, '_', num2str(harmonic), 'P']) ...
                    = {sim_out_list_tmp(case_no).fft_peaks.blade(h, l)};
            end
        end

        [~, x, ~] = fileparts(case_list(ctrl_case_idx).InflowWind.FileName_BTS);
        Controllers_simulation_case_table(ctrl_case_idx, 'WindField') = {x};
        Controllers_fft_case_table(ctrl_case_idx, 'WindField') = {x};
        x = split(x, '_');
        Controllers_simulation_case_table(ctrl_case_idx, 'WindTurbulence') = x(1);
        Controllers_simulation_case_table(ctrl_case_idx, 'WindMean') = {str2double(x(2))};
        Controllers_simulation_case_table(ctrl_case_idx, 'WindSeed') = {str2double(x(3))};
        Controllers_fft_case_table(ctrl_case_idx, 'WindTurbulence') = x(1);
        Controllers_fft_case_table(ctrl_case_idx, 'WindMean') = {str2double(x(2))};
        Controllers_fft_case_table(ctrl_case_idx, 'WindSeed') = {str2double(x(3))};

        Controllers_simulation_case_table(ctrl_case_idx, ...
            {'ADC', 'RootMycBlade1 RMSE', 'RootMycD RMSE', 'RootMycQ RMSE', 'GenPwr Mean'}) = ...
            table(sim_out_list_tmp(case_no).ADC, ...
            sim_out_list_tmp(case_no).RootMycRMSE.blade, ...
            sim_out_list_tmp(case_no).RootMycRMSE.dq(1), ...
            sim_out_list_tmp(case_no).RootMycRMSE.dq(2), ...
            mean(sim_out_list_tmp(case_no).GenPwr), ...
                'VariableNames', {'ADC', 'RootMycBlade1 RMSE', 'RootMycD RMSE', 'RootMycQ RMSE', 'GenPwr Mean'});
        
        for f = fieldnames(sim_out_list_tmp(case_no))'
            sim_out_list.controller(ctrl_case_idx).(f{1}) = sim_out_list_tmp(case_no).(f{1});
        end

        % if (Controllers_simulation_case_table(ctrl_case_idx, 'WindMean').Variables == NONLPV_CONTROLLER_WIND_SPEED) ...
        %         && 
        if VARY_WU && (case_list(ctrl_case_idx).WuGain.Numerator{1,1} == case_basis.WuGain.x{end-1}.Numerator{1,1})
            % mean_beta_dq = [mean_beta_dq; mean((sim_out_list_tmp(ctrl_case_idx).beta_ipc.dq), 1)];
            absmax_beta_ipc = [absmax_beta_ipc; max(abs(sim_out_list_tmp(case_no).beta_ipc.blade), [], 1)];
        end

    end
    clear sim_out_list_tmp;
    sortrows(Controllers_simulation_case_table((Controllers_simulation_case_table.("TunedWindSpeed") == 16) & (Controllers_simulation_case_table.("WindMean") == 16), :), "RootMycBlade1 RMSE")
    sortrows(Controllers_simulation_case_table((Controllers_simulation_case_table.("TunedWindSpeed") == 16) & (Controllers_simulation_case_table.("WindMean") == 16), :), "RootMycD RMSE")
    sortrows(Controllers_simulation_case_table((Controllers_simulation_case_table.("TunedWindSpeed") == 16) & (Controllers_simulation_case_table.("WindMean") == 16), :), "RootMycQ RMSE")
    % Beta_dq_saturation = round(mean(mean_beta_dq, 1), 4);
    % save(fullfile(mat_save_dir, 'Beta_dq_saturation.mat'), "Beta_dq_saturation");
    if VARY_WU
        Beta_ipc_blade_saturation = round(mean(absmax_beta_ipc, 1), 4); % mean over all VARY_WU controller cases for each blade
        Beta_ipc_blade_saturation = max(Beta_ipc_blade_saturation);
        save(fullfile(mat_save_dir, 'Beta_ipc_blade_saturation.mat'), "Beta_ipc_blade_saturation");
    end
    
    % Controllers_simulation_case_table = sortrows(Controllers_simulation_case_table, 'Case Desc.');
    
    % average Untuned_case_table and Controllers_case_table over
    % wind field seeds for each mean wind speed and case no.
    % Controllers_case_table = renamevars(Controllers_case_table, {'Wind Speed'}, {'TunedWindSpeed'})
    
    agg_cols = Controllers_fft_case_table.Properties.VariableNames;
    agg_cols(ismember(agg_cols, 'Case Desc.')) = [];
    agg_cols(ismember(agg_cols, 'WindField')) = [];
    agg_cols(ismember(agg_cols, 'WindMean')) = [];
    agg_cols(ismember(agg_cols, 'WindTurbulence')) = [];
    agg_cols(ismember(agg_cols, 'WindSeed')) = [];

    Controllers_fft_case_agg_table = ...
        groupsummary(Controllers_fft_case_table, ...
        ["Case Desc.", "WindMean", "WindTurbulence"], "mean", agg_cols);
    
    Controllers_fft_case_agg_table = renamevars(Controllers_fft_case_agg_table, {'mean_A_W1', 'mean_A_W2', 'mean_A_We', 'mean_A_Wu', ['mean_', gain_col]}, {'A_W1', 'A_W2', 'A_We', 'A_Wu', gain_col})
        
    % clear Controllers_fft_case_table;

    save(fullfile(mat_save_dir, [sim_type, '_fft_case_agg_table.mat']), 'Controllers_fft_case_agg_table');
    save(fullfile(mat_save_dir, [sim_type, '_fft_case_table.mat']), 'Controllers_fft_case_table');

    agg_cols = Controllers_simulation_case_table.Properties.VariableNames;
    agg_cols(ismember(agg_cols, 'Case No.')) = [];
    agg_cols(ismember(agg_cols, 'Case Desc.')) = [];
    agg_cols(ismember(agg_cols, 'Stable')) = [];
    agg_cols(ismember(agg_cols, 'TunedWindSpeed')) = [];
    agg_cols(ismember(agg_cols, 'WindField')) = [];
    agg_cols(ismember(agg_cols, 'WindMean')) = [];
    agg_cols(ismember(agg_cols, 'WindTurbulence')) = [];
    agg_cols(ismember(agg_cols, 'WindSeed')) = [];

    Controllers_simulation_case_agg_table = groupsummary(Controllers_simulation_case_table, ["Case Desc.", "TunedWindSpeed", "WindMean", "WindTurbulence"], "mean", agg_cols)
    
    Controllers_simulation_case_agg_table = renamevars(Controllers_simulation_case_agg_table, {'mean_A_W1', 'mean_A_W2', 'mean_A_We', 'mean_A_Wu', ['mean_', gain_col]}, {'A_W1', 'A_W2', 'A_We', 'A_Wu', gain_col})
    
    % clear Controllers_simulation_case_table;

    save(fullfile(mat_save_dir, [sim_type, '_simulation_case_agg_table.mat']), 'Controllers_simulation_case_agg_table');
    save(fullfile(mat_save_dir, [sim_type, '_simulation_case_table.mat']), 'Controllers_simulation_case_table');
else
    load(fullfile(mat_save_dir, [sim_type, '_simulation_case_agg_table.mat']));
    load(fullfile(mat_save_dir, [sim_type, '_fft_case_table.mat']));
    load(fullfile(mat_save_dir, [sim_type, '_fft_case_agg_table.mat']));
    load(fullfile(mat_save_dir, [sim_type, '_simulation_case_table.mat']));
end

%% Generate PSD data
if  OPTIMAL_K_COLLECTION && 1

    metric_cols = Controllers_fft_case_agg_table.Properties.VariableNames;
    metric_cols(ismember(metric_cols, 'Case Desc.')) = [];
    metric_cols(ismember(metric_cols, 'WindMean')) = [];
    metric_cols(ismember(metric_cols, 'WindTurbulence')) = [];
    metric_cols(ismember(metric_cols, 'GroupCount')) = [];
    
    metric_cols(ismember(metric_cols, {'A_W1', 'A_W2', 'A_We', 'A_Wu', gain_col})) = [];
    % metric_cols(ismember(metric_cols, ...
    %     {'mean_OoPDeflD_0P', 'mean_OoPDeflD_3P', 'mean_OoPDeflQ_0P', 'mean_OoPDeflQ_3P', ...
    %      'mean_IPDeflD_0P', 'mean_IPDeflD_3P', 'mean_IPDeflQ_0P', 'mean_IPDeflQ_3P', ...
    %      'mean_TwstDeflD_0P', 'mean_TwstDeflD_3P', 'mean_TwstDeflQ_0P', 'mean_TwstDeflQ_3P'})) = [];
    
    wind_cols = arrayfun(@num2str, LPV_CONTROLLER_WIND_SPEEDS, 'UniformOutput', 0);
    op_cols = wind_cols;
    op_cols{end + 1} = "Load";
    
    op_cols{end + 1} = gain_col;
    psd_out_table = table();

    for m = metric_cols
        untuned_cases = UntunedControllers_fft_case_agg_table(:, ["Case Desc.", "WindMean", m{1}]);
        baseline_cases = sortrows(untuned_cases(strcmp(untuned_cases.("Case Desc."), "baseline_controller"), :), "WindMean");
        noipc_cases = sortrows(untuned_cases(strcmp(untuned_cases.("Case Desc."), "noipc"), :), "WindMean");
        baseline_cases.(gain_col) = zeros(height(baseline_cases), 1);
        % baseline_cases(:, m{1}) = (baseline_cases(:, m{1}) - noipc_cases(:, m{1})) ./ noipc_cases(:, m{1});
        
        tmp_table = table();
        n_ctrl_cases = height(unique(Controllers_fft_case_agg_table(:, "Case Desc.")));
        load_vals = cell(n_ctrl_cases + 1, 1); % add 1 for baseline case
        load_vals(:) = {replace(m{1}, 'mean_', '')};
        tmp_table.("Load") = load_vals;
        [x, i, ~] = unique(Controllers_fft_case_agg_table(:, "Case Desc."));
        x(end + 1, "Case Desc.") = {{'baseline_controller'}};
        tmp_table(:, "Case Desc.") = x;
        tmp_table.(gain_col) = [Controllers_fft_case_agg_table(i, gain_col).Variables; 0];
        tmp_table = sortrows(tmp_table, "Case Desc.");

        for ws = unique(Controllers_fft_case_agg_table.("WindMean"))'
            ctrl_cases = Controllers_fft_case_agg_table(Controllers_fft_case_agg_table.WindMean == ws, ["Case Desc.", "WindMean", m{1}, gain_col]);
            cases = [ctrl_cases; baseline_cases(baseline_cases.WindMean == ws, :)];
            cases(:, m{1}) = (cases(:, m{1}) - noipc_cases(noipc_cases.WindMean == ws, m{1})) ./ noipc_cases(noipc_cases.WindMean == ws, m{1});
            tmp_table.(num2str(ws)) = sortrows(cases, "Case Desc.").(m{1});
        end
        
        psd_out_table = [psd_out_table; tmp_table];
    end
    controller_types = {'rob', 'adc', 'y_mse'};
    for c = 1:length(controller_types)
        ctrl_type = controller_types{c};
        cond = contains(psd_out_table.("Case Desc."), ctrl_type);
        n_cases = sum(cond);
        vals = cell(n_cases, 1);
        vals(:) = {ctrl_type};
        psd_out_table(cond, "Case Desc.") = vals; 
    end
    cond = zeros(height(psd_out_table), 1);
    for op = {"RootMyc1_1P", "RootMyc1_2P", "YawBrMyp_0P", "YawBrMyp_3P", ...
            "YawBrMzp_0P", "YawBrMzp_3P"}
        cond = cond | strcmp(psd_out_table.("Load"), op{1});
    end
    % psd_out_table = psd_out_table(cond, :);
    % psd_out_table = sortrows(psd_out_table, ["Load", "Case Desc.", gain_col]);
    % cond = psd_out_table(:, wind_cols) <= -0.1;
    % cond = all(cond.Variables, 2);
    % psd_out_table = psd_out_table(cond, :);
    % psd_out_table.("mean_Change") = mean(psd_out_table(:, wind_cols), 2).Variables;
    % red_psd_out_table = table();
    % for l = unique(psd_out_table.("Load"))'
    %     x = sortrows(psd_out_table(strcmp(psd_out_table.("Load"), l{1}), :), "mean_Change");
    %     red_psd_out_table = [red_psd_out_table; x(1, :)];
    % end
    sortrows(psd_out_table(strcmp(psd_out_table.("Load"), "RootMyc1_1P"), :), "16")
    sortrows(psd_out_table(strcmp(psd_out_table.("Load"), "RootMyc1_2P"), :), "16")
    sortrows(psd_out_table(strcmp(psd_out_table.("Load"), "YawBrMyp_0P"), :), "16")
    sortrows(psd_out_table(strcmp(psd_out_table.("Load"), "YawBrMyp_3P"), :), "16")
    sortrows(psd_out_table(strcmp(psd_out_table.("Load"), "YawBrMzp_0P"), :), "16")
    sortrows(psd_out_table(strcmp(psd_out_table.("Load"), "YawBrMzp_3P"), :), "16")
    % writetable(red_psd_out_table, fullfile(fig_dir, [sim_type, '_psd_table.csv']));
    save(fullfile(mat_save_dir, [sim_type, '_psd_table']), 'psd_out_table');
        
    % RootMyc1_1P, RootMyc1_2P, YawBrMyp_0P, YawBrMyp_3P, YawBrMzp_0P, YawBrMzp_3P
end
%% Generate DEL data
if OPTIMAL_K_COLLECTION
    % run python DEL analysis
    % if ~exist(fullfile(postprocessing_save_dir, [sim_type, '_DELs.csv']), 'file') || 0
    %     % read python DEL analysis output
    %       save(fullfile(mat_save_dir, [sim_type, '_del_data.mat']), 'del_data_tuned');
    % else
    %     load(fullfile(mat_save_dir, [sim_type, '_del_data']));
    % end 
    del_data_tuned = sortrows(readtable(fullfile(postprocessing_save_dir, [sim_type, '_DELs.csv']), 'Delimiter', ','), "Var1");
    sortrows(del_data_tuned, "RootMyc1")
    % if ~exist(fullfile(postprocessing_save_dir, 'baseline_k_turbsim_DELs.csv'), 'file') || 0
    %     status = system(['/Users/aoifework/miniconda3/envs/weis_dev/bin/python3 ', fullfile(project_dir, 'postprocessing', 'main.py'), ' -st ', 'baseline_k_turbsim']);
    %     del_data_untuned = sortrows(readtable(fullfile(postprocessing_save_dir, 'baseline_k_turbsim_DELs.csv'), 'Delimiter', ','), "Var1");
    %     save(fullfile(mat_save_dir, 'baseline_k_turbsim_del_data.mat'), 'del_data_untuned');
    % else
    %     load(fullfile(mat_save_dir, 'baseline_k_turbsim_del_data.mat'));
    % end

    
    del_data_untuned = sortrows(readtable(fullfile(postprocessing_save_dir, 'baseline_k_turbsim_DELs.csv'), 'Delimiter', ','), "Var1");
        
    
    % del_data_untuned(strcmp(del_data_untuned.("Var1"), 'case_9_baseline_k_turbsim_outdata.mat'), :) = [];
    
    % add variables to del data for tuned wind speed, windmean, windturbulence,
    % case desc.

    wind_cols = arrayfun(@num2str, LPV_CONTROLLER_WIND_SPEEDS, 'UniformOutput', 0);
    % op_cols = wind_cols;
    
    new_cols = ["Case Desc.", "TunedWindSpeed", "WindMean", "WindTurbulence", "ADC", "GenPwr Mean"];
    
    % controller, not to minimum ADC controller
    new_cols(end + 1) = gain_col;

    for case_idx = 1:length(sim_out_list.controller)
        save_fn = sim_out_list.controller(case_idx).outdata_save_fn;
        [~, save_fn, ~] = fileparts(save_fn);
        del_data_idx = find(contains(del_data_tuned.("Var1"), save_fn));
        case_desc = sim_out_list.controller(case_idx).CaseDesc;
        wind_field =  sim_out_list.controller(case_idx).InflowWind.FileName_BTS;
        [~, wind_field, ~] = fileparts(wind_field);
        sim_data_idx = find(strcmp(Controllers_simulation_case_table.("Case Desc."), case_desc) ...
            & strcmp(Controllers_simulation_case_table.("WindField"), wind_field));
        del_data_tuned(del_data_idx, new_cols) = Controllers_simulation_case_table(sim_data_idx, new_cols);
    end
    agg_cols = del_data_tuned.Properties.VariableNames;
    agg_cols(ismember(agg_cols, ...
        {'Var1', 'Case Desc.', 'TunedWindSpeed', 'WindMean', 'WindTurbulence', ...
        gain_col})) = [];
    
% groupsummary(del_data_tuned, ...
%             ["Case Desc.", "TunedWindSpeed", "WindMean", gain_col])

    del_data_tuned = sortrows(groupsummary(del_data_tuned, ...
            ["Case Desc.", "TunedWindSpeed", "WindMean", "WindTurbulence", gain_col], ...
            "mean", agg_cols), ["Case Desc.", "WindMean", gain_col])
    
    new_cols = ["Case Desc.", "TunedWindSpeed", "WindMean", "WindTurbulence", "ADC", "GenPwr Mean"];
    for untuned_type = {'noipc', 'baseline_controller'}
        for untuned_case_idx = 1:length(sim_out_list.(untuned_type{1}))
            save_fn = sim_out_list.(untuned_type{1})(untuned_case_idx).outdata_save_fn;
            [~, save_fn, ~] = fileparts(save_fn);
            del_data_idx = find(contains(del_data_untuned.("Var1"), save_fn));
            wind_field =  sim_out_list.(untuned_type{1})(untuned_case_idx).InflowWind.FileName_BTS;
            [~, wind_field, ~] = fileparts(wind_field);
            sim_data_idx = find(strcmp(UntunedControllers_simulation_case_table.("Case Desc."), untuned_type{1}) ...
                & strcmp(UntunedControllers_simulation_case_table.("WindField"), wind_field));
            
            del_data_untuned(del_data_idx, new_cols) = UntunedControllers_simulation_case_table(sim_data_idx, new_cols);
        end
    end
    
    agg_cols = del_data_untuned.Properties.VariableNames;
    agg_cols(ismember(agg_cols, ...
        {'Var1', 'Case Desc.', 'TunedWindSpeed', 'WindMean', 'WindTurbulence'})) = [];
    del_data_untuned = sortrows(groupsummary(del_data_untuned, ...
        ["Case Desc.", "TunedWindSpeed", "WindMean", "WindTurbulence"], ...
        "mean", agg_cols), ["Case Desc.", "WindMean"])
    
    del_data_untuned.(gain_col) = zeros(height(del_data_untuned), 1);
    

    % aggregate over wind seeds for each controller type
    
    % plot percentage DEL reduction @ each mean wind speed for blade
    % out-of-plane load, nacelle yaw moment, nacelle pitch moment, shaft
    % bending moment around y-axis
    % del_out_table = [del_data_untuned(:, op_cols); del_data_tuned(:, op_cols)];

    % op_cols = del_data_tuned.Properties.VariableNames;
    % op_cols(ismember(op_cols, {'TunedWindSpeed', 'WindTurbulence', 'GroupCount'})) = [];
    
    metric_cols = del_data_tuned.Properties.VariableNames;
    metric_cols(ismember(metric_cols, 'Case Desc.')) = [];
    metric_cols(ismember(metric_cols, 'TunedWindSpeed')) = [];
    metric_cols(ismember(metric_cols, 'WindMean')) = [];
    metric_cols(ismember(metric_cols, 'WindTurbulence')) = [];
    metric_cols(ismember(metric_cols, 'GroupCount')) = [];
    metric_cols(ismember(metric_cols, {'A_W1', 'A_W2', 'A_We', 'A_Wu', gain_col})) = [];
    % metric_cols(ismember(metric_cols, {'mean_RootMc1', 'mean_OoPDefl1', 'mean_IPDefl1'})) = [];
    
    op_cols = wind_cols;
    op_cols{end + 1} = "Load";
    op_cols{end + 1} = gain_col;
    del_out_table = table();

    for m = metric_cols
        untuned_cases = del_data_untuned(:, ["Case Desc.", "WindMean", m{1}]);
        baseline_cases = sortrows(untuned_cases(strcmp(untuned_cases.("Case Desc."), "baseline_controller"), :), "WindMean");
        noipc_cases = sortrows(untuned_cases(strcmp(untuned_cases.("Case Desc."), "noipc"), :), "WindMean");
        baseline_cases.(gain_col) = zeros(height(baseline_cases), 1);
        % baseline_cases(:, m{1}) = (baseline_cases(:, m{1}) - noipc_cases(:, m{1})) ./ noipc_cases(:, m{1});
        
        tmp_table = table();
        n_ctrl_cases = height(unique(del_data_tuned(:, "Case Desc.")));
        load_vals = cell(n_ctrl_cases + 1, 1); % add 1 for baseline case
        load_vals(:) = {replace(m{1}, 'mean_', '')};
        tmp_table.("Load") = load_vals;
        [x, i, ~] = unique(del_data_tuned(:, "Case Desc."));
        x(end + 1, "Case Desc.") = {{'baseline_controller'}};
        tmp_table(:, "Case Desc.") = x;
        tmp_table.(gain_col) = [del_data_tuned(i, gain_col).Variables; 0];
        tmp_table = sortrows(tmp_table, "Case Desc.");

        for ws = unique(del_data_tuned.("WindMean"))'
            ctrl_cases = del_data_tuned(del_data_tuned.WindMean == ws, ["Case Desc.", "WindMean", m{1}, gain_col]);
            cases = [ctrl_cases; baseline_cases(baseline_cases.WindMean == ws, :)];
            cases(:, m{1}) = (cases(:, m{1}) - noipc_cases(noipc_cases.WindMean == ws, m{1})) ./ noipc_cases(noipc_cases.WindMean == ws, m{1});
            tmp_table.(num2str(ws)) = sortrows(cases, "Case Desc.").(m{1});
        end
        
        del_out_table = [del_out_table; tmp_table];
    end
    for c = 1:length(controller_types)
        ctrl_type = controller_types{c};
        cond = contains(del_out_table.("Case Desc."), ctrl_type);
        n_cases = sum(cond);
        vals = cell(n_cases, 1);
        vals(:) = {ctrl_type};
        del_out_table(cond, "Case Desc.") = vals; 
    end
    del_out_table = sortrows(del_out_table, ["Load", "Case Desc.", gain_col])

    del_out_table_tmp = del_out_table;
    cond = zeros(height(del_out_table), 1);
    
    for op = {"RootMyc1", "RootMxb1", "RootMyb1", "TTDspFA", "TTDspSS", "YawBrMyp", "YawBrMzp", "ADC", "GenPwr Mean"}
        cond = cond | strcmp(del_out_table.("Load"), op{1});
    end
    del_out_table_tmp = del_out_table(cond, :);
    del_out_table_tmp = sortrows(del_out_table, ["Load", "Case Desc.", gain_col])
    % NOTE: focusing on particular frequencies can increase overall DELs
    cond = del_out_table_tmp(:, wind_cols) < 0;
    cond = all(cond.Variables, 2);
    del_out_table_tmp = del_out_table_tmp(cond, :);
    del_out_table_tmp.("mean_Change") = mean(del_out_table_tmp(:, wind_cols), 2).Variables;
    red_del_out_table = table();
    for l = unique(del_out_table_tmp.("Load"))'
        x = sortrows(del_out_table_tmp(strcmp(del_out_table_tmp.("Load"), l{1}), :), "mean_Change");
        red_del_out_table = [red_del_out_table; x(1, :)];
    end

    writetable(red_del_out_table, fullfile(fig_dir, [sim_type, '_del_table.csv']));
    save(fullfile(mat_save_dir, [sim_type, '_del_table']), 'del_out_table');

    % for c = 1:length(controller_types)
    %     ctrl_type = controller_types{c};
    %     ctrl_type_cond = contains(del_data_tuned.("Case Desc."), ctrl_type);
    %     x = sortrows(del_data_tuned(ctrl_type_cond, op_cols), ["WindMean", gain_col]);
    %     writetable(x, fullfile(fig_dir, [ctrl_type, '_del_vs_ws.csv']));
    %     save(fullfile(fig_dir, [ctrl_type, '_del_vs_ws_table']), 'x');
    % end
end

    
%% Generate data from different controllers
% ctrl_cond = ~strcmp(Controllers_simulation_case_agg_table.("Case Desc."), 'noipc') ...
%     & ~strcmp(Controllers_simulation_case_agg_table.("Case Desc."), 'baseline');

sortrows(Controllers_fft_case_agg_table(Controllers_fft_case_agg_table.WindMean == NONLPV_CONTROLLER_WIND_SPEED, :), 'mean_RootMyc1_2P').("mean_RootMyc1_2P")(1)
sortrows(Controllers_fft_case_agg_table(Controllers_fft_case_agg_table.WindMean == NONLPV_CONTROLLER_WIND_SPEED, :), 'mean_RootMyc1_4P').("mean_RootMyc1_4P")(1)
UntunedControllers_fft_case_agg_table(...
    UntunedControllers_fft_case_agg_table.WindMean == NONLPV_CONTROLLER_WIND_SPEED, ["mean_RootMyc1_2P", "mean_RootMyc1_4P"])
sortrows(Controllers_fft_case_agg_table(Controllers_fft_case_agg_table.WindMean == NONLPV_CONTROLLER_WIND_SPEED, :), 'mean_RootMyc1_2P')
op_cols = ["Case Desc.", "WindMean", "mean_MultiDiskIO_DM", "mean_wc", "mean_ADC", "mean_RootMycBlade1 RMSE"];

n_top_cases_per_wind_case = 5;
% group by WindMean
n_top_cases = n_top_cases_per_wind_case * length(unique(Controllers_simulation_case_agg_table(:, "WindMean")).Variables);
most_robust_table = array2table(zeros(0, length(op_cols)), 'VariableNames', op_cols);
lowest_adc_table = array2table(zeros(0, length(op_cols)), 'VariableNames', op_cols);
lowest_mse_table = array2table(zeros(0, length(op_cols)), 'VariableNames', op_cols);
wind_idx = 1;

% group by mean wind speed and seed (ie. .bts file used in simulation)
for wind_case = unique(Controllers_simulation_case_agg_table(:, "WindMean")).Variables'
    wind_cond = Controllers_simulation_case_agg_table.("WindMean") == wind_case;
    x = sortrows(Controllers_simulation_case_agg_table(wind_cond, :), 'mean_MultiDiskIO_DM', 'descend');

    table_start_idx = (wind_idx - 1) * min(n_top_cases_per_wind_case, height(x)) + 1;
    table_end_idx = (wind_idx - 1) * min(n_top_cases_per_wind_case, height(x)) + min(n_top_cases_per_wind_case, height(x));
    
    most_robust_table = [most_robust_table; x(1:min(n_top_cases_per_wind_case, height(x)), op_cols)];
    
    x = sortrows(Controllers_simulation_case_agg_table(wind_cond, :), 'mean_ADC', 'ascend');
    lowest_adc_table = [lowest_adc_table; x(1:min(n_top_cases_per_wind_case, height(x)), op_cols)];
    
    x = sortrows(Controllers_simulation_case_agg_table(wind_cond, :), 'mean_RootMycBlade1 RMSE', 'ascend');
    lowest_mse_table = [lowest_mse_table; x(1:min(n_top_cases_per_wind_case, height(x)), op_cols)];
    wind_idx = wind_idx + 1;
end
most_robust_table(1, 'Case Desc.') % W1 = 1 W2 = 0.01 ->  Wu = 10 We = 0.1
lowest_adc_table(1, 'Case Desc.') %   W1 = 1 W2 = 0.01 ->  Wu = 10 We = 0.1
lowest_mse_table(1, 'Case Desc.') %  W1 = 1 W2 = 0.1 ->  Wu = 7.5 We = 10 SAME AS GREATEST FFT PEAK DECREASE AT 2P and 4P for ROOTMYC!

% add noipc and baseline cases from tuned wind speed
for wind_case = unique(most_robust_table.("WindMean"))'
    wind_cond = UntunedControllers_simulation_case_agg_table.("WindMean") == wind_case;
    most_robust_table = ...
        [UntunedControllers_simulation_case_agg_table(wind_cond, op_cols); ...
         most_robust_table];

    lowest_adc_table = ...
        [UntunedControllers_simulation_case_agg_table(wind_cond, op_cols); ...
         lowest_adc_table];

    lowest_mse_table = ...
        [UntunedControllers_simulation_case_agg_table(wind_cond, op_cols); ...
         lowest_mse_table];
end

% compare to non-scheduled case
if SCHEDULING
     nonScheduledControllers_simulation_case_agg_table = load(fullfile(mat_save_dir, ['optimal_k_cases_turbsim_wu', '_simulation_case_agg_table'])).Controllers_simulation_case_agg_table;
     % get cases corresponding to Wu weighting for scheduled tests
     nonScheduledControllers_simulation_case_agg_table = ...
         nonScheduledControllers_simulation_case_agg_table(nonScheduledControllers_simulation_case_agg_table.("A_Wu") == ...
         unique(Controllers_simulation_case_agg_table.("A_Wu")), :);
     nonScheduledControllers_simulation_case_agg_table = sortrows(nonScheduledControllers_simulation_case_agg_table, ["Case Desc.", "WindMean"]);
     Controllers_simulation_case_agg_table = sortrows(Controllers_simulation_case_agg_table, ["Case Desc.", "WindMean"]);
     nonlin_cols = ["mean_ADC", "mean_RootMycBlade1 RMSE", "mean_RootMycD RMSE", "mean_RootMycQ RMSE"];
     x = (Controllers_simulation_case_agg_table(:, nonlin_cols) - nonScheduledControllers_simulation_case_agg_table(:, nonlin_cols)) ./ nonScheduledControllers_simulation_case_agg_table(:, nonlin_cols)
     x(:, ["Case Desc.", "WindMean"]) = Controllers_simulation_case_agg_table(:, ["Case Desc.", "WindMean"]);
     mean(x(:, ["mean_RootMycBlade1 RMSE", "mean_ADC"]), 1)

     % nofilt_delta_sched = x;
    
     mean(nofilt_delta_sched(:, ["mean_RootMycBlade1 RMSE", "mean_ADC"]), 1)
     mean(vlowbp_delta_sched(:, ["mean_RootMycBlade1 RMSE", "mean_ADC"]), 1)
     mean(lowbp_delta_sched(:, ["mean_RootMycBlade1 RMSE", "mean_ADC"]), 1)
     mean(highbp_delta_sched(:, ["mean_RootMycBlade1 RMSE", "mean_ADC"]), 1)
     vlowbp_delta_sched = load(fullfile(mat_save_dir, "vlowbp_delta_sched.mat")).x;
     highbp_delta_sched = load(fullfile(mat_save_dir, "highbp_delta_sched.mat")).new_diff;
     lowbp_delta_sched = load(fullfile(mat_save_dir, "lowbp_delta_sched.mat")).old_diff;
     % save(fullfile(mat_save_dir, "highbp_delta_sched.mat"), 'new_diff');
     % save(fullfile(mat_save_dir, "lowbp_delta_sched.mat"), 'old_diff');
     % save(fullfile(mat_save_dir, "vlowbp_delta_sched.mat"), 'x');
end

% choose best of three types of tuned controllers
cond = (most_robust_table.("WindMean") == NONLPV_CONTROLLER_WIND_SPEED); % & (contains(most_robust_table.("Case Desc."), 'W1'));
most_robust_table = sortrows(most_robust_table(cond, :), "mean_MultiDiskIO_DM", 'descend')
cond = (lowest_adc_table.("WindMean") == NONLPV_CONTROLLER_WIND_SPEED); % & (contains(lowest_adc_table.("Case Desc."), 'W1'));
lowest_adc_table = sortrows(lowest_adc_table(cond, :), "mean_ADC", 'ascend')
cond = (lowest_mse_table.("WindMean") == NONLPV_CONTROLLER_WIND_SPEED); % & (contains(lowest_mse_table.("Case Desc."), 'W1'));
lowest_mse_table = sortrows(lowest_mse_table(cond, :), "mean_RootMycBlade1 RMSE", 'ascend') 

if EXTREME_K_COLLECTION
    best_weightings_table = ...
        [UntunedControllers_simulation_case_agg_table(wind_cond, op_cols)
        most_robust_table(1, :)
        lowest_adc_table(2, :)
        lowest_mse_table(1, :)];
    best_weightings_table = removevars(best_weightings_table, 'WindMean'); % OUT TABLE 1
    writetable(best_weightings_table, fullfile(fig_dir, 'extreme_controllers_table.csv'));
end

%% save data
if OPTIMAL_K_COLLECTION
    writetable(most_robust_table, fullfile(fig_dir, [sim_type '_optimal_robust_controller_table.csv']));
    writetable(lowest_adc_table, fullfile(fig_dir, [sim_type '_optimal_lowadc_controller_table.csv']));
    writetable(lowest_mse_table, fullfile(fig_dir, [sim_type '_optimal_lowmse_controller_table.csv']));
    
    % table2latex(table_tmp, fullfile(fig_dir, 'optimal_controller_table.tex'));
elseif EXTREME_K_COLLECTION
    writetable(most_robust_table(:, op_cols), fullfile(fig_dir, 'extreme_robust_controller_table.csv'));
    writetable(lowest_adc_table(:, op_cols), fullfile(fig_dir, 'extreme_lowadc_controller_table.csv'));
    writetable(lowest_mse_table(:, op_cols), fullfile(fig_dir, 'extreme_lowmse_controller_table.csv'));

    % writetable(table_tmp, char(fullfile(fig_dir, 'extreme_controller_table.tex')));
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
    
if STRUCT_PARAM_SWEEP
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

%% Analysis of loads from Open-Loop nonlinear simulations
if 0
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
    % Yaw bearing yaw moment (Mz&My), YawBrMyp, YawBrMzp, YawBrMyp, YawBrMzp
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
end