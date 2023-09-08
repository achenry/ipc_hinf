%% Overview
% Tune full-order controller for 14m/s with only tracking error metric
% Ask Manuel Questions, correct weighting matrices and structured
% controller
% Add BladePitch penalty with greater penalty Wu gain and with saturation
% block
% Write Linear Analysis code: studying robust stability pareto front for
% different controllers
% Construct Simulink models for structured/full-order IPC controllers
% Write Nonlinear Analysis code: run 10 simulations with Turbsim for each
% controller plugged into simulink model.
% Write code with pcrunch to compute DELs

% LPV vs Gain-Scheduled vs Single above-rated tuning => Adhoc
% Gain-scheduling vs single-point. No guarantees. Can then advance to LPV.

% X fix normalization in loop plot
% ~ achieve maximum performance for each channel by changing weights
% ~ compare controllers with notch on input vs output
% ~ evaluate robustness for given controller

%% TODO 
% approximate full-order controller with structured controller
% plot GenPlant for Extreme Controllers
% based on results of Extreme Controllers, adjust the weighting matrices

%% Setup workspace
init_hinf_controller;

%% Synthesize LPV Continuous-Time Controller w/ lpvsyn for gain controller, full-order controller
if 0

    nmeas = 2; % number of measurement outputs y, K has nmeas inputs
    ncont = 2; % number of control inputs u, K has ncont outputs
    
    % Basis function, QUESTION how to choose these?
    bf = basis(GenPlant_lpv.Parameter.u, 'u', 1);
    Xb = [1; bf; bf^2; bf^3];
    Yb = Xb;

    % LPV Rate-Bounded Control Design
    lpv_syn_opt = lpvsynOptions('BackOffFactor', 1.02);
    [K_full_lpv, gamma_lpv] = lpvsyn(GenPlant_lpv, nmeas, ncont, Xb, Yb, lpv_syn_opt);

%     SF_lpv = loopsens()
% sigma(SF.So,'b',SF.Si,'r--')
% DMI = loopmargin(G(:,2:3),Klpv,'di');
% lpvmin(DMI.GainMargin(2))
% lpvmin(DMI.PhaseMargin(2))
end

%% Prepare controller synthesis cases
if 1
nmeas = 2; % number of measurement outputs y, K has nmeas inputs
ncont = 2; % number of control inputs u, K has ncont outputs

% consider case of only Wy to penalize To, 
% then only Wy and We to penalize To and So, 
% then Wy and We and Wu to penalize To and So and KSi and Ti
% full_controller_case_basis.WindSpeedIndex = 1:length(LPV_CONTROLLER_WIND_SPEEDS);
full_controller_case_basis.WindSpeedIndex = find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED); % TODO only works for non-scheduled
full_controller_case_basis.WuGain = case_basis.WuGain;
full_controller_case_basis.WeGain = case_basis.WeGain;
full_controller_case_basis.W1Gain = case_basis.W1Gain;
full_controller_case_basis.W2Gain = case_basis.W2Gain;

[FullOrderControllers_case_list, FullOrderControllers_case_name_list, FullOrderControllers_n_cases] ...
    = generateCases(full_controller_case_basis, 'full_order_tuned_controllers', false);

if EXTREME_K_COLLECTION
    % remove cases
    if 0
        exc_idx = [];
        for c_idx = 1:FullOrderControllers_n_cases
            % want exactly 2 weighting matrices to be 'on'
            % and only one weighting filter to have a notch
            if (~((length(FullOrderControllers_case_list(c_idx).WeGain.Numerator{1,1}) > 1) ...
                    && (length(FullOrderControllers_case_list(c_idx).W1Gain.Numerator{1,1}) > 1))) ...
                    && (ss(FullOrderControllers_case_list(c_idx).W1Gain).D(1, 1) ...
                    + ss(FullOrderControllers_case_list(c_idx).W2Gain).D(1, 1) == IP_HIGH_GAIN + IP_LOW_GAIN) ... % want one (but not both) to have high gain
                    && ((ss(FullOrderControllers_case_list(c_idx).WuGain).D(1, 1) ...
                    + ss(FullOrderControllers_case_list(c_idx).WeGain).D(1, 1) == WU_HIGH_GAIN + OP_LOW_GAIN) ... 
                    || (ss(FullOrderControllers_case_list(c_idx).WuGain).D(1, 1) ...
                    + ss(FullOrderControllers_case_list(c_idx).WeGain).D(1, 1) == WE_HIGH_GAIN + OP_LOW_GAIN))
                continue;
            end
            exc_idx = [exc_idx, c_idx];
        end
        FullOrderControllers_case_list = FullOrderControllers_case_list(~ismember(1:FullOrderControllers_n_cases, exc_idx));
        FullOrderControllers_n_cases = FullOrderControllers_n_cases - length(exc_idx);
        FullOrderControllers_case_name_list = arrayfun(@(n) ['tuned_controllers_', num2str(n)], 1:FullOrderControllers_n_cases, 'UniformOutput', false);
    end
% elseif OPTIMAL_K_COLLECTION
    
end

%% Synthesize Gain-Scheduled Continuous-Time Controller w/ hinfsyn for gain controller, full-order controller
if 1
    FullOrderTuning = repmat(struct(), FullOrderControllers_n_cases, 1 );
    parfor case_idx = 1:FullOrderControllers_n_cases
        controller_case = FullOrderControllers_case_list(case_idx);
        c_ws_idx = controller_case.WindSpeedIndex;
        % Wy_tmp = controller_case.Wy * Wy;
        Wu_tmp = controller_case.WuGain * Wu;
        We_tmp = controller_case.WeGain * We;
        W1_tmp = controller_case.W1Gain * W1;
        W2_tmp = controller_case.W2Gain * W2;
        Wy_tmp = Wy;

        [GenPlant_tmp, Win_tmp, Wout_tmp] = generateGenPlant(...
            Plant_scaled(:, :, c_ws_idx), ...
            Wu_tmp, We_tmp, Wy_tmp, W1_tmp, W2_tmp);
        
        opts = hinfsynOptions('Display', 'on'); 
        [K_tmp, CL_full_tuned_tmp, gamma_tmp] = hinfsyn(GenPlant_tmp, nmeas, ncont, opts); % NOTE does not compute for pure integrator in weighting matrix
        
        % weighting_case_idx = floor(case_idx / length(LPV_CONTROLLER_WIND_SPEEDS)) + 1;

        FullOrderTuning(case_idx).WuGain = controller_case.WuGain;
        FullOrderTuning(case_idx).WeGain = controller_case.WeGain;
        FullOrderTuning(case_idx).W1Gain = controller_case.W1Gain;
        FullOrderTuning(case_idx).W2Gain = controller_case.W2Gain;
                
        FullOrderTuning(case_idx).GenPlant = GenPlant_tmp;
        FullOrderTuning(case_idx).Win = Win_tmp;
        FullOrderTuning(case_idx).Wout = Wout_tmp;
        FullOrderTuning(case_idx).CL = CL_full_tuned_tmp;
        FullOrderTuning(case_idx).Controller = ss(K_tmp); 
        FullOrderTuning(case_idx).Controller.InputName = {'Measured RootMycD Tracking Error', 'Measured RootMycQ Tracking Error'};
        FullOrderTuning(case_idx).Controller.OutputName = {'BldPitchD Control Input', 'BldPitchQ Control Input'};
        FullOrderTuning(case_idx).gamma = gamma_tmp;

        FullOrderTuning(case_idx).Controller_scaled = ...
                    ss(ip_scaling(:, :, c_ws_idx) ...
                    * FullOrderTuning(case_idx).Controller ...
                    * inv(op_scaling(:, :, c_ws_idx)));
        
        % Lo = Plant(:, :, c_ws_idx) ...
        %                 * FullOrderTuning(case_idx).Controller_scaled;
        % Li = FullOrderTuning(case_idx).Controller_scaled ...
        %                 * Plant(:, :, c_ws_idx);

        SF_tmp = loopsens(-Plant(:, :, c_ws_idx), -FullOrderTuning(case_idx).Controller_scaled);

        [DMo_tmp, ~] = diskmargin(SF_tmp.Lo);
        [DMi_tmp, ~] = diskmargin(SF_tmp.Li);
        FullOrderTuning(case_idx).DMo = DMo_tmp;
        FullOrderTuning(case_idx).DMi = DMi_tmp;
        
        dc = abs(dcgain(FullOrderTuning(case_idx).Controller_scaled));
        FullOrderTuning(case_idx).wc = getGainCrossover(FullOrderTuning(case_idx).Controller_scaled, min(dc, [], 'all') / sqrt(2));

        % bodemag(Li); xline(FullOrderTuning(case_idx).wci); yline(mag2db(dci(2,2)));

        % check norms of KSi, Ti, So, GSo (4 channels) and To
        
        % Ti_tmp = Ti(Plant(:, :, c_ws_idx), FullOrderTuning(case_idx).Controller_scaled); % 
        % To_tmp = To(Plant(:, :, c_ws_idx), FullOrderTuning(case_idx).Controller_scaled);
        % KSi_tmp = KSi(Plant(:, :, c_ws_idx), FullOrderTuning(case_idx).Controller_scaled);
        % So_tmp = So(Plant(:, :, c_ws_idx), FullOrderTuning(case_idx).Controller_scaled);
        % GSo_tmp = GSo(Plant(:, :, c_ws_idx), FullOrderTuning(case_idx).Controller_scaled);
        
        FullOrderTuning(case_idx).SF = SF_tmp;

        FullOrderTuning(case_idx).inf_norms.Ti = norm(SF_tmp.Ti, Inf);
        FullOrderTuning(case_idx).inf_norms.To = norm(SF_tmp.To, Inf);
        % FullOrderTuning(case_idx).inf_norms.KSi = norm(KSi_tmp);
        FullOrderTuning(case_idx).inf_norms.Si = norm(SF_tmp.Si, Inf);
        FullOrderTuning(case_idx).inf_norms.So = norm(SF_tmp.So, Inf);
        % FullOrderTuning(case_idx).inf_norms.GSo = norm(GSo_tmp);

    end
   
    % restructure synthesized controller cases per weighting matrix
    % combination and wind speed
    % for each wind speed
    FullOrderControllers = struct;
    for c_ws_idx = full_controller_case_basis.WindSpeedIndex
        weighting_case_idx = 1;
        % find all of the controller cases that correspond to this wind speed
        for case_idx = 1:FullOrderControllers_n_cases
            controller_case = FullOrderControllers_case_list(case_idx);
            if c_ws_idx == controller_case.WindSpeedIndex
                cc = find(full_controller_case_basis.WindSpeedIndex == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED));
                FullOrderControllers(weighting_case_idx).WuGain = FullOrderControllers_case_list(case_idx).WuGain;
                FullOrderControllers(weighting_case_idx).WeGain = FullOrderControllers_case_list(case_idx).WeGain;
                FullOrderControllers(weighting_case_idx).W1Gain = FullOrderControllers_case_list(case_idx).W1Gain;
                FullOrderControllers(weighting_case_idx).W2Gain = FullOrderControllers_case_list(case_idx).W2Gain;

                FullOrderControllers(weighting_case_idx).GenPlant(:, :, cc) = FullOrderTuning(case_idx).GenPlant;
                FullOrderControllers(weighting_case_idx).Win(:, :, cc) = FullOrderTuning(case_idx).Win;
                FullOrderControllers(weighting_case_idx).Wout(:, :, cc) = FullOrderTuning(case_idx).Wout;
                FullOrderControllers(weighting_case_idx).gamma(cc) = FullOrderTuning(case_idx).gamma;

                FullOrderControllers(weighting_case_idx).Controller(:, :, cc) = FullOrderTuning(case_idx).Controller;
                FullOrderControllers(weighting_case_idx).CL(:, :, cc) = FullOrderTuning(case_idx).CL;

                FullOrderControllers(weighting_case_idx).Controller_scaled(:, :, cc) = ...
                   FullOrderTuning(case_idx).Controller_scaled;

                FullOrderControllers(weighting_case_idx).DMo(:, cc) = FullOrderTuning(case_idx).DMo;
                FullOrderControllers(weighting_case_idx).DMi(:, cc) = FullOrderTuning(case_idx).DMi;

                FullOrderControllers(weighting_case_idx).wc(:, cc) = FullOrderTuning(case_idx).wc;
                % FullOrderControllers(weighting_case_idx).n_wc(:, cc) = FullOrderTuning(case_idx).n_wc;
                
                FullOrderControllers(weighting_case_idx).SF(cc) = FullOrderTuning(case_idx).SF;
                FullOrderControllers(weighting_case_idx).inf_norms.Ti(cc) = FullOrderTuning(case_idx).inf_norms.Ti;
                FullOrderControllers(weighting_case_idx).inf_norms.To(cc) = FullOrderTuning(case_idx).inf_norms.To;
                % FullOrderControllers(weighting_case_idx).inf_norms.KSi(c_ws_idx) = FullOrderTuning(case_idx).inf_norms.KSi;
                FullOrderControllers(weighting_case_idx).inf_norms.Si(cc) = FullOrderTuning(case_idx).inf_norms.Si;
                FullOrderControllers(weighting_case_idx).inf_norms.So(cc) = FullOrderTuning(case_idx).inf_norms.So;
                % FullOrderControllers(weighting_case_idx).inf_norms.GSo(c_ws_idx) = FullOrderTuning(case_idx).inf_norms.GSo;

                weighting_case_idx = weighting_case_idx + 1;
            end
        end
    end
    if length(FullOrderControllers) == 1
        FullOrderControllers = [FullOrderControllers];
    end

    n_weighting_cases = (FullOrderControllers_n_cases / length(full_controller_case_basis.WindSpeedIndex));

    for w_idx = 1:n_weighting_cases
        FullOrderControllers(w_idx).Controller = ss(FullOrderControllers(w_idx).Controller);
        FullOrderControllers(w_idx).Controller.InputName = {'Measured RootMycD Tracking Error', 'Measured RootMycQ Tracking Error'};
        FullOrderControllers(w_idx).Controller.OutputName = {'BldPitchD Control Input', 'BldPitchQ Control Input'};
        FullOrderControllers(w_idx).Controller.SamplingGrid = struct('u', LPV_CONTROLLER_WIND_SPEEDS(full_controller_case_basis.WindSpeedIndex));
        FullOrderControllers(w_idx).Controller_scaled = ss(FullOrderControllers(w_idx).Controller_scaled);
        FullOrderControllers(w_idx).Controller_scaled.InputName = {'Measured RootMycD Tracking Error', 'Measured RootMycQ Tracking Error'};
        FullOrderControllers(w_idx).Controller_scaled.OutputName = {'BldPitchD Control Input', 'BldPitchQ Control Input'};
        FullOrderControllers(w_idx).Controller_scaled.SamplingGrid = struct('u', LPV_CONTROLLER_WIND_SPEEDS(full_controller_case_basis.WindSpeedIndex));
    end
    
    % go through each controller case and find the corresponding controller in
    % FullOrderControllers
    for c_idx = 1:Controllers_n_cases
        if ~strcmp(Controllers_case_list(c_idx).Structure, 'Full-Order')
            continue;
        end
        % weighting_case_idx = floor(c_idx / length(LPV_CONTROLLER_WIND_SPEEDS)) + 1;
        
        for w_idx = 1:n_weighting_cases
            
            if (sum(ss(FullOrderControllers(w_idx).WuGain - Controllers_case_list(c_idx).WuGain).D, 'all') || ...
                sum(ss(FullOrderControllers(w_idx).WeGain - Controllers_case_list(c_idx).WeGain).D, 'all') || ...
                sum(ss(FullOrderControllers(w_idx).W1Gain - Controllers_case_list(c_idx).W1Gain).D, 'all') || ...
                sum(ss(FullOrderControllers(w_idx).W2Gain - Controllers_case_list(c_idx).W2Gain).D, 'all'))
                continue;
            end
            
            K_tmp = FullOrderControllers(w_idx).Controller;
            K_tmp_scaled = FullOrderControllers(w_idx).Controller_scaled;
    
            % Add gain-scheduled controller to Controller_list
            if strcmp(Controllers_case_list(c_idx).Scheduling, 'Yes')
                Controllers_case_list(c_idx).Controller = K_tmp;
                Controllers_case_list(c_idx).Controller_scaled = K_tmp_scaled;
            % Add non gain-scheduled controller to Controller_list by repeating
            % same system for every parameter value
            elseif strcmp(Controllers_case_list(c_idx).Scheduling, 'No')
                for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
                    Controllers_case_list(c_idx).Controller(:, :, c_ws_idx) ...
                        = K_tmp(:, :, full_controller_case_basis.WindSpeedIndex == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED));
                    Controllers_case_list(c_idx).Controller_scaled(:, :, c_ws_idx) ...
                        = K_tmp_scaled(:, :, full_controller_case_basis.WindSpeedIndex == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED));
                end     
            end
            
            Controllers_case_list(c_idx).GenPlant = FullOrderControllers(w_idx).GenPlant;
            Controllers_case_list(c_idx).Wout = FullOrderControllers(w_idx).Wout;
            Controllers_case_list(c_idx).Win = FullOrderControllers(w_idx).Win;
            Controllers_case_list(c_idx).gamma = FullOrderControllers(w_idx).gamma;

            Controllers_case_list(c_idx).DMi = FullOrderControllers(w_idx).DMi;
            Controllers_case_list(c_idx).DMo = FullOrderControllers(w_idx).DMo;

            Controllers_case_list(c_idx).n_wc = length(FullOrderControllers(w_idx).wc);

            if Controllers_case_list(c_idx).n_wc == 0
                Controllers_case_list(c_idx).wc = -1;
            else
                Controllers_case_list(c_idx).wc = FullOrderControllers(w_idx).wc(1);
            end
            
            
            Controllers_case_list(c_idx).SF.Stable = FullOrderControllers(w_idx).SF.Stable;
            Controllers_case_list(c_idx).SF.Ti = FullOrderControllers(w_idx).SF.Ti;
            Controllers_case_list(c_idx).SF.To = FullOrderControllers(w_idx).SF.To;
            % PI_ParameterSweep.tfs.KSi(:, :, i, j) = PI_ParameterSweep_tmp(v).tfs.KSi;
            Controllers_case_list(c_idx).SF.Si = FullOrderControllers(w_idx).SF.Si;
            Controllers_case_list(c_idx).SF.So = FullOrderControllers(w_idx).SF.So;
    
            Controllers_case_list(c_idx).inf_norms.Ti = FullOrderControllers(w_idx).inf_norms.Ti;
            Controllers_case_list(c_idx).inf_norms.To = FullOrderControllers(w_idx).inf_norms.To;
            % Controllers_case_list(c_idx).inf_norms.KSi = FullOrderControllers(w_idx).inf_norms.KSi;
            Controllers_case_list(c_idx).inf_norms.Si = FullOrderControllers(w_idx).inf_norms.Si;
            Controllers_case_list(c_idx).inf_norms.So = FullOrderControllers(w_idx).inf_norms.So;
            % Controllers_case_list(c_idx).inf_norms.GSo = FullOrderControllers(w_idx).inf_norms.GSo;
            
        end
        Controllers_case_list(c_idx).Controller_scaled.SamplingGrid = struct('u', LPV_CONTROLLER_WIND_SPEEDS);
        
    end
   
    
    fileID = fopen('./weighting_cases.txt','w');
    case_desc = {'No IPC'};
    % loop through weighting cases and print information
    for w_idx = 1:n_weighting_cases
        fprintf(fileID, '\nCase %d\n', w_idx);
        
        ip_msg = [];
        % if (ss(FullOrderControllers(w_idx).W1Gain).D(1, 1) == IP_HIGH_GAIN)
            ip_msg = [ip_msg, ' W1 = ', num2str(ss(FullOrderControllers(w_idx).W1Gain).D(1, 1))];
        % end
    
        if length(ss(FullOrderControllers(w_idx).W1Gain).C)
            ip_msg = [ip_msg, ' with notch '];
        end
        
        % if (ss(FullOrderControllers(w_idx).W2Gain).D(1, 1) == IP_HIGH_GAIN)
            ip_msg = [ip_msg, ' W2 = ', num2str(ss(FullOrderControllers(w_idx).W2Gain).D(1, 1))];
        % end
        ip_msg = [ip_msg, ' -> '];
        fprintf(fileID, ip_msg);
        
        op_msg = [];
        % if (ss(FullOrderControllers(w_idx).WuGain).D(1, 1) == OP_HIGH_GAIN)
            op_msg = [op_msg, ' Wu = ', num2str(ss(FullOrderControllers(w_idx).WuGain).D(1, 1))];
        % end
    
        % if (ss(FullOrderControllers(w_idx).WeGain).D(1, 1) == OP_HIGH_GAIN)
            op_msg = [op_msg, ' We = ', num2str(ss(FullOrderControllers(w_idx).WeGain).D(1, 1))];
        % end
        if length(ss(FullOrderControllers(w_idx).WeGain).C)
            op_msg = [op_msg, ' with notch '];
        end
        
        op_msg = [op_msg];
        case_desc = [case_desc, [ip_msg op_msg]];
        fprintf(fileID, op_msg);
        
    end
    fclose(fileID);
    
    % QUESTION MANUEL how to find bandwidth of controller (DM), robustness margins
    % for noipc case?
     n_weighting_cases = (FullOrderControllers_n_cases / length(full_controller_case_basis.WindSpeedIndex));
    for w_idx = 1:n_weighting_cases
        Stable_tmp(w_idx) = Controllers_case_list(w_idx).SF.Stable;
        Ti_infnorm_tmp(w_idx) = Controllers_case_list(w_idx).inf_norms.Ti;
        To_infnorm_tmp(w_idx) = Controllers_case_list(w_idx).inf_norms.To;
        Si_infnorm_tmp(w_idx) = Controllers_case_list(w_idx).inf_norms.Si;
        So_infnorm_tmp(w_idx) = Controllers_case_list(w_idx).inf_norms.So;
        DMi_tmp(w_idx, :) = [Controllers_case_list(w_idx).DMi(1).DiskMargin Controllers_case_list(w_idx).DMi(2).DiskMargin];
        DMo_tmp(w_idx, :) = [Controllers_case_list(w_idx).DMo(1).DiskMargin Controllers_case_list(w_idx).DMo(2).DiskMargin];
        wc_tmp(w_idx, :) = Controllers_case_list(w_idx).wc;
        % n_wc_tmp(w_idx, :) = Controllers_case_list(w_idx).n_wc;
    end

    % Make table comparing controllers
    Controllers_case_table = table( ...
        (0:n_weighting_cases)', case_desc', ... 
        [0; Stable_tmp'], ...
        [0; Ti_infnorm_tmp'], ...
        [0; To_infnorm_tmp'], ... % [0; PI_ParameterSweep.inf_norms.KSi], ...
        [0; Si_infnorm_tmp'], ...
        [0; So_infnorm_tmp'], ... % [0; PI_ParameterSweep.inf_norms.GSo], ... % Ti_tmp, To_tmp, KSi_tmp, So_tmp, GSo_tmp, ...
        [0; DMi_tmp(:, 1)], [0; DMi_tmp(:, 2)], ...
        [0; DMo_tmp(:, 1)], [0; DMo_tmp(:, 2)], ...
        [0; wc_tmp], ...
        zeros(n_weighting_cases + 1, 1), zeros(n_weighting_cases + 1, 1), ...
        'VariableNames', ...
        {'Case No.', 'Case Desc.', 'Stable', ...
        'Hinf(Ti)', 'Hinf(To)', 'Hinf(Si)', 'Hinf(So)', ... 
        'DMiD', 'DMiQ', 'DMoD', 'DMoQ', ...
        'wc', 'ADC', 'RootMycBlade1 MSE'});

    Controllers_case_table.("Mean DMi") = mean(Controllers_case_table(:, ["DMiD", "DMiQ"]), 2).Variables;
    Controllers_case_table.("Mean DMo") = mean(Controllers_case_table(:, ["DMoD", "DMoQ"]), 2).Variables;
    Controllers_case_table.("Mean DM") = mean(Controllers_case_table(:, ["Mean DMi", "Mean DMo"]), 2).Variables;

    sortrows(Controllers_case_table, 'DMoD', 'descend')
    % QUESTION MANUEL how to estimate bandwidth when tf never crosses unity
    sortrows(Controllers_case_table, 'wc', 'descend')

    if EXTREME_K_COLLECTION
        save(fullfile(save_dir, 'Extreme_Controllers_case_list.mat'), "Controllers_case_list", '-v7.3');
        save(fullfile(save_dir, 'Extreme_Controllers_case_table.mat'), "Controllers_case_table", '-v7.3');
    elseif OPTIMAL_K_COLLECTION
        save(fullfile(save_dir, 'Optimal_Controllers_case_list.mat'), "Controllers_case_list", '-v7.3')
        save(fullfile(save_dir, 'Optimal_Controllers_case_table.mat'), "Controllers_case_table", '-v7.3');
    end

else
    if EXTREME_K_COLLECTION
        load(fullfile(save_dir, 'Extreme_Controllers_case_list.mat'));
        load(fullfile(save_dir, 'Extreme_Controllers_case_table.mat'));
    elseif OPTIMAL_K_COLLECTION
        load(fullfile(save_dir, 'Optimal_Controllers_case_list.mat'));
        load(fullfile(save_dir, 'Optimal_Controllers_case_table.mat'));
    end
    
end

if 0
    
    figure;
    for w_idx = 1:(FullOrderControllers_n_cases / length(LPV_CONTROLLER_WIND_SPEEDS))
        bodemag(Controllers_case_list(w_idx).Controller_scaled(:, :, C_WS_IDX));
        hold on;
        axh = findall(gcf, 'type', 'axes');
        for a = 1:length(axh)
            xline(axh(a), omega_1P_rad * HARMONICS);
        end
    end
    hold off;
    legend([arrayfun(@(n) ['Case ', num2str(n)], ... 
    1:n_weighting_cases, 'UniformOutput', false) {'', '', '', ''}], ...
    'NumColumns', 2);
    
    % plot robustness margins
    figure;
    tcf = tiledlayout(2, 2);
    n_weighting_cases = (FullOrderControllers_n_cases / length(LPV_CONTROLLER_WIND_SPEEDS));
    for w_idx = 1:n_weighting_cases
        nexttile(1);
        outputs = cell(1, length(LPV_CONTROLLER_WIND_SPEEDS));
        [outputs{:}] = Controllers_case_list(w_idx).DMi(1, :).DiskMargin;
        plot(LPV_CONTROLLER_WIND_SPEEDS, cell2mat(outputs));
        title('DM_i D Loop');
        hold on;

        nexttile(2);
        outputs = cell(1, length(LPV_CONTROLLER_WIND_SPEEDS));
        [outputs{:}] = Controllers_case_list(w_idx).DMi(2, :).DiskMargin;
        plot(LPV_CONTROLLER_WIND_SPEEDS, cell2mat(outputs));
        title('DM_i Q Loop');
        hold on;

        nexttile(3);
        outputs = cell(1, length(LPV_CONTROLLER_WIND_SPEEDS));
        [outputs{:}] = Controllers_case_list(w_idx).DMo(1, :).DiskMargin;
        plot(LPV_CONTROLLER_WIND_SPEEDS, cell2mat(outputs));
        title('DM_o D Loop');
        hold on;

        nexttile(4);
        outputs = cell(1, length(LPV_CONTROLLER_WIND_SPEEDS));
        [outputs{:}] = Controllers_case_list(w_idx).DMo(2, :).DiskMargin;
        plot(LPV_CONTROLLER_WIND_SPEEDS, cell2mat(outputs));
        title('DM_o Q Loop');
        hold on;
    end
    legend([arrayfun(@(n) ['Case ', num2str(n)], ... 
    1:n_weighting_cases, 'UniformOutput', false) {'', '', '', ''}], ...
    'NumColumns', 2);
    set(gcf, 'Position', [0 0 1500 900]);
    savefig(gcf, fullfile(fig_dir, 'fullorder_diskmargins.fig'));
    saveas(gcf, fullfile(fig_dir, 'fullorder_diskmargins.png'));
end
end

%% Plot transfer functions and weighting functions for full-order controller
PLOTTING = 1;
% TODO plot all extreme controllers
if PLOTTING
    n_weighting_cases = (FullOrderControllers_n_cases / length(full_controller_case_basis.WindSpeedIndex));
    cc = find(full_controller_case_basis.WindSpeedIndex == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED));
    for w_idx = 1:n_weighting_cases
        close all;
        GenPlant_inner = inv(Controllers_case_list(w_idx).Wout(:, :, cc)) ...
            * Controllers_case_list(w_idx).GenPlant(:, :, cc) ...
            * inv(Controllers_case_list(w_idx).Win(:, :, cc));
        GenPlant_inner.OutputName = Controllers_case_list(w_idx).GenPlant(:, :, cc).OutputName;
        GenPlant_inner.InputName = Controllers_case_list(w_idx).GenPlant(:, :, cc).InputName;
        
        % GenPlant_inner.InputGroup.dist = 1:4;
        GenPlant_inner.InputGroup.dist = find(ismember(...
            GenPlant_inner.InputName, ...
            {'RootMycD Reference', 'RootMycQ Reference', ...
            'BldPitchD Disturbance', 'BldPitchQ Disturbance'}))';
    
        GenPlant_inner.InputGroup.acts = find(ismember(...
            GenPlant_inner.InputName, ...
            {'BldPitchD Control Input', 'BldPitchQ Control Input'}))';
            
        GenPlant_inner.OutputGroup.perf = find(ismember(GenPlant_inner.OutputName, ...
            {'Weighted BldPitchD Control Input', 'Weighted BldPitchQ Control Input', ...
            'Weighted RootMycD Tracking Error', 'Weighted RootMycQ Tracking Error', ...
            'Weighted RootMycD Output', 'Weighted RootMycQ Output'}))';
    
        GenPlant_inner.OutputGroup.meas = find(ismember(GenPlant_inner.OutputName, ...
            {'Measured RootMycD Tracking Error', 'Measured RootMycQ Tracking Error'}))';
         
        Win_arr = {};
        Wout_arr = {};
        for i = 1:size(Controllers_case_list(w_idx).Win, 1)
            Win_arr{end+1} = Controllers_case_list(w_idx).Win(i, i, cc);
        end
        for i = 1:size(Controllers_case_list(w_idx).Wout, 1)
            Wout_arr{end+1} = Controllers_case_list(w_idx).Wout(i, i, cc);
        end
        % yellow should touch blue at high freq from
        % bldpitch disturbance to tracking error
        % no active shaping in off-diag elements from bldpitch disturbance to
        % tracking error
        % performance loops are bottom two rows, control effort top two rows
        % (nothing shaped on right col, shaping occuring on left col)
        loopData = getLoopData(GenPlant_inner, ...
            Controllers_case_list(w_idx).Controller(:, :, cc), ...
            Win_arr, Wout_arr);
        w = logspace(-2, 4, 400);
        [h, loopData] = plotLoopBode(loopData, [], [], w);
        axh = findall(gcf, 'type', 'axes');
        for a = 1:length(axh)
            if loopData.peaks.wGAM > 0
            xline(axh(a), [loopData.peaks.wGAM]);
            axh(a).YLabel.Rotation = 0;
            end
        end
        set(gcf, 'Position', [0 0 1500 900]);
        savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_genplant_bodemag.fig']));
        saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_genplant_bodemag.png']))
    
        figure;
        K_tmp = Controllers_case_list(w_idx).Controller(:, :, cc);
        K_tmp.InputName = {'\tilde{M}_d Tracking Error', '\tilde{M}_q Tracking Error'};
        K_tmp.OutputName = {'\tilde{\beta}_d Control Input', '\tilde{\beta}_q Control Input'};
        bodeplot(K_tmp, bode_plot_opt);

        axh = findall(gcf, 'type', 'axes');
        xline(axh(3), omega_1P_rad * [2, 3]);
        xline(axh(5), omega_1P_rad * [2, 3]);
        xline(axh(7), omega_1P_rad * [2, 3]);
        xline(axh(9), omega_1P_rad * [2, 3]);

        set(gcf, 'Position', [0 0 1500 900]);
        savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_controller_bodemag.fig']));
        saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_controller_bodemag.jpg']));
        
        if 0
        Wu_tmp = Controllers_case_list(w_idx).WuGain * Wu;
        We_tmp = Controllers_case_list(w_idx).WeGain * We;
        W1_tmp = Controllers_case_list(w_idx).W1Gain * W1;
        W2_tmp = Controllers_case_list(w_idx).W2Gain * W2;

        SF = loopsens(-Plant(:, :, cc), ...
            -Controllers_case_list(w_idx).Controller_scaled(:, :, cc));
        
        % Plot output sensitivity function and its weighting matrix, dy-y or
        % n->e or r->e (ideally 0)
        % S and T are coupled : S + T = 1, so choosing 1 determines the other
        % need penalty on both bc want both to have small magnitdues at particular
        % frequencies (ie could add 2 complex numbers with large magnitudes at
        % high phases that add to 1, or have one with large mag and the other with very small in alignment)
        % OpenLoop case: So = 1, ClosedLoop case: can shape So, ideally to 0.
        % Real So: hpf with roll-off with high-freq gain of 1.
        figure;
        sys = SF.So * W1_tmp;
        % sys.InputName = GenPlant.InputName(1:2); % output disturbance/reference
        sys.InputName = {'\tilde{M}_d Reference', '\tilde{M}_q Reference'};
        We_tmp.OutputName = {'\tilde{M}_d Reference', '\tilde{M}_q Reference'};
        sys.OutputName = {'\tilde{M}_d Tracking Error', '\tilde{M}_q Tracking Error'};
        We_tmp.InputName = {'\tilde{M}_d Tracking Error', '\tilde{M}_q Tracking Error'};
        h = bodeplot(...
            sys, ...
            inv(We_tmp) * Controllers_case_list(w_idx).gamma(cc), w, bode_plot_opt);
        hYLabel = get(gca,'YLabel');
        set(hYLabel,'rotation',0,'VerticalAlignment','middle')
        title('Output Sensitivity Function, S_o');

        axh = findall(gcf, 'type', 'axes');
        % for a = 1:length(axh)
        %     if loopData.peaks.wGAM > 0
        %     xline(axh(a), [loopData.peaks.wGAM]);
        %     end
        % end
        xline(axh(3), omega_1P_rad * [2, 3]);
        xticks(axh(3), [1e-1, 1e0, 1e1, 1e2, 1e3, 1e4]);
        xline(axh(5), omega_1P_rad * [2, 3]);
        xticks(axh(5), [1e-1, 1e0, 1e1, 1e2, 1e3, 1e4]);
        xline(axh(7), omega_1P_rad * [2, 3]);
        xticks(axh(7), [1e-1, 1e0, 1e1, 1e2, 1e3, 1e4]);
        xline(axh(9), omega_1P_rad * [2, 3]);
        xticks(axh(9), [1e-1, 1e0, 1e1, 1e2, 1e3, 1e4]);


        set(gcf, 'Position', [0 0 1500 900]);
        legend('S_o W_1', 'W_e^{-1}\gamma');
        savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_So_bodemag.fig']));
        saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_So_bodemag.jpg']));
    
        figure;
        % sigma(CL_full_tuned(:, :, 1, cc), ss(gamma_full_tuned(1, cc)));
        sys = SF.To * W2_tmp; % requirements defined on output side
        sys.InputName = GenPlant.InputName(1:2); % output disturbance/references
        sys.OutputName = GenPlant.OutputName(5:6); % output
        sys.InputName = {'M_d Reference', 'M_q Reference'};
        Wy.OutputName = {'M_d Reference', 'M_q Reference'};
        sys.OutputName = {'M_d Output', 'M_q Output'};
        Wy.InputName = {'M_d Output', 'M_q Output'};
        bodeplot(...
            sys, ...
            inv(Wy) * Controllers_case_list(w_idx).gamma(cc), w, bode_plot_opt);
        title('Complementary Output Sensitivity Function, T_o');
        axh = findall(gcf, 'type', 'axes');
        for a = 1:length(axh)
            xline(axh(a), [loopData.peaks.wGAM]);
        end
        set(gcf, 'Position', [0 0 1500 900]);
        legend('T_o W_2', 'W_y^{-1}\gamma');
        savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'To_full_bodemag.fig']));
        saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'To_full_bodemag.png']));
        
        % Plot complementary input sensitivity function and its weighting matrix
        figure;
        sys = SF.Ti * W2_tmp;
        sys.InputName = GenPlant.InputName(3:4); % control disturbance
        sys.OutputName = Wu_tmp.InputName; % control input
        sys.InputName = {'\beta_d Disturbance', '\beta_q Disturbance'};
        Wu_tmp.OutputName = {'\beta_d Disturbance', '\beta_q Disturbance'};
        sys.OutputName = {'\beta_d Control Input', '\beta_q Control Input'};
        Wu_tmp.InputName = {'\beta_d Control Input', '\beta_q Control Input'};
        bodeplot(...
            sys, ...
            inv(Wu_tmp) * Controllers_case_list(w_idx).gamma(cc), w, bode_plot_opt);
        title('Complementary Input Sensitivity Function, T_i');
        axh = findall(gcf, 'type', 'axes');
        for a = 1:length(axh)
            xline(axh(a), [loopData.peaks.wGAM]);
        end
        set(gcf, 'Position', [0 0 1500 900]);
        legend('T_i W_2', 'W_u^{-1}\gamma');
        savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'Ti_full_bodemag.fig']));
        saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'Ti_full_bodemag.png']));
    
        % Plot GSo with two weighting functions
        figure;
        sys = GSo(Plant(:, :, cc), ...
            Controllers_case_list(w_idx).Controller_scaled(:, :, cc)) * W2_tmp;
        sys.InputName = GenPlant.InputName(3:4); % control disturbance
        sys.OutputName = We_tmp.InputName; % tracking error
        bodeplot(...
            sys, ...
            inv(We_tmp) * Controllers_case_list(w_idx).gamma(cc), ...
            inv(Wy) * Controllers_case_list(w_idx).gamma(cc), w, bode_plot_opt);
        title('GS_o Function');
        axh = findall(gcf, 'type', 'axes');
        for a = 1:length(axh)
            xline(axh(a), [loopData.peaks.wGAM]);
        end
        set(gcf, 'Position', [0 0 1500 900]);
        legend('GS_o W_2', 'W_e^{-1}\gamma', 'W_y^{-1}\gamma');
        savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'GSo_full_bodemag.fig']));
        saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'GSo_full_bodemag.png']));

        % Plot KS_i with two weighting functions
        figure;
        sys = KSi(Plant(:, :, cc), ...
            Controllers_case_list(w_idx).Controller_scaled(:, :, cc)) * W1_tmp;
        sys.InputName = GenPlant.InputName(1:2); % output disturbance/reference
        sys.OutputName = Wu_tmp.InputName; % control input
        bodeplot(...
            sys, ...
            inv(Wu_tmp) * Controllers_case_list(w_idx).gamma(cc), w, ...
             bode_plot_opt);
        title('KS_i Function');
        axh = findall(gcf, 'type', 'axes');
        for a = 1:length(axh)
            xline(axh(a), [loopData.peaks.wGAM]);
        end
        set(gcf, 'Position', [0 0 1500 900]);
        legend('GS_o W_1', 'W_u^{-1}\gamma');
        savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'KSi_full_bodemag.fig']));
        saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'KSi_full_bodemag.png']));
        end
    end
end