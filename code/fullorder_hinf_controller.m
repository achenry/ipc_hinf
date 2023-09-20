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
full_controller_case_basis.RootMyc_ref = case_basis.Reference;
full_controller_case_basis.BldPitch_sat = case_basis.Saturation;

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
            Wu_tmp, We_tmp, W1_tmp, W2_tmp);
        
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
        FullOrderTuning(case_idx).Controller.InputName = {'Measured M_d Tracking Error', 'Measured M_q Tracking Error'};
        FullOrderTuning(case_idx).Controller.OutputName = {'\beta_d Control Input', '\beta_q Control Input'};
        FullOrderTuning(case_idx).gamma = gamma_tmp;

        FullOrderTuning(case_idx).Controller_scaled = ...
                    ss(ip_scaling(:, :, c_ws_idx) ...
                    * FullOrderTuning(case_idx).Controller ...
                    * inv(op_scaling(:, :, c_ws_idx)));
        
        % Lo = Plant(:, :, c_ws_idx) ...
        %                 * FullOrderTuning(case_idx).Controller_scaled;
        % Li = FullOrderTuning(case_idx).Controller_scaled ...
        %                 * Plant(:, :, c_ws_idx);
        % negative Plant st e = r(dy) - y(yP) is input to controller, 
        % negative Controller for positive u input to Plant
        SF_tmp = loopsens(-Plant(:, :, c_ws_idx), -FullOrderTuning(case_idx).Controller_scaled);
        
        % classical gain/phase margins at plant outputs
        Mrgo_tmp = allmargin(SF_tmp.Lo);

        % classical gain/phase margins at plant inputs
        Mrgi_tmp = allmargin(SF_tmp.Li);

        % vary gain/phase perturbation at all plant outputs
        [DMo_tmp, MMo_tmp] = diskmargin(SF_tmp.Lo);

        % vary gain/phase perturbation at all plant inputs
        [DMi_tmp, MMi_tmp] = diskmargin(SF_tmp.Li);

        % vary gain/phase perturbation at all inputs and outputs
        MMio_tmp = diskmargin(Plant(:, :, c_ws_idx), FullOrderTuning(case_idx).Controller_scaled);
        
        FullOrderTuning(case_idx).Mrgo = Mrgo_tmp;
        FullOrderTuning(case_idx).Mrgi = Mrgi_tmp;
        FullOrderTuning(case_idx).DMo = DMo_tmp;
        FullOrderTuning(case_idx).DMi = DMi_tmp;
        FullOrderTuning(case_idx).MMo = MMo_tmp;
        FullOrderTuning(case_idx).MMi = MMi_tmp;
        FullOrderTuning(case_idx).MMio = MMio_tmp;
        
        dc = abs(dcgain(FullOrderTuning(case_idx).Controller_scaled));

        % QUESTION MANUEL is this how to compute bandwidth
        FullOrderTuning(case_idx).wc = getGainCrossover(FullOrderTuning(case_idx).Controller_scaled, min(dc, [], 'all') / sqrt(2));

        % bodemag(Li); xline(FullOrderTuning(case_idx).wci); yline(mag2db(dci(2,2)));

        % check norms of KSi, Ti, So, GSo (4 channels) and To
        
        % Ti_tmp = Ti(Plant(:, :, c_ws_idx), FullOrderTuning(case_idx).Controller_scaled); % 
        % To_tmp = To(Plant(:, :, c_ws_idx), FullOrderTuning(case_idx).Controller_scaled);
        % KSi_tmp = KSi(Plant(:, :, c_ws_idx), FullOrderTuning(case_idx).Controller_scaled);
        % So_tmp = So(Plant(:, :, c_ws_idx), FullOrderTuning(case_idx).Controller_scaled);
        % GSo_tmp = GSo(Plant(:, :, c_ws_idx), FullOrderTuning(case_idx).Controller_scaled);
        
        FullOrderTuning(case_idx).SF = SF_tmp;

        % FullOrderTuning(case_idx).inf_norms.Ti = norm(SF_tmp.Ti, Inf);
        % FullOrderTuning(case_idx).inf_norms.To = norm(SF_tmp.To, Inf);
        % FullOrderTuning(case_idx).inf_norms.KSi = norm(KSi_tmp);
        % FullOrderTuning(case_idx).inf_norms.Si = norm(SF_tmp.Si, Inf);
        % FullOrderTuning(case_idx).inf_norms.So = norm(SF_tmp.So, Inf);
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
                
                FullOrderControllers(weighting_case_idx).Mrgo(:, cc) = FullOrderTuning(case_idx).Mrgo;
                FullOrderControllers(weighting_case_idx).Mrgi(:, cc) = FullOrderTuning(case_idx).Mrgi;
                FullOrderControllers(weighting_case_idx).DMo(:, cc) = FullOrderTuning(case_idx).DMo;
                FullOrderControllers(weighting_case_idx).DMi(:, cc) = FullOrderTuning(case_idx).DMi;
                FullOrderControllers(weighting_case_idx).MMo(:, cc) = FullOrderTuning(case_idx).MMo;
                FullOrderControllers(weighting_case_idx).MMi(:, cc) = FullOrderTuning(case_idx).MMi;
                FullOrderControllers(weighting_case_idx).MMio(:, cc) = FullOrderTuning(case_idx).MMio;

                FullOrderControllers(weighting_case_idx).wc(:, cc) = FullOrderTuning(case_idx).wc;
                % FullOrderControllers(weighting_case_idx).n_wc(:, cc) = FullOrderTuning(case_idx).n_wc;
                
                FullOrderControllers(weighting_case_idx).SF(cc) = FullOrderTuning(case_idx).SF;
                % FullOrderControllers(weighting_case_idx).inf_norms.Ti(cc) = FullOrderTuning(case_idx).inf_norms.Ti;
                % FullOrderControllers(weighting_case_idx).inf_norms.To(cc) = FullOrderTuning(case_idx).inf_norms.To;
                % FullOrderControllers(weighting_case_idx).inf_norms.KSi(c_ws_idx) = FullOrderTuning(case_idx).inf_norms.KSi;
                % FullOrderControllers(weighting_case_idx).inf_norms.Si(cc) = FullOrderTuning(case_idx).inf_norms.Si;
                % FullOrderControllers(weighting_case_idx).inf_norms.So(cc) = FullOrderTuning(case_idx).inf_norms.So;
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
        FullOrderControllers(w_idx).Controller.InputName = {'Measured M_d Tracking Error', 'Measured M_q Tracking Error'};
        FullOrderControllers(w_idx).Controller.OutputName = {'\beta_d Control Input', '\beta_q Control Input'};
        FullOrderControllers(w_idx).Controller.SamplingGrid = struct('u', LPV_CONTROLLER_WIND_SPEEDS(full_controller_case_basis.WindSpeedIndex));
        FullOrderControllers(w_idx).Controller_scaled = ss(FullOrderControllers(w_idx).Controller_scaled);
        FullOrderControllers(w_idx).Controller_scaled.InputName = {'Measured M_d Tracking Error', 'Measured M_q Tracking Error'};
        FullOrderControllers(w_idx).Controller_scaled.OutputName = {'\beta_d Control Input', '\beta_q Control Input'};
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
            
            Controllers_case_list(c_idx).Mrgi = FullOrderControllers(w_idx).Mrgi;
            Controllers_case_list(c_idx).Mrgo = FullOrderControllers(w_idx).Mrgo;
            Controllers_case_list(c_idx).DMi = FullOrderControllers(w_idx).DMi;
            Controllers_case_list(c_idx).DMo = FullOrderControllers(w_idx).DMo;
            Controllers_case_list(c_idx).MMo = FullOrderControllers(w_idx).MMo;
            Controllers_case_list(c_idx).MMi = FullOrderControllers(w_idx).MMi;
            Controllers_case_list(c_idx).MMio = FullOrderControllers(w_idx).MMio;

            Controllers_case_list(c_idx).n_wc = length(FullOrderControllers(w_idx).wc);

            if Controllers_case_list(c_idx).n_wc == 0
                Controllers_case_list(c_idx).wc = -1;
            else
                Controllers_case_list(c_idx).wc = FullOrderControllers(w_idx).wc(1);
            end
            
            
            Controllers_case_list(c_idx).SF.Stable = FullOrderControllers(w_idx).SF.Stable;
            % Controllers_case_list(c_idx).SF.Ti = FullOrderControllers(w_idx).SF.Ti;
            % Controllers_case_list(c_idx).SF.To = FullOrderControllers(w_idx).SF.To;
            % PI_ParameterSweep.tfs.KSi(:, :, i, j) = PI_ParameterSweep_tmp(v).tfs.KSi;
            % Controllers_case_list(c_idx).SF.Si = FullOrderControllers(w_idx).SF.Si;
            % Controllers_case_list(c_idx).SF.So = FullOrderControllers(w_idx).SF.So;
    
            % Controllers_case_list(c_idx).inf_norms.Ti = FullOrderControllers(w_idx).inf_norms.Ti;
            % Controllers_case_list(c_idx).inf_norms.To = FullOrderControllers(w_idx).inf_norms.To;
            % Controllers_case_list(c_idx).inf_norms.KSi = FullOrderControllers(w_idx).inf_norms.KSi;
            % Controllers_case_list(c_idx).inf_norms.Si = FullOrderControllers(w_idx).inf_norms.Si;
            % Controllers_case_list(c_idx).inf_norms.So = FullOrderControllers(w_idx).inf_norms.So;
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
        % Ti_infnorm_tmp(w_idx) = Controllers_case_list(w_idx).inf_norms.Ti;
        % To_infnorm_tmp(w_idx) = Controllers_case_list(w_idx).inf_norms.To;
        % Si_infnorm_tmp(w_idx) = Controllers_case_list(w_idx).inf_norms.Si;
        % So_infnorm_tmp(w_idx) = Controllers_case_list(w_idx).inf_norms.So;

        Mrgi_tmp(w_idx, 1, 1) = Controllers_case_list(w_idx).Mrgi(1).GainMargin(1);
        if length(Controllers_case_list(w_idx).Mrgi(1).PhaseMargin)
            Mrgi_tmp(w_idx, 1, 2) = Controllers_case_list(w_idx).Mrgi(1).PhaseMargin(1);
        else
            Mrgi_tmp(w_idx, 1, 2) = 0;
        end

        Mrgi_tmp(w_idx, 2, 1) = Controllers_case_list(w_idx).Mrgi(2).GainMargin(1);
        if length(Controllers_case_list(w_idx).Mrgi(2).PhaseMargin)
            Mrgi_tmp(w_idx, 2, 2) = Controllers_case_list(w_idx).Mrgi(2).PhaseMargin(1);
        else
            Mrgi_tmp(w_idx, 2, 2) = 0;
        end

        Mrgo_tmp(w_idx, 1, 1) = Controllers_case_list(w_idx).Mrgo(1).GainMargin(1);

        if length(Controllers_case_list(w_idx).Mrgo(1).PhaseMargin)
            Mrgo_tmp(w_idx, 1, 2) = Controllers_case_list(w_idx).Mrgo(1).PhaseMargin(1);
        else
            Mrgo_tmp(w_idx, 1, 2) = 0;
        end

        Mrgo_tmp(w_idx, 2, 1) = Controllers_case_list(w_idx).Mrgo(2).GainMargin(1);

        if length(Controllers_case_list(w_idx).Mrgo(2).PhaseMargin)
            Mrgo_tmp(w_idx, 2, 2) = Controllers_case_list(w_idx).Mrgo(2).PhaseMargin(1);
        else
            Mrgo_tmp(w_idx, 2, 2) = 0;
        end
        
        DMi_tmp(w_idx, 1, 1) = Controllers_case_list(w_idx).DMi(1).GainMargin(2);
        DMi_tmp(w_idx, 1, 2) = Controllers_case_list(w_idx).DMi(1).PhaseMargin(2);
        DMi_tmp(w_idx, 1, 3) = Controllers_case_list(w_idx).DMi(1).DiskMargin;

        DMi_tmp(w_idx, 2, 1) = Controllers_case_list(w_idx).DMi(2).GainMargin(2);
        DMi_tmp(w_idx, 2, 2) = Controllers_case_list(w_idx).DMi(2).PhaseMargin(2);
        DMi_tmp(w_idx, 2, 3) = Controllers_case_list(w_idx).DMi(2).DiskMargin;
        
        DMo_tmp(w_idx, 1, 1) = Controllers_case_list(w_idx).DMo(1).GainMargin(2);
        DMo_tmp(w_idx, 1, 2) = Controllers_case_list(w_idx).DMo(1).PhaseMargin(2);
        DMo_tmp(w_idx, 1, 3) = Controllers_case_list(w_idx).DMo(1).DiskMargin;
        
        DMo_tmp(w_idx, 2, 1) = Controllers_case_list(w_idx).DMo(2).GainMargin(2);
        DMo_tmp(w_idx, 2, 2) = Controllers_case_list(w_idx).DMo(2).PhaseMargin(2);
        DMo_tmp(w_idx, 2, 3) = Controllers_case_list(w_idx).DMo(2).DiskMargin;
        
        MMi_tmp(w_idx, 1) = Controllers_case_list(w_idx).MMi.GainMargin(2);
        MMi_tmp(w_idx, 2) = Controllers_case_list(w_idx).MMi.PhaseMargin(2);
        MMi_tmp(w_idx, 3) = Controllers_case_list(w_idx).MMi.DiskMargin;
        
        MMo_tmp(w_idx, 1) = Controllers_case_list(w_idx).MMo.GainMargin(2);
        MMo_tmp(w_idx, 2) = Controllers_case_list(w_idx).MMo.PhaseMargin(2);
        MMo_tmp(w_idx, 3) = Controllers_case_list(w_idx).MMo.DiskMargin;
        
        MMio_tmp(w_idx, 1) = Controllers_case_list(w_idx).MMio.GainMargin(2);
        MMio_tmp(w_idx, 2) = Controllers_case_list(w_idx).MMio.PhaseMargin(2);
        MMio_tmp(w_idx, 3) = Controllers_case_list(w_idx).MMio.DiskMargin;
        
        wc_tmp(w_idx) = Controllers_case_list(w_idx).wc;
        % n_wc_tmp(w_idx, :) = Controllers_case_list(w_idx).n_wc;
    end
    
    % [0; Ti_infnorm_tmp'], ...
        % [0; To_infnorm_tmp'], ... % [0; PI_ParameterSweep.inf_norms.KSi], ...
        % [0; Si_infnorm_tmp'], ...
        % [0; So_infnorm_tmp'], ... % [0; PI_ParameterSweep.inf_norms.GSo], ... % Ti_tmp, To_tmp, KSi_tmp, So_tmp, GSo_tmp, ...

    % Make table comparing controllers
    Controllers_case_table = table( ...
        (0:n_weighting_cases)', case_desc', ... 
        [0; Stable_tmp'], ...
        [0; mag2db(Mrgi_tmp(:, 1, 1))], [0; Mrgi_tmp(:, 1, 2)], ...
        [0; mag2db(Mrgi_tmp(:, 2, 1))], [0; Mrgi_tmp(:, 2, 2)], ...
        [0; mag2db(Mrgo_tmp(:, 1, 1))], [0; Mrgo_tmp(:, 1, 2)], ...
        [0; mag2db(Mrgo_tmp(:, 2, 1))], [0; Mrgo_tmp(:, 2, 2)], ...
        [0; mag2db(DMi_tmp(:, 1, 1))], [0; DMi_tmp(:, 1, 2)], [0; DMi_tmp(:, 1, 3)], ...
        [0; mag2db(DMi_tmp(:, 2, 1))], [0; DMi_tmp(:, 2, 2)], [0; DMi_tmp(:, 2, 3)], ...
        [0; mag2db(DMo_tmp(:, 1, 1))], [0; DMo_tmp(:, 1, 2)], [0; DMo_tmp(:, 1, 3)], ...
        [0; mag2db(DMo_tmp(:, 2, 1))], [0; DMo_tmp(:, 2, 2)], [0; DMo_tmp(:, 2, 3)], ...
        [0; mag2db(MMi_tmp(:, 1))], [0; MMi_tmp(:, 2)], [0; MMi_tmp(:, 3)], ...
        [0; mag2db(MMo_tmp(:, 1))], [0; MMo_tmp(:, 2)], [0; MMo_tmp(:, 3)], ...
        [0; mag2db(MMio_tmp(:, 1))], [0; MMio_tmp(:, 2)], [0; MMio_tmp(:, 3)], ...
        [0; wc_tmp'], ...
        zeros(n_weighting_cases + 1, 1), zeros(n_weighting_cases + 1, 1), ...
        'VariableNames', ...
        {'Case No.', 'Case Desc.', 'Stable', ...
        'ClassicalInput_GMD', 'ClassicalInput_PMD', ...
        'ClassicalInput_GMQ', 'ClassicalInput_PMQ', ...
        'ClassicalOutput_GMD', 'ClassicalOutput_PMD', ...
        'ClassicalOutput_GMQ', 'ClassicalOutput_PMQ', ...
        'SingleDiskInput_GMD', 'SingleDiskInput_PMD', 'SingleDiskInput_DMD', ...
        'SingleDiskInput_GMQ', 'SingleDiskInput_PMQ', 'SingleDiskInput_DMQ', ...
        'SingleDiskOutput_GMD', 'SingleDiskOutput_PMD', 'SingleDiskOutput_DMD', ...
        'SingleDiskOutput_GMQ', 'SingleDiskOutput_PMQ', 'SingleDiskOutput_DMQ', ...
        'MultiDiskInput_GM', 'MultiDiskInput_PM', 'MultiDiskInput_DM', ...
        'MultiDiskOutput_GM', 'MultiDiskOutput_PM', 'MultiDiskOutput_DM', ...
        'MultiDiskIO_GM', 'MultiDiskIO_PM', 'MultiDiskIO_DM', ...
        'wc', 'ADC', 'RootMycBlade1 MSE'});
% 'Hinf(Ti)', 'Hinf(To)', 'Hinf(Si)', 'Hinf(So)', ... 

    
    Controllers_case_table.("WorstCase_SingleClassical_GM") = ...
        min(Controllers_case_table(:, ...
        ["ClassicalInput_GMD", "ClassicalInput_GMQ", ...
        "ClassicalOutput_GMD", "ClassicalOutput_GMQ"]), [], 2).Variables;
    Controllers_case_table.("WorstCase_SingleClassical_PM") = ...
        min(Controllers_case_table(:, ...
        ["ClassicalInput_PMD", "ClassicalInput_PMQ", ...
        "ClassicalOutput_PMD", "ClassicalOutput_PMQ"]), [], 2).Variables;

    [M, I] = min(Controllers_case_table(:, ...
        ["SingleDiskInput_DMD", "SingleDiskInput_DMQ", ...
        "SingleDiskOutput_DMD", "SingleDiskOutput_DMQ"]), [], 2);
    x = Controllers_case_table(:, ...
        ["SingleDiskInput_GMD", "SingleDiskInput_GMQ", ...
        "SingleDiskOutput_GMD", "SingleDiskOutput_GMQ"]).Variables;
    Controllers_case_table.("WorstCase_SingleDisk_GM") = x(I.Variables);
    x = Controllers_case_table(:, ...
        ["SingleDiskInput_PMD", "SingleDiskInput_PMQ", ...
        "SingleDiskOutput_PMD", "SingleDiskOutput_PMQ"]).Variables;
    Controllers_case_table.("WorstCase_SingleDisk_PM") = x(I.Variables);

    % Controllers_case_table.("Mean DMo") = mean(Controllers_case_table(:, ["DMoD", "DMoQ"]), 2).Variables;
    % Controllers_case_table.("Mean DM") = mean(Controllers_case_table(:, ["Mean DMi", "Mean DMo"]), 2).Variables;

    sortrows(Controllers_case_table, 'MultiDiskIO_DM', 'descend')
    % QUESTION MANUEL how to estimate bandwidth when tf never crosses unity
    sortrows(Controllers_case_table, 'wc', 'descend')

    if EXTREME_K_COLLECTION
        save(fullfile(mat_save_dir, 'Extreme_Controllers_case_list.mat'), "Controllers_case_list", '-v7.3');
        save(fullfile(mat_save_dir, 'Extreme_Controllers_case_table.mat'), "Controllers_case_table", '-v7.3');
    elseif OPTIMAL_K_COLLECTION
        save(fullfile(mat_save_dir, 'Optimal_Controllers_case_list.mat'), "Controllers_case_list", '-v7.3')
        save(fullfile(mat_save_dir, 'Optimal_Controllers_case_table.mat'), "Controllers_case_table", '-v7.3');
    end

else
    if EXTREME_K_COLLECTION
        load(fullfile(mat_save_dir, 'Extreme_Controllers_case_list.mat'));
        load(fullfile(mat_save_dir, 'Extreme_Controllers_case_table.mat'));
    elseif OPTIMAL_K_COLLECTION
        load(fullfile(mat_save_dir, 'Optimal_Controllers_case_list.mat'));
        load(fullfile(mat_save_dir, 'Optimal_Controllers_case_table.mat'));
    end
    
end

end

%% Plot transfer functions and weighting functions for full-order controller
PLOTTING = 0;
% TODO plot transfer functions for single wind speed
if PLOTTING

    cc = find(full_controller_case_basis.WindSpeedIndex == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED));
    n_weighting_cases = (FullOrderControllers_n_cases / length(full_controller_case_basis.WindSpeedIndex));

    % TODO Plot singular values of KSi, Ti; So, GSo vs f for single wind speed on
    % 2*2 plot for open-loop vs closed-loop
    % QUESTION what meaning does the open-loop singular value have for
    % these transfer functions, only defined and non-identity for So?

    for w_idx = 1:n_weighting_cases
        figure;
        % bodemag(Wu_tmp, We_tmp, W1_tmp, W2_tmp); legend('Wu', 'We', 'W1', 'W2');
        Wu_tmp = Controllers_case_list(w_idx).WuGain * Wu;
        We_tmp = Controllers_case_list(w_idx).WeGain * We;
        W1_tmp = Controllers_case_list(w_idx).W1Gain * W1;
        W2_tmp = Controllers_case_list(w_idx).W2Gain * W2;

        SF = loopsens(-Plant(:, :, cc), ...
            -Controllers_case_list(w_idx).Controller_scaled(:, :, cc));
        % SF = Controllers_case_list(w_idx).SF;

        sys_KSi = Controllers_case_list(w_idx).Controller_scaled(:, :, cc) * SF.Si * W1_tmp;
        sys_Ti = SF.Ti * W2_tmp;
        % sys_Ti2 = (Controllers_case_list(w_idx).Controller_scaled(:, :, cc) * Plant(:, :, cc)) / ...
        %           (eye(size(Plant(:, :, cc))) + Controllers_case_list(w_idx).Controller_scaled(:, :, cc) * Plant(:, :, cc)) * W2_tmp;
        % QUESTION MANUEL does this make sense?
        sys_So_cl = SF.So * W1_tmp;
        % sys_So_cl2 = inv(eye(size(Plant(:, :, cc))) + Plant(:, :, cc) * Controllers_case_list(w_idx).Controller_scaled(:, :, cc)) * W1_tmp;
        sys_So_ol = Plant(:, :, cc) * W1_tmp;
        sys_GSo = Plant(:, :, cc) * SF.So * W2_tmp;

        ax = subplot(2, 2, 1);
        sigmaplot(ax, sys_KSi, inv(Wu_tmp) * Controllers_case_list(w_idx).gamma(cc), sigma_plot_opt);
        legend('Closed-Loop', 'Bound');
        title(ax, 'KS_i');

        ax = subplot(2, 2, 2);
        sigmaplot(ax, sys_Ti, inv(Wu_tmp) * Controllers_case_list(w_idx).gamma(cc), sigma_plot_opt);
        legend('Closed-Loop', 'Bound');
        title(ax, 'T_i');
        

        ax = subplot(2, 2, 3);
        sigmaplot(ax, sys_So_ol, sys_So_cl, inv(We_tmp) * Controllers_case_list(w_idx).gamma(cc), sigma_plot_opt);
        legend('Open-Loop', 'Closed-Loop', 'Bound');
        title(ax, 'S_o');
        % QUESTION MANUEL should these bounds limit the singular values?
        ax = subplot(2, 2, 4);
        sigmaplot(ax, sys_GSo, inv(We_tmp) * Controllers_case_list(w_idx).gamma(cc), sigma_plot_opt);
        legend('Closed-Loop', 'Bound');
        title(ax, 'GS_o');
    end

    % TODO Plot classical, disk and MIMO stability margins of designed
    % controller over wind speeds for controller tuned at 16m/s and
    % controller tuned for different wind speeds OUTPLOT

    %% Plot robustness margins OUTPLOT
    figure;
    tcf = tiledlayout(1, 2);
    for w_idx = 1:n_weighting_cases
        nexttile(1);
        outputs = cell(1, length(full_controller_case_basis.WindSpeedIndex));
        [outputs{:}] = mag2db(Controllers_case_list(w_idx).MMio.GainMargin(2));
        scatter(LPV_CONTROLLER_WIND_SPEEDS(full_controller_case_basis.WindSpeedIndex), cell2mat(outputs));
        set(gca,'yscale','log');
        title('Multi-Input-Output Gain Margin');
        ylabel('dB');
        hold on;

        nexttile(2);
        outputs = cell(1, length(full_controller_case_basis.WindSpeedIndex));
        [outputs{:}] = Controllers_case_list(w_idx).MMio.PhaseMargin(2);
        scatter(LPV_CONTROLLER_WIND_SPEEDS(full_controller_case_basis.WindSpeedIndex), cell2mat(outputs));
        title('Multi-Input-Output Phase Margin');
        ylabel('deg');
        hold on;

        % nexttile(3);
        % outputs = cell(1, length(LPV_CONTROLLER_WIND_SPEEDS));
        % [outputs{:}] = Controllers_case_list(w_idx).DMo(1, :).DiskMargin;
        % plot(LPV_CONTROLLER_WIND_SPEEDS, cell2mat(outputs));
        % title('DM_o D Loop');
        % hold on;
        % 
        % nexttile(4);
        % outputs = cell(1, length(LPV_CONTROLLER_WIND_SPEEDS));
        % [outputs{:}] = Controllers_case_list(w_idx).DMo(2, :).DiskMargin;
        % plot(LPV_CONTROLLER_WIND_SPEEDS, cell2mat(outputs));
        % title('DM_o Q Loop');
        % hold on;
    end
    legend([arrayfun(@(n) ['Case ', num2str(n)], ... 
    1:n_weighting_cases, 'UniformOutput', false) {'', '', '', ''}], ...
    'NumColumns', 2);
    xticks(LPV_CONTROLLER_WIND_SPEEDS);
    set(gcf, 'Position', [0 0 1500 900]);
    savefig(gcf, fullfile(fig_dir, 'fullorder_diskmargins.fig'));
    saveas(gcf, fullfile(fig_dir, 'fullorder_diskmargins.png'));

    for w_idx = 1:n_weighting_cases
        close all;

        %% Plot Generalized Plant transfer functions OUTPLOT
        GenPlant_inner = inv(Controllers_case_list(w_idx).Wout(:, :, cc)) ...
            * Controllers_case_list(w_idx).GenPlant(:, :, cc) ...
            * inv(Controllers_case_list(w_idx).Win(:, :, cc));
        GenPlant_inner.OutputName = Controllers_case_list(w_idx).GenPlant(:, :, cc).OutputName;
        GenPlant_inner.InputName = Controllers_case_list(w_idx).GenPlant(:, :, cc).InputName;

        % GenPlant_inner.InputGroup.dist = 1:4;
        GenPlant_inner.InputGroup.dist = find(ismember(...
            GenPlant_inner.InputName, ...
            {'M_d Reference', 'M_q Reference', ...
            '\beta_d Disturbance', '\beta_q Disturbance'}))';
    
        GenPlant_inner.InputGroup.acts = find(ismember(...
            GenPlant_inner.InputName, ...
            {'\beta_d Control Input', '\beta_q Control Input'}))';
            
        GenPlant_inner.OutputGroup.perf = find(ismember(...
            GenPlant_inner.OutputName, ...
            {'Weighted \beta_d Control Input', 'Weighted \beta_q Control Input', ...
            'Weighted M_d Tracking Error', 'Weighted M_q Tracking Error'}))';
            % 'Weighted M_d Output', 'Weighted M_q Output'}))';
    
        GenPlant_inner.OutputGroup.meas = find(ismember(...
            GenPlant_inner.OutputName, ...
            {'Measured M_d Tracking Error', 'Measured M_q Tracking Error'}))';
         
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
        % So (bottom left corner, d1->ze) and GSo (bottom right corner,
        % d2->ze) have been shifted
        loopData = getLoopData(GenPlant_inner, ...
            Controllers_case_list(w_idx).Controller(:, :, cc), ...
            Win_arr, Wout_arr);
        w = logspace(-2, 4, 400);
        [h, loopData] = plotLoopBode(loopData, [], [], w, bode_plot_opt);
    
        axh = findall(gcf, 'type', 'axes');
        for a = 1:length(axh)
            if loopData.peaks.wGAM > 0
            xline(axh(a), [loopData.peaks.wGAM]);
            axh(a).YLabel.Rotation = 0;
            end
        end
        % legend('W_{out}^{-1}\gamma', 'Open-Loop', 'Closed-Loop', '');
        set(gcf, 'Position', [0 0 1500 900]);
        savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_genplant_bodemag.fig']));
        saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_genplant_bodemag.png']));

        %% Plot transfer functions
        figure;
        Wu_tmp = Controllers_case_list(w_idx).WuGain * Wu;
        We_tmp = Controllers_case_list(w_idx).WeGain * We;
        W1_tmp = Controllers_case_list(w_idx).W1Gain * W1;
        W2_tmp = Controllers_case_list(w_idx).W2Gain * W2;
        SF = loopsens(-Plant(:, :, cc), ...
            -Controllers_case_list(w_idx).Controller_scaled(:, :, cc));
        sys_KSi = Controllers_case_list(w_idx).Controller_scaled(:, :, cc) * SF.Si * W1_tmp;
        sys_Ti = SF.Ti * W2_tmp;
        sys_So_cl = SF.So * W1_tmp;
        sys_So_ol = Plant(:, :, cc) * W1_tmp;
        sys_GSo = Plant(:, :, cc) * SF.So * W2_tmp;

        % Plot output sensitivity function and its weighting matrix, dy-y or
        % n->e or r->e (ideally 0)
        % S and T are coupled : S + T = 1, so choosing 1 determines the other
        % need penalty on both bc want both to have small magnitdues at particular
        % frequencies (ie could add 2 complex numbers with large magnitudes at
        % high phases that add to 1, or have one with large mag and the other with very small in alignment)
        % OpenLoop case: So = 1, ClosedLoop case: can shape So, ideally to 0.
        % Real So: hpf with roll-off with high-freq gain of 1.
        % So = e/W1d1, output disturbance -> tracking error
        subplot(2,2,3);
        % sys.InputName = GenPlant.InputName(1:2); % output disturbance/reference
        sys_So_cl.InputName = GenPlant.InputName(1:2); % output disturbance/reference
        % We_tmp.OutputName = {'M_d Reference', 'M_q Reference'};
        sys_So_cl.OutputName = We_tmp.InputName;
        % We_tmp.InputName = {'M_d Tracking Error', 'M_q Tracking Error'};
        h = bodeplot(...
            sys_So_cl, ...
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
        xticks(axh(9), [1e-1, 1e0, 1e1, 1e2, 1e3, 1e4])

        % set(gcf, 'Position', [0 0 1500 900]);
        legend('S_o W_1', 'W_e^{-1}\gamma');
        % savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_So_bodemag.fig']));
        % saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_So_bodemag.jpg']));
        
        % Plot complementary output sensitivity function and its weighting matrix
        % To = y/W2d2, input disturbance -> control input
        % figure;
        % % sigma(CL_full_tuned(:, :, 1, cc), ss(gamma_full_tuned(1, cc)));
        % sys = SF.To * W2_tmp; % requirements defined on output side
        % sys.InputName = GenPlant.InputName(1:2); % output disturbance/references
        % sys.OutputName = GenPlant.OutputName(5:6); % output
        % sys.InputName = {'M_d Reference', 'M_q Reference'};
        % Wy.OutputName = {'M_d Reference', 'M_q Reference'};
        % sys.OutputName = {'M_d Output', 'M_q Output'};
        % Wy.InputName = {'M_d Output', 'M_q Output'};
        % bodeplot(...
        %     sys, ...
        %     inv(Wy) * Controllers_case_list(w_idx).gamma(cc), w, bode_plot_opt);
        % title('Complementary Output Sensitivity Function, T_o');
        % axh = findall(gcf, 'type', 'axes');
        % for a = 1:length(axh)
        %     xline(axh(a), [loopData.peaks.wGAM]);
        % end
        % set(gcf, 'Position', [0 0 1500 900]);
        % legend('T_o W_2', 'W_y^{-1}\gamma');
        % savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'To_full_bodemag.fig']));
        % saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'To_full_bodemag.png']));
        
        % Plot complementary input sensitivity function and its weighting matrix
        % Ti = -u/W2d2, input disturbance -> control input
        subplot(2,2,2);
        sys_Ti.InputName = GenPlant.InputName(3:4); % control disturbance
        sys_Ti.OutputName = Wu_tmp.InputName; % control input
        sys_Ti.InputName = {'\beta_d Disturbance', '\beta_q Disturbance'};
        % Wu_tmp.OutputName = {'\beta_d Disturbance', '\beta_q Disturbance'};
        sys_Ti.OutputName = {'\beta_d Control Input', '\beta_q Control Input'};
        % Wu_tmp.InputName = {'\beta_d Control Input', '\beta_q Control Input'};
        bodeplot(...
            sys_Ti, ...
            inv(Wu_tmp) * Controllers_case_list(w_idx).gamma(cc), w, bode_plot_opt);
        title('Complementary Input Sensitivity Function, T_i');
        axh = findall(gcf, 'type', 'axes');
        for a = 1:length(axh)
            xline(axh(a), [loopData.peaks.wGAM]);
        end
        set(gcf, 'Position', [0 0 1500 900]);
        legend('T_i W_2', 'W_u^{-1}\gamma');
        % savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'Ti_full_bodemag.fig']));
        % saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'Ti_full_bodemag.png']));
    
        % Plot GSo with two weighting functions
        % GSo = -e/W2d2, input disturbance -> tracking error
        subplot(2,2,4);
        sys_GSo.InputName = GenPlant.InputName(3:4); % control disturbance
        sys_GSo.OutputName = We_tmp.InputName; % tracking error
        bodeplot(...
            sys_GSo, ...
            inv(We_tmp) * Controllers_case_list(w_idx).gamma(cc), w, bode_plot_opt);
        title('GS_o Function');
        axh = findall(gcf, 'type', 'axes');
        for a = 1:length(axh)
            xline(axh(a), [loopData.peaks.wGAM]);
        end
        set(gcf, 'Position', [0 0 1500 900]);
        legend('GS_o W_2', 'W_e^{-1}\gamma', 'W_y^{-1}\gamma');
        % savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'GSo_full_bodemag.fig']));
        % saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'GSo_full_bodemag.png']));

        % Plot KS_i with two weighting functions
        % KSi = u/W1d1, output disturbance -> control input
        subplot(2,2,1);
        sys_KSi.InputName = GenPlant.InputName(1:2); % output disturbance/reference
        sys_KSi.OutputName = Wu_tmp.InputName; % control input
        bodeplot(...
            sys_KSi, ...
            inv(Wu_tmp) * Controllers_case_list(w_idx).gamma(cc), w, ...
             bode_plot_opt);
        title('KS_i Function');
        axh = findall(gcf, 'type', 'axes');
        for a = 1:length(axh)
            xline(axh(a), [loopData.peaks.wGAM]);
        end
        set(gcf, 'Position', [0 0 1500 900]);
        legend('GS_o W_1', 'W_u^{-1}\gamma');
        
        savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_tfs_bodemag.fig']));
        saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_tfs_bodemag.png']));
    end
    
    %% Plot tuned controller for this weighting case OUTPLOT
    for w_idx = 1:n_weighting_cases
        
        figure;
        bcol = copper(length(LPV_CONTROLLER_WIND_SPEEDS)); % Define the color order based on the number of models
        % rcol = jet(length(LPV_CONTROLLER_WIND_SPEEDS));
        % ycol = hot(length(LPV_CONTROLLER_WIND_SPEEDS)); % https://www.mathworks.com/help/matlab/colors-1.html?s_tid=CRUX_lftnav
    
        omega = logspace(-2, 4, 300);
    
        for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
            if false || LPV_CONTROLLER_WIND_SPEEDS(c_ws_idx) ~= NONLPV_CONTROLLER_WIND_SPEED
                continue;
            end
            
            % QUESTION MANUEL problem with scaling, amplitudes are small!!
            K_tmp_scaled = Controllers_case_list(w_idx).Controller_scaled(:, :, c_ws_idx);
            K_tmp = Controllers_case_list(w_idx).Controller(:, :, c_ws_idx);
            % K_tmp = Controllers_case_list(w_idx).Controller(:, :, cc);
            K_tmp.InputName = {'M_d Tracking Error', 'M_q Tracking Error'};
            K_tmp.OutputName = {'\beta_d Control Input', '\beta_q Control Input'};
            K_tmp_scaled.InputName = {'M_d Tracking Error', 'M_q Tracking Error'};
            K_tmp_scaled.OutputName = {'\beta_d Control Input', '\beta_q Control Input'};

            % Plot baseline and tuned controllers
            bodeplot(K_tmp_scaled, omega, bode_plot_opt);
    
            % Find handles of all lines in the figure that have the color blue
            blineHandle = findobj(gcf,'Type','line','-and','Color','b');
            % rlineHandle = findobj(gcf,'Type','line','-and','Color','r');
            % ylineHandle = findobj(gcf,'Type','line','-and','Color','y');
    
            % Change the color to the one you defined
            set(blineHandle,'Color',bcol(c_ws_idx,:));
            % set(rlineHandle,'Color',rcol(c_ws_idx,:));
            % set(ylineHandle,'Color',ycol(c_ws_idx,:));
            
            hold on
        end
    
    %      legend([arrayfun(@(n) ['Case ', num2str(n)], ... 
    % 1:n_weighting_cases, 'UniformOutput', false) {'', '', '', ''}], ...
    % 'NumColumns', 2);

        axh = findall(gcf, 'type', 'axes');
        xline(axh(3), omega_1P_rad * HARMONICS);
        xline(axh(5), omega_1P_rad * HARMONICS);
        xline(axh(7), omega_1P_rad * HARMONICS);
        xline(axh(9), omega_1P_rad * HARMONICS);
        set(gcf, 'Position', [0 0 1500 900]);
        % legend('Structured Baseline', 'Structured Tuned', 'Full-Order Tuned', '', '', '', '');
        %         title('Frequency Response of Tuned Controllers');
        hold off;

        set(gcf, 'Position', [0 0 1500 900]);
        savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_controller_bodemag.fig']));
        saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_controller_bodemag.png']));
    end 

end