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
% then Wy and We and Wu to penalize To and So and KSo and Ti
% full_controller_case_basis.WindSpeedIndex = 1:length(LPV_CONTROLLER_WIND_SPEEDS);
if EXTREME_K_COLLECTION
    full_controller_case_basis.WindSpeedIndex.x = find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED);
    full_controller_case_basis.WuGain.x = case_basis.WuGain.x;
    full_controller_case_basis.WeGain.x = case_basis.WeGain.x;
    full_controller_case_basis.W1Gain.x = case_basis.W1Gain.x;
    full_controller_case_basis.W2Gain.x = case_basis.W2Gain.x;
    full_controller_case_basis.Reference = case_basis.Reference;
    full_controller_case_basis.Saturation = case_basis.Saturation;
elseif OPTIMAL_K_COLLECTION
    full_controller_case_basis.WindSpeedIndex.x = 1:length(LPV_CONTROLLER_WIND_SPEEDS);
    full_controller_case_basis.WuGain = case_basis.WuGain;
    full_controller_case_basis.WeGain = case_basis.WeGain;
    full_controller_case_basis.W1Gain = case_basis.W1Gain;
    full_controller_case_basis.W2Gain = case_basis.W2Gain;
    full_controller_case_basis.Reference = case_basis.Reference;
    full_controller_case_basis.Saturation = case_basis.Saturation;
end

[FullOrderControllers_case_list, FullOrderControllers_case_name_list, FullOrderControllers_n_cases] ...
    = generateCases(full_controller_case_basis, 'full_order_tuned_controllers', true);

if EXTREME_K_COLLECTION
    % remove cases
    if 0
        exc_idx = [];
        for c_idx = 1:FullOrderControllers_n_cases
            % want exactly 2 weighting matrices to be 'on'
            % and only one weighting filter to have a notch
            if (~((length(FullOrderControllers_case_list(c_idx).WeGains.Numerator{1,1}) > 1) ...
                    && (length(FullOrderControllers_case_list(c_idx).W1Gains.Numerator{1,1}) > 1))) ...
                    && (ss(FullOrderControllers_case_list(c_idx).W1Gains).D(1, 1) ...
                    + ss(FullOrderControllers_case_list(c_idx).W2Gains).D(1, 1) == IP_HIGH_GAIN + IP_LOW_GAIN) ... % want one (but not both) to have high gain
                    && ((ss(FullOrderControllers_case_list(c_idx).WuGains).D(1, 1) ...
                    + ss(FullOrderControllers_case_list(c_idx).WeGains).D(1, 1) == WU_HIGH_GAIN + OP_LOW_GAIN) ... 
                    || (ss(FullOrderControllers_case_list(c_idx).WuGains).D(1, 1) ...
                    + ss(FullOrderControllers_case_list(c_idx).WeGains).D(1, 1) == WE_HIGH_GAIN + OP_LOW_GAIN))
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
    for f = fieldnames(case_basis.W1Gain)'
    parfor case_idx = 1:FullOrderControllers_n_cases
        controller_case = FullOrderControllers_case_list(case_idx);
        c_ws_idx = controller_case.WindSpeedIndex.x;

        FullOrderTuning(case_idx).WuGain = controller_case.WuGain;
        FullOrderTuning(case_idx).WeGain.(f{1}) = controller_case.WeGain.(f{1});
        FullOrderTuning(case_idx).W1Gain.(f{1}) = controller_case.W1Gain.(f{1});
        FullOrderTuning(case_idx).W2Gain.(f{1}) = controller_case.W2Gain.(f{1});

        % for f = fieldnames(controller_case.WuGain)'
            Wu_tmp = controller_case.WuGain.x * Wu;
            We_tmp = controller_case.WeGain.(f{1}) * We;
            W1_tmp = controller_case.W1Gain.(f{1}) * W1;
            W2_tmp = controller_case.W2Gain.(f{1}) * W2;

            [GenPlant_tmp, Win_tmp, Wout_tmp] = generateGenPlant(...
                Plant_scaled(:, :, c_ws_idx), ...
                Wu_tmp, We_tmp, W1_tmp, W2_tmp);
            
            opts = hinfsynOptions('Display', 'on'); 
            [K_tmp, CL_full_tuned_tmp, gamma_tmp] = hinfsyn(GenPlant_tmp, nmeas, ncont, opts); % NOTE does not compute for pure integrator in weighting matrix
    
            FullOrderTuning(case_idx).GenPlant.(f{1}) = GenPlant_tmp;
            FullOrderTuning(case_idx).Win.(f{1}) = Win_tmp;
            FullOrderTuning(case_idx).Wout.(f{1}) = Wout_tmp;
            FullOrderTuning(case_idx).CL.(f{1}) = CL_full_tuned_tmp;
            FullOrderTuning(case_idx).Controller.(f{1}) = ss(K_tmp); 
            FullOrderTuning(case_idx).Controller.(f{1}).InputName = {'Measured M_d Tracking Error', 'Measured M_q Tracking Error'};
            FullOrderTuning(case_idx).Controller.(f{1}).OutputName = {'\beta_d Control Input', '\beta_q Control Input'};
            FullOrderTuning(case_idx).gamma.(f{1}) = gamma_tmp;
    
            FullOrderTuning(case_idx).Controller_scaled.(f{1}) = ...
                        ss(ip_scaling(:, :, c_ws_idx) ...
                        * FullOrderTuning(case_idx).Controller.(f{1}) ...
                        * inv(op_scaling(:, :, c_ws_idx)));
            
            % negative Plant st e = r(dy) - y(yP) is input to controller, 
            % negative Controller for positive u input to Plant
            SF_tmp = loopsens(-Plant(:, :, c_ws_idx), -FullOrderTuning(case_idx).Controller_scaled.(f{1}));
            
            % classical gain/phase margins at plant outputs
            Mrgo_tmp = allmargin(SF_tmp.Lo);
    
            % classical gain/phase margins at plant inputs
            Mrgi_tmp = allmargin(SF_tmp.Li);
    
            % vary gain/phase perturbation at all plant outputs
            [DMo_tmp, MMo_tmp] = diskmargin(SF_tmp.Lo);
    
            % vary gain/phase perturbation at all plant inputs
            [DMi_tmp, MMi_tmp] = diskmargin(SF_tmp.Li);
    
            % vary gain/phase perturbation at all inputs and outputs
            MMio_tmp = diskmargin(Plant(:, :, c_ws_idx), FullOrderTuning(case_idx).Controller_scaled.(f{1}));
            
            FullOrderTuning(case_idx).Mrgo.(f{1}) = Mrgo_tmp;
            FullOrderTuning(case_idx).Mrgi.(f{1}) = Mrgi_tmp;
            FullOrderTuning(case_idx).DMo.(f{1}) = DMo_tmp;
            FullOrderTuning(case_idx).DMi.(f{1}) = DMi_tmp;
            FullOrderTuning(case_idx).MMo.(f{1}) = MMo_tmp;
            FullOrderTuning(case_idx).MMi.(f{1}) = MMi_tmp;
            FullOrderTuning(case_idx).MMio.(f{1}) = MMio_tmp;
    
            % FullOrderTuning(case_idx).ß = getGainCrossover(FullOrderTuning(case_idx).Controller_scaled, min(dc, [], 'all') / sqrt(2));
            % [s, w] = sigma(SF_tmp.So);
            % FullOrderTuning(case_idx).wc = getGainCrossover(max(s), 0.707);
            % QUESTION MANUEL shouldn't Lo be unity at low freq? Only the
            % case for CL output from hinfsyn...due to weightings?
            % is this the best way to compute bandwidth since we don't
            % cross unity?
            % Lo does not look like an integrator, is that okay?

            % [s, ~] = sigma(SF_tmp.So);
            % s = max(s);
            % dc = abs(dcgain(s));
            % dc = max(s(:, 1));
            x = getGainCrossover(inv(SF_tmp.So), 1);
            FullOrderTuning(case_idx).wc.(f{1}) = x(1);
            % bode(SF_tmp.Lo, SF_tmp.To, bode_plot_opt)
            % xline()
            FullOrderTuning(case_idx).SF.(f{1}) = SF_tmp;
        % end

    end
    end

    % restructure synthesized controller cases per weighting matrix
    % combination and wind speed
    % for each wind speed
    FullOrderControllers = struct;
    for c_ws_idx = 1:length(full_controller_case_basis.WindSpeedIndex.x)
        weighting_case_idx = 1;
        % find all of the controller cases that correspond to this wind speed
        for case_idx = 1:FullOrderControllers_n_cases
            controller_case = FullOrderControllers_case_list(case_idx);
            if full_controller_case_basis.WindSpeedIndex.x(c_ws_idx) == controller_case.WindSpeedIndex.x
                FullOrderControllers(weighting_case_idx).WuGain = FullOrderControllers_case_list(case_idx).WuGain;
                for f = fieldnames(case_basis.W1Gain)'
                    % cc = find(full_controller_case_basis.WindSpeedIndex == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED));
                    
                    FullOrderControllers(weighting_case_idx).WeGain.(f{1}) = FullOrderControllers_case_list(case_idx).WeGain.(f{1});
                    FullOrderControllers(weighting_case_idx).W1Gain.(f{1}) = FullOrderControllers_case_list(case_idx).W1Gain.(f{1});
                    FullOrderControllers(weighting_case_idx).W2Gain.(f{1}) = FullOrderControllers_case_list(case_idx).W2Gain.(f{1});
    
                    FullOrderControllers(weighting_case_idx).GenPlant.(f{1})(:, :, c_ws_idx) = FullOrderTuning(case_idx).GenPlant.(f{1});
                    FullOrderControllers(weighting_case_idx).Win.(f{1})(:, :, c_ws_idx) = FullOrderTuning(case_idx).Win.(f{1});
                    FullOrderControllers(weighting_case_idx).Wout.(f{1})(:, :, c_ws_idx) = FullOrderTuning(case_idx).Wout.(f{1});
                    FullOrderControllers(weighting_case_idx).gamma.(f{1})(c_ws_idx) = FullOrderTuning(case_idx).gamma.(f{1});
    
                    FullOrderControllers(weighting_case_idx).Controller.(f{1})(:, :, c_ws_idx) = ...
                        FullOrderTuning(case_idx).Controller.(f{1});
                    FullOrderControllers(weighting_case_idx).CL.(f{1})(:, :, c_ws_idx) = FullOrderTuning(case_idx).CL.(f{1});
    
                    FullOrderControllers(weighting_case_idx).Controller_scaled.(f{1})(:, :, c_ws_idx) = ...
                       FullOrderTuning(case_idx).Controller_scaled.(f{1});
                    
                    FullOrderControllers(weighting_case_idx).Mrgo.(f{1})(:, c_ws_idx) = FullOrderTuning(case_idx).Mrgo.(f{1});
                    FullOrderControllers(weighting_case_idx).Mrgi.(f{1})(:, c_ws_idx) = FullOrderTuning(case_idx).Mrgi.(f{1});
                    FullOrderControllers(weighting_case_idx).DMo.(f{1})(:, c_ws_idx) = FullOrderTuning(case_idx).DMo.(f{1});
                    FullOrderControllers(weighting_case_idx).DMi.(f{1})(:, c_ws_idx) = FullOrderTuning(case_idx).DMi.(f{1});
                    FullOrderControllers(weighting_case_idx).MMo.(f{1})(:, c_ws_idx) = FullOrderTuning(case_idx).MMo.(f{1});
                    FullOrderControllers(weighting_case_idx).MMi.(f{1})(:, c_ws_idx) = FullOrderTuning(case_idx).MMi.(f{1});
                    FullOrderControllers(weighting_case_idx).MMio.(f{1})(:, c_ws_idx) = FullOrderTuning(case_idx).MMio.(f{1});
    
                    FullOrderControllers(weighting_case_idx).wc.(f{1})(c_ws_idx) = FullOrderTuning(case_idx).wc.(f{1});
                    % FullOrderControllers(weighting_case_idx).n_wc(:, cc) = FullOrderTuning(case_idx).n_wc;
                    
                    FullOrderControllers(weighting_case_idx).SF.(f{1})(c_ws_idx) = FullOrderTuning(case_idx).SF.(f{1});
                    FullOrderControllers(weighting_case_idx).Stable.(f{1})(c_ws_idx) = FullOrderTuning(case_idx).SF.(f{1}).Stable;
                    
                    if 1
                        K_tmp = FullOrderTuning(case_idx).Controller.(f{1});
                        CL_tmp = FullOrderTuning(case_idx).CL.(f{1});
                        GenPlant_tmp = FullOrderTuning(case_idx).GenPlant.(f{1});
                        y = lft(GenPlant_tmp, K_tmp);
                        figure; bodemag(CL_tmp); hold on; bodemag(y); grid on;

                        SF_tmp = FullOrderTuning(case_idx).SF.(f{1});
                        figure; bodemag(SF_tmp.So, SF_tmp.To)
                        figure; bodemag(SF_tmp.So + SF_tmp.To) % good
                        
                        K_tmp.OutputName = {'\beta_d Control Input', '\beta_q Control Input'};
                        Plant_scaled.InputName = {'\beta_d Control Input', '\beta_q Control Input'};
                        K_tmp.InputName = {'Measured M_d Tracking Error', 'Measured M_q Tracking Error'};
                        Plant.OutputName = {'Measured M_d Output', 'Measured M_q Output'};
                        x = feedback(series(K_tmp, Plant(:, :, c_ws_idx)), eye(2));
                        
                        figure; bodemag(x); bodemag(y);
                        bode(SF_tmp.Lo, SF_tmp.To, bode_plot_opt)
                        % weighting_case_idx = floor(case_idx / length(LPV_CONTROLLER_WIND_SPEEDS)) + 1;
                    end

                end
                weighting_case_idx = weighting_case_idx + 1;
            end
        end
    end
    if length(FullOrderControllers) == 1
        FullOrderControllers = [FullOrderControllers];
    end
    
    if 0
        FullOrderControllers(weighting_case_idx).SF.(f{1})(c_ws_idx)
        weighting_case_idx = 6; % 0db at high freq for So for all ctlr types, good
        weighting_case_idx = 1; % 0db at high freq for So for all ctlr types, good
        figure(1);
        f = 'y_mse'; % 0db at high freq for So, good
        f = 'rob'; % % 0db at high freq for So, good
        f = 'adc'; % % 0db at high freq for So, good
        bode(FullOrderControllers(weighting_case_idx).CL.(f)(3:4, 1:2, 3), bode_plot_opt);
        inner_CL = inv(FullOrderControllers(weighting_case_idx).Wout.(f)(1:4, 1:4, 3)) * FullOrderControllers(weighting_case_idx).CL.(f)(:, :, 3) * inv(FullOrderControllers(weighting_case_idx).Win.(f)(1:4, 1:4, 3));
        inner_CL.InputName = cellfun(@(l) ['Weighted ', l], FullOrderControllers(weighting_case_idx).CL.(f)(:, :, 3).InputName, 'UniformOutput', false);
        inner_CL.OutputName = cellfun(@(l) strrep(l, 'Weighted', 'Unweighted'), FullOrderControllers(weighting_case_idx).CL.(f)(:, :, 3).OutputName, 'UniformOutput', false);
        figure(2);
        bode(inner_CL(3:4, 1:2), bode_plot_opt);
        
        G = Plant(:, :, 3); C = FullOrderControllers(weighting_case_idx).Controller_scaled.(f)(:, :, 3);
        C.u = 'e'; C.y = 'u'; G.u = 'u'; G.y = 'y';
        Sum = sumblk('e = r - y', 2);
        To = connect(G, C, Sum, 'r', 'y');
        So = connect(G, C, Sum, 'r', 'e');
        figure(3);
        bode(To, So, bode_plot_opt); legend('To', 'So');
    end

    n_weighting_cases = (FullOrderControllers_n_cases / length(full_controller_case_basis.WindSpeedIndex.x));
    
    for w_idx = 1:n_weighting_cases
        for f = fieldnames(case_basis.W1Gain)'
            FullOrderControllers(w_idx).Controller.(f{1}) = ss(FullOrderControllers(w_idx).Controller.(f{1}));
            FullOrderControllers(w_idx).Controller.(f{1}).InputName = {'Measured M_d Tracking Error', 'Measured M_q Tracking Error'};
            FullOrderControllers(w_idx).Controller.(f{1}).OutputName = {'\beta_d Control Input', '\beta_q Control Input'};
            FullOrderControllers(w_idx).Controller.(f{1}).SamplingGrid = struct('u', LPV_CONTROLLER_WIND_SPEEDS);
            FullOrderControllers(w_idx).Controller_scaled.(f{1}) = ss(FullOrderControllers(w_idx).Controller_scaled.(f{1}));
            FullOrderControllers(w_idx).Controller_scaled.(f{1}).InputName = {'Measured M_d Tracking Error', 'Measured M_q Tracking Error'};
            FullOrderControllers(w_idx).Controller_scaled.(f{1}).OutputName = {'\beta_d Control Input', '\beta_q Control Input'};
            FullOrderControllers(w_idx).Controller_scaled.(f{1}).SamplingGrid = struct('u', LPV_CONTROLLER_WIND_SPEEDS);
        end
    end

        % case_desc = {'No IPC'};
    case_desc = {};
    case_wind_speeds = [];
    % loop through weighting cases and print information
    for f = fieldnames(case_basis.W1Gain)'
        fileID = fopen([f{1}, '_weighting_cases.txt'],'w');
        for w_idx = 1:n_weighting_cases
            for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
                fprintf(fileID, '\nCase %d Wind Speed %f\n', w_idx, LPV_CONTROLLER_WIND_SPEEDS(c_ws_idx));
                
                ip_msg = [];
                % if (ss(FullOrderControllers(w_idx).W1Gain).D(1, 1) == IP_HIGH_GAIN)
                    ip_msg = [ip_msg, ' W1 = ', num2str(ss(FullOrderControllers(w_idx).W1Gain.(f{1})).D(1, 1))];
                % end
            
                if length(ss(FullOrderControllers(w_idx).W1Gain.(f{1})).C)
                    ip_msg = [ip_msg, ' with notch '];
                end
                
                % if (ss(FullOrderControllers(w_idx).W2Gain).D(1, 1) == IP_HIGH_GAIN)
                    ip_msg = [ip_msg, ' W2 = ', num2str(ss(FullOrderControllers(w_idx).W2Gain.(f{1})).D(1, 1))];
                % end
                ip_msg = [ip_msg, ' -> '];
                fprintf(fileID, ip_msg);
                
                op_msg = [];
                % if (ss(FullOrderControllers(w_idx).WuGain).D(1, 1) == OP_HIGH_GAIN)
                    op_msg = [op_msg, ' Wu = ', num2str(ss(FullOrderControllers(w_idx).WuGain.x).D(1, 1))];
                % end
            
                % if (ss(FullOrderControllers(w_idx).WeGain).D(1, 1) == OP_HIGH_GAIN)
                    op_msg = [op_msg, ' We = ', num2str(ss(FullOrderControllers(w_idx).WeGain.(f{1})).D(1, 1))];
                % end
                if length(ss(FullOrderControllers(w_idx).WeGain.(f{1})).C)
                    op_msg = [op_msg, ' with notch '];
                end
                
                op_msg = [op_msg];
                case_desc = [case_desc, [f{1} ip_msg op_msg]];
                case_wind_speeds = [case_wind_speeds, LPV_CONTROLLER_WIND_SPEEDS(c_ws_idx)];

                fprintf(fileID, op_msg);
            end
        end
        fclose(fileID);
    end
    
    % go through each controller case and find the corresponding controller in
    % FullOrderControllers
    for c_idx = 1:Controllers_n_cases
        if ~strcmp(Controllers_case_list(c_idx).Structure.x, 'Full-Order')
            continue;
        end
        
        for w_idx = 1:n_weighting_cases
            for f = fieldnames(case_basis.W1Gain)'
                if (sum(ss(FullOrderControllers(w_idx).WuGain.x - Controllers_case_list(c_idx).WuGain.x).D, 'all') || ...
                    sum(ss(FullOrderControllers(w_idx).WeGain.(f{1}) - Controllers_case_list(c_idx).WeGain.(f{1})).D, 'all') || ...
                    sum(ss(FullOrderControllers(w_idx).W1Gain.(f{1}) - Controllers_case_list(c_idx).W1Gain.(f{1})).D, 'all') || ...
                    sum(ss(FullOrderControllers(w_idx).W2Gain.(f{1}) - Controllers_case_list(c_idx).W2Gain.(f{1})).D, 'all'))
                    continue;
                end
                
                K_tmp = FullOrderControllers(w_idx).Controller.(f{1});
                K_tmp_scaled = FullOrderControllers(w_idx).Controller_scaled.(f{1});
        
                % Add gain-scheduled controller to Controller_list
                if Controllers_case_list(c_idx).Scheduling.x
                    Controllers_case_list(c_idx).Controller.(f{1}) = K_tmp;
                    Controllers_case_list(c_idx).Controller_scaled.(f{1}) = K_tmp_scaled;
                % Add non gain-scheduled controller to Controller_list by repeating
                % same system for every parameter value
                elseif ~Controllers_case_list(c_idx).Scheduling.x
                    for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
                        Controllers_case_list(c_idx).Controller.(f{1})(:, :, c_ws_idx) ...
                            = K_tmp(:, :, full_controller_case_basis.WindSpeedIndex.x == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED));
                        Controllers_case_list(c_idx).Controller_scaled.(f{1})(:, :, c_ws_idx) ...
                            = K_tmp_scaled(:, :, full_controller_case_basis.WindSpeedIndex.x == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED));
                    end     
                end
                
                Controllers_case_list(c_idx).GenPlant.(f{1}) = FullOrderControllers(w_idx).GenPlant.(f{1});
                Controllers_case_list(c_idx).Wout.(f{1}) = FullOrderControllers(w_idx).Wout.(f{1});
                Controllers_case_list(c_idx).Win.(f{1}) = FullOrderControllers(w_idx).Win.(f{1});
                Controllers_case_list(c_idx).gamma.(f{1}) = FullOrderControllers(w_idx).gamma.(f{1});
                
                Controllers_case_list(c_idx).Mrgi.(f{1}) = FullOrderControllers(w_idx).Mrgi.(f{1});
                Controllers_case_list(c_idx).Mrgo.(f{1}) = FullOrderControllers(w_idx).Mrgo.(f{1});
                Controllers_case_list(c_idx).DMi.(f{1}) = FullOrderControllers(w_idx).DMi.(f{1});
                Controllers_case_list(c_idx).DMo.(f{1}) = FullOrderControllers(w_idx).DMo.(f{1});
                Controllers_case_list(c_idx).MMo.(f{1}) = FullOrderControllers(w_idx).MMo.(f{1});
                Controllers_case_list(c_idx).MMi.(f{1}) = FullOrderControllers(w_idx).MMi.(f{1});
                Controllers_case_list(c_idx).MMio.(f{1}) = FullOrderControllers(w_idx).MMio.(f{1});
    
                Controllers_case_list(c_idx).wc.(f{1}) = FullOrderControllers(w_idx).wc.(f{1});

                Controllers_case_list(c_idx).Stable.(f{1}) = FullOrderControllers(w_idx).Stable.(f{1});
                
            end   
        end
        Controllers_case_list(c_idx).Controller_scaled.(f{1}).SamplingGrid = struct('u', LPV_CONTROLLER_WIND_SPEEDS);
        
    end
    
    i = 1;
    for f = fieldnames(case_basis.W1Gain)'
        for w_idx = 1:n_weighting_cases
            for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
                Controllers_case_list(w_idx).CaseDesc.(f{1}) = case_desc(i);
                i = i + 1;
            end
        end
    end
    
    n_weighting_cases = (FullOrderControllers_n_cases / length(full_controller_case_basis.WindSpeedIndex.x));
    fs = fieldnames(case_basis.W1Gain)';
    i = 1;
    for f_idx = 1:length(fs)
        for w_idx = 1:n_weighting_cases
            for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)

                f = fs(f_idx);
                % i = (f_idx - 1) + w_idx;
    
                Stable_tmp(i) = Controllers_case_list(w_idx).Stable.(f{1})(c_ws_idx);
                Mrgi_tmp(i, 1, 1) = Controllers_case_list(w_idx).Mrgi.(f{1})(1, c_ws_idx).GainMargin(1);
                pm = Controllers_case_list(w_idx).Mrgi.(f{1})(1, c_ws_idx).PhaseMargin;
                if length(pm)
                    Mrgi_tmp(i, 1, 2) = pm(1);
                else
                    Mrgi_tmp(i, 1, 2) = 0;
                end
            
                Mrgi_tmp(i, 2, 1) = Controllers_case_list(w_idx).Mrgi.(f{1})(2, c_ws_idx).GainMargin(1);
                pm = Controllers_case_list(w_idx).Mrgi.(f{1})(2, c_ws_idx).PhaseMargin;
                if length(pm)
                    Mrgi_tmp(i, 2, 2) = pm(1);
                else
                    Mrgi_tmp(i, 2, 2) = 0;
                end
        
                Mrgo_tmp(i, 1, 1) = Controllers_case_list(w_idx).Mrgo.(f{1})(1).GainMargin(1);
                % SF_tmp = loopsens(-Plant(:, :, c_ws_idx), -¸.Controller_scaled.(f{1}));
                % bode(SF_tmp.To)
                pm = Controllers_case_list(w_idx).Mrgo.(f{1})(1, c_ws_idx).PhaseMargin;
                if length(pm)
                    Mrgo_tmp(i, 1, 2) = pm(1);
                else
                    Mrgo_tmp(i, 1, 2) = 0;
                end
        
                Mrgo_tmp(i, 2, 1) = Controllers_case_list(w_idx).Mrgo.(f{1})(2).GainMargin(1);
                pm = Controllers_case_list(w_idx).Mrgo.(f{1})(2, c_ws_idx).PhaseMargin;
                if length(pm)
                    Mrgo_tmp(i, 2, 2) = pm(1);
                else
                    Mrgo_tmp(i, 2, 2) = 0;
                end
                
                DMi_tmp(i, 1, 1) = Controllers_case_list(w_idx).DMi.(f{1})(1, c_ws_idx).GainMargin(2);
                DMi_tmp(i, 1, 2) = Controllers_case_list(w_idx).DMi.(f{1})(1, c_ws_idx).PhaseMargin(2);
                DMi_tmp(i, 1, 3) = Controllers_case_list(w_idx).DMi.(f{1})(1, c_ws_idx).DiskMargin;
        
                DMi_tmp(i, 2, 1) = Controllers_case_list(w_idx).DMi.(f{1})(2, c_ws_idx).GainMargin(2);
                DMi_tmp(i, 2, 2) = Controllers_case_list(w_idx).DMi.(f{1})(2, c_ws_idx).PhaseMargin(2);
                DMi_tmp(i, 2, 3) = Controllers_case_list(w_idx).DMi.(f{1})(2, c_ws_idx).DiskMargin;
                
                DMo_tmp(i, 1, 1) = Controllers_case_list(w_idx).DMo.(f{1})(1, c_ws_idx).GainMargin(2);
                DMo_tmp(i, 1, 2) = Controllers_case_list(w_idx).DMo.(f{1})(1, c_ws_idx).PhaseMargin(2);
                DMo_tmp(i, 1, 3) = Controllers_case_list(w_idx).DMo.(f{1})(1, c_ws_idx).DiskMargin;
                
                DMo_tmp(i, 2, 1) = Controllers_case_list(w_idx).DMo.(f{1})(2, c_ws_idx).GainMargin(2);
                DMo_tmp(i, 2, 2) = Controllers_case_list(w_idx).DMo.(f{1})(2, c_ws_idx).PhaseMargin(2);
                DMo_tmp(i, 2, 3) = Controllers_case_list(w_idx).DMo.(f{1})(2, c_ws_idx).DiskMargin;
                
                MMi_tmp(i, 1) = Controllers_case_list(w_idx).MMi.(f{1})(c_ws_idx).GainMargin(2);
                MMi_tmp(i, 2) = Controllers_case_list(w_idx).MMi.(f{1})(c_ws_idx).PhaseMargin(2);
                MMi_tmp(i, 3) = Controllers_case_list(w_idx).MMi.(f{1})(c_ws_idx).DiskMargin;
                
                MMo_tmp(i, 1) = Controllers_case_list(w_idx).MMo.(f{1})(c_ws_idx).GainMargin(2);
                MMo_tmp(i, 2) = Controllers_case_list(w_idx).MMo.(f{1})(c_ws_idx).PhaseMargin(2);
                MMo_tmp(i, 3) = Controllers_case_list(w_idx).MMo.(f{1})(c_ws_idx).DiskMargin;
                
                MMio_tmp(i, 1) = Controllers_case_list(w_idx).MMio.(f{1})(c_ws_idx).GainMargin(2);
                MMio_tmp(i, 2) = Controllers_case_list(w_idx).MMio.(f{1})(c_ws_idx).PhaseMargin(2);
                MMio_tmp(i, 3) = Controllers_case_list(w_idx).MMio.(f{1})(c_ws_idx).DiskMargin;
                
                wc_tmp(i) = Controllers_case_list(w_idx).wc.(f{1})(c_ws_idx);
                
                i = i + 1;
            end
        end
    end
    
    n_total_cases = length(wc_tmp);
    % Make table comparing controllers
    Controllers_case_table = table( ...
        (1:n_total_cases)', case_desc', ... 
        case_wind_speeds', Stable_tmp', ...
        mag2db(Mrgi_tmp(:, 1, 1)), Mrgi_tmp(:, 1, 2), ...
        mag2db(Mrgi_tmp(:, 2, 1)), Mrgi_tmp(:, 2, 2), ...
        mag2db(Mrgo_tmp(:, 1, 1)), Mrgo_tmp(:, 1, 2), ...
        mag2db(Mrgo_tmp(:, 2, 1)), Mrgo_tmp(:, 2, 2), ...
        mag2db(DMi_tmp(:, 1, 1)), DMi_tmp(:, 1, 2), DMi_tmp(:, 1, 3), ...
        mag2db(DMi_tmp(:, 2, 1)), DMi_tmp(:, 2, 2), DMi_tmp(:, 2, 3), ...
        mag2db(DMo_tmp(:, 1, 1)), DMo_tmp(:, 1, 2), DMo_tmp(:, 1, 3), ...
        mag2db(DMo_tmp(:, 2, 1)), DMo_tmp(:, 2, 2), DMo_tmp(:, 2, 3), ...
        mag2db(MMi_tmp(:, 1)), MMi_tmp(:, 2), MMi_tmp(:, 3), ...
        mag2db(MMo_tmp(:, 1)), MMo_tmp(:, 2), MMo_tmp(:, 3), ...
        mag2db(MMio_tmp(:, 1)), MMio_tmp(:, 2), MMio_tmp(:, 3), ...
        wc_tmp', ...
        zeros(n_total_cases, 1), zeros(n_total_cases, 1), ...
        'VariableNames', ...
        {'Case No.', 'Case Desc.', 'TunedWindSpeed', 'Stable', ...
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

    for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
        ws_cond = Controllers_case_table.("TunedWindSpeed") == LPV_CONTROLLER_WIND_SPEEDS(c_ws_idx);
        
        Controllers_case_table(ws_cond, "WorstCase_SingleClassical_GM") = ...
            min(Controllers_case_table(ws_cond, ...
            ["ClassicalInput_GMD", "ClassicalInput_GMQ", ...
            "ClassicalOutput_GMD", "ClassicalOutput_GMQ"]), [], 2);
        
        Controllers_case_table(ws_cond, "WorstCase_SingleClassical_PM") = ...
            min(Controllers_case_table(ws_cond, ...
            ["ClassicalInput_PMD", "ClassicalInput_PMQ", ...
            "ClassicalOutput_PMD", "ClassicalOutput_PMQ"]), [], 2);
        
        % get indices of cases with lowest disk margins, then get
        % corresponding gain and phase margins
        [M, I] = min(Controllers_case_table(ws_cond, ...
            ["SingleDiskInput_DMD", "SingleDiskInput_DMQ", ...
            "SingleDiskOutput_DMD", "SingleDiskOutput_DMQ"]), [], 2);

        x = Controllers_case_table(ws_cond, ...
            ["SingleDiskInput_GMD", "SingleDiskInput_GMQ", ...
            "SingleDiskOutput_GMD", "SingleDiskOutput_GMQ"]);
        x = table2array(x);
        i = I.Variables;
        Controllers_case_table(ws_cond, "WorstCase_SingleDisk_GM") = table(x(i), 'VariableNames', {'WorstCase_SingleDisk_GM'});

        x = Controllers_case_table(ws_cond, ...
            ["SingleDiskInput_PMD", "SingleDiskInput_PMQ", ...
            "SingleDiskOutput_PMD", "SingleDiskOutput_PMQ"]);
        x = table2array(x);
        i = I.Variables;
        Controllers_case_table(ws_cond, "WorstCase_SingleDisk_PM") = table(x(i), 'VariableNames', {'WorstCase_SingleDisk_PM'});

    end
    % Controllers_case_table.("Mean DMo") = mean(Controllers_case_table(:, ["DMoD", "DMoQ"]), 2).Variables;
    % Controllers_case_table.("Mean DM") = mean(Controllers_case_table(:, ["Mean DMi", "Mean DMo"]), 2).Variables;

    % sortrows(Controllers_case_table, 'MultiDiskIO_DM', 'descend')
    % sortrows(Controllers_case_table, 'wc', 'descend')

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
if PLOTTING

    cc = find(full_controller_case_basis.WindSpeedIndex.x == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED));
    n_weighting_cases = (FullOrderControllers_n_cases / length(full_controller_case_basis.WindSpeedIndex.x));

    %% TODO Plot transfer functions of KSo, Ti; So, GSi vs f for single wind speed on
    % 2*2 bodeplot OR singular values for open-loop vs closed-loop OUTPLOT
    % QUESTION MANUEL do these plots make sense, bound on GSi seems
    % ineffective, should I choose a single singular value to plot, which
    % one? Messy to plot gamma bounds for all weighting cases, do you think
    % its necessary?
    % for each controller type: robustness, adc, y_mse
    % controller over wind speedfs = fieldnames(case_basis.W1Gain)';
    for f = fieldnames(case_basis.W1Gain)'
        figure;
        ax_KSo = subplot(2, 2, 1);
        ax_Ti = subplot(2, 2, 2);
        ax_So = subplot(2, 2, 3);
        ax_GSi = subplot(2, 2, 4);
        ax_KSo_del_idx = []; ax_Ti_del_idx = []; 
        ax_So_del_idx = []; ax_GSi_del_idx = [];
        Wu_gains = [];
        for w_idx = 1:n_weighting_cases
            % bodemag(Wu_tmp, We_tmp, W1_tmp, W2_tmp); legend('Wu', 'We', 'W1', 'W2');
           
            Wu_gains = [Wu_gains,  ss(Controllers_case_list(w_idx).WuGain.x).D(1, 1)];
            Wu_tmp = Controllers_case_list(w_idx).WuGain.x * Wu;
            We_tmp = Controllers_case_list(w_idx).WeGain.(f{1}) * We;
            W1_tmp = Controllers_case_list(w_idx).W1Gain.(f{1}) * W1;
            W2_tmp = Controllers_case_list(w_idx).W2Gain.(f{1}) * W2;
    
            SF = loopsens(-Plant(:, :, cc), ...
                -Controllers_case_list(w_idx).Controller_scaled.(f{1})(:, :, cc));
    
            % sys_KSi = SF.Si * Controllers_case_list(w_idx).Controller_scaled.(f{1})(:, :, cc) * W1_tmp;
            sys_KSo = SF.CSo * W1_tmp;
            sys_Ti = SF.Ti * W2_tmp;
            % sys_Ti2 = (Controllers_case_list(w_idx).Controller_scaled(:, :, cc) * Plant(:, :, cc)) / ...
            %           (eye(size(Plant(:, :, cc))) + Controllers_case_list(w_idx).Controller_scaled(:, :, cc) * Plant(:, :, cc)) * W2_tmp;
            
            sys_So = SF.So * W1_tmp;
            % sys_So_cl2 = inv(eye(size(Plant(:, :, cc))) + Plant(:, :, cc) * Controllers_case_list(w_idx).Controller_scaled(:, :, cc)) * W1_tmp;
            % sys_GSi = Plant(:, :, cc) * SF.So * W2_tmp;
            sys_GSi = SF.PSi * W2_tmp;
            
            % [s, w] = sigma(sys_KSi);
            sigmaplot(ax_KSo, sys_KSo, sigma_plot_opt); hold on;
            a = findobj(ax_KSo, 'Type', 'line');
            ax_KSo_del_idx = [ax_KSo_del_idx, length(ax_KSo_del_idx) + 2:2];
            % legend(ax_KSi, ['Wu = ', num2str(Wu_gains(end))], 'Bound', 'Location', 'westoutside');
            subtitle(ax_KSo, '$KS_o$', 'Interpreter','latex');
            
            % [s, w] = sigma(sys_Ti);
            sigmaplot(ax_Ti, sys_Ti, sigma_plot_opt); hold on;
            a = findobj(ax_Ti, 'Type', 'line');
            ax_Ti_del_idx = [ax_Ti_del_idx, length(ax_Ti_del_idx) + 2:2];
            
            % legend(['Wu = ', num2str(Wu_gains(end))], 'Bound');
            subtitle(ax_Ti, '$T_i$', 'Interpreter','latex');
            
            sigmaplot(ax_So, sys_So, sigma_plot_opt); hold on;
            a = findobj(ax_So, 'Type', 'line');
            ax_So_del_idx = [ax_So_del_idx, length(ax_So_del_idx) + 2:2];
            
            % legend(['Wu = ', num2str(Wu_gains(end))], 'Bound');
            subtitle(ax_So, '$S_o$', 'Interpreter','latex');

            sigmaplot(ax_GSi, sys_GSi, sigma_plot_opt); hold on;
            a = findobj(ax_GSi, 'Type', 'line');
            ax_GSi_del_idx = [ax_GSi_del_idx, length(ax_GSi_del_idx) + 2:2];
            
            % legend(['Wu = ', num2str(Wu_gains(end))], 'Bound');
            subtitle(ax_GSi, '$GS_i$', 'Interpreter','latex');

            % sigmaplot(ax_KSo, inv(Wu_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc), '--', sigma_plot_opt);
            % sigmaplot(ax_Ti, inv(Wu_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc), '--', sigma_plot_opt);
            % sigmaplot(ax_So, inv(We_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc), '--', sigma_plot_opt);
            % sigmaplot(ax_GSi, inv(We_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc), '--', sigma_plot_opt);
        end
        % sgtitle([f{1}, ' Controller'], 'Interpreter', 'latex');
        % TODO no legend save seperately
        % legend(ax_KSo, [arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
        %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex');
        % legend(ax_Ti, [arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ... % 'Bound'], ...
        %     'NumColumns', 1, 'Location', 'eastoutside', 'Interpreter', 'latex');
        % legend(ax_So, [arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ... %
        %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex');
        % legend(ax_GSi, [arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ... %
        %     'NumColumns', 1, 'Location', 'eastoutside', 'Interpreter', 'latex');

        axesHandlesToChildObjects = findobj(ax_KSo, 'Type', 'line');
        delete(axesHandlesToChildObjects(ax_KSo_del_idx)); % remove first singluar value
        axesHandlesToChildObjects = findobj(ax_Ti, 'Type', 'line');
        delete(axesHandlesToChildObjects(ax_Ti_del_idx)); % remove first singluar value
        axesHandlesToChildObjects = findobj(ax_So, 'Type', 'line');
        delete(axesHandlesToChildObjects(ax_So_del_idx)); % remove first singluar value
        axesHandlesToChildObjects = findobj(ax_GSi, 'Type', 'line');
        delete(axesHandlesToChildObjects(ax_GSo_del_idx)); % remove first singluar value
    
        set(gcf, 'Position', [0 0 1500 900]);
        savefig(gcf, fullfile(fig_dir, [f{1}, '_fullorder_singular_vals.fig']));
        saveas(gcf, fullfile(fig_dir, [f{1}, '_fullorder_singular_vals.png']));
    
    end
    
    %% Plot classical, disk margins for controller tuned for different wind speeds OUTPLOT
    
    for f = fieldnames(case_basis.W1Gain)'
        ctrl_cond = contains(Controllers_case_table.("Case Desc."), f{1});

        figure;
        % top row: gain margins vs wind speed for worst case single channel
        % classical margin, worst case single channel disk margin, multi
        % input/output disk margin
        % bottom row: phase margins vs wind speed for worst case single channel
        % classical margin, worst case single channel disk margin, multi
        % input/output disk margin
        ax_gm_1 = subplot(2, 3, 1);
        ax_gm_2 = subplot(2, 3, 2);
        ax_gm_3 = subplot(2, 3, 3);
        ax_pm_1 = subplot(2, 3, 4);
        ax_pm_2 = subplot(2, 3, 5);
        ax_pm_3 = subplot(2, 3, 6);
        Wu_gains = [];
        for w_idx = 1:n_weighting_cases
            wu = ss(Controllers_case_list(w_idx).WuGain.x).D(1, 1);
            Wu_gains = [Wu_gains,  wu];

            data_gm_1 = []; data_gm_2 = []; data_gm_3 = [];
            data_pm_1 = []; data_pm_2 = []; data_pm_3 = [];
            for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
                ws_cond = Controllers_case_table.("TunedWindSpeed") == LPV_CONTROLLER_WIND_SPEEDS(c_ws_idx);
                x = Controllers_case_table(ctrl_cond & ws_cond, "WorstCase_SingleClassical_GM").Variables;
                data_gm_1 = [data_gm_1 x(w_idx)];
                x = Controllers_case_table(ctrl_cond & ws_cond, "WorstCase_SingleDisk_GM").Variables;
                data_gm_2 = [data_gm_2 x(w_idx)];
                x = Controllers_case_table(ctrl_cond & ws_cond, "MultiDiskIO_GM").Variables;
                data_gm_3 = [data_gm_3 x(w_idx)];

                x = Controllers_case_table(ctrl_cond & ws_cond, "WorstCase_SingleClassical_PM").Variables;
                data_pm_1 = [data_pm_1 x(w_idx)];
                x = Controllers_case_table(ctrl_cond & ws_cond, "WorstCase_SingleDisk_PM").Variables;
                data_pm_2 = [data_pm_2 x(w_idx)];
                x = Controllers_case_table(ctrl_cond & ws_cond, "MultiDiskIO_PM").Variables;
                data_pm_3 = [data_pm_3 x(w_idx)];
            end
            subplot(2, 3, 1); plot(LPV_CONTROLLER_WIND_SPEEDS, data_gm_1); hold on;
            subplot(2, 3, 2); plot(LPV_CONTROLLER_WIND_SPEEDS, data_gm_2); hold on;
            subplot(2, 3, 3); plot(LPV_CONTROLLER_WIND_SPEEDS, data_gm_3); hold on;
            subplot(2, 3, 4); plot(LPV_CONTROLLER_WIND_SPEEDS, data_pm_1); hold on;
            subplot(2, 3, 5); plot(LPV_CONTROLLER_WIND_SPEEDS, data_pm_2); hold on;
            subplot(2, 3, 6); plot(LPV_CONTROLLER_WIND_SPEEDS, data_pm_3); hold on;

        end
        % TODO no legend save seperately
        subplot(2, 3, 1); 
        % legend([arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
        %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex'); 
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlabel('Wind Speed [m/s]');
        subtitle("Worst Case Classical Gain Margin [dB]");

        subplot(2, 3, 2); 
        % legend([arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
        %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex'); 
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlabel('Wind Speed [m/s]');
        subtitle("Worst Case Loop-at-a-Time Disk Gain Margin [dB]");

        subplot(2, 3, 3); 
        % legend([arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
        %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex'); 
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlabel('Wind Speed [m/s]');
        subtitle("Multiloop Disk Gain Margin [dB]");

        subplot(2, 3, 4); 
        % legend([arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
        %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex'); 
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlabel('Wind Speed [m/s]');
        subtitle("Worst Case Classical Phase Margin [deg]");

        subplot(2, 3, 5); 
        % legend([arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
        %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex'); 
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlabel('Wind Speed [m/s]');
        subtitle("Worst Case Loop-at-a-Time Disk Phase Margin [deg]");

        subplot(2, 3, 6); 
        % legend([arrayfun(@(wu) ['$W_u = ', num2str(wu), '$'], Wu_gains, 'UniformOutput', false)], ...
        %     'NumColumns', 1, 'Location', 'westoutside', 'Interpreter', 'latex'); 
        xticks(LPV_CONTROLLER_WIND_SPEEDS);
        xlabel('Wind Speed [m/s]');
        subtitle("Multiloop Disk Phase Margin [deg]");
        % 
        % ax1 = subplot(2, 3, 1); ax2 = subplot(2, 3, 2); ax3 = subplot(2, 3, 3);
        % linkaxes([ax1, ax2, ax3]);
        % ax1 = subplot(2, 3, 4); ax2 = subplot(2, 3, 5); ax3 = subplot(2, 3, 6);
        % linkaxes([ax1, ax2, ax3]);

        % sgtitle([f{1}, ' Controller'], 'Interpreter', 'latex'); 
        set(gcf, 'Position', [0 0 1500 900]);
        savefig(gcf, fullfile(fig_dir, [f{1}, '_fullorder_rob_margins.fig']));
        saveas(gcf, fullfile(fig_dir, [f{1}, '_fullorder_rob_margins.png']));
    end

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
        % So (bottom left corner, d1->ze) and GSi (bottom right corner,
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
        % sys_KSi = SF.Si * Controllers_case_list(w_idx).Controller_scaled(:, :, cc) * W1_tmp;
        sys_KSo = SF.CSo * W1_tmp;
        sys_Ti = SF.Ti * W2_tmp;
        sys_So = SF.So * W1_tmp;
        % sys_So_ol = Plant(:, :, cc) * W1_tmp;
        % sys_GSi = SF.So * Plant(:, :, cc) * W2_tmp;
        sys_GSi = Sf.PSi * W2_tmp;

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
    
        % Plot GSi with two weighting functions
        % GSi = -e/W2d2, input disturbance -> tracking error
        subplot(2,2,4);
        sys_GSi.InputName = GenPlant.InputName(3:4); % control disturbance
        sys_GSi.OutputName = We_tmp.InputName; % tracking error
        bodeplot(...
            sys_GSi, ...
            inv(We_tmp) * Controllers_case_list(w_idx).gamma(cc), w, bode_plot_opt);
        title('GS_i Function');
        axh = findall(gcf, 'type', 'axes');
        for a = 1:length(axh)
            xline(axh(a), [loopData.peaks.wGAM]);
        end
        set(gcf, 'Position', [0 0 1500 900]);
        legend('GS_i W_2', 'W_e^{-1}\gamma', 'W_y^{-1}\gamma');
        % savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'GSi_full_bodemag.fig']));
        % saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'GSi_full_bodemag.png']));

        % Plot KS_o with two weighting functions
        % KSo = u/W1d1, output disturbance -> control input
        subplot(2,2,1);
        sys_KSo.InputName = GenPlant.InputName(1:2); % output disturbance/reference
        sys_KSo.OutputName = Wu_tmp.InputName; % control input
        bodeplot(...
            sys_KSo, ...
            inv(Wu_tmp) * Controllers_case_list(w_idx).gamma(cc), w, ...
             bode_plot_opt);
        title('KS_o Function');
        axh = findall(gcf, 'type', 'axes');
        for a = 1:length(axh)
            xline(axh(a), [loopData.peaks.wGAM]);
        end
        set(gcf, 'Position', [0 0 1500 900]);
        legend('KS_o W_1', 'W_u^{-1}\gamma');
        
        savefig(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_tfs_bodemag.fig']));
        saveas(gcf, fullfile(fig_dir, ['case', num2str(w_idx), '_', 'fullorder_tfs_bodemag.png']));
    end
    
    %% Plot tuned controller for this weighting case OUTPLOT
    cc = find(full_controller_case_basis.WindSpeedIndex.x == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED));
    % first case = black = highest Wu_gain
    for f = fieldnames(case_basis.W1Gain)'
        ctrl_cond = contains(Controllers_case_table.("Case Desc."), f{1});

        figure;
        Wu_gains = [];

        clear controllers;
        % 
        for w_idx = 1:n_weighting_cases

            wu = ss(Controllers_case_list(w_idx).WuGain.x).D(1, 1);
            Wu_gains = [Wu_gains,  wu];
            
            % bcol = copper(n_weighting_cases); % Define the color order based on the number of models
            % rcol = jet(length(LPV_CONTROLLER_WIND_SPEEDS));
            % ycol = hot(length(LPV_CONTROLLER_WIND_SPEEDS)); % https://www.mathworks.com/help/matlab/colors-1.html?s_tid=CRUX_lftnav
        
            omega = logspace(-2, 4, 300);
        
            % for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
                
                K_tmp = Controllers_case_list(w_idx).Controller.(f{1})(:, :, cc);
                K_tmp.InputName = {'$M_d$ Tracking Error', '$M_q$ Tracking Error'};
                K_tmp.OutputName = {'$\beta_d$ Control Input', '$\beta_q$ Control Input'};
                controllers(:, :, w_idx) = K_tmp;
                % K_tmp_scaled.InputName = {'$M_d$ Tracking Error', '$M_q$ Tracking Error'};
                % K_tmp_scaled.OutputName = {'$\beta_d$ Control Input', '$\beta_q$ Control Input'};
    
                
        
                % Find handles of all lines in the figure that have the color blue
                % blineHandle = findobj(gcf, 'Type', 'line', '-and', 'Color', 'b');
                % rlineHandle = findobj(gcf,'Type','line','-and','Color','r');
                % ylineHandle = findobj(gcf,'Type','line','-and','Color','y');
        
                % Change the color to the one you defined
                % set(blineHandle,'Color', bcol(w_idx, :));
                % set(rlineHandle,'Color',rcol(c_ws_idx,:));
                % set(ylineHandle,'Color',ycol(c_ws_idx,:));
                
                % hold on
            % end
        
        %      legend([arrayfun(@(n) ['Case ', num2str(n)], ... 
        % 1:n_weighting_cases, 'UniformOutput', false) {'', '', '', ''}], ...
        % 'NumColumns', 2);
            
            % legend('Structured Baseline', 'Structured Tuned', 'Full-Order Tuned', '', '', '', '');
            %         title('Frequency Response of Tuned Controllers');
            % hold off;
    
        end

        bcol = copper(n_weighting_cases); % Define the color order based on the number of models

        % Plot baseline and tuned controllers
        bodeplot(controllers, 'b', omega, bode_plot_opt);

        % Find handles of all lines in the figure that have the color blue
        axh = findall(gcf, 'type', 'axes');
        
        for ax_idx = [3, 5, 7, 9]
            xline(axh(ax_idx), omega_1P_rad * HARMONICS);
            blineHandle = findobj(axh(ax_idx), 'Type', 'line', '-and', 'Color', 'b');
            for w_idx = 1:n_weighting_cases
                % Change the color to the one you defined
                set(blineHandle(w_idx), 'Color', bcol(w_idx, :));
            end
        end

        set(gcf, 'Position', [0 0 1500 900]);
        savefig(gcf, fullfile(fig_dir, [f{1} '_fullorder_controller_bodemag.fig']));
        saveas(gcf, fullfile(fig_dir, [f{1} '_fullorder_controller_bodemag.png']));
    end

end