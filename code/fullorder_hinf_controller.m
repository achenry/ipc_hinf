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
nmeas = 2; % number of measurement outputs y, K has nmeas inputs
ncont = 2; % number of control inputs u, K has ncont outputs

% consider case of only Wy to penalize To, 
% then only Wy and We to penalize To and So, 
% then Wy and We and Wu to penalize To and So and KSo and Ti

[FullOrderControllers_case_list, FullOrderControllers_case_name_list, FullOrderControllers_n_cases] ...
    = generateCases(case_basis, 'full_order_tuned_controllers', true);

save(fullfile(mat_save_dir, [sim_type, '_full_controller_cases']), 'case_basis');

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
    if 1
    FullOrderTuning = repmat(struct(), FullOrderControllers_n_cases, 1 );
    for f = fieldnames(case_basis.W1Gain)'
    parfor case_idx = 1:FullOrderControllers_n_cases
        controller_case = FullOrderControllers_case_list(case_idx);
        c_ws_idx = controller_case.WindSpeedIndex.x;

        % FullOrderTuning(case_idx).Scheduling = controller_case.Scheduling;
     
        if VARY_REFERENCE
            FullOrderTuning(case_idx).Reference = VARY_REFERENCE_BASIS(controller_case.Reference.x);
        else
            FullOrderTuning(case_idx).Reference = 0;
        end
        if VARY_SATURATION
            FullOrderTuning(case_idx).Saturation = VARY_SATURATION_BASIS(controller_case.Saturation.x);
        else
            FullOrderTuning(case_idx).Saturation = 0;
        end
        % controller_case.WuGain.x.Numerator{1,1}(1)
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
            FullOrderTuning(case_idx).Controller.(f{1}).InputName = {'M_d Tracking Error', 'M_q Tracking Error'};
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

            x = getGainCrossover(SF_tmp.Lo, 1);
            if length(x) >= 1
                FullOrderTuning(case_idx).wc.(f{1}) = x(1); % TODO not right but there is no crossover for Lo, must repeat for extreme and optimal controllers
            else
                FullOrderTuning(case_idx).wc.(f{1}) = 0;
            end
            % bode(SF_tmp.Lo, SF_tmp.To, bode_plot_opt)
            % xline()
            FullOrderTuning(case_idx).SF.(f{1}) = SF_tmp;
        % end

    end
    end
        save(fullfile(mat_save_dir, [sim_type, '_FullOrderTuning.mat']), 'FullOrderTuning', '-v7.3');
    else
        load(fullfile(mat_save_dir, [sim_type, '_FullOrderTuning.mat']));
    end
    % restructure synthesized controller cases per weighting matrix
    % combination and wind speed
    % for each wind speed
    FullOrderControllers = struct;
    for c_ws_idx = 1:length(case_basis.WindSpeedIndex.x)
        weighting_case_idx = 1;
        % find all of the controller cases that correspond to this wind speed
        for case_idx = 1:FullOrderControllers_n_cases
            controller_case = FullOrderControllers_case_list(case_idx);
            if case_basis.WindSpeedIndex.x(c_ws_idx) == controller_case.WindSpeedIndex.x
                % FullOrderControllers(weighting_case_idx).WuGain = FullOrderControllers_case_list(case_idx).WuGain;
                for f = fieldnames(case_basis.W1Gain)'
                    % cc = find(case_basis.WindSpeedIndex == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED));
                    % FullOrderControllers(weighting_case_idx).Scheduling.(f{1}) = FullOrderTuning(case_idx).Scheduling.x;
                    FullOrderControllers(weighting_case_idx).Reference.(f{1}) = FullOrderTuning(case_idx).Reference;
                    FullOrderControllers(weighting_case_idx).Saturation.(f{1}) = FullOrderTuning(case_idx).Saturation;
                    
                    FullOrderControllers(weighting_case_idx).WeGain.(f{1}) = FullOrderTuning(case_idx).WeGain.(f{1});
                    FullOrderControllers(weighting_case_idx).WuGain.(f{1}) = FullOrderTuning(case_idx).WuGain.x;
                    FullOrderControllers(weighting_case_idx).W1Gain.(f{1}) = FullOrderTuning(case_idx).W1Gain.(f{1});
                    FullOrderControllers(weighting_case_idx).W2Gain.(f{1}) = FullOrderTuning(case_idx).W2Gain.(f{1});
    
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
                     
                    if 0
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
                        K_tmp.InputName = {'M_d Tracking Error', 'M_q Tracking Error'};
                        Plant.OutputName = {'M_d Output', 'M_q Output'};
                        x = feedback(series(K_tmp, Plant(:, :, c_ws_idx)), eye(2));
                        
                        figure; bodemag(x); bodemag(y);
                        bode(SF_tmp.Lo, SF_tmp.So, SF_tmp.To, bode_plot_opt)
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

    n_weighting_cases = (FullOrderControllers_n_cases / length(case_basis.WindSpeedIndex.x));

    for w_idx = 1:n_weighting_cases
        for f = fieldnames(case_basis.W1Gain)'
            K_tmp = FullOrderControllers(w_idx).Controller.(f{1});
            K_tmp_scaled = FullOrderControllers(w_idx).Controller_scaled.(f{1});
    
            % Add gain-scheduled controller to Controller_list
            % if FullOrderControllers(w_idx).Scheduling.(f{1})
            if SCHEDULING
                FullOrderControllers(w_idx).Controller.(f{1}) = K_tmp;
                FullOrderControllers(w_idx).Controller_scaled.(f{1}) = K_tmp_scaled;
            % Add non gain-scheduled controller to Controller_list by repeating
            % same system for every parameter value
            % elseif ~FullOrderControllers(w_idx).Scheduling.(f{1})
            else
                for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
                    FullOrderControllers(w_idx).Controller.(f{1})(:, :, c_ws_idx) ...
                        = K_tmp(:, :, case_basis.WindSpeedIndex.x == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED));
                    FullOrderControllers(w_idx).Controller_scaled.(f{1})(:, :, c_ws_idx) ...
                        = K_tmp_scaled(:, :, case_basis.WindSpeedIndex.x == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED));
                end     
            end

            FullOrderControllers(w_idx).Controller.(f{1}) = ss(FullOrderControllers(w_idx).Controller.(f{1}));
            FullOrderControllers(w_idx).Controller.(f{1}).InputName = {'M_d Tracking Error', 'M_q Tracking Error'};
            FullOrderControllers(w_idx).Controller.(f{1}).OutputName = {'\beta_d Control Input', '\beta_q Control Input'};
            % FullOrderControllers(w_idx).Controller.(f{1}).SamplingGrid = struct('u', LPV_CONTROLLER_WIND_SPEEDS);
            FullOrderControllers(w_idx).Controller_scaled.(f{1}) = ss(FullOrderControllers(w_idx).Controller_scaled.(f{1}));
            FullOrderControllers(w_idx).Controller_scaled.(f{1}).InputName = {'M_d Tracking Error', 'M_q Tracking Error'};
            FullOrderControllers(w_idx).Controller_scaled.(f{1}).OutputName = {'\beta_d Control Input', '\beta_q Control Input'};
            % FullOrderControllers(w_idx).Controller_scaled.(f{1}).SamplingGrid = struct('u', LPV_CONTROLLER_WIND_SPEEDS);
        end
    end

        % case_desc = {'No IPC'};
    case_desc = {};
    case_wind_speeds = [];
    % loop through weighting cases and print information
    for f = fieldnames(case_basis.W1Gain)'
        fileID = fopen([f{1}, '_weighting_cases.txt'],'w');
        for w_idx = 1:n_weighting_cases
            for c_ws_idx = 1:length(case_basis.WindSpeedIndex.x)
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
                    op_msg = [op_msg, ' Wu = ', num2str(ss(FullOrderControllers(w_idx).WuGain.(f{1})).D(1, 1))];
                % end
            
                % if (ss(FullOrderControllers(w_idx).WeGain).D(1, 1) == OP_HIGH_GAIN)
                    op_msg = [op_msg, ' We = ', num2str(ss(FullOrderControllers(w_idx).WeGain.(f{1})).D(1, 1))];
                % end
                if length(ss(FullOrderControllers(w_idx).WeGain.(f{1})).C)
                    op_msg = [op_msg, ' with notch '];
                end

                if VARY_REFERENCE
                    refs = FullOrderControllers(w_idx).Reference.(f{1});
                     % op_msg = [op_msg, ' M_dq_ref = [', num2str(refs * M_dq_reference(1)), ', ', num2str(refs * M_dq_reference(2)), ']'];
                     op_msg = [op_msg, ' M_dq_ref = [', num2str(refs(1)), ', ', num2str(refs), ']'];
                end

                if VARY_SATURATION
                    % TODO why is this zero
                    sats = FullOrderControllers(w_idx).Saturation.(f{1});
                    % op_msg = [op_msg, ' Beta_dq_sat = [', num2str(sats * Beta_dq_saturation(1)), ', ', num2str(sats * Beta_dq_saturation(2)), ']'];
                     op_msg = [op_msg, ' Beta_dq_sat = [', num2str(sats), ', ', num2str(sats), ']'];
                end
                
                op_msg = [op_msg];
                case_desc = [case_desc, [f{1} ip_msg op_msg]];
                case_wind_speeds = [case_wind_speeds, LPV_CONTROLLER_WIND_SPEEDS(case_basis.WindSpeedIndex.x(c_ws_idx))];

                fprintf(fileID, op_msg);
            end
        end
        fclose(fileID);
    end

    % for jdx = 1:length(FullOrderControllers)
    %     for idx = 1:length(case_basis.WeGain.x)
    %         % x = (case_basis.WeGain.x{idx}) - case_list(jdx).WeGain;
    %         x = (case_basis.WeGain.x{idx}) - FullOrderControllers(jdx).WeGain.x;
    %         if (sum(x(1,1).Numerator{1,1}) == 0) && (FullOrderControllers(jdx).WuGain.x(1,1).Numerator{1,1} == 1)
    %             idx
    %             jdx
    %             x(1,1)
    %             break;
    %         end
    %     end
    % end
    
    % go through each controller case and find the corresponding controller in
    % FullOrderControllers
    % for c_idx = 1:Controllers_n_cases
    %     if ~strcmp(Controllers_case_list(c_idx).Structure.x, 'Full-Order')
    %         continue;
    %     end
        
    % for f = fieldnames(case_basis.W1Gain)'
    %     for w_idx = 1:n_weighting_cases
    %         wu_diff = FullOrderControllers(w_idx).WuGain.(f{1}) - Controllers_case_list(c_idx).WuGain.x;
    %         we_diff = FullOrderControllers(w_idx).WeGain.(f{1}) - Controllers_case_list(c_idx).WeGain.(f{1});
    %         w1_diff = FullOrderControllers(w_idx).W1Gain.(f{1}) - Controllers_case_list(c_idx).W1Gain.(f{1});
    %         w2_diff = FullOrderControllers(w_idx).W2Gain.(f{1}) - Controllers_case_list(c_idx).W2Gain.(f{1});
    %         if (sum(wu_diff(1,1).Numerator{1,1}) || ...
    %             sum(we_diff(1,1).Numerator{1,1}) || ...
    %             sum(w1_diff(1,1).Numerator{1,1}) || ...
    %             sum(w2_diff(1,1).Numerator{1,1}))
    %             continue;
    %         end
    %         for c_ws_idx = 1:length(case_basis.WindSpeedIndex.x)
    %             K_tmp = FullOrderControllers(w_idx).Controller.(f{1});
    %             K_tmp_scaled = FullOrderControllers(w_idx).Controller_scaled.(f{1});
    % 
    %             % Add gain-scheduled controller to Controller_list
    %             if Controllers_case_list(c_idx).Scheduling.x
    %                 Controllers_case_list(c_idx).Controller.(f{1}) = K_tmp;
    %                 Controllers_case_list(c_idx).Controller_scaled.(f{1}) = K_tmp_scaled;
    %             % Add non gain-scheduled controller to Controller_list by repeating
    %             % same system for every parameter value
    %             elseif ~Controllers_case_list(c_idx).Scheduling.x
    %                 for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
    %                     Controllers_case_list(c_idx).Controller.(f{1})(:, :, c_ws_idx) ...
    %                         = K_tmp(:, :, case_basis.WindSpeedIndex.x == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED));
    %                     Controllers_case_list(c_idx).Controller_scaled.(f{1})(:, :, c_ws_idx) ...
    %                         = K_tmp_scaled(:, :, case_basis.WindSpeedIndex.x == find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED));
    %                 end     
    %             end
    % 
    %             Controllers_case_list(c_idx).Reference.(f{1}) = FullOrderControllers(w_idx).Reference.(f{1});
    %             Controllers_case_list(c_idx).Saturation.(f{1}) = FullOrderControllers(w_idx).Saturation.(f{1});
    % 
    %             Controllers_case_list(c_idx).W1Gain.(f{1}) = FullOrderControllers(w_idx).W1Gain.(f{1});
    %             Controllers_case_list(c_idx).W2Gain.(f{1}) = FullOrderControllers(w_idx).W2Gain.(f{1});
    %             Controllers_case_list(c_idx).WeGain.(f{1}) = FullOrderControllers(w_idx).WeGain.(f{1});
    %             Controllers_case_list(c_idx).WuGain.(f{1}) = FullOrderControllers(w_idx).WuGain.(f{1});
    % 
    %             Controllers_case_list(c_idx).GenPlant.(f{1}) = FullOrderControllers(w_idx).GenPlant.(f{1});
    %             Controllers_case_list(c_idx).Wout.(f{1}) = FullOrderControllers(w_idx).Wout.(f{1});
    %             Controllers_case_list(c_idx).Win.(f{1}) = FullOrderControllers(w_idx).Win.(f{1});
    %             Controllers_case_list(c_idx).gamma.(f{1}) = FullOrderControllers(w_idx).gamma.(f{1});
    % 
    %             Controllers_case_list(c_idx).Mrgi.(f{1}) = FullOrderControllers(w_idx).Mrgi.(f{1});
    %             Controllers_case_list(c_idx).Mrgo.(f{1}) = FullOrderControllers(w_idx).Mrgo.(f{1});
    %             Controllers_case_list(c_idx).DMi.(f{1}) = FullOrderControllers(w_idx).DMi.(f{1});
    %             Controllers_case_list(c_idx).DMo.(f{1}) = FullOrderControllers(w_idx).DMo.(f{1});
    %             Controllers_case_list(c_idx).MMo.(f{1}) = FullOrderControllers(w_idx).MMo.(f{1});
    %             Controllers_case_list(c_idx).MMi.(f{1}) = FullOrderControllers(w_idx).MMi.(f{1});
    %             Controllers_case_list(c_idx).MMio.(f{1}) = FullOrderControllers(w_idx).MMio.(f{1});
    % 
    %             Controllers_case_list(c_idx).wc.(f{1}) = FullOrderControllers(w_idx).wc.(f{1});
    % 
    %             Controllers_case_list(c_idx).Stable.(f{1}) = FullOrderControllers(w_idx).Stable.(f{1});
    % 
    %         end   
    %     end
    %     % Controllers_case_list(c_idx).Controller_scaled.(f{1}).SamplingGrid = struct('u', LPV_CONTROLLER_WIND_SPEEDS);
    % 
    % end

    % for jdx = 1:length(Controllers_case_list)
    %     for idx = 1:length(case_basis.WeGain.x)
    %         % x = (case_basis.WeGain.x{idx}) - case_list(jdx).WeGain;
    %         x = (case_basis.WeGain.x{idx}) - Controllers_case_list(jdx).WeGain.x;
    %         if (sum(x(1,1).Numerator{1,1}) == 0) && (Controllers_case_list(jdx).WuGain.x(1,1).Numerator{1,1} == 1)
    %             idx
    %             jdx
    %             x(1,1)
    %             break;
    %         end
    %     end
    % end
    
    i = 1;
    for f = fieldnames(case_basis.W1Gain)'
        for w_idx = 1:n_weighting_cases
            for c_ws_idx = 1:length(case_basis.WindSpeedIndex.x)
                FullOrderControllers(w_idx).CaseDesc.(f{1}) = case_desc(i);
                i = i + 1;
            end
        end
    end
    Controllers_case_list = FullOrderControllers;

%% Plot Generalized Plant transfer functions 
n_weighting_cases = (FullOrderControllers_n_cases / length(case_basis.WindSpeedIndex.x));
if n_weighting_cases == 1 || false% if testing
% for f = {'rob', 'adc', 'y_mse'}
    f = {'x'};
    for w_idx = 1:n_weighting_cases
        close all;
        % cc = find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED);
        cc = 1;
        GenPlant_inner = inv(Controllers_case_list(w_idx).Wout.(f{1})(:, :, cc)) ...
            * Controllers_case_list(w_idx).GenPlant.(f{1})(:, :, cc) ...
            * inv(Controllers_case_list(w_idx).Win.(f{1})(:, :, cc));
        GenPlant_inner.OutputName = Controllers_case_list(w_idx).GenPlant.(f{1})(:, :, cc).OutputName;
        GenPlant_inner.InputName = Controllers_case_list(w_idx).GenPlant.(f{1})(:, :, cc).InputName;

        % GenPlant_inner.InputGroup.dist = 1:4;
        GenPlant_inner.InputGroup.dist = find(ismember(...
            GenPlant_inner.InputName, ...
            {'\beta_d Disturbance', '\beta_q Disturbance', ...
            'M_d Reference', 'M_q Reference'}))';
    
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
            {'M_d Tracking Error', 'M_q Tracking Error'}))';
         
        Win_arr = {};
        Wout_arr = {};
        for i = 1:size(Controllers_case_list(w_idx).Win.(f{1}), 1)
            Win_arr{end+1} = Controllers_case_list(w_idx).Win.(f{1})(i, i, cc);
        end
        for i = 1:size(Controllers_case_list(w_idx).Wout.(f{1}), 1)
            Wout_arr{end+1} = Controllers_case_list(w_idx).Wout.(f{1})(i, i, cc);
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
            Controllers_case_list(w_idx).Controller.(f{1})(:, :, cc), ...
            Win_arr, Wout_arr);
        w = logspace(-3, 2, 400); % TODO only plot wout on diagonal 
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
        Wu_tmp = Controllers_case_list(w_idx).WuGain.(f{1}) * Wu;
        We_tmp = Controllers_case_list(w_idx).WeGain.(f{1}) * We;
        W1_tmp = Controllers_case_list(w_idx).W1Gain.(f{1}) * W1;
        W2_tmp = Controllers_case_list(w_idx).W2Gain.(f{1}) * W2;
        SF = loopsens(-Plant(:, :, cc), ...
            -Controllers_case_list(w_idx).Controller_scaled.(f{1})(:, :, cc));
        sys_KSo = SF.CSo * W1_tmp;
        sys_Ti = SF.Ti * W2_tmp;
        sys_So = SF.So * W1_tmp;
        sys_GSi = SF.PSi * W2_tmp;
        % bodemag(SF.To)
        % Plot output sensitivity function and its weighting matrix, dy-y or
        % n->e or r->e (ideally 0)
        % S and T are coupled : S + T = 1, so choosing 1 determines the other
        % need penalty on both bc want both to have small magnitdues at particular
        % frequencies (ie could add 2 complex numbers with large magnitudes at
        % high phases that add to 1, or have one with large mag and the other with very small in alignment)
        % OpenLoop case: So = 1, ClosedLoop case: can shape So, ideally to 0.
        % Real So: hpf with roll-off with high-freq gain of 1.
        % So = e/W2d2, output disturbance -> tracking error
        subplot(2,2,4);
        sys_So.InputName = GenPlant.InputName(1:2); % output disturbance/reference
        sys_So.OutputName = We_tmp.InputName;
        h = bodeplot(...
            sys_So, ...
            inv(We_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc), w, bode_plot_opt);
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
        
        % Plot complementary input sensitivity function and its weighting matrix
        % Ti = -u/W1d1, input disturbance -> control input
        subplot(2,2,1);
        sys_Ti.InputName = GenPlant.InputName(3:4); % control disturbance
        sys_Ti.OutputName = Wu_tmp.InputName; % control input
        sys_Ti.InputName = {'\beta_d Disturbance', '\beta_q Disturbance'};
        % Wu_tmp.OutputName = {'\beta_d Disturbance', '\beta_q Disturbance'};
        sys_Ti.OutputName = {'\beta_d Control Input', '\beta_q Control Input'};
        % Wu_tmp.InputName = {'\beta_d Control Input', '\beta_q Control Input'};
        bodeplot(...
            sys_Ti, ...
            inv(Wu_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc), w, bode_plot_opt);
        title('Complementary Input Sensitivity Function, T_i');
        axh = findall(gcf, 'type', 'axes');
        for a = 1:length(axh)
            xline(axh(a), [loopData.peaks.wGAM]);
        end
        set(gcf, 'Position', [0 0 1500 900]);
        legend('T_i W_2', 'W_u^{-1}\gamma');
    
        % Plot GSi with two weighting functions
        % GSi = -e/W1d1, input disturbance -> tracking error
        subplot(2,2,3);
        sys_GSi.InputName = GenPlant.InputName(3:4); % control disturbance
        sys_GSi.OutputName = We_tmp.InputName; % tracking error
        bodeplot(...
            sys_GSi, ...
            inv(We_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc), w, bode_plot_opt);
        title('GS_i Function');
        axh = findall(gcf, 'type', 'axes');
        for a = 1:length(axh)
            xline(axh(a), [loopData.peaks.wGAM]);
        end
        set(gcf, 'Position', [0 0 1500 900]);
        legend('GS_i W_2', 'W_e^{-1}\gamma', 'W_y^{-1}\gamma');

        % Plot KS_o with two weighting functions
        % KSo = u/W2d2, output disturbance -> control input
        subplot(2,2,2);
        sys_KSo.InputName = GenPlant.InputName(1:2); % output disturbance/reference
        sys_KSo.OutputName = Wu_tmp.InputName; % control input
        bodeplot(...
            sys_KSo, ...
            inv(Wu_tmp) * Controllers_case_list(w_idx).gamma.(f{1})(cc), w, ...
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
% end
end

n_weighting_cases = (FullOrderControllers_n_cases / length(case_basis.WindSpeedIndex.x));
controller_types = fieldnames(case_basis.W1Gain)';
i = 1;
clear Reference_tmp Saturation_tmp Stable_tmp Mrgi_tmp Mrgo_tmp DMi_tmp DMo_tmp MMi_tmp MMo_tmp MMio_tmp wc_tmp W1_gain_tmp W2_gain_tmp We_gain_tmp We_gain_tmp
for f = 1:length(controller_types)
     for w_idx = 1:n_weighting_cases
          for c_ws_idx = 1:length(case_basis.WindSpeedIndex.x)
    % for c_idx = 1:Controllers_n_cases
            ctrl_type = controller_types{f};
            % i = (f_idx - 1) + w_idx;

            Reference_tmp(i) = Controllers_case_list(w_idx).Reference.(ctrl_type);
            Saturation_tmp(i) = Controllers_case_list(w_idx).Saturation.(ctrl_type);

            Stable_tmp(i) = Controllers_case_list(w_idx).Stable.(ctrl_type)(c_ws_idx);
            Mrgi_tmp(i, 1, 1) = Controllers_case_list(w_idx).Mrgi.(ctrl_type)(1, c_ws_idx).GainMargin(1);
            pm = Controllers_case_list(w_idx).Mrgi.(ctrl_type)(1, c_ws_idx).PhaseMargin;
            if length(pm)
                Mrgi_tmp(i, 1, 2) = pm(1);
            else
                Mrgi_tmp(i, 1, 2) = 0;
            end
        
            Mrgi_tmp(i, 2, 1) = Controllers_case_list(w_idx).Mrgi.(ctrl_type)(2, c_ws_idx).GainMargin(1);
            pm = Controllers_case_list(w_idx).Mrgi.(ctrl_type)(2, c_ws_idx).PhaseMargin;
            if length(pm)
                Mrgi_tmp(i, 2, 2) = pm(1);
            else
                Mrgi_tmp(i, 2, 2) = 0;
            end
    
            Mrgo_tmp(i, 1, 1) = Controllers_case_list(w_idx).Mrgo.(ctrl_type)(1).GainMargin(1);
            % SF_tmp = loopsens(-Plant(:, :, c_ws_idx), -Controllers_case_list(w_idx).Controller_scaled.y_mse(:,:,c_ws_idx));
            % bodemag(SF_tmp.To, SF_tmp.So, SF_tmp.Lo)
            pm = Controllers_case_list(w_idx).Mrgo.(ctrl_type)(1, c_ws_idx).PhaseMargin;
            if length(pm)
                Mrgo_tmp(i, 1, 2) = pm(1);
            else
                Mrgo_tmp(i, 1, 2) = 0;
            end
    
            Mrgo_tmp(i, 2, 1) = Controllers_case_list(w_idx).Mrgo.(ctrl_type)(2).GainMargin(1);
            pm = Controllers_case_list(w_idx).Mrgo.(ctrl_type)(2, c_ws_idx).PhaseMargin;
            if length(pm)
                Mrgo_tmp(i, 2, 2) = pm(1);
            else
                Mrgo_tmp(i, 2, 2) = 0;
            end
            
            DMi_tmp(i, 1, 1) = Controllers_case_list(w_idx).DMi.(ctrl_type)(1, c_ws_idx).GainMargin(2);
            DMi_tmp(i, 1, 2) = Controllers_case_list(w_idx).DMi.(ctrl_type)(1, c_ws_idx).PhaseMargin(2);
            DMi_tmp(i, 1, 3) = Controllers_case_list(w_idx).DMi.(ctrl_type)(1, c_ws_idx).DiskMargin;
    
            DMi_tmp(i, 2, 1) = Controllers_case_list(w_idx).DMi.(ctrl_type)(2, c_ws_idx).GainMargin(2);
            DMi_tmp(i, 2, 2) = Controllers_case_list(w_idx).DMi.(ctrl_type)(2, c_ws_idx).PhaseMargin(2);
            DMi_tmp(i, 2, 3) = Controllers_case_list(w_idx).DMi.(ctrl_type)(2, c_ws_idx).DiskMargin;
            
            DMo_tmp(i, 1, 1) = Controllers_case_list(w_idx).DMo.(ctrl_type)(1, c_ws_idx).GainMargin(2);
            DMo_tmp(i, 1, 2) = Controllers_case_list(w_idx).DMo.(ctrl_type)(1, c_ws_idx).PhaseMargin(2);
            DMo_tmp(i, 1, 3) = Controllers_case_list(w_idx).DMo.(ctrl_type)(1, c_ws_idx).DiskMargin;
            
            DMo_tmp(i, 2, 1) = Controllers_case_list(w_idx).DMo.(ctrl_type)(2, c_ws_idx).GainMargin(2);
            DMo_tmp(i, 2, 2) = Controllers_case_list(w_idx).DMo.(ctrl_type)(2, c_ws_idx).PhaseMargin(2);
            DMo_tmp(i, 2, 3) = Controllers_case_list(w_idx).DMo.(ctrl_type)(2, c_ws_idx).DiskMargin;
            
            MMi_tmp(i, 1) = Controllers_case_list(w_idx).MMi.(ctrl_type)(c_ws_idx).GainMargin(2);
            MMi_tmp(i, 2) = Controllers_case_list(w_idx).MMi.(ctrl_type)(c_ws_idx).PhaseMargin(2);
            MMi_tmp(i, 3) = Controllers_case_list(w_idx).MMi.(ctrl_type)(c_ws_idx).DiskMargin;
            
            MMo_tmp(i, 1) = Controllers_case_list(w_idx).MMo.(ctrl_type)(c_ws_idx).GainMargin(2);
            MMo_tmp(i, 2) = Controllers_case_list(w_idx).MMo.(ctrl_type)(c_ws_idx).PhaseMargin(2);
            MMo_tmp(i, 3) = Controllers_case_list(w_idx).MMo.(ctrl_type)(c_ws_idx).DiskMargin;
            
            MMio_tmp(i, 1) = Controllers_case_list(w_idx).MMio.(ctrl_type)(c_ws_idx).GainMargin(2);
            MMio_tmp(i, 2) = Controllers_case_list(w_idx).MMio.(ctrl_type)(c_ws_idx).PhaseMargin(2);
            MMio_tmp(i, 3) = Controllers_case_list(w_idx).MMio.(ctrl_type)(c_ws_idx).DiskMargin;
            
            wc_tmp(i) = Controllers_case_list(w_idx).wc.(ctrl_type)(c_ws_idx);

            W1_gain_tmp(i) = Controllers_case_list(w_idx).W1Gain.(ctrl_type).Numerator{1, 1}(1);
            W2_gain_tmp(i) = Controllers_case_list(w_idx).W2Gain.(ctrl_type).Numerator{1, 1}(1);
            We_gain_tmp(i) = Controllers_case_list(w_idx).WeGain.(ctrl_type).Numerator{1, 1}(1);
            Wu_gain_tmp(i) = Controllers_case_list(w_idx).WuGain.(ctrl_type).Numerator{1, 1}(1);
            
            i = i + 1;
         end
    end
end

n_total_cases = length(wc_tmp);
% Make table comparing controllers
Controllers_case_table = table( ...
    (1:n_total_cases)', case_desc', ... 
    case_wind_speeds', Stable_tmp', ...
    Reference_tmp', Saturation_tmp', ...
    W1_gain_tmp', W2_gain_tmp', We_gain_tmp', Wu_gain_tmp', ...
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
    'Reference', 'Saturation', ...
    'A_W1', 'A_W2', 'A_We', 'A_Wu', ...
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
    'wc', 'ADC', 'RootMycBlade1 RMSE'});

for c_ws_idx = case_basis.WindSpeedIndex.x
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

save(fullfile(mat_save_dir, [sim_type, '_Controllers_case_list.mat']), "Controllers_case_list", '-v7.3');
save(fullfile(mat_save_dir, [sim_type, '_Controllers_case_table.mat']), "Controllers_case_table", '-v7.3');

else
    load(fullfile(mat_save_dir, [sim_type, '_Controllers_case_list.mat']));
    load(fullfile(mat_save_dir, [sim_type, '_Controllers_case_table.mat']));
end

