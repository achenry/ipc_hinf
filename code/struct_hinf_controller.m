%% TODO
% MIMO wind-up: check if blade-pitch actuation is saturating, check signal after saturator
% Check GenPlant derivation, signs in SL models

%% Setup workspace
init_hinf_controller;
fig_dir = fullfile(fig_dir, 'struct_controllers');

%% TODO Formulate Tunable PI+Notch+LPF Continuous-Time Controller Structure that resembles full-order controller
exact_int = 1 / tf('s');
approx_int = 1 / (tf('s') + 1e-12);
% approx_int = 1 / (tf('s'));

% MIMO tunable PI
Kp_diag = realp('Kp_diag', baseline_PI.Kp); % baseline_PI.Kp
Ki_diag = realp('Ki_diag', baseline_PI.Ki); % baseline_PI.Ki
Kp_offdiag = realp('Kp_offdiag', baseline_PI.Kp);
Ki_offdiag = realp('Ki_offdiag', baseline_PI.Ki);
PI_diag = eye(2) * (Kp_diag + Ki_diag * approx_int);
PI_offdiag = rot90(eye(2)) * (Kp_offdiag + Ki_offdiag * approx_int);
PI = PI_diag + PI_offdiag;

% MIMO tunable 3p notch filter (bandpass)
% tune height of peak and width of inverse notch filter/bandpass (more
% intuitive but can use coefficients).
% Compare with (MIMO) and without (SISO) off-diagonal
zeta_diag = realp('zeta_diag', zeta); % damping, greater value leads to greater width
zeta_offdiag = realp('zeta_offdiag', zeta);
gbar_diag = realp('gbar_diag', gbar); % gain at notch frequency
gbar_offdiag = realp('gbar_offdiag', gbar);

% TODO test variations of structured controller with/without 2p off diag,
% 2p bandstop
Notch_3P_diag = tf([1 2*zeta_diag*gbar_diag*omega_3P_rad omega_3P_rad^2], ...
    [1 2*zeta_diag*omega_3P_rad omega_3P_rad^2]);

% Notch_2P_offdiag = tf([1 2*gbar_offdiag*zeta_offdiag*omega_2P_rad omega_2P_rad^2], ...
%     [1 2*zeta_offdiag*omega_2P_rad omega_2P_rad^2]);

% Notch = [Notch_3P_diag, Notch_2P_offdiag; Notch_2P_offdiag, Notch_3P_diag];
Notch = Notch_3P_diag * eye(2);

% MIMO tunable low-pass filter => not tunable TODO above 3P, fix Wu
% roll-off keep low-freq magnitude tunable (static gain) QUESTION is this
% too high for a bp frequency?
LPF = 1 / (s + w1_u) * eye(2);

% Full MIMO Structured Controller
K_struct = [PI(1,1)*Notch(1,1)*LPF(1,1), PI(1,2)*Notch(1,2)*LPF(1,2);
            PI(2,1)*Notch(2,1)*LPF(2,1), PI(2,2)*Notch(2,2)*LPF(2,2)];
K_struct.InputName = {'Measured RootMycD Tracking Error', 'Measured RootMycQ Tracking Error'};
K_struct.OutputName = {'BldPitchD Control Input', 'BldPitchQ Control Input'};

if 0
    figure;
    bodeplot(K_struct, bode_plot_opt); 
    title('Frequency Response of Initial Structured Controller');
end


%% Conduct Parameter Sweep for MIMO PI Controller
% omega = logspace(-2, 2, 200);
if BASELINE_K % QUESTION does it need to be negative??
    Sweep.Vary.Kp_diag = [Parameters.cIPC.DQ_Kp_1P];
    Sweep.Vary.Kp_offdiag = [0];
    Sweep.Vary.Ki_diag = [Parameters.cIPC.DQ_Ki_1P];
    Sweep.Vary.Ki_offdiag = [0];
    % Sweep.Vary.WindSpeedIdx = 1:length(LPV_CONTROLLER_WIND_SPEEDS);
    Sweep.Vary.WindSpeedIdx = [find(C_WS_IDX)];
    
    K_grid = combinations(Sweep.Vary.Kp_diag, Sweep.Vary.Kp_offdiag, ...
                                  Sweep.Vary.Ki_diag, Sweep.Vary.Ki_offdiag, ...
                                  Sweep.Vary.WindSpeedIdx);

    % add no ipc case
    K_grid = [combinations([0], [0], [0], [0], Sweep.Vary.WindSpeedIdx); ...
              K_grid];

elseif STRUCT_PARAM_SWEEP
    % TODO robustness margin 6dB, 45deg. Evaluate robustness margins, if not
    % achieved need to tune weighting filters.
    % run a sweep for each of the parameters, keeping all other params constant
    % at their baseline values
    
    % want to know robustness of baseline controller (PI), wc of L,
    % how do freq-domain specs translate to time-domain
    
    Sweep.Vary.Kp_diag = logspace(-3, 4, 8); % linspace(approx_Kp.KD_range(1), approx_Kp.KD_range(2), 5);
    Sweep.Vary.Kp_offdiag = logspace(-3, 4, 8); %logspace(-9, 0, 10); % linspace(approx_Kp.KQD_range(1), approx_Kp.KQD_range(2), 5);
    Sweep.Vary.Ki_diag = logspace(-3, 4, 8); % linspace(approx_Ki.KD_range(1), approx_Ki.KD_range(2), 5);
    Sweep.Vary.Ki_offdiag = logspace(-3, 4, 8); %logspace(-9, 0, 10); % linspace(approx_Ki.KDQ_range(1), approx_Ki.KDQ_range(2), 5);
    Sweep.Vary.WindSpeedIdx = [find(C_WS_IDX)];
    
    K_grid = combinations(Sweep.Vary.Kp_diag, Sweep.Vary.Kp_offdiag, ...
                                  Sweep.Vary.Ki_diag, Sweep.Vary.Ki_offdiag, ...
                                  Sweep.Vary.WindSpeedIdx);
end

REPROCESS_SWEEP = 1;
if REPROCESS_SWEEP
    Sweep.Params = {'Kp_diag', 'Kp_offdiag', 'Ki_diag', 'Ki_offdiag', 'WindSpeedIdx'};
    
    
                                  % [find(C_WS_IDX)]);
                                  % 1:length(LPV_CONTROLLER_WIND_SPEEDS));
    K_grid.Properties.VariableNames = {'Kp_diag', 'Kp_offdiag', 'Ki_diag', 'Ki_offdiag', 'WindSpeedIdx'};
    
    
    % for each gridpoint
    PI_ParameterSweep = repmat(struct(), size(K_grid, 1), 1);
    parfor case_idx = 1:size(K_grid, 1)
        K_tmp = PI;
        % parameter sweep for PI based on diskmargin, hinfnorm of To for tracking,
        % (bandwidth of To) for one wind-speed => to know how critical
        % robustness is, followed by nonlinear simulation to evaluate ADC and
        % Mdq tracking performance
        K_tmp.Blocks.Kp_diag.Value = K_grid(case_idx, 'Kp_diag').Variables;
        K_tmp.Blocks.Kp_offdiag.Value = K_grid(case_idx, 'Kp_offdiag').Variables;
        K_tmp.Blocks.Ki_diag.Value = K_grid(case_idx, 'Ki_diag').Variables;
        K_tmp.Blocks.Ki_offdiag.Value = K_grid(case_idx, 'Ki_offdiag').Variables;
        c_ws_idx = K_grid(case_idx, 'WindSpeedIdx').Variables;
    
        PI_ParameterSweep(case_idx).Gains = K_grid(case_idx, :);

        PI_ParameterSweep(case_idx).Controller = ss(K_tmp);

        % negative Plant st e = r(dy) - y(yP) is input to controller, 
        % negative Controller for positive u input to Plant
        SF_tmp = loopsens(-Plant(:, :, c_ws_idx), -PI_ParameterSweep(case_idx).Controller);
        
        % classical gain/phase margins at plant outputs
        Mrgo_tmp = allmargin(SF_tmp.Lo);

        % classical gain/phase margins at plant inputs
        Mrgi_tmp = allmargin(SF_tmp.Li);

        % vary gain/phase perturbation at all plant outputs
        [DMo_tmp, MMo_tmp] = diskmargin(SF_tmp.Lo);

        % vary gain/phase perturbation at all plant inputs
        [DMi_tmp, MMi_tmp] = diskmargin(SF_tmp.Li);

        % vary gain/phase perturbation at all inputs and outputs
        MMio_tmp = diskmargin(Plant(:, :, c_ws_idx), PI_ParameterSweep(case_idx).Controller);
        
        PI_ParameterSweep(case_idx).Mrgo = Mrgo_tmp;
        PI_ParameterSweep(case_idx).Mrgi = Mrgi_tmp;
        PI_ParameterSweep(case_idx).DMo = DMo_tmp;
        PI_ParameterSweep(case_idx).DMi = DMi_tmp;
        PI_ParameterSweep(case_idx).MMo = MMo_tmp;
        PI_ParameterSweep(case_idx).MMi = MMi_tmp;
        PI_ParameterSweep(case_idx).MMio = MMio_tmp;
        
        dc = abs(dcgain(PI_ParameterSweep(case_idx).Controller));

        % QUESTION MANUEL is this how to compute bandwidth
        if dc(1, 1) > 0
        PI_ParameterSweep(case_idx).wc = getGainCrossover(...
            PI_ParameterSweep(case_idx).Controller, ...
            min(dc(1, 1), [], 'all') / sqrt(2));
        else
            PI_ParameterSweep(case_idx).wc = nan;
        end
        
        % K_tmp = ip_scaling(:, :, c_ws_idx) * K_tmp * inv(op_scaling(:, :, c_ws_idx));
        % PI_ParameterSweep(case_idx).Controller_scaled = K_tmp;
        % QUESTION MANUEL does it make sense to sweep over unscaled
        % controller here  
        
        PI_ParameterSweep(case_idx).SF = SF_tmp;
    
    end
    
    % restructure sweep controller cases per gains
    % combination for each wind speed
    % v_in_stable = 1;
    % v_out_stable = 1;
    PI_ParameterSweepControllers = struct;
    for c_ws_idx = Sweep.Vary.WindSpeedIdx
        weighting_case_idx = 1;
        for case_idx = 1:length(PI_ParameterSweep)
            if c_ws_idx == PI_ParameterSweep(case_idx).Gains(:, 'WindSpeedIdx').Variables

                % cc = find(c_ws_idx == Sweep.Vary.WindSpeedIdx);

                PI_ParameterSweepControllers(weighting_case_idx).Controller = ...
                    PI_ParameterSweep(case_idx).Controller;
                % PI_ParameterSweep.Controller_scaled(:, :, w_idx, j) = PI_ParameterSweep_tmp(case_idx).Controller_scaled;
                
                Gains_tmp(weighting_case_idx, :) = PI_ParameterSweep(case_idx).Gains(:, 1:4).Variables;
                PI_ParameterSweepControllers(weighting_case_idx).Gains = ...
                    Gains_tmp(weighting_case_idx, :);
                
                if length(PI_ParameterSweep(case_idx).Mrgo(1).GainMargin)
                    Mrgo_tmp(weighting_case_idx, c_ws_idx, :, :) = ...
                        [PI_ParameterSweep(case_idx).Mrgo(1).GainMargin(1) PI_ParameterSweep(case_idx).Mrgo(1).PhaseMargin(1);
                        PI_ParameterSweep(case_idx).Mrgo(2).GainMargin(1) PI_ParameterSweep(case_idx).Mrgo(2).PhaseMargin(1)];
                else
                    Mrgo_tmp(weighting_case_idx, c_ws_idx, :, :) = [nan nan; nan nan];
                end

                PI_ParameterSweepControllers(weighting_case_idx).Mrgo(:, :, c_ws_idx) = ...
                        Mrgo_tmp(weighting_case_idx, c_ws_idx);
                
                if length(PI_ParameterSweep(case_idx).Mrgi(1).GainMargin)
                    Mrgi_tmp(weighting_case_idx, c_ws_idx, :, :) = ...    
                    [PI_ParameterSweep(case_idx).Mrgi(1).GainMargin(1) PI_ParameterSweep(case_idx).Mrgi(1).PhaseMargin(1);
                        PI_ParameterSweep(case_idx).Mrgi(2).GainMargin(1) PI_ParameterSweep(case_idx).Mrgi(2).PhaseMargin(1)];
                else
                    Mrgi_tmp(weighting_case_idx, c_ws_idx, :, :) = ... 
                        [nan nan; nan nan]; 
                end

                PI_ParameterSweepControllers(weighting_case_idx).Mrgi(:, :, c_ws_idx) = ... 
                    Mrgi_tmp(weighting_case_idx, c_ws_idx);
                
                DMo_tmp(weighting_case_idx, c_ws_idx, :, :) = ...
                    [PI_ParameterSweep(case_idx).DMo(1).GainMargin(2) PI_ParameterSweep(case_idx).DMo(1).PhaseMargin(2) PI_ParameterSweep(case_idx).DMo(1).DiskMargin;
                     PI_ParameterSweep(case_idx).DMo(2).GainMargin(2) PI_ParameterSweep(case_idx).DMo(2).PhaseMargin(2) PI_ParameterSweep(case_idx).DMo(2).DiskMargin];
                
                PI_ParameterSweepControllers(weighting_case_idx).DMo(:, :, c_ws_idx) = DMo_tmp(weighting_case_idx, c_ws_idx, :, :);

                DMi_tmp(weighting_case_idx, c_ws_idx, :, :) = ...
                    [PI_ParameterSweep(case_idx).DMi(1).GainMargin(2) PI_ParameterSweep(case_idx).DMi(1).PhaseMargin(2) PI_ParameterSweep(case_idx).DMi(1).DiskMargin;
                     PI_ParameterSweep(case_idx).DMi(2).GainMargin(2) PI_ParameterSweep(case_idx).DMi(2).PhaseMargin(2) PI_ParameterSweep(case_idx).DMi(2).DiskMargin];
                
                PI_ParameterSweepControllers(weighting_case_idx).DMi(:, :, c_ws_idx) ...
                    = DMi_tmp(weighting_case_idx, c_ws_idx, :, :);

                MMo_tmp(weighting_case_idx, c_ws_idx, :) = ...
                    [PI_ParameterSweep(case_idx).MMo.GainMargin(2) PI_ParameterSweep(case_idx).MMo.PhaseMargin(2) PI_ParameterSweep(case_idx).MMo.DiskMargin];
                PI_ParameterSweepControllers(weighting_case_idx).MMo(:, c_ws_idx) = ...
                    MMo_tmp(weighting_case_idx, c_ws_idx, :);


                MMi_tmp(weighting_case_idx, c_ws_idx, :) = ...
                    [PI_ParameterSweep(case_idx).MMi.GainMargin(2) PI_ParameterSweep(case_idx).MMi.PhaseMargin(2) PI_ParameterSweep(case_idx).MMi.DiskMargin];
                PI_ParameterSweepControllers(weighting_case_idx).MMi(:, c_ws_idx) = ...
                    MMi_tmp(weighting_case_idx, c_ws_idx, :);

                MMio_tmp(weighting_case_idx, c_ws_idx, :) = ...
                    [PI_ParameterSweep(case_idx).MMio.GainMargin(2) PI_ParameterSweep(case_idx).MMio.PhaseMargin(2) PI_ParameterSweep(case_idx).MMio.DiskMargin];
                PI_ParameterSweepControllers(weighting_case_idx).MMio(:, c_ws_idx) = ...
                    MMio_tmp(weighting_case_idx, c_ws_idx, :);
                
                % PI_ParameterSweep.n_wc(w_idx, j) = 
                % 
                % if PI_ParameterSweep.n_wc(w_idx, j) == 0
                %     PI_ParameterSweep.wc(w_idx, j) = -1;
                % else
                wc_tmp(weighting_case_idx) = PI_ParameterSweep(case_idx).wc(1);
                PI_ParameterSweepControllers(weighting_case_idx).wc = wc_tmp(weighting_case_idx);
                % end
                
                Stable_tmp(weighting_case_idx, c_ws_idx) = PI_ParameterSweep(case_idx).SF.Stable;
                PI_ParameterSweepControllers(weighting_case_idx).SF.Stable(c_ws_idx) = Stable_tmp(weighting_case_idx, c_ws_idx);

                weighting_case_idx = weighting_case_idx + 1;
            end
        end
    end
    if length(PI_ParameterSweepControllers) == 1
        PI_ParameterSweepControllers = [PI_ParameterSweepControllers];
    end

    % Make table comparing controllers
    % ws_idx = unique(PI_ParameterSweep_tmp(v).Gains(:, 'WindSpeedIdx').Variables) == find(C_WS_IDX);
    ws_idx = find(C_WS_IDX);
    PI_ParameterSweep_table = table( ...
        (1:size(PI_ParameterSweepControllers, 2))', ... 
        Gains_tmp(:, 1), ...
        Gains_tmp(:, 2), ...
        Gains_tmp(:, 3), ...
        Gains_tmp(:, 4), ...
        Stable_tmp(:, ws_idx), ...
        mag2db(Mrgi_tmp(:, ws_idx, 1, 1)), ...
        Mrgi_tmp(:, ws_idx, 1, 2), ...
        mag2db(Mrgi_tmp(:, ws_idx, 2, 1)), ...
        Mrgi_tmp(:, ws_idx, 2, 2), ...
        mag2db(Mrgo_tmp(:, ws_idx, 1, 1)), ...
        Mrgo_tmp(:, ws_idx, 1, 2), ...
        mag2db(Mrgo_tmp(:, ws_idx, 2, 1)), ...
        Mrgo_tmp(:, ws_idx, 2, 2), ...
        mag2db(DMi_tmp(:, ws_idx, 1, 1)), ...
        DMi_tmp(:, ws_idx, 1, 2), ...
        DMi_tmp(:, ws_idx, 1, 3), ...
        mag2db(DMi_tmp(:, ws_idx, 2, 1)), ...
        DMi_tmp(:, ws_idx, 2, 2), ...
        DMi_tmp(:, ws_idx, 2, 3), ...
        mag2db(DMo_tmp(:, ws_idx, 1, 1)), ...
        DMo_tmp(:, ws_idx, 1, 2), ...
        DMo_tmp(:, ws_idx, 1, 3), ...
        mag2db(DMo_tmp(:, ws_idx, 2, 1)), ...
        DMo_tmp(:, ws_idx, 2, 2), ...
        DMo_tmp(:, ws_idx, 2, 3), ...
        mag2db(MMi_tmp(:, ws_idx, 1)), ...
        MMi_tmp(:, ws_idx, 2), ...
        MMi_tmp(:, ws_idx, 3), ...
        mag2db(MMo_tmp(:, ws_idx, 1)), ...
        MMo_tmp(:, ws_idx, 2), ...
        MMo_tmp(:, ws_idx, 3), ...
        mag2db(MMio_tmp(:, ws_idx, 1)), ...
        MMio_tmp(:, ws_idx, 2), ...
        MMio_tmp(:, ws_idx, 3), ...
        wc_tmp', ...
        zeros(size(PI_ParameterSweepControllers, 2), 1), ...
        zeros(size(PI_ParameterSweepControllers, 2), 1), ...
        'VariableNames', ...
        {'Case No.', 'Kp_diag', 'Kp_offdiag', 'Ki_diag', 'Ki_offdiag', ...
        'Stable', ...
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
        'wc', ...
        'ADC', 'RootMycBlade1 MSE'});
    % PI_ParameterSweep_table = sortrows(PI_ParameterSweep_table, 'Mean DMi', 'descend');
    % sortrows(PI_ParameterSweep_table(~isnan(PI_ParameterSweep_table.("Hinf(To)")), :), 'Hinf(To)', 'descend')
    
    PI_ParameterSweep_table.("WorstCase_SingleClassical_GM") = ...
        min(PI_ParameterSweep_table(:, ...
        ["ClassicalInput_GMD", "ClassicalInput_GMQ", ...
        "ClassicalOutput_GMD", "ClassicalOutput_GMQ"]), [], 2).Variables;
    PI_ParameterSweep_table.("WorstCase_SingleClassical_PM") = ...
        min(PI_ParameterSweep_table(:, ...
        ["ClassicalInput_PMD", "ClassicalInput_PMQ", ...
        "ClassicalOutput_PMD", "ClassicalOutput_PMQ"]), [], 2).Variables;

    [M, I] = min(PI_ParameterSweep_table(:, ...
        ["SingleDiskInput_DMD", "SingleDiskInput_DMQ", ...
        "SingleDiskOutput_DMD", "SingleDiskOutput_DMQ"]), [], 2);
    x = PI_ParameterSweep_table(:, ...
        ["SingleDiskInput_GMD", "SingleDiskInput_GMQ", ...
        "SingleDiskOutput_GMD", "SingleDiskOutput_GMQ"]).Variables;
    PI_ParameterSweep_table.("WorstCase_SingleDisk_GM") = x(I.Variables);
    x = PI_ParameterSweep_table(:, ...
        ["SingleDiskInput_PMD", "SingleDiskInput_PMQ", ...
        "SingleDiskOutput_PMD", "SingleDiskOutput_PMQ"]).Variables;
    PI_ParameterSweep_table.("WorstCase_SingleDisk_PM") = x(I.Variables);
    
    
    if BASELINE_K
        save(fullfile(mat_save_dir, 'PI_BaselineParameters.mat'), "PI_ParameterSweep");
        save(fullfile(mat_save_dir, 'PI_BaselineParameters_table.mat'), "PI_ParameterSweep_table");
    elseif STRUCT_PARAM_SWEEP
        save(fullfile(mat_save_dir, 'PI_ParameterSweep.mat'), "PI_ParameterSweep");
        save(fullfile(mat_save_dir, 'PI_ParameterSweep_table.mat'), "PI_ParameterSweep_table");
    end

else
    if BASELINE_K
        load(fullfile(mat_save_dir, 'PI_BaselineParameters.mat'));
        load(fullfile(mat_save_dir, 'PI_BaselineParameters_table.mat'));
    elseif STRUCT_PARAM_SWEEP
        load(fullfile(mat_save_dir, 'PI_ParameterSweep.mat'));
        load(fullfile(mat_save_dir, 'PI_ParameterSweep_table.mat'));
    end
end

if 1
    if BASELINE_K
        robust_idx = PI_ParameterSweep_table.('Case No.') == PI_ParameterSweep_table.('Case No.');
        PI_ParameterSweep_redtable = PI_ParameterSweep_table(robust_idx, :);
        robust_idx = PI_ParameterSweep_redtable.('Case No.');
    elseif STRUCT_PARAM_SWEEP
        tol = 1e-3;
        robust_idx = (PI_ParameterSweep_table(:, "Hinf(To)").Variables <= 1+tol) ... % idx where peak To is close to 1
            & (PI_ParameterSweep_table(:, "Hinf(To)").Variables >= 1-tol) ...
            & (PI_ParameterSweep_table(:, "Hinf(Ti)").Variables <= 1+tol) ...
            & (PI_ParameterSweep_table(:, "Hinf(Ti)").Variables >- 1-tol) ...
            & (PI_ParameterSweep_table(:, "Hinf(So)").Variables < 1.22) ...
            & (PI_ParameterSweep_table(:, "Hinf(So)").Variables >= 1) ...
            & (PI_ParameterSweep_table(:, "Hinf(Si)").Variables < 1.22) ...
            & (PI_ParameterSweep_table(:, "Hinf(Si)").Variables >= 1) ...
            & (PI_ParameterSweep_table(:, "Mean DM").Variables < 1.5) ...
            & (PI_ParameterSweep_table(:, "Mean DM").Variables >= 1);
        % nnan_idx = ~isnan(PI_ParameterSweep_table.("Hinf(To)"));
        stable_idx = (PI_ParameterSweep_table.("Stable") == 1);
        % robust_idx = (PI_ParameterSweep_table.("Mean DMi") > 0) & (PI_ParameterSweep_table.("Mean DMo") > 0);
        % wco_idx = (PI_ParameterSweep_table(:, "wco").Variables <= omega_3P_rad * 10) ...
        %     & (PI_ParameterSweep_table(:, "wco").Variables >= omega_3P_rad /
        %     10)
        
        % PI_ParameterSweep_redtable = sortrows(PI_ParameterSweep_table(stable_idx & robust_idx, :), 'Mean DM', 'ascend'); % last one (with gre
        % approx 1 for Ti, To; 1.2 for Si, So, 1.4 for DM values
        PI_ParameterSweep_redtable = sortrows(PI_ParameterSweep_table(stable_idx & robust_idx, :), 'Mean DM', 'ascend');
        % unity_idx = ones(length(nnan_idx), 1)
        % sortrows(PI_ParameterSweep_table, 'DMoD', 'descend')
        % PI_ParameterSweep_redtable = PI_ParameterSweep_redtable(1:10, :); %...
            % PI_ParameterSweep_redtable(end-5:end, :)];
        
        % plot To
        % QUESTION How to choose best controllers
        % based on linear metrics?
        % QUESTION off_diag components look odd... should they be positive...
        % bodemag(PI_ParameterSweep.SF.To(:, :, PI_ParameterSweep_redtable(:, "Case No.").Variables))
        % xline(PI_ParameterSweep_redtable(1, "wco").Variables);

        robust_idx = PI_ParameterSweep_redtable.('Case No.');
    
    end
    clear PI_ParameterSweep_case_list;

    % if height(PI_ParameterSweep_table) == 2 % only single controller case + no ipc case
    %      PI_ParameterSweep.Controller(:, :, 1, 1) = PI_ParameterSweep.Controller;
    %      % PI_ParameterSweep.Controller_scaled(:, :, 1, 1) = PI_ParameterSweep.Controller_scaled;
    %      PI_ParameterSweep.Gains(:, 1, 1) = PI_ParameterSweep.Gains;
    % end

    vv = 1;
    for v = robust_idx'
        PI_ParameterSweep_case_list(vv).CaseNo = v;
        % if v ~= 0 % using ipc
            % PI_ParameterSweep_case_list(vv).Controller = PI_ParameterSweepControllers(v).Controller;
            % PI_ParameterSweep_case_list(vv).Controller_scaled = ss(PI_ParameterSweep.Controller_scaled(:, :, v, :));
            
            
            for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
                PI_ParameterSweep_case_list(vv).Controller(:, :, c_ws_idx) ...
                    = PI_ParameterSweepControllers(v).Controller;
                % PI_ParameterSweep_case_list(vv).Controller_scaled(:, :, c_ws_idx) ...
                %     = ss(PI_ParameterSweep.Controller_scaled(:, :, v, :));
            end     
            
            PI_ParameterSweep_case_list(vv).Controller.InputName = {'Measured M_d Tracking Error', 'Measured M_q Tracking Error'};
            PI_ParameterSweep_case_list(vv).Controller.OutputName = {'\beta_d Control Input', '\beta_q Control Input'};
            PI_ParameterSweep_case_list(vv).Controller.SamplingGrid = struct('u', LPV_CONTROLLER_WIND_SPEEDS);
    
            % PI_ParameterSweep_case_list(vv).Controller_scaled.InputName = {'Measured RootMycD Tracking Error', 'Measured RootMycQ Tracking Error'};
            % PI_ParameterSweep_case_list(vv).Controller_scaled.OutputName = {'BldPitchD Control Input', 'BldPitchQ Control Input'};
            % PI_ParameterSweep_case_list(vv).Controller_scaled.SamplingGrid = struct('u', LPV_CONTROLLER_WIND_SPEEDS);
            
            PI_ParameterSweep_case_list(vv).Kp_diag = PI_ParameterSweepControllers(v).Gains(1);
            PI_ParameterSweep_case_list(vv).Kp_offdiag = PI_ParameterSweepControllers(v).Gains(2);
            PI_ParameterSweep_case_list(vv).Ki_diag = PI_ParameterSweepControllers(v).Gains(3);
            PI_ParameterSweep_case_list(vv).Ki_offdiag = PI_ParameterSweepControllers(v).Gains(4);
        % end
    
        vv = vv + 1;
    end
    
    % PI_ParameterSweep_redtable = [num2cell(zeros(1, size(PI_ParameterSweep_redtable, 2))); PI_ParameterSweep_redtable]; % NoIPC case

    if BASELINE_K
        save(fullfile(mat_save_dir, 'PI_BaselineParameters_case_list.mat'), "PI_ParameterSweep_case_list");
        save(fullfile(mat_save_dir, 'PI_BaselineParameters_redtable.mat'), "PI_ParameterSweep_redtable");
    elseif STRUCT_PARAM_SWEEP
        save(fullfile(mat_save_dir, 'PI_ParameterSweep_case_list.mat'), "PI_ParameterSweep_case_list");
        save(fullfile(mat_save_dir, 'PI_ParameterSweep_redtable.mat'), "PI_ParameterSweep_redtable");
    end
else
    if BASELINE_K
        load(fullfile(mat_save_dir, 'PI_BaselineParameters_case_list.mat'));
        load(fullfile(mat_save_dir, 'PI_BaselineParameters_redtable.mat'));        
    elseif STRUCT_PARAM_SWEEP
        load(fullfile(mat_save_dir, 'PI_ParameterSweep_case_list.mat'));
        load(fullfile(mat_save_dir, 'PI_ParameterSweep_redtable.mat'));
    end
end

if 0
% Plot loop gain for particular controller
% K_tmp = PI;
% K_tmp.Blocks.Kp_diag.Value = PI_ParameterSweep_redtable(1, "Kp_diag").Variables;
% K_tmp.Blocks.Kp_offdiag.Value = PI_ParameterSweep_redtable(1, "Kp_offdiag").Variables;
% K_tmp.Blocks.Ki_diag.Value = PI_ParameterSweep_redtable(1, "Ki_diag").Variables;
% K_tmp.Blocks.Ki_offdiag.Value = PI_ParameterSweep_redtable(1, "Ki_offdiag").Variables;
% K_tmp = ip_scaling(:, :, c_ws_idx) * K_tmp * inv(op_scaling(:, :, c_ws_idx));
% bodeplot(series(K_tmp, Plant(:, :, C_WS_IDX)))
% bodeplot(feedback(series(K_tmp, Plant(:, :, C_WS_IDX)), eye(2), -1))

figure;
for c_ws_idx = unique(PI_ParameterSweep(1).Gains(:, 'WindSpeedIdx'))
    stable_gains{c_ws_idx} = PI_ParameterSweep.Gains(:, PI_ParameterSweep.Stable == 1, c_ws_idx);
    scatter(1:size(PI_ParameterSweep.Gains, 1), log10(stable_gains{c_ws_idx}));
    hold on;
end
hold off;

% stable_gains = Gains(:, all(DMo > 0, 3));

% Plot variation in MMIO vs. variation in each of parameters
figure;
tcf = tiledlayout(2, 2);
tol = 1e-13;

Sweep.Baseline.Kp_diag = -1;
Sweep.Baseline.Kp_offdiag = -1;
Sweep.Baseline.Ki_diag = -1;
Sweep.Baseline.Ki_offdiag = -1;
    

% for c_ws_idx = [find(C_WS_IDX)]
    Gains = PI_ParameterSweep.Gains;
    DMs = PI_ParameterSweep_table(:, "Mean DM").Variables;
    Kp_diag_idx = find((abs(Gains(2, :) - Sweep.Baseline.Kp_offdiag) < tol) ...
        .* (abs(Gains(3, :) - Sweep.Baseline.Ki_diag) < tol) ...
        .* (abs(Gains(4, :) - Sweep.Baseline.Ki_offdiag) < tol));
    Kp_offdiag_idx = find((abs(Gains(1, :) - Sweep.Baseline.Kp_diag) < tol) ...
        .* (abs(Gains(3, :) - Sweep.Baseline.Ki_diag) < tol) ...
        .* (abs(Gains(4, :) - Sweep.Baseline.Ki_offdiag) < tol));
    Ki_diag_idx = find(...
        (abs(Gains(1, :) - Sweep.Baseline.Kp_diag) < tol) ...
        .* (abs(Gains(2, :) - Sweep.Baseline.Kp_offdiag) < tol) ...
        .* (abs(Gains(4, :) - Sweep.Baseline.Ki_offdiag) < tol));
    Ki_offdiag_idx = find( ...
        (abs(Gains(1, :) - Sweep.Baseline.Kp_diag) < tol) ...
        .* (abs(Gains(2, :) - Sweep.Baseline.Kp_offdiag) < tol) ...
        .* (abs(Gains(3, :) - Sweep.Baseline.Ki_diag) < tol));

    nexttile(1);
    scatter(log10(Gains(1, Kp_diag_idx)), DMs(Kp_diag_idx));
    hold on;
    title('Variation in Mean Disk Margin vs. k^p_d');
    nexttile(2);
    scatter(log10(Gains(2, Kp_offdiag_idx)), DMs(Kp_offdiag_idx));
    hold on;
    title('Variation in Mean Disk Margin vs. k^p_{od}');
    nexttile(3);
    scatter(log10(Gains(3, Ki_diag_idx)), DMs(Ki_diag_idx));
    hold on;
    title('Variation in Mean Disk Margin vs. k^i_{d}');
    nexttile(4);
    scatter(log10(Gains(4, Ki_offdiag_idx)), DMs(Ki_offdiag_idx));
    hold on;
    title('Variation in Mean Disk Margin vs. k^i_{od}');
% end
    % legend(arrayfun(@(n) [num2str(n) ' m/s'], LPV_CONTROLLER_WIND_SPEEDS, 'UniformOutput', false));
    set(gcf, 'Position', [0 0 1500 900]);
    savefig(gcf, fullfile(fig_dir, 'param_sweep.fig'));
    saveas(gcf, fullfile(fig_dir, 'param_sweep.png'));
    
    % Plot hinfnorm, h2norm {'KSi', 'So', 'To'}

    for p = 1:length(Sweep.Params)
        figure;
        tcl = tiledlayout(4, 1);

            % ax = nexttile(1);
            % plot(Sweep.Vary.(Sweep.Params{p}), Sweep.KSi_hinfnorm.(Sweep.Params{p}));
            % title(ax, 'KS_i hinfnorm');
            % 
            % ax = nexttile(2);
            % plot(Sweep.Vary.(Sweep.Params{p}), Sweep.KSi_h2norm.(Sweep.Params{p}));
            % title(ax, 'KS_i h2norm');

        ax = nexttile(1);
        plot(Sweep.Vary.(Sweep.Params{p}), Sweep.So_hinfnorm.(Sweep.Params{p}));
        title(ax, 'S_o hinfnorm');

        % ax = nexttile(4);
        % plot(Sweep.Vary.(Sweep.Params{p}), Sweep.So_h2norm.(Sweep.Params{p}));
        % title(ax, 'S_o h2norm');

        ax = nexttile(2);
        plot(Sweep.Vary.(Sweep.Params{p}), Sweep.To_hinfnorm.(Sweep.Params{p}));
        title(ax, 'T_o hinfnorm');

        % ax = nexttile(6);
        % plot(Sweep.Vary.(Sweep.Params{p}), Sweep.To_h2norm.(Sweep.Params{p}));
        % title(ax, 'T_o h2norm');

        ax = nexttile(3);
        plot(Sweep.Vary.(Sweep.Params{p}), Sweep.Si_hinfnorm.(Sweep.Params{p}));
        title(ax, 'S_i hinfnorm');

        % ax = nexttile(8);
        % plot(Sweep.Vary.(Sweep.Params{p}), Sweep.Si_h2norm.(Sweep.Params{p}));
        % title(ax, 'S_i h2norm');

        ax = nexttile(4);
        plot(Sweep.Vary.(Sweep.Params{p}), Sweep.Ti_hinfnorm.(Sweep.Params{p}));
        title(ax, 'T_i hinfnorm');

        % ax = nexttile(10);
        % plot(Sweep.Vary.(Sweep.Params{p}), Sweep.Ti_h2norm.(Sweep.Params{p}));
        % title(ax, 'T_i h2norm');

        % ax = nexttile(11);
        % plot(Sweep.Vary.(Sweep.Params{p}), Sweep.GSo_hinfnorm.(Sweep.Params{p}));
        % title(ax, 'GS_o hinfnorm');

        % ax = nexttile(12);
        % plot(Sweep.Vary.(Sweep.Params{p}), Sweep.GSo_h2norm.(Sweep.Params{p}));
        % title(ax, 'GS_o h2norm');

        title(tcl, ['Structured Controller Gain Sweep for ', Sweep.Params{p}]);
        set(gcf, 'Position', [0 0 1500 900]);
    end
end

%% Conduct MIMO Parameter Sweep for Structured Controller
if 0

% run a sweep for each of the parameters, keeping all other params constant
% at their baseline values
Sweep.Baseline.Kp_diag = Kp_diag.Value;
Sweep.Baseline.Kp_offdiag = Kp_offdiag.Value;
Sweep.Baseline.Ki_diag = Ki_diag.Value;
Sweep.Baseline.Ki_offdiag = Ki_offdiag.Value;
% Sweep.Baseline.zeta_1_diag = zeta_1_diag.Value;
% Sweep.Baseline.zeta_1_offdiag = zeta_1_offdiag.Value;
% Sweep.Baseline.fac_2_diag = fac_2_diag.Value;
% Sweep.Baseline.fac_2_offdiag = fac_2_offdiag.Value;

approx_Kp = load(fullfile(mat_save_dir, 'p_ctrl_range.mat'));
approx_Ki = load(fullfile(mat_save_dir, 'i_ctrl_range.mat'));

Sweep.Vary.Kp_diag = linspace(1e-8, 1e-4, 10); % linspace(approx_Kp.KD_range(1), approx_Kp.KD_range(2), 5);
Sweep.Vary.Kp_offdiag = linspace(1e-7, 1e-3, 10); % linspace(approx_Kp.KQD_range(1), approx_Kp.KQD_range(2), 5);
Sweep.Vary.Ki_diag = linspace(1e-8, 1e-4, 10); % linspace(approx_Ki.KD_range(1), approx_Ki.KD_range(2), 5);
Sweep.Vary.Ki_offdiag = linspace(1e-7, 1e-3, 10); % linspace(approx_Ki.KDQ_range(1), approx_Ki.KDQ_range(2), 5);
% Sweep.Vary.zeta_1_diag = logspace(-3, 1, 5);
% Sweep.Vary.zeta_1_offdiag = logspace(-3, 1, 5);
% Sweep.Vary.fac_2_diag = logspace(0, 4, 5);
% Sweep.Vary.fac_2_offdiag = logspace(0, 4, 5);

Sweep.Params = fieldnames(Sweep.Baseline);

omega = logspace(-2, 2, 200);

trans_funcs = {'KSi', 'Ti', 'So', 'GSo', 'To'};
upper_bounds = [];
upper_bounds(1, :, :, :) = bode(inv(Wu), omega);
upper_bounds(2, :, :, :) = bode(inv(Wu), omega);
upper_bounds(3, :, :, :) = bode(inv(We), omega);
upper_bounds(4, :, :, :) = bode(inv(We), omega);
upper_bounds(5, :, :, :) = bode(inv(Wy), omega);

K_grid = combinations(Sweep.Vary.Kp_diag, Sweep.Vary.Kp_offdiag, ...
                             Sweep.Vary.Ki_diag, Sweep.Vary.Ki_offdiag);
K_grid.Properties.VariableNames = {'Kp_diag', 'Kp_offdiag', 'Ki_diag', 'Ki_offdiag'};

% for each gridpoint
parfor v = 1:size(K_grid, 1)    
    K_tmp = K_struct;
    for p = 1:length(Sweep.Params)
%         val = Sweep.Vary.(Sweep.Params{p})(v);
        val = K_grid(v, Sweep.Params{p}).Variables;
        K_tmp.Blocks.(Sweep.Params{p}).Value = val;
    end    

end

% Plot hinfnorm, h2norm {'KSi', 'So', 'To'}
if 1
    for p = 1:length(Sweep.Params)
        figure;
        tcl = tiledlayout(6, 2);

        ax = nexttile(1);
        plot(Sweep.Vary.(Sweep.Params{p}), Sweep.KSi_hinfnorm.(Sweep.Params{p}));
        title(ax, 'KS_i hinfnorm');

        ax = nexttile(2);
        plot(Sweep.Vary.(Sweep.Params{p}), Sweep.KSi_h2norm.(Sweep.Params{p}));
        title(ax, 'KS_i h2norm');

        ax = nexttile(3);
        plot(Sweep.Vary.(Sweep.Params{p}), Sweep.So_hinfnorm.(Sweep.Params{p}));
        title(ax, 'S_o hinfnorm');

        ax = nexttile(4);
        plot(Sweep.Vary.(Sweep.Params{p}), Sweep.So_h2norm.(Sweep.Params{p}));
        title(ax, 'S_o h2norm');

        ax = nexttile(5);
        plot(Sweep.Vary.(Sweep.Params{p}), Sweep.To_hinfnorm.(Sweep.Params{p}));
        title(ax, 'T_o hinfnorm');

        ax = nexttile(6);
        plot(Sweep.Vary.(Sweep.Params{p}), Sweep.To_h2norm.(Sweep.Params{p}));
        title(ax, 'T_o h2norm');

        ax = nexttile(7);
        plot(Sweep.Vary.(Sweep.Params{p}), Sweep.Si_hinfnorm.(Sweep.Params{p}));
        title(ax, 'S_i hinfnorm');

        ax = nexttile(8);
        plot(Sweep.Vary.(Sweep.Params{p}), Sweep.Si_h2norm.(Sweep.Params{p}));
        title(ax, 'S_i h2norm');

        ax = nexttile(9);
        plot(Sweep.Vary.(Sweep.Params{p}), Sweep.Ti_hinfnorm.(Sweep.Params{p}));
        title(ax, 'T_i hinfnorm');

        ax = nexttile(10);
        plot(Sweep.Vary.(Sweep.Params{p}), Sweep.Ti_h2norm.(Sweep.Params{p}));
        title(ax, 'T_i h2norm');

        ax = nexttile(11);
        plot(Sweep.Vary.(Sweep.Params{p}), Sweep.GSo_hinfnorm.(Sweep.Params{p}));
        title(ax, 'GS_o hinfnorm');

        ax = nexttile(12);
        plot(Sweep.Vary.(Sweep.Params{p}), Sweep.GSo_h2norm.(Sweep.Params{p}));
        title(ax, 'GS_o h2norm');

        title(tcl, ['Structured Controller Gain Sweep for ', Sweep.Params{p}]);
        set(gcf, 'Position', [0 0 1500 900]);
    end
end

% Plot transfer functions
if 0
for p = 1:length(Sweep.Params)
    for trans_func_i = 1:length(trans_funcs)
        trans_func = trans_funcs{trans_func_i};
        ub = squeeze(upper_bounds(trans_func_i, :, :, :));

        figure;
        tcl = tiledlayout(2, 2);
        for v = 1:length(Sweep.Vary.(Sweep.Params{p}))
            
            mag = squeeze(Sweep.(trans_func).(Sweep.Params{p})(v, :, :, :));
            
            nexttile(1)
            loglog(omega, squeeze(mag(1, 1, :)));
            xlabel('Frequency [rad/sec]'); ylabel('dB'); title('D->D');
            hold on;
            loglog(omega, squeeze(ub(1, 1, :)), 'k--');
        
            nexttile(2)
            loglog(omega, squeeze(mag(1, 2, :)));
            xlabel('Frequency [rad/sec]'); ylabel('dB'); title('Q->D');
            hold on;
            loglog(omega, squeeze(ub(1, 2, :)), 'k--');
        
            nexttile(3)
            loglog(omega, squeeze(mag(2, 1, :)));
            xlabel('Frequency [rad/sec]'); ylabel('dB'); title('D->Q');
            hold on;
            loglog(omega, squeeze(ub(2, 1, :)), 'k--');
        
            nexttile(4)
            loglog(omega, squeeze(mag(2, 2, :)));
            xlabel('Frequency [rad/sec]'); ylabel('dB'); title('Q->Q');
            hold on;
            loglog(omega, squeeze(ub(2, 2, :)), 'k--');

        end
        title(tcl, [trans_func, ' Frequency Response Structured Controller Gain Sweep for ', Sweep.Params{p}]);
        legend(arrayfun(@(s) num2str(s), Sweep.Vary.(Sweep.Params{p}), 'UniformOutput', false));
        set(gcf, 'Position', [0 0 1500 900]);
    end
end
end
end

%% Synthesize PI+Notch+LPF Continuous-Time Controller Structure with hinfstruct
% gamma_struct is best closed-loop H-infinity norm
% TODO full-order first, the describe w/ minimum number of filters for
% structured controller
if 0
    parfor c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
        opt = hinfstructOptions('Display', 'iter'); %, 'TolGain', 1e-6);
        [K_tmp, gamma_tmp, opt_info] = hinfstruct(GenPlant(:, :, c_ws_idx), K_struct, opt); % lft(P, PI_DQ)

        StructuredTuning(c_ws_idx).K = ss(K_tmp);
        StructuredTuning(c_ws_idx).gamma = gamma_tmp;
        
    end
    for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
        K_struct_tuned(:, :, c_ws_idx) = StructuredTuning(c_ws_idx).K;
        gamma_struct_tuned(c_ws_idx) = StructuredTuning(c_ws_idx).gamma;
    end
    K_struct_tuned = ss(K_struct_tuned);
    K_struct_tuned.SamplingGrid = struct('u', LPV_CONTROLLER_WIND_SPEEDS);
    save(fullfile(mat_save_dir, 'StructuredTuning.mat'), "K_struct_tuned", "gamma_struct_tuned");

else
    load(fullfile(mat_save_dir, 'StructuredTuning.mat'));
end

for c_idx = 1:Controllers_n_cases
    if ~strcmp(Controllers_case_list(c_idx).Structure.x, 'Structured')
        continue;
    end
    % Add LPV controller to Controller_list
    if strcmp(Controllers_case_list(c_idx).Scheduling.x, 'Yes')
        Controllers_case_list(c_idx).Controller = K_struct_tuned;
    % Add nonLPV controller to Controller_list
    elseif strcmp(Controllers_case_list(c_idx).Scheduling.x, 'No')
        Controllers_case_list(c_idx).Controller = K_struct_tuned(:, :, LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED);
    end
end
save(fullfile(mat_save_dir, 'Controllers_case_list.mat'), "Controllers_case_list");

% QUESTION need to filter wind speed for LPV parameter input?


%% Plot frequency response of Controllers
if 0
   
    load(fullfile(mat_save_dir, 'K0.mat'));
    load(fullfile(mat_save_dir, 'Controllers_case_list.mat'));
    
    figure;
    bcol = copper(length(LPV_CONTROLLER_WIND_SPEEDS)); % Define the color order based on the number of models
    rcol = jet(length(LPV_CONTROLLER_WIND_SPEEDS));
    ycol = hot(length(LPV_CONTROLLER_WIND_SPEEDS)); % https://www.mathworks.com/help/matlab/colors-1.html?s_tid=CRUX_lftnav

    omega = logspace(-2, 4, 300);
    
    for c_ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
        if LPV_CONTROLLER_WIND_SPEEDS(c_ws_idx) ~= NONLPV_CONTROLLER_WIND_SPEED
            continue;
        end
        % Plot baseline and tuned controllers
        bodeplot(K0, K_struct_tuned(:, :, c_ws_idx), K_full_tuned(:, :, c_ws_idx), omega, bode_plot_opt);

        % Find handles of all lines in the figure that have the color blue
        blineHandle = findobj(gcf,'Type','line','-and','Color','b');
        rlineHandle = findobj(gcf,'Type','line','-and','Color','r');
        ylineHandle = findobj(gcf,'Type','line','-and','Color','y');

        % Change the color to the one you defined
        set(blineHandle,'Color',bcol(c_ws_idx,:));
        set(rlineHandle,'Color',rcol(c_ws_idx,:));
        set(ylineHandle,'Color',ycol(c_ws_idx,:));
        
        hold on
    end
    
    axh = findall(gcf, 'type', 'axes');
    xline(axh(3), omega_1P_rad * HARMONICS);
    xline(axh(5), omega_1P_rad * HARMONICS);
    xline(axh(7), omega_1P_rad * HARMONICS);
    xline(axh(9), omega_1P_rad * HARMONICS);
    set(gcf, 'Position', [0 0 1500 900]);
    legend('Structured Baseline', 'Structured Tuned', 'Full-Order Tuned', '', '', '', '');
    %         title('Frequency Response of Tuned Controllers');
    hold off;

    
    savefig(gcf, fullfile(fig_dir, 'K_bodemag.fig'));
    saveas(gcf, fullfile(fig_dir, 'K_bodemag.png'));

    figure;
    SF_0 = loopsens(...
            inv(op_scaling(:, :, c_ws_idx)) ...
            * Plant(:, :, c_ws_idx) ...
            * ip_scaling(:, :, c_ws_idx), K0);
    % SF_ol = loopsens(Plant(:, :, C_WS_IDX), K0); % QUESTION how to
    % compute this?
    SF_struct = loopsens(...
            Plant_scaled(:, :, c_ws_idx), K_struct_tuned(:, :, C_WS_IDX));
    SF_full = loopsens(...
            Plant_scaled(:, :, c_ws_idx), K_full_tuned(:, :, C_WS_IDX));
    % QUESTION frequency the larges peak (hinf norm) occurs
    sigmaplot(SF_0.So, SF_struct.So, SF_full.So);
    legend('Baseline', 'Structured Tuned', 'Full-Order Tuned');
    set(gcf, 'Position', [0 0 1500 900]);
    savefig(gcf, fullfile(fig_dir, 'sigma_bodemag.fig'));
    saveas(gcf, fullfile(fig_dir, 'sigma_bodemag.png'));

end

%% Generate Latex Table of step/impulse response characteristics
if 0
% ismember(GenPlant(C_WS_IDX).InputName, {'RootMycD Disturbance', 'RootMycQ Disturbance'})
% ismember(GenPlant(C_WS_IDX).InputName, {'Blade D pitch command', 'Blade Q pitch command'})
% ismember(GenPlant(C_WS_IDX).OutputName, {'RootMycD Output', 'RootMycQ Output'})
% OL0 = GenPlant(C_WS_IDX)(...
%     ismember(GenPlant(C_WS_IDX).OutputName, {'RootMycD Output', 'RootMycQ Output'}), ...
%     ismember(GenPlant(C_WS_IDX).InputName, {'Blade D pitch command', 'Blade Q pitch command'}));


[Y, T] = step(CL0);
ess0 = Y(end, [3, 4], [1, 2]);
ess0 = squeeze(ess0);
step0 = stepinfo(CL0);
step0 = step0([3, 4], [1, 2]);
% D->D, D->Q, Q->D, Q->Q
step0_tab = [...
    struct2table(step0(1, 1)); ...
    struct2table(step0(2, 1)); ...
    struct2table(step0(1, 2)); ...
    struct2table(step0(2, 2))];
step0_tab = step0_tab(:, {'RiseTime', 'SettlingMin', 'SettlingTime', 'Overshoot', 'PeakTime'});
step0_tab.Properties.VariableNames = {'$t_r$', '$e_{ss}$', '$t_s$', '$M_p$', '$t_p$'};
step0_tab.Row = {'$M_d$ Dist $\rightarrow$ $M_d$ Error', ...
                 '$M_d$ Dist $\rightarrow$ $M_q$ Error', ...
                 '$M_q$ Dist $\rightarrow$ $M_d$ Error', ...
                 '$M_q$ Dist $\rightarrow$ $M_q$ Error'};
step0_tab(:, '$e_{ss}$') = table([ess0(1,1); ess0(2, 1); ess0(1, 2); ess0(2, 2)]);


table2latex(step0_tab, fullfile(results_mat_save_dir, 'step0.tex'));

[Y, T] = step(CL_full_tuned);
ess_full_tuned = Y(end, [3, 4], [1, 2]);
ess_full_tuned = squeeze(ess_full_tuned);
step_full_tuned = stepinfo(CL_full_tuned);
step_full_tuned = step_full_tuned([3, 4], [1, 2]);
% D->D, D->Q, Q->D, Q->Q
step_full_tuned_tab = [...
    struct2table(step_full_tuned(1, 1)); ...
    struct2table(step_full_tuned(2, 1)); ...
    struct2table(step_full_tuned(1, 2)); ...
    struct2table(step_full_tuned(2, 2))];
step_full_tuned_tab = step_full_tuned_tab(:, {'RiseTime', 'SettlingMin', 'SettlingTime', 'Overshoot', 'PeakTime'});
step_full_tuned_tab.Properties.VariableNames = {'$t_r$', '$e_{ss}$', '$t_s$', '$M_p$', '$t_p$'};
step_full_tuned_tab.Row = {'$M_d$ Dist $\rightarrow$ $M_d$ Error', ...
                 '$M_d$ Dist $\rightarrow$ $M_q$ Error', ...
                 '$M_q$ Dist $\rightarrow$ $M_d$ Error', ...
                 '$M_q$ Dist $\rightarrow$ $M_q$ Error'};
step_full_tuned_tab(:, '$e_{ss}$') = table([ess_full_tuned(1,1); ess_full_tuned(2, 1); ess_full_tuned(1, 2); ess_full_tuned(2, 2)]);
table2latex(step_full_tuned_tab, fullfile(results_mat_save_dir, 'step_full_tuned_tab.tex'));

[Y, T] = step(CL_struct_tuned);
ess_struct_tuned = Y(end, [3, 4], [1, 2]);
ess_struct_tuned = squeeze(ess_struct_tuned);
step_struct_tuned = stepinfo(CL_struct_tuned);
step_struct_tuned = step_struct_tuned([3, 4], [1, 2]);
% D->D, D->Q, Q->D, Q->Q
step_struct_tuned_tab = [...
    struct2table(step_struct_tuned(1, 1)); ...
    struct2table(step_struct_tuned(2, 1)); ...
    struct2table(step_struct_tuned(1, 2)); ...
    struct2table(step_struct_tuned(2, 2))];
step_struct_tuned_tab = step_struct_tuned_tab(:, {'RiseTime', 'SettlingMin', 'SettlingTime', 'Overshoot', 'PeakTime'});
step_struct_tuned_tab.Properties.VariableNames = {'$t_r$', '$e_{ss}$', '$t_s$', '$M_p$', '$t_p$'};
step_struct_tuned_tab.Row = {'$M_d$ Dist $\rightarrow$ $M_d$ Error', ...
                 '$M_d$ Dist $\rightarrow$ $M_q$ Error', ...
                 '$M_q$ Dist $\rightarrow$ $M_d$ Error', ...
                 '$M_q$ Dist $\rightarrow$ $M_q$ Error'};
step_struct_tuned_tab(:, '$e_{ss}$') = table([ess_struct_tuned(1,1); ess_struct_tuned(2, 1); ess_struct_tuned(1, 2); ess_struct_tuned(2, 2)]);
table2latex(step_struct_tuned_tab, fullfile(results_mat_save_dir, 'step_struct_tuned_tab.tex'));

% [DM_L0, MM_L0] = diskmargin(-L0);
% [DM_L_full_tuned, MM_L_full_tuned] = diskmargin(-L_full_tuned);
% [DM_L_struct_tuned, MM_L_struct_tuned] = diskmargin(-L_struct_tuned);
% % Each Feedback Channel Disk Margin
% DM_tab = array2table([...
%     DM_L0(1).DiskMargin, DM_L0(2).DiskMargin; ...
%     DM_L_struct_tuned(1).DiskMargin, DM_L_struct_tuned(2).DiskMargin; ...
%     DM_L_full_tuned(1).DiskMargin, DM_L_full_tuned(2).DiskMargin], ...
%     'VariableNames', {'RootMycD Disk Margin', 'RootMycQ Disk Margin'}, ...
%     'RowNames', {'Baseline', 'Structured', 'Full-Order'});
% table2latex(DM_tab, fullfile(results_mat_save_dir, 'DM_tab.tex'));

end


%% Questions
% how to enforce structure of controller ie PI
% => Ossman did full-order controller design (same order as plant more complex), 
% we could look at MIMO structured, need to use hinfstruct (contained by
% systune)

% how to implement low vs high order controller?
% => structured (ie PI) => convex problem vs. non-structured (plant/full)
% => nonlinear, nonsmooth, starting points matter, need to minimize number
% of variables bc solutions will not converge
% how to consider tf for deviations of Mdq and Betadq AND absolute values ?
% => define additional z output and put weights on output
% => define artificial output y_beta = 0x + 1u
% => dy_beta/dt = Cdx/dt + Ddu/dt = C(Ax + Bu) + D du/dt
% => Check D matrix. At infinite gain, for real systems, there is zero
% gain. Can make zero by adding low-pass filter bc required for many
% algorithms w/ cutoff frequency just above system dynamics.
% Getting drvt of gain is equiv to multi;lying by s, equiv to rotation in
% Bode plot => can shape frequency response of gain to shape rate of gain
% Can make weighting filter that shapes gain

% how to implement LPV vs non-LPV?
% non-LPV = one LTI, non-varying controller for all wind speeds (const
% gains for all wind speeds)
% Better performance with gain-scheduling, LPV is one approach
% First, establish control design at one wind-speed for as long as
% possible,
% then extend to multiple wind speeds
% (i) test that controller for full range of wind speeds
% (ii) design controller at different wind speed and test for full range
% (iii) tune LTI controllers for different wind speeds, interpolate
% inbetween. Investigate how they are different. Best performance at partic
% wind speed is one from controller tuned at that wind speed
% (iv) Interpolation step. Assume basis function for gain 
% e.g. linear model k_p = kp0 + v kp1 and tune kp0 and kp1 
%  => use regression to approximate kp with minimum number of coefficients
% increase order of basis function incrementally and compare scheduling
% impact

% how to implement MIMO vs SISO?
% SISO: eye block controller, (i) individual blocks by just consider d or q
% components, then together tuning by consider dq system

% how to structure discrete-time generalized plant P, are disturbances
% considered ?
% what margins to use for MIMO system

% Including filters in tuning process, weighting filters are tunable

% TODO
% Structured SISO PI with hinfstruct
% => keep Overleaf with Problem Formulation & Notes
% Read Tutorial
% Read IPC Ossman papers
% Read Multivariable Feedback Hinf control

