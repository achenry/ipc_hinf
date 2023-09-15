%% Initialize workspace and add paths
close all;
clc;

initialize;

%% Compute maximum absolute values from steady-state simulations (with IPC off) to normalize transfer functions later
if ~exist('op_absmax.mat', 'file')
    load('sim_out_list_ss.mat');
    [ss_vals, op_absmax] = compute_ss_vals(sim_out_list, OutList, dqOutList, Parameters);
    save('op_absmax', 'op_absmax');
else
    load('op_absmax.mat')
end

%% Investigate Sensitivity of Outputs to Individual BldPitch1 Variation for Linear model

% Import and MBC-transform linear models

if false && exist('sys_arr.mat', 'file') && exist('sys_red_arr.mat', 'file') && exist('xop_arr.mat', 'file') && exist('xop_red_arr.mat', 'file')
    load('xop_arr.mat')
    load('xop_red_arr.mat')
    load('sys_arr.mat')
    load('sys_red_arr.mat')
else
    
    all_linfiles = dir(lin_models_dir);
    
    input_arr = {'Blade D pitch command', ...
            'Blade Q pitch command'};
    
    for ws_idx = 1:length(LPV_CONTROLLER_WIND_SPEEDS)
        linfile_prefix = ['lin_' num2str(ws_idx - 1, '%02u') '.'];
        linfiles = {};
        linfile_idx = 1;
        for f_idx = 1:length(all_linfiles)
            filepath = fullfile(lin_models_dir, all_linfiles(f_idx).name);
            if any(strfind(all_linfiles(f_idx).name, linfile_prefix)) && isfile(filepath)
                linfiles{linfile_idx} = filepath;
                linfile_idx = linfile_idx + 1;
            end
        end
        
        % 1,2,3 -> c(0), d(cos), q(sin)
        [MBC, matData, FAST_linData, VTK] = fx_mbc3(linfiles);
    
        % Rename labels
        MBC.DescStates = transformLabels(MBC.DescStates);
        matData.DescCntrlInpt = transformLabels(matData.DescCntrlInpt);
        matData.DescOutput = transformLabels(matData.DescOutput);
    
        sys = ss(matData.AvgA, matData.AvgB, matData.AvgC, matData.AvgD, ...
            ...
            'StateName', MBC.DescStates, ...
            'InputName', matData.DescCntrlInpt, ...
            'OutputName', matData.DescOutput);
        
        % generate indices of OutList outputs corresponding to sys
        % outputs
        % if ~exist('dqOutList2Lin_idx', 'var')
            dqOutList2Lin_idx = -1 * ones(length(dqOutList), 1);
            for lin_op_idx = 1:length(matData.DescOutput)
                for nonlin_op_idx = 1:length(dqOutList)
                    if contains(matData.DescOutput(lin_op_idx), dqOutList(nonlin_op_idx))
                        dqOutList2Lin_idx(lin_op_idx) = nonlin_op_idx;%find(matData.DescOutput(lin_op_idx), OutList);
                    end
                end
            end
        % end
        % remove outputs in sys not present in OutList
        y_drop_idx = dqOutList2Lin_idx == -1;
        sys = sys(~y_drop_idx, :);
        matData.DescOutput = matData.DescOutput(~y_drop_idx);
        dqOutList2Lin_idx = dqOutList2Lin_idx(~y_drop_idx);
        
        sys = sysclean(sys); % zero almost zero elements
        
        az_idx = ismember(MBC.DescStates, 'ED Variable speed generator DOF (internal DOF index = DOF_GeAz), rad');
        sys = modred(sys, az_idx, 'truncate'); % remove azimuth state
        
        u_zero_idx = sum(abs(sys.B), 1) == 0; % remove cols from B corresponding to zero sum, rows from C, D corresponding to zero
        y_zero_idx = (sum(abs(sys.C), 2) == 0) .* (sum(abs(sys.D), 2) == 0);
        exc_ops = {'IfW Wind1VelX, (m/s)', 'IfW Wind1VelY, (m/s)', ...
            'SrvD GenPwr, (kW)', 'SrvD GenTq, (kN-m)', 'ED RotTorq, (kN-m)', ...
            'ED BldPitchC, (deg)', 'ED BldPitchD, (deg)', 'ED BldPitchQ, (deg)'};
        y_zero_idx(ismember(matData.DescOutput, exc_ops)) = 1;
        sys = sys(~y_zero_idx, ~u_zero_idx);
    
        % spy(sys.D) % nonzero elements
    
        op_arr = matData.DescOutput(~y_zero_idx);
        ip_arr = matData.DescCntrlInpt(~u_zero_idx);
        state_arr = MBC.DescStates(~az_idx);
        short_op_arr = cellfun(@(op) simplifyOpName(op), op_arr', 'UniformOutput', false);
        short_ip_arr = cellfun(@(ip) simplifyOpName(ip), ip_arr', 'UniformOutput', false);
        short_state_arr = cellfun(@(state) simplifyOpName(state), state_arr', 'UniformOutput', false);
        
        % remove any outputs not output in nonlinear model
        y_zero_idx = zeros(length(op_arr), 1);
        for op_idx = 1:length(short_op_arr)
            if sum(ismember(dqOutList, short_op_arr(op_idx))) == 0
                y_zero_idx(op_idx) = 1;
            end
        end
    
        sys = sys(~y_zero_idx, :);
        op_arr = op_arr(~y_zero_idx);
        short_op_arr = short_op_arr(~y_zero_idx);
    
        sys.StateName = short_state_arr';
    

        % control inputs u
        min_input_arr = {
            'Blade D pitch command', ...
              'Blade Q pitch command'
                };
        % measured outputs y
        min_op_arr = {
                'RootMycD', ...
                'RootMycQ'
                };

        % sys = sys(short_op_arr, short_ip_arr);
        % sys.InputName = short_ip_arr';
        % sys.OutputName = short_op_arr';
        sys.InputName = min_input_arr';
        sys.OutputName = min_op_arr';
        sys = sys(min_op_arr, min_input_arr);
        sys_red = sys(min_op_arr, min_input_arr);
        
        % remove 2nd bending modes: 
        % Options 
        % a) modred, truncate (removing rows/cols in A, B matrix) 
        % b) modred, matchdc ie residualization (assume states to remove are
        % settled, Enforce matching DC gains)
        % c) regenerate linmodels
        %  and plot bode before and after, confirm that it is not changing much in relevant freq
        % range
        second_bm_states = contains(sys_red.StateName, '2nd');
        sys_red = modred(sys_red, second_bm_states, 'MatchDC');
        
        % lpf state is applied to each output, so new states are added
        max_freq = max(abs(eig(sys))) * 2;
        lpf = tf([max_freq], [1 max_freq]);
        sys = lpf * sys; % zero out D matrix

        max_freq = max(abs(eig(sys_red))) * 2;
        lpf = tf([max_freq], [1 max_freq]);
        sys_red = lpf * sys_red;
%         sys_red = xbalred(sys_red); % balanced-order reduction to compute minimum order plant
        
        
        % TODO reduce system first as much as possible before normalizing and filtering,
    
        % output side normalization
        % TODO if normalization is 0, take 1
        % only care about Mdq, so wait to do normalization until reduced model
        % do not need to normalize if all same units
    %         sys = diag(1 ./ table2array(op_absmax.dq(ws_idx, dqOutList2Lin_idx))) * sys;
    
         % input side normalization TODO
         % beta_dq = maximum pitch angle in degrees (?)
         % don't care about others
    %         sys = sys * diag(1 ./ table2array(op_absmax.dq(ws_idx, dqOutList2Lin_idx)));
    
        xop = matData.Avgxop(~az_idx);

        % additional state added for each output due to low-pass filter
        xop_red = [zeros(size(sys_red.OutputName, 1), 1); xop(~second_bm_states)];
        
        xop_arr(:, ws_idx) = xop;
        xop_red_arr(:, ws_idx) = xop_red;
        Plant(:, :, ws_idx) = sys;
        Plant_red(:, :, ws_idx) = sys_red;
    
    end
    
    % save(fullfile(save_dir, 'xop_arr'), 'xop_arr');
    % save(fullfile(save_dir, 'xop_red_arr'), 'xop_red_arr');
    save(fullfile(save_dir, 'Plant'), 'Plant');
    save(fullfile(save_dir, 'Plant_red'), 'Plant_red');
end

%% PLOTTING LINEAR MODELS
PLOTTING = 0;
plotting_idx = find(LPV_CONTROLLER_WIND_SPEEDS == NONLPV_CONTROLLER_WIND_SPEED);

if PLOTTING
    figure;
    bodeplot(sys_arr(:, :, plotting_idx), sys_red_arr(:, :, plotting_idx));
    legend('w/ second-order bending modes', 'wo/ second-order bending modes');
    

end