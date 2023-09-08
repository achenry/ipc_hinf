%% Initialize workspace and add paths
close all;
clc;
clear all;

WIND_SPEEDS = 10:0.5:22;

username = char(java.lang.System.getProperty('user.name'));
% char(java.net.InetAddress.getLocalHost.getHostName)
if strcmp(username, 'aoifework')
    home_dir = '/Users/aoifework/Documents';
    
    project_dir = fullfile(home_dir, 'Research', 'ipc_tuning');

    plant_setup_dir = fullfile(project_dir, 'plant_setup_package');
    chdir(plant_setup_dir);
    
    toolbox_dir = fullfile(home_dir, 'toolboxes');
    addpath(fullfile(toolbox_dir, 'PMtools'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox/A_Functions'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox/Utilities'));
    addpath(fullfile(toolbox_dir, 'matlab-toolbox', 'MBC', 'Source'));
    
    lin_models_dir = fullfile(project_dir, 'SOAR-25-V2f_IF', 'linearization', 'steady_wind-CL', 'linfiles');

elseif strmp(username, 'manuel')
    
    plant_setup_dir = fullfile(project_dir, 'plant_setup_package');
    chdir(plant_setup_dir);

    addpath(fullfile('CHANGE ME', 'PMtools')); % MANUEL: add your filepaths here
    addpath(fullfile('CHANGE ME', 'matlab-toolbox/A_Functions'));
    addpath(fullfile('CHANGE ME', 'matlab-toolbox/Utilities'));
    addpath(fullfile('CHANGE ME', 'matlab-toolbox', 'MBC', 'Source')); % Need fx_mbc3 from this toolbox
    lin_models_dir = fullfile('CHANGE ME', 'linfiles');
end

autrun;
CpCtCqFile = 'weis_job_00_Cp_Ct_Cq.txt';
C_BL_SOAR25_V2f_c73_Clean;

% Build and Test Mandar's controller with linear model: 
% original controller gains in Matlab
% 1) tune controller with reduced model
% 2) consider full-order model for linear analysis
% 3) nonlinear analysis

%% OutList for Blade & DQ

if exist('OutList.mat')
    load OutList.mat
end

dqOutList = OutList;
for op_label = OutList'
    % if rotating quantity
    if strcmp(op_label{1}(end), '1')
        % get all corresponding quantities
        blade_op_labels = cellfun(@(b) [op_label{1}(1:end-1) b], {'1', '2', '3'}, 'UniformOutput', false);
        cdq_op_labels = cellfun(@(b) [op_label{1}(1:end-1) b], {'C', 'D', 'Q'}, 'UniformOutput', false);
        
        % replace in transformed signal matrix
        dqOutList(ismember(dqOutList, blade_op_labels)) = cdq_op_labels;
    end
end


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
    
    input_arr = {'Blade C pitch command', ...
            'Blade D pitch command', ...
            'Blade Q pitch command'};
    plotting_idx = find(WIND_SPEEDS == 14);
    
    for ws_idx = 1:length(WIND_SPEEDS)
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
        if ~exist('dqOutList2Lin_idx', 'var')
            dqOutList2Lin_idx = -1 * ones(length(dqOutList), 1);
            for lin_op_idx = 1:length(matData.DescOutput)
                for nonlin_op_idx = 1:length(dqOutList)
                    if contains(matData.DescOutput(lin_op_idx), dqOutList(nonlin_op_idx))
                        dqOutList2Lin_idx(lin_op_idx) = nonlin_op_idx;%find(matData.DescOutput(lin_op_idx), OutList);
                    end
                end
            end
        end
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
        sys.InputName = short_ip_arr';
        sys.OutputName = short_op_arr';
    
        sys = sys(short_op_arr, short_ip_arr);

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
        sys_red = sys(min_op_arr, input_arr);
        
        % remove 2nd bending modes: 
        % Options 
        % a) modred, truncate (removing rows/cols in A, B matrix) 
        % b) modred, matchdc ie residualization (assume states to remove are
        % settled, Enforce matching DC gains)
        % c) TODO regenerate linmodels
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
        
%         bodeplot(sys, sys_red);
%         legend('original', 'reduced');
        
        
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
        sys_arr(:, :, ws_idx) = sys;
        sys_red_arr(:, :, ws_idx) = sys_red;
    
    end
    
    save('/xop_arr', 'xop_arr');
    save('./xop_red_arr', 'xop_red_arr');
    save('./sys_arr', 'sys_arr');
    save('./sys_red_arr', 'sys_red_arr');
end

%% Other Functions
function [op_out] = simplifyOpName(op_in)
    op_out = split(op_in, ',');
    op_out = split(op_out(1), ' ');
    op_out = join(op_out(2:end), ' ');
    op_out = op_out{1};
end

function [transformedLabels] = transformLabels(bLabels)
    transformedLabels = bLabels;
    for op_label = bLabels'
        % if rotating quantity
        if strfind(op_label{1}, 'blade 1,')
            transformedLabels(ismember(transformedLabels, op_label{1})) = {replace(op_label{1}, 'blade 1,', 'C,')};
        elseif strfind(op_label{1}, 'blade 2,')
            transformedLabels(ismember(transformedLabels, op_label{1})) = {replace(op_label{1}, 'blade 2,', 'D,')};
        elseif strfind(op_label{1}, 'blade 3,')
            transformedLabels(ismember(transformedLabels, op_label{1})) = {replace(op_label{1}, 'blade 3,', 'Q,')};
        elseif strfind(op_label{1}, '1, ')
            % get all corresponding quantities
            blade_op_labels = arrayfun(@(b) replace(replace(op_label{1}, '1, ', [num2str(b) ', ']), ' 1 ', [' ' num2str(b) ' ']), [1, 2, 3], 'UniformOutput', false);
            cdq_op_labels = cellfun(@(c) replace(op_label{1}, '1, ', c), {'C, ', 'D, ', 'Q, '}, 'UniformOutput', false);
            
            % replace in output describption
            transformedLabels(ismember(transformedLabels, blade_op_labels)) = cdq_op_labels;
        elseif strfind(op_label{1}, ' 1 ')
            % get all corresponding quantities
            blade_op_labels = arrayfun(@(b) replace(replace(op_label{1}, '(1,', ['(' num2str(b) ',']), ' 1 ', [' ' num2str(b) ' ']), [1, 2, 3], 'UniformOutput', false);
            cdq_op_labels = cellfun(@(c) replace(op_label{1}, ' 1 ', c), {' C ', ' D ', ' Q '}, 'UniformOutput', false);
            
            % replace in output describption
            transformedLabels(ismember(transformedLabels, blade_op_labels)) = cdq_op_labels;
        end
    end
end

function ss_val = get_ss_val(time, data)
    DT = time(2) - time(1);
    [~, locs] = findpeaks(data(end-(50/DT):end), time(end-(50/DT):end));
    if length(locs)
        period = max(diff(locs)) / DT;
    else
        period = 12 / DT;
    end
    tmp = movmean(data, period);
    ss_val = tmp(end);
end

function [ss_vals, op_absmax] = compute_ss_vals(sim_out_list, OutList, dqOutList, Parameters)
    time_data = getData(sim_out_list(1).OutData.signals.values, OutList, 'Time');
    DT = time_data(2) - time_data(1);
    TMax = time_data(end);
    
    figure(1)
    tiledlayout(5, 1);
    title('Time Series');
    labels = {};
    
    l = 1;
    for c = 1:length(sim_out_list)
        sim_out = sim_out_list(c);

        if length(sim_out.ErrorMessage) > 0
            continue;
        end

        wind_speed = mean(getData(sim_out.OutData.signals.values, OutList, 'Wind1VelX'));
    
        labels{l} = num2str(wind_speed);
    
        nexttile(1);
        b = 1;
        plot(time_data, getData(sim_out.OutData.signals.values, OutList, ['BldPitch' num2str(b)]) * (pi / 180));
        hold on;
        title('BldPitch1 [rad]')
    
        nexttile(2);
        plot(time_data, getData(sim_out.OutData.signals.values, OutList, 'RotSpeed') * (2 * pi / 60));
        hold on;
        title('RotSpeed [rad/s]');

        nexttile(3);
        plot(time_data, getData(sim_out.OutData.signals.values, OutList, 'GenTq'));
        hold on;
        title('GenTq [kNm]');

        nexttile(4);
        plot(time_data, getData(sim_out.OutData.signals.values, OutList, 'GenPwr'));
        hold on;
        title('GenPwr [kW]');

        nexttile(5);
        TSR = (getData(sim_out.OutData.signals.values, OutList, 'RotSpeed') * (pi / 30) * Parameters.Turbine.R) ./ getData(sim_out.OutData.signals.values, OutList, 'Wind1VelX');
        plot(time_data, TSR);
        hold on;
        title('TSR [-]');

    
        l = l + 1;
    end
    legend(nexttile(1), labels);
    linkaxes([nexttile(1) nexttile(2)  nexttile(3) nexttile(4) nexttile(5)], 'x');
    xlabel('Time [s]');
    xlim([0 TMax]);
    hold off;
    
    windspeed_ss = [];
    bldpitch_ss = [];
    rotspeed_ss = [];
    tsr_ss = [];
    gentq_ss = [];
    op_absmax.blade = [];
    op_absmax.dq = [];
    c = 1;
    for sim_out = sim_out_list
        if length(sim_out.ErrorMessage) > 0
            continue;
        end
        bldpitch_data = getData(sim_out.OutData.signals.values, OutList, 'BldPitch1') * (pi / 180);
        rotspeed_data = getData(sim_out.OutData.signals.values, OutList, 'RotSpeed') * (pi / 30);
        gentq_data = getData(sim_out.OutData.signals.values, OutList, 'GenTq');
        tsr_data = (getData(sim_out.OutData.signals.values, OutList, 'RotSpeed') * (pi / 30) * Parameters.Turbine.R) ./ getData(sim_out.OutData.signals.values, OutList, 'Wind1VelX');
        
        ss_val = get_ss_val(time_data, bldpitch_data);
        bldpitch_ss = [bldpitch_ss ss_val];

        ss_val = get_ss_val(time_data, rotspeed_data);
        rotspeed_ss = [rotspeed_ss ss_val];
        
        ss_val = get_ss_val(time_data, gentq_data);
        gentq_ss = [gentq_ss ss_val];

        ss_val = get_ss_val(time_data, tsr_data);
        tsr_ss = [tsr_ss ss_val];
        
        wind_speed = mean(getData(sim_out.OutData.signals.values, OutList, 'Wind1VelX'));
        windspeed_ss = [windspeed_ss wind_speed];
        
        op_absmax.blade = [op_absmax.blade; abs(max(sim_out.OutData.signals.values, [], 1,'ComparisonMethod', 'abs'))];
        op_absmax.dq = [op_absmax.dq; abs(max(sim_out.OutData.signals.dqValues, [], 1,'ComparisonMethod', 'abs'))];

        c = c + 1;
    end
    
    [windspeed_ss, sort_idx] = sort(windspeed_ss);
    bldpitch_ss = bldpitch_ss(sort_idx);
    rotspeed_ss = rotspeed_ss(sort_idx);
    gentq_ss = gentq_ss(sort_idx);
    tsr_ss = tsr_ss(sort_idx);
    op_absmax.blade = op_absmax.blade(sort_idx, :);
    op_absmax.dq = op_absmax.dq(sort_idx, :);

    ss_vals.Wind1VelX = windspeed_ss;
    ss_vals.BlPitch1 = bldpitch_ss;
    ss_vals.RotSpeed = rotspeed_ss;
    ss_vals.GenTq = gentq_ss;
    ss_vals.TSR = tsr_ss;

    op_absmax.blade = array2table(op_absmax.blade, 'VariableNames', OutList, 'RowNames', ...
        arrayfun(@(ws) num2str(ws), windspeed_ss, 'UniformOutput', false));
    
    op_absmax.dq = array2table(op_absmax.dq, 'VariableNames', dqOutList, 'RowNames', ...
        arrayfun(@(ws) num2str(ws), windspeed_ss, 'UniformOutput', false));
    
    figure(2)
    tiledlayout(4, 1);
    nexttile
    plot(windspeed_ss, bldpitch_ss);
    title('BldPitch1 [rad]');
    nexttile
    plot(windspeed_ss, rotspeed_ss);
    title('RotSpeed [rad/s]');
    nexttile
    plot(windspeed_ss, gentq_ss);
    title('GenTq [kNm]');
    nexttile
    plot(windspeed_ss, tsr_ss);
    title('TSR [-]');
    linkaxes([nexttile(1) nexttile(2) nexttile(3) nexttile(4)], 'x');
    xlabel('Wind Speed [m/s]');
    xticks(windspeed_ss);
end

function [normalizedValues] = normalizeOutData(OutData_values, norm_factors)
    normalizedValues = OutData_values ./ norm_factors;
end

% function [transformedValues] = mbcTransformBladeOpPoints(OpPoints, StateNames)
%     Az = getData(OutData_values, OutList, 'Azimuth');
%     data_len = length(getData(OutData_values, OutList, 'Time'));
%     transformedValues = OutData_values;
% 
%     for op_label = OutList'
%         % if rotating quantity
%         
%         if strcmp(op_label{1}(end), '1')
%             % get all corresponding quantities
%             blade_op_labels = cellfun(@(b) [op_label{1}(1:end-1) b], {'1', '2', '3'}, 'UniformOutput', false);
%             cdq_op_labels = cellfun(@(b) [op_label{1}(1:end-1) b], {'C', 'D', 'Q'}, 'UniformOutput', false);
%             
%             % perform MBC transform
%             blade_ops = [getData(OutData_values, OutList, blade_op_labels{1}) ...
%                 getData(OutData_values, OutList, blade_op_labels{2})...
%                 getData(OutData_values, OutList, blade_op_labels{3})];
%             c_comp = (1/3) .* sum(blade_ops, 2);
%             d_comp = (2/3) .* arrayfun(@(tt) [cosd(Az(tt)) cosd(Az(tt)+120) cosd(Az(tt)+240)] * blade_ops(tt, :)', 1:data_len);
%             q_comp =  (2/3) .* arrayfun(@(tt) [sind(Az(tt)) sind(Az(tt)+120) sind(Az(tt)+240)] * blade_ops(tt, :)', 1:data_len);
% 
%             % replace in transformed signal matrix
%             transformedValues(:, ismember(OutList, blade_op_labels)) = [c_comp, d_comp', q_comp'];
% %             OutList_op(ismember(OutList_op, blade_op_labels)) = cdq_op_labels;
%         end
%     end
% end

function [transformedValues] = mbcTransformOutData(OutData_values, OutList)
    Az = getData(OutData_values, OutList, 'Azimuth');
    data_len = length(getData(OutData_values, OutList, 'Time'));
    transformedValues = OutData_values;

    for op_label = OutList'
        % if rotating quantity
        
        if strcmp(op_label{1}(end), '1')
            % get all corresponding quantities
            blade_op_labels = cellfun(@(b) [op_label{1}(1:end-1) b], {'1', '2', '3'}, 'UniformOutput', false);
            cdq_op_labels = cellfun(@(b) [op_label{1}(1:end-1) b], {'C', 'D', 'Q'}, 'UniformOutput', false);
            
            % perform MBC transform
            blade_ops = [getData(OutData_values, OutList, blade_op_labels{1}) ...
                getData(OutData_values, OutList, blade_op_labels{2})...
                getData(OutData_values, OutList, blade_op_labels{3})];
            c_comp = (1/3) .* sum(blade_ops, 2);
            d_comp = (2/3) .* arrayfun(@(tt) [cosd(Az(tt)) cosd(Az(tt)+120) cosd(Az(tt)+240)] * blade_ops(tt, :)', 1:data_len);
            q_comp =  (2/3) .* arrayfun(@(tt) [sind(Az(tt)) sind(Az(tt)+120) sind(Az(tt)+240)] * blade_ops(tt, :)', 1:data_len);

            % replace in transformed signal matrix
            transformedValues(:, ismember(OutList, blade_op_labels)) = [c_comp, d_comp', q_comp'];
%             OutList_op(ismember(OutList_op, blade_op_labels)) = cdq_op_labels;
        end
    end
end

function [data] = getData(OutData_values, OutList, op) 
    data = OutData_values(:, strmatch(op, OutList));
end