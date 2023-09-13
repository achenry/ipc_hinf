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