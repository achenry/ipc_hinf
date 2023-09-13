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