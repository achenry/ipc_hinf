function [transformedValues] = invMbcTransformOutData(OutData_dqValues, dqOutList)
    Az = getData(OutData_dqValues, dqOutList, 'Azimuth');
    data_len = length(getData(OutData_dqValues, dqOutList, 'Time'));
    transformedValues = OutData_dqValues;

    for op_label = dqOutList'
        % if rotating quantity
        
        if strcmp(op_label{1}(end), '1')
            % get all corresponding quantities
            blade_op_labels = cellfun(@(b) [op_label{1}(1:end-1) b], {'1', '2', '3'}, 'UniformOutput', false);
            cdq_op_labels = cellfun(@(b) [op_label{1}(1:end-1) b], {'C', 'D', 'Q'}, 'UniformOutput', false);
            
            % perform inverse MBC transform
            cdq_ops = [getData(OutData_dqValues, dqOutList, cdq_op_labels{1}) ...
                getData(OutData_dqValues, dqOutList, cdq_op_labels{2})...
                getData(OutData_dqValues, dqOutList, cdq_op_labels{3})];
            bl1_comp = arrayfun(@(tt) [1 cosd(Az(tt)) sin(Az(tt))] * cdq_ops(tt, :)', 1:data_len);
            bl2_comp = arrayfun(@(tt) [1 cosd(Az(tt) + 120) sin(Az(tt) + 120)] * cdq_ops(tt, :)', 1:data_len);
            bl3_comp =  arrayfun(@(tt) [1 cosd(Az(tt) + 240) sin(Az(tt) + 240)] * cdq_ops(tt, :)', 1:data_len);

            % replace in transformed signal matrix
            transformedValues(:, ismember(dqOutList, cdq_op_labels)) = [bl1_comp, bl2_comp', bl3_comp'];
%             OutList_op(ismember(OutList_op, blade_op_labels)) = cdq_op_labels;
        end
    end
end