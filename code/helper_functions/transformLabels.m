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