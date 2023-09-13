function [data] = getData(OutData_values, OutList, op) 
    data = OutData_values(:, strmatch(op, OutList));
end