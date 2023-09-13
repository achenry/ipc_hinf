function [normalizedValues] = normalizeOutData(OutData_values, norm_factors)
    normalizedValues = OutData_values ./ norm_factors;
end