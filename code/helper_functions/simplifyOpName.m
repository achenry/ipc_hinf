function [op_out] = simplifyOpName(op_in)
    op_out = split(op_in, ',');
    op_out = split(op_out(1), ' ');
    op_out = join(op_out(2:end), ' ');
    op_out = op_out{1};
end