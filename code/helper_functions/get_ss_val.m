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