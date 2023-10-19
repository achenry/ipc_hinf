function peaks = computeFFTPeaks(fft_vals, type, DT, base_f, harmonics)
    peaks.harmonics = harmonics;
    peaks.base_f = base_f;

    L = size(fft_vals, 1);
    f = (1 / (DT * L)) * (0:L/2);
    x = (1 / L) * abs(fft_vals(1:L/2 + 1, :));
    x(2:end-1, :) = 2 * x(2:end-1, :);
    peaks.(type) = zeros(length(harmonics), size(x, 2));

    for h = 1:length(harmonics)
        har = harmonics(h);
        idx_rng = (f > (base_f * max((har - 0.5), 0))) & (f < (base_f * (har + 0.5)));
        peaks.(type)(h, :) = max(x(idx_rng, :), [], 1);
    end
end