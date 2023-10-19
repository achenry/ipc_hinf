function plotSpectra(...
    blade_fft, dq_fft, blade_outputs, dq_outputs, plot_type, DT, harmonics)
    figure;
    ax1 = subplot(2, 1, 1);
    
    L = size(values, 1);
    % t = (0:L-1)*DT;
    f = (1 / (DT * L)) * (0:L/2);
    
    if strcmp(plot_type, 'fft')
        P2 = (1 / L) * abs(blade_fft);
     elseif strcmp(plot_type, 'psd')
        P2 = (1 / (DT * L)) * abs(blade_fft).^2;
    end
    P1 = P2(1:L/2 + 1, :);
    P1(2:end-1, :) = 2 * P1(2:end-1, :);
    if strcmp(plot_type, 'fft')
        plot(f, P1,"LineWidth", 2);
        title("Blade Single-Sided Amplitude Spectrum", 'Interpreter', 'latex')
     elseif strcmp(plot_type, 'psd')
        plot(f, mag2db(P1),"LineWidth", 2);
        title("Blade Single-Sided Amplitude PSD [dB/Hz]", 'Interpreter', 'latex')
    end

    xline(harmonics);
    xlim([0, harmonics(end) * 1.25])
    xlabel(['$\omega$', ' (rad/s)'], 'Interpreter', 'latex')
        
    ax2 = subplot(2, 1, 2);
    L = size(dqValues, 1);
    % t = (0:L-1)*DT;
    f = (1 / (DT * L)) * (0:L/2) * 2 * pi;
    if strcmp(plot_type, 'fft')
        P2 = (1 / L) * abs(dq_fft);
     elseif strcmp(plot_type, 'psd')
        P2 = (1 / (DT * L)) * abs(dq_fft).^2;
    end

    P1 = P2(1:L/2 + 1, :);
    P1(2:end-1, :) = 2 * P1(2:end-1, :);

    if strcmp(plot_type, 'fft')
        plot(f, P1,"LineWidth", 2);
        title("DQ Single-Sided Amplitude Spectrum", 'Interpreter', 'latex')
     elseif strcmp(plot_type, 'psd')
        plot(f, mag2db(P1),"LineWidth", 2);
        title("DQ Single-Sided Amplitude PSD [dB/Hz]", 'Interpreter', 'latex')
    end

    xline(harmonics);
    xlim([0, harmonics(end) * 1.25])
    xlabel(['$f$', ' [Hz]'], 'Interpreter', 'latex')

    linkaxes([ax1, ax2], 'xy')
end