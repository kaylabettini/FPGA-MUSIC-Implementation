clear all; close all; clc;

Na=4; % Number of antennas/sensors
Ns=1; % number of sources

% Folders with the recordings from the Pi (the 4-channel 8 kHz *_sent.wav
% files = exactly what the FPGA received)
folders = {'synthetic'};
skip = 2;          % seconds dropped at the start (same as the FPGA results)
makePlots = false; % true = plot the signals and MUSIC spectrum for every file

%Using range of 300-3400 for human speech
%Wideband: run the narrowband MUSIC in 100 Hz bands from 300 to 3400 Hz
%and add the spectra together. The band centres include 600, 1000, 1400,
%2200 and 3000 Hz exactly, so the test tones sit in the middle of a band.

bandSet = 'hfma';   % 'wide' = every 100 Hz from 300-3400 Hz,  'hfma' = only the 4 HFMA bands
if strcmp(bandSet, 'wide')
    f0s = 300:100:3400; bw = 100;
else
    f0s = [600 1400 2200 3000]; bw = 180;   % the HFMA's 4 bins; 180 Hz ≈ its 90 Hz low-pass after mixing
end

% f0s = 300:100:3400; %band centres
% bw = 100;           %passband width
out = fopen(['music_doa_' bandSet '.csv'], 'w');
fprintf(out, 'type,angle,trial,estimate\n');

for fo = 1:numel(folders)
files = dir(fullfile(folders{fo}, '*_sent.wav'));
for k = 1:numel(files)
    name = files(k).name;

    % Work out the signal type, angle and trial number from the file name
    % synthetic: syn_1400_m60deg_sent.wav   acoustic: m60deg_pink_t2_sent.wav
    t = regexp(name, '^syn_(.+)_(m?\d+)deg_sent', 'tokens', 'once');
    if ~isempty(t)
        type = ['syn_' t{1}]; ang = t{2}; trial = 1;
    else
        t = regexp(name, '^(m?\d+)deg_(.+)_t(\d+)_sent', 'tokens', 'once');
        if isempty(t), continue; end
        ang = t{1}; type = t{2}; trial = str2double(t{3});
    end
    ang = str2double(strrep(ang, 'm', '-'));

    [sig,fs]=audioread(fullfile(folders{fo}, name)); % 4 columns = Mic1..Mic4, already 8 kHz
    fs_actual = fs;
    sig = sig(round(skip*fs)+1:end, :); % drop the first 2 s

    % Plot the signal
    if makePlots
        t_sig = (0:size(sig,1)-1) / fs_actual;
        figure;
        plot(t_sig, sig(:,1), t_sig, sig(:,2), t_sig, sig(:,3), t_sig, sig(:,4));
        grid on;
        xlabel('Time (s)');
        ylabel('Amplitude');
        title('Signals Received at Four Microphones');
        legend('Mic 1','Mic 2','Mic 3','Mic 4');
    end

    %BP filter so that we can apply narrowband music algorithm, once per band
    scanAngles = -90:0.1:90;
    spectrumSum = zeros(length(scanAngles), 1);
    power = zeros(numel(f0s), 1);
    res = zeros(length(scanAngles), numel(f0s));
    for b = 1:numel(f0s)
        if strcmp(bandSet, 'wide')
            use = power >= max(power)/100; 
        else
            use = true(size(power)); 
        end
        f0 = f0s(b);
        [bf, af] = butter(4, [(f0-bw/2) (f0+bw/2)]/(fs_actual/2), 'bandpass');
        sigB = filtfilt(bf, af, sig);
        power(b) = mean(sigB(:).^2);
        [~, ~, res(:,b)] = musicDOA(sigB, fs_actual, f0, Ns);
    end
    %only use bands with real signal in them (within 20 dB of the strongest),
    %so a single tone is not drowned out by the empty bands
    use = power >= max(power)/100;
    spectrumSum = sum(res(:, use), 2);
    spectrumSum = spectrumSum/max(spectrumSum);
    [~, idx] = max(spectrumSum);
    DOA = scanAngles(idx);

    fprintf(out, '%s,%d,%d,%.1f\n', type, ang, trial, DOA);
    fprintf('%-30s true %4d   MUSIC %6.1f\n', name, ang, DOA);

    %MUSIC PLOTTING
    if makePlots
        figure;
        plot(scanAngles, 10*log10(spectrumSum), 'LineWidth', 1.5);
        grid on;
        xlabel('Angle (degrees)');
        ylabel('Normalised MUSIC Spectrum (dB)');
        title(sprintf('%s - Estimated DOA = %.1f°', strrep(name,'_','\_'), DOA));
        xlim([-90 90]);
        ylim([-40 0]);
    end
end
end
fclose(out);
disp('Saved music_doa.csv');

% This function emulates a 2D planar wave front received
% (not used for the recordings, kept for generating test signals)
function [w] = ULA_planewave(x,DOA, Na, fs)

    c = 343;   %speed of sound
    d = 0.042; %distance between microphones

    Tau  = (d/c)*sin(DOA); % fixed time delay between consecutive elements

    lh=length(x);
    f = (-lh/2 : lh/2-1)' * (fs/lh); % Frequency bin vector
    omega = 2*pi*f;
    w=zeros(lh,Na);
    X=fftshift(fft(x)); % FFT of the input
    w(:,1) = x;
    % Applying fractional time delays to the input signal to generate
    % signal at each sensor.
    % 1st sensor: x(t)
    % 2nd sensor: x(t-tau)
    % 3rd sensor: x(t-2*tau)
    % Last sensor: x(t-(Na-1)*tau)
    % Here, tau is the fractional time delay given by
    % tau=(dx/c)*sin(DOA);
    for p = 2:Na
        delay = (p-1)*Tau;
        kk = X .* exp(-1j*omega*delay);
        w(:,p) = ifft(ifftshift(kk),'symmetric');
    end
end

%Calculates the MUSIC Algorithm & Outputs the DOA
function [DOA, res_dB, res] = musicDOA(sig, fs, f0, Ns)
    M = size(sig, 2);
    p = size(sig, 1);

    %start of MUSIC code
    sig_analytic = hilbert(sig);
    x = sig_analytic.'; %transpose

    %%MUSIC STEP 1: COVARIANCE MATRIX
    R = (x*x')/p;

    %%STEP 2: Eigenvalue Decomposition
    [V, D] = eig(R);
    [~, idx] = sort(real(diag(D)), 'ascend');
    V = V(:, idx);

    noiseSub = V(:, 1:M-Ns); %Noise subspace

    %%STEP 3: MUSIC angle scan
    scanAngles = -90:0.1:90;
    omega0 = 2*pi*f0;
    res = zeros(length(scanAngles), 1);

    c = 343;
    d = 0.042;

    for i = 1:length(scanAngles)
        theta = deg2rad(scanAngles(i));
        Tau = (d/c)*sin(theta);
        a = exp(-1j*omega0*Tau*(0:M-1)');
        res(i) = 1/(norm(a' * noiseSub)^2);
    end

    % Normalise
    res = res/max(res);
    res_dB = 10*log10(res);

    % Peak(s)
    [~,idx] = sort(res,'descend');
    DOA = scanAngles(idx(1:Ns));
end