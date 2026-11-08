clear all; close all; clc;

Na=4;    % Number of antennas/sensors
NN=2^10; % number of time samples taken from the audio signals

Ns=1;           % number of sources
DOA1=30*pi/180; % Direction of arrival of the wave fronts

[s1,fs]=audioread('Audio Lab Sweep/0deg_600_t1_fpga.csv'); % Audio source 1 - female singing

sstart=2000;           % Start time sample index of the audio inputs - to remove the initial silent period
sstop=sstart+NN-1;     % Stop time sample index of the audio input to have NN many samples
s1=s1(sstart:sstop,1); % Resizing the audio inputs to be of the same length

s1=s1/max(abs(s1));    % Normalising the inputs to max amplitude one

% The original signals are sampled at 16KHz, whereas for voice 0-4KHz
% frequency content is sufficient. Thus, we can down sample by a factor of
% 2 (i.e. make the sampling rate 8kHz) to have fewer samples to deal with
s11=resample(s1,1,2); 

ns=0; % Noise amplitude
fs_actual = fs/2;
sig1=ULA_planewave(s11,DOA1,Na, fs_actual); % Emulating the first signal arriving at the array at angle DOA1
noise=ns*rand(size(sig1)); % Additive white noise

NN2 = length(s11);

SS1=zeros(NN2,NN2);
SS1(:,1:Na)=sig1;
SSS1=fftshift(fft2(SS1));
SSS1=SSS1/max(max(abs(SSS1)));
figure
mesh(abs(SSS1));

% "sig" is the final signal received by the array, which consists of one
% audio signal arriving at angle DOA1, another interfereing audio signal
% arrivign at angle DOA2 and additive noise. sig is a matrix of size
% (NN,Na) where each column represents the 1D signal received by a single
% sensor. For example, you can listen to sig(:,Na) to listen to the
% composite signal received by the last microphone in the array. You can
% control the signal to interference ratio (SIR) by changing the amplitude
% factors As and Ai. As=Ai implied SIR=0dB
As=1; Ai=0;
sig=As*sig1+Ai*sig2+ns*noise; 

% Plot the signal
t_sig = (0:size(sig,1)-1) / fs_actual;

figure;
plot(t_sig, sig(:,1), t_sig, sig(:,2), t_sig, sig(:,3), t_sig, sig(:,4));

grid on;
xlabel('Time (s)');
ylabel('Amplitude');
title('Signals Received at Four Microphones');
legend('Mic 1','Mic 2','Mic 3','Mic 4');

%Using range of 300-3400 for human speech
f0 = 1850; %centered around narrowband center of 300hz-3400hz
bw = 300; %passband width

%BP filter so that we can apply narrowband music algorithm & so that we
%dont have to apply the MUSIC algortihm on every frequency bin
[bf, af] = butter(4, [(f0-bw/2) (f0+bw/2)]/(fs_actual/2), 'bandpass');
sig = filtfilt(bf, af, sig);

scanAngles = -90:0.1:90;
[DOA, spectrum] = musicDOA(sig, fs_actual, f0, Ns);

%MUSIC PLOTTING
figure;
plot(scanAngles, spectrum, 'LineWidth', 1.5);
grid on;

xlabel('Angle (degrees)');
ylabel('Normalised MUSIC Spectrum (dB)');
title(sprintf('MUSIC DOA Spectrum - Estimated DOA = %.1f°', DOA));

xlim([-90 90]);
ylim([-40 0]);

% This function emulates a 2D planar wave front received
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
function [DOA, res_dB] = musicDOA(sig, fs, f0, Ns);
    M = size(sig, 2);
    p = size(sig, 1);

    %start of MUSIC code
    sig_analytic = hilbert(sig)
    x = sig_analytic.' %transpose
    
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