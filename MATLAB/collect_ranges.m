%% collect_ranges.m
% Records min/max of every signal in the HFMA model (in double precision),
% merged over several representative test cases, into the Fixed-Point Tool
% run 'BaselineRun'. Run this, then open the Fixed-Point Tool and click
% Propose Data Types.

clear; clc;
mdl     = 'HFMA_OPTIMISED_BUTTERWORTH_TEST';
RUNNAME = 'BaselineRun'; % must match the run shown in the Fixed-Point Tool

load_system(mdl);
load('a_luts.mat'); load('iq_luts.mat'); load('lpf_sos.mat');
fs = 8000; Ts = 1/fs; N = 4096; k_avg = 10; Na = 4; NN = 4096;

% --- Instrumentation settings -------------------------------------------
set_param(mdl, 'SimulationMode',            'normal');          % range logging needs Normal mode
set_param(mdl, 'DataTypeOverride',          'Double');          % measure ideal ranges
set_param(mdl, 'MinMaxOverflowLogging',     'MinMaxAndOverflow');
set_param(mdl, 'MinMaxOverflowArchiveMode', 'Overwrite');       % first case starts a clean run
set_param(mdl, 'FPTRunName',                RUNNAME);

% --- Test signals --------------------------------------------------------
[s, fsin] = audioread('Testing 28-07/0.wav');
s  = resample(s(:,1), fs, fsin);
i0 = find(abs(s) > 0.1*max(abs(s)), 1);
speech = s(i0:i0+NN-1);  speech = speech / max(abs(speech));

rng(1);
t0    = (0:NN-1)'/fs;
tones = sum(cos(2*pi*(400:200:3400).*t0 + 2*pi*rand(1,16)), 2);
tones = tones / max(abs(tones));

cases = { speech, 0,  'speech broadside'
          speech, 45, 'speech 45'
          speech,-85, 'speech endfire'
          tones,  30, 'tones 30'
          0.03*speech, 20, 'quiet speech (-30 dB)'
          speech + 0.3*randn(NN,1),-60,'noisy speech'};

% --- Run each case, merging ranges ---------------------------------------
for c = 1:size(cases,1)
    sig = ULA_planewave(cases{c,1}, deg2rad(cases{c,2}), Na, fs);
    pk  = max(abs(sig(:)));
    if pk > 0.999, sig = sig * (0.999/pk); end      % real 16-bit audio never reaches +1.0

    t = (0:size(sig,1)-1)' * Ts;
    Mic1 = [t sig(:,1)];  Mic2 = [t sig(:,2)];
    Mic3 = [t sig(:,3)];  Mic4 = [t sig(:,4)];
    RESET = [t zeros(size(t))];

    sim(mdl, 'StopTime', num2str(t(end)));
    fprintf('Range case %d/%d done: %s\n', c, size(cases,1), cases{c,3});

    if c == 1
        set_param(mdl, 'MinMaxOverflowArchiveMode', 'Merge');   % later cases add to the run
    end
end

fprintf('\nRanges collected in run "%s".\n', RUNNAME);
fprintf('Now open the Fixed-Point Tool, select that run, and click Propose Data Types.\n');

%% ------------------------------------------------------------------------
function w = ULA_planewave(x, DOA, Na, fs)
c = 343; d = 0.042;
Tau = (d/c)*sin(DOA);
lh = length(x);
f = (-lh/2 : lh/2-1)' * (fs/lh);
omega = 2*pi*f;
X = fftshift(fft(x));
w = zeros(lh, Na);  w(:,1) = x;
for p = 2:Na
    w(:,p) = ifft(ifftshift(X .* exp(-1j*omega*(p-1)*Tau)), 'symmetric');
end
end