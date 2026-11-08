%% HFMA_Sweep_Background.m
% DOA error sweep for any HFMA model variant, run in parallel with parsim.
%
% HOW TO USE
%   1. Set the options in the USER SETTINGS section below.
%   2. Run SECTION 1 (Ctrl+Enter inside it, or the "Run Section" button).
%      With RUN_IN_BACKGROUND = true, MATLAB is free again straight away.
%   3. When the progress window shows that every run has finished, run
%      SECTION 2 to collect the results, print the table and plot the figures.
%      (With RUN_IN_BACKGROUND = false, run the whole file and it waits.)
%
% Requires Parallel Computing Toolbox for parsim. Without it, set
% USE_PARSIM = false and the sweep runs one simulation at a time instead.
%
% CREATED WITH THE HELP OF CLAUDE OPUS 5.5
%
%% ======================= SECTION 1: SETUP + LAUNCH =======================
clc;
% Stop any sweep still running from a previous launch BEFORE losing its handle
if exist('fut','var') && ~isempty(fut) && any(~strcmp({fut.State},'finished'))
    fprintf('Cancelling the previous sweep that is still running...\n');
    cancel(fut);
end
clear out fut in;                      % clear results from any previous run

% ----------------------------- USER SETTINGS -----------------------------
mdl               = 'HFMA_FIXED_FINAL';  % <-- model to test (no .slx)
angles            = [-80 -30 0 30 60 80];          % true DOAs to test (deg); e.g. -90:5:90 for quick checks
SIGNAL_TYPE       = 'speech';          % 'speech' or 'tones' (16 tones at the band centres)
WAV_FILE          = 'Testing 28-07/0.wav';
SNR_dB            = Inf;               % Inf = no noise, otherwise e.g. 20, 10, 5
EXTRA_CYCLES      = 200;               % zero samples appended after the data
USE_PARSIM        = false;             % false = plain for-loop (no toolbox needed)
RUN_IN_BACKGROUND = false;              % true = return immediately; run Section 2 later
SIM_MODE          = 'normal';     % 'normal', 'accelerator' or 'rapid-accelerator'
LUT_FILES         = {'a_luts.mat','iq_luts.mat','lpf_sos.mat'};  % loaded into base workspace
% -------------------------------------------------------------------------

Na    = 4;       % microphones
NN    = 2^12;    % samples per test
fs    = 8000;    % processing rate (Hz)
N_COV = NN;      % value of 'N' used by the 1/N gains in Covariance Matrix
                 % (your old script had N = 4096; keep it the same for comparable results)
Ts    = 1/fs;
k_avg = 10;
assignin('base','N',N_COV);  assignin('base','Ts',Ts);   % so load/compile in the client also works

% --- Load lookup tables / coefficients into the base workspace -----------
for f = 1:numel(LUT_FILES)
    if isfile(LUT_FILES{f})
        evalin('base', sprintf('load(''%s'');', LUT_FILES{f}));
    else
        warning('LUT file not found (skipped): %s', LUT_FILES{f});
    end
end

load_system(mdl);
set_param(mdl, 'ParameterPrecisionLossMsg', 'none');

% --- Source signal --------------------------------------------------------
switch lower(SIGNAL_TYPE)
    case 'speech'
        [s1, fs_in] = audioread(WAV_FILE);
        s1     = resample(s1(:,1), fs, fs_in);
        thr    = 0.1*max(abs(s1));
        sstart = find(abs(s1) > thr, 1);
        sstop  = sstart + NN - 1;
        assert(sstop <= numel(s1), 'Audio too short: need %d samples, have %d', sstop, numel(s1));
        s11 = s1(sstart:sstop);
    case 'tones'
        rng(1);                                        % repeatable random phases
        t0  = (0:NN-1)'/fs;
        f0  = 400:200:3400;
        s11 = sum(cos(2*pi*f0.*t0 + 2*pi*rand(1,numel(f0))), 2);
    otherwise
        error('SIGNAL_TYPE must be ''speech'' or ''tones''.');
end
s11 = s11 / max(abs(s11));

% --- Build one SimulationInput per angle ----------------------------------
Ts     = 1/fs;
nAng   = numel(angles);
in(1:nAng) = Simulink.SimulationInput(mdl);

for k = 1:nAng
    sig = ULA_planewave(s11, deg2rad(angles(k)), Na, fs);

    if isfinite(SNR_dB)                                % white noise, independent per mic
        Ps  = mean(sig(:,1).^2);
        sig = sig + sqrt(Ps/10^(SNR_dB/10)) * randn(size(sig));
    end

    Ndata  = size(sig,1);
    Ntotal = Ndata + EXTRA_CYCLES;
    t      = (0:Ntotal-1)' * Ts;
    pad    = @(x) [x; zeros(EXTRA_CYCLES,1)];

    in(k) = in(k).setModelParameter('DataTypeOverride','Double');

    in(k) = in(k).setVariable('Mic1', [t pad(sig(:,1))]);
    in(k) = in(k).setVariable('Mic2', [t pad(sig(:,2))]);
    in(k) = in(k).setVariable('Mic3', [t pad(sig(:,3))]);
    in(k) = in(k).setVariable('Mic4', [t pad(sig(:,4))]);
    in(k) = in(k).setVariable('RESET', [t zeros(Ntotal,1)]);

    % Model parameters the blocks read from the workspace
    in(k) = in(k).setVariable('N',  N_COV);   % Covariance Matrix gains use 1/N
    in(k) = in(k).setVariable('Ts', Ts);      % counter / source sample time

    % Only used by older model versions; harmless for the new ones
    in(k) = in(k).setVariable('F0', [t 1850*ones(Ntotal,1)]);
    in(k) = in(k).setVariable('Fs', [t fs*ones(Ntotal,1)]);
    in(k) = in(k).setVariable('Sample_Valid', [t [ones(Ndata,1); zeros(EXTRA_CYCLES,1)]]);

    in(k) = in(k).setModelParameter('StopTime', num2str(t(end)), ...
                                    'SimulationMode', SIM_MODE);
end

% Keep what Section 2 needs
sweep = struct('mdl',mdl, 'angles',angles, 'signal',SIGNAL_TYPE, 'snr',SNR_dB);

% --- Launch ---------------------------------------------------------------
fprintf('Running %d simulations of %s (%s, SNR = %g dB)...\n', nAng, mdl, SIGNAL_TYPE, SNR_dB);
tStart = tic;

if USE_PARSIM
    if RUN_IN_BACKGROUND
        bgOpts = {'RunInBackground','on', 'ShowProgress','on', ...
                  'ShowSimulationManager','on', ...         % opens the progress window
                  'TransferBaseWorkspaceVariables','on'};
        try
            fut = parsim(in, bgOpts{:});
        catch ME
            if contains(ME.message, 'busy')
                % An earlier sweep is still occupying the pool and its handle is lost:
                % restart the pool, then launch again.
                fprintf('Parallel pool busy with an old job - restarting the pool...\n');
                delete(gcp('nocreate'));
                fut = parsim(in, bgOpts{:});
            else
                rethrow(ME);
            end
        end
        fprintf('Launched in the background. MATLAB is free.\n');
        fprintf('Check progress any time by typing:\n');
        fprintf('   fprintf(''%%d / %%d finished\\n'', sum(strcmp({fut.State},''finished'')), numel(fut))\n');
        fprintf('When all runs are finished, run SECTION 2.\n');
        return;                                        % stop here; Section 2 collects later
    else
        out = parsim(in, 'ShowProgress','on', 'TransferBaseWorkspaceVariables','on');
    end
else
    for k = 1:nAng
        out(k) = sim(in(k));
        fprintf('  %d/%d done\n', k, nAng);
    end
end
% (falls through to Section 2 automatically when not running in the background)

%% ===================== SECTION 2: COLLECT + PLOT ========================
if exist('fut','var') && ~exist('out','var')
    out = fetchOutputs(fut);                           % waits if anything is still running
end
fprintf('Sweep finished (%.1f s since launch).\n', toc(tStart));

DOAs   = nan(size(sweep.angles));
for k = 1:numel(out)
    if ~isempty(out(k).ErrorMessage)
        warning('Angle %+d failed: %s', sweep.angles(k), out(k).ErrorMessage);
        continue;
    end
    DOAs(k) = out(k).DOA.Data(end);
end
errors = abs(DOAs - sweep.angles); 

for k = 1:numel(sweep.angles)
    fprintf('True angle = %+4d, Estimated = %6.1f, Error = %5.1f\n', ...
            sweep.angles(k), DOAs(k), errors(k));
end

% --- Figures ----------------------------------------------------------------
ttl = sprintf('%s | %s | SNR = %g dB', strrep(sweep.mdl,'_','\_'), sweep.signal, sweep.snr);

figure('Name',[sweep.mdl ' - estimate']);
plot(sweep.angles, DOAs, 'LineWidth', 1.5); hold on;
plot(sweep.angles, sweep.angles, '--', 'LineWidth', 1.2);
grid on; xlim([-90 90]); ylim([-90 90]);
xlabel('Actual DOA (degrees)'); ylabel('Estimated DOA (degrees)');
title({'HFMA Estimated vs Actual DOA', ttl});
legend('HFMA estimate', 'Ideal', 'Location', 'northwest');

figure('Name',[sweep.mdl ' - error']);
plot(sweep.angles, errors, 'LineWidth', 1.5);
grid on; xlim([-90 90]);
xlabel('Actual DOA (degrees)'); ylabel('Absolute error (degrees)');
title({'HFMA DOA Error', ttl});

% --- Statistics (full range and +/-60 deg) --------------------------------
ok   = ~isnan(errors);
in60 = ok & abs(sweep.angles) <= 60;
fprintf('\nHFMA DOA accuracy: %s\n', sweep.mdl);
fprintf('-----------------------------------------------\n');
fprintf('                     all angles    |angle|<=60\n');
fprintf('Mean abs error  : %10.3f    %10.3f deg\n', mean(errors(ok)),            mean(errors(in60)));
fprintf('RMSE            : %10.3f    %10.3f deg\n', sqrt(mean(errors(ok).^2)),   sqrt(mean(errors(in60).^2)));
fprintf('Max error       : %10.3f    %10.3f deg\n', max(errors(ok)),             max(errors(in60)));

% Save so different models can be compared later
resFile = sprintf('results_%s_%s_snr%s.mat', sweep.mdl, sweep.signal, num2str(sweep.snr));
save(resFile, 'sweep', 'DOAs', 'errors');
fprintf('Saved %s\n', resFile);

%% ============================ FUNCTIONS =================================

function w = ULA_planewave(x, DOA, Na, fs)
% Plane wave arriving at a uniform linear array (fractional delays via FFT).
% DOA in radians, measured from broadside.
    c   = 343;        % speed of sound (m/s)
    d   = 0.042;      % mic spacing (m)
    Tau = (d/c)*sin(DOA);

    lh    = length(x);
    f     = (-lh/2 : lh/2-1)' * (fs/lh);
    omega = 2*pi*f;
    X     = fftshift(fft(x));

    w      = zeros(lh, Na);
    w(:,1) = x;
    for p = 2:Na
        kk     = X .* exp(-1j*omega*(p-1)*Tau);
        w(:,p) = ifft(ifftshift(kk), 'symmetric');
    end
end