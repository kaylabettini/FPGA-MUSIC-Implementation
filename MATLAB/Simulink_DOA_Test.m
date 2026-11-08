% simulink_doa.m - run the HFMA Simulink model on every *_sent.wav file
% Put this next to HFMA_4B_1_5_9_13.slx, a_luts.mat and your 'synthetic'
% and 'acoustic' folders, then press Run.
% Output: simulink_doa.csv  (type, angle, trial, estimate, median, std)
% The model reads Mic1..Mic4 and RESET from the workspace (From Workspace
% blocks, sample time Ts) and writes DOA to the workspace (To Workspace).
% The _sent.wav files are the exact int16 samples the FPGA received;
% audioread scales them by 1/32768, which is exactly the fixdt(1,16,15)
% value the model's input conversion expects, so Simulink sees bit-identical
% input to the board.

%OPTIMISED WITH THE HELP oF CLAUDE OPUS 5.5 FOR PARALLEL PROCESSING CODE

model   = 'HFMA_4B_1_5_9_13';
folders = {'synthetic'};
onlyTrial = [];     % e.g. 1 to run only trial 1 of each acoustic signal (faster); [] = all
skip    = 2;        % seconds dropped at the start (same as the FPGA results)
Ts      = 1/8000;   % model sample time (8 kHz)

load('a_luts.mat');   % lookup tables: a2/a3 steering, sin/cos
load('OLDER HFMA/iq_luts.mat');
k_avg = 10;           % leaky average gain = 2^-k_avg (same value used for HDL generation)
load_system(model);                                   % also loads a_luts.mat (PreLoadFcn)
set_param(model, 'SimulationMode', 'accelerator');    % much faster than normal mode; use 'normal' if it complains

out = fopen('simulink_doa.csv', 'w');
fprintf(out, 'type,angle,trial,estimate,median,std\n');

for fo = 1:numel(folders)
    files = dir(fullfile(folders{fo}, '*_sent.wav'));
    for k = 1:numel(files)
        name = files(k).name;

        % signal type, angle and trial from the file name (same as music_doa.m)
        t = regexp(name, '^syn_(.+)_(m?\d+)deg_sent', 'tokens', 'once');
        if ~isempty(t)
            type = ['syn_' t{1}]; ang = t{2}; trial = 1;
        else
            t = regexp(name, '^(m?\d+)deg_(.+)_t(\d+)_sent', 'tokens', 'once');
            if isempty(t), continue; end
            ang = t{1}; type = t{2}; trial = str2double(t{3});
            if ~isempty(onlyTrial) && trial ~= onlyTrial, continue; end
        end
        ang = str2double(strrep(ang, 'm', '-'));

        % --- inputs for the From Workspace blocks: [time, value] ---
        x  = audioread(fullfile(folders{fo}, name));      % N x 4, = int16/32768
        N  = size(x,1);
        tt = (0:N-1)' * Ts;
        Mic1 = [tt x(:,1)];  Mic2 = [tt x(:,2)];  Mic3 = [tt x(:,3)];  Mic4 = [tt x(:,4)];
        RESET = [tt double(tt < 10*Ts)];                  % reset for the first 10 samples, then run (as the FPGA does)
        stopTime = (N-1)*Ts;

        % --- run the model ---
        tic;
        simOut = sim(model, 'SrcWorkspace', 'current', 'StopTime', num2str(stopTime));
        doaTs  = simOut.get('DOA');
        doa    = double(squeeze(doaTs.Data));
        td     = doaTs.Time;

        % keep one reading every 20 ms after the first 2 s (the same rate as the FPGA log)
        keep = find(td >= skip);
        keep = keep(1:160:end);
        e    = doa(keep);

        fprintf(out, '%s,%d,%d,%.1f,%.1f,%.1f\n', type, ang, trial, mean(e), median(e), std(e));
        fprintf('%-30s true %4d   Simulink %6.1f  (median %6.1f, std %4.1f)   %.0f s\n', ...
            name, ang, mean(e), median(e), std(e), toc);
    end
end
fclose(out);
disp('Saved simulink_doa.csv');