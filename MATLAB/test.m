%% fix_fixed_point_types.m
% Corrects the data types the Fixed-Point Tool proposed in the places where
% they are unsafe for this design, and sets the Biquads by hand.
% Apply to the model AFTER "Apply Data Types" (keeps the tool's choices for
% the bandpass filters, I/Q mixers, counters and lookup tables).
%
% Design rules used:
%   * Biquad: 18-bit coefficients, 40-bit accumulator, 18-bit output ~±1
%   * Covariance path (products + leaky averages): signed 32-bit, saturating
%     (R can be negative; the >>k_avg update needs extra fraction bits)
%   * HFMA spectrum P(theta): unsigned 40-bit (wide dynamic range: loud vs quiet)
%   * Inports between these stages inherit from their source (no re-quantising)

mdl = 'HFMA_OPTIMISED_BUTTERWORTH_TEST';     % <-- model to fix
load_system(mdl);
opts = {'LookUnderMasks','all','FollowLinks','on'};
cnt  = @(c) fprintf('  %-55s %4d blocks\n', c{1}, c{2});

%% 0. Run in fixed point (the saved model still had Data Type Override = Double)
set_param(mdl, 'DataTypeOverride', 'UseLocalSettings');
set_param(mdl, 'MinMaxOverflowLogging', 'UseLocalSettings');
fprintf('Data Type Override -> UseLocalSettings\n');

%% 1. Inputs: 16-bit audio, Q1.15
mics = {};
for k = 1:4
    b = sprintf('%s/Subsystem/Mic%d', mdl, k);
    set_param(b, 'OutDataTypeStr', 'fixdt(1,16,15)');  mics{end+1} = b; %#ok<SAGROW>
end
cnt({'Subsystem Mic inports -> fixdt(1,16,15)', numel(mics)});

%% 2. Biquad low-pass filters (128)
% Library-linked blocks are found by the library block they point to
bq = find_system(mdl, opts{:}, 'ReferenceBlock', sprintf('dsphdlfiltering2/Biquad Filter'));
bqTypes = { 'NumeratorDataTypeStr',    'fixdt(1,18,25)'   % b ~ 0.0012..0.0024
            'DenominatorDataTypeStr',  'fixdt(1,18,16)'   % a1 ~ -1.94 (needs +/-2)
            'ScaleValuesDataTypeStr',  'fixdt(1,18,16)'   % scale values = 1
            'CoefficientsDataTypeStr', 'fixdt(1,18,16)'
            'AccumulatorDataTypeStr',  'fixdt(1,40,30)'   % poles at |z|~0.97: wide accumulator
            'OutputDataTypeStr',       'fixdt(1,18,16)' };% baseband I/Q, |x| < 2
for i = 1:numel(bq)
    for p = 1:size(bqTypes,1)
        try
            set_param(bq{i}, bqTypes{p,1}, bqTypes{p,2});
        catch ME
            if i == 1, fprintf('  (Biquad: could not set %s: %s)\n', bqTypes{p,1}, ME.message); end
        end
    end
end
cnt({'Biquad filters', numel(bq)});

%% 3. Covariance Matrix: inports inherit, products and leaky averages signed 32-bit
cov = find_system(mdl, opts{:}, 'BlockType','SubSystem', 'Name','Covariance Matrix');
nIn = 0; nProd = 0; nLeak = 0;
for c = 1:numel(cov)
    % every Inport inside the covariance (except RESET) inherits its type
    ins = find_system(cov{c}, opts{:}, 'BlockType','Inport');
    for i = 1:numel(ins)
        if ~strcmpi(get_param(ins{i},'Name'),'RESET')
            set_param(ins{i}, 'OutDataTypeStr', 'Inherit: auto');  nIn = nIn + 1;
        end
    end
    % ac_bd / bc_ad complex-product blocks
    pb = [find_system(cov{c}, opts{:}, 'BlockType','SubSystem','Name','ac_bd'); ...
          find_system(cov{c}, opts{:}, 'BlockType','SubSystem','Name','bc_ad')];
    for j = 1:numel(pb)
        pr = find_system(pb{j}, opts{:}, 'SearchDepth',1, 'BlockType','Product');
        sm = find_system(pb{j}, opts{:}, 'SearchDepth',1, 'BlockType','Sum');
        for i = 1:numel(pr), set_param(pr{i}, 'OutDataTypeStr','fixdt(1,32,30)', 'SaturateOnIntegerOverflow','on'); end
        for i = 1:numel(sm), set_param(sm{i}, 'OutDataTypeStr','fixdt(1,32,30)', 'SaturateOnIntegerOverflow','on'); end
        nProd = nProd + 1;
    end
    % leaky averages
    lk = find_system(cov{c}, 'RegExp','on', opts{:}, 'BlockType','SubSystem', 'Name','^Leaky');
    for j = 1:numel(lk)
        sm = find_system(lk{j}, opts{:}, 'SearchDepth',1, 'BlockType','Sum');
        for i = 1:numel(sm), set_param(sm{i}, 'OutDataTypeStr','fixdt(1,32,30)', 'SaturateOnIntegerOverflow','on'); end
        g = find_system(lk{j}, opts{:}, 'SearchDepth',1, 'BlockType','Gain');
        for i = 1:numel(g)
            set_param(g{i}, 'ParamDataTypeStr','Inherit: Inherit via internal rule', ...   % exact 2^-k
                            'OutDataTypeStr',  'fixdt(1,32,30)', 'SaturateOnIntegerOverflow','on');
        end
        ip = find_system(lk{j}, opts{:}, 'SearchDepth',1, 'BlockType','Inport', 'Name','p[n]');
        for i = 1:numel(ip), set_param(ip{i}, 'OutDataTypeStr','Inherit: auto'); end
        nLeak = nLeak + 1;
    end
end
cnt({'Covariance inports set to Inherit', nIn});
cnt({'ac_bd / bc_ad blocks -> signed 32-bit', nProd});
cnt({'Leaky averages -> signed 32-bit, saturating', nLeak});

%% 4. HFMA spectrum core: inports inherit, wide types for P(theta)
core = find_system(mdl, opts{:}, 'BlockType','SubSystem', 'Name','Hardware Friendly MUSIC Algorithm');
for c = 1:numel(core)
    ins = find_system(core{c}, opts{:}, 'BlockType','Inport');
    for i = 1:numel(ins), set_param(ins{i}, 'OutDataTypeStr','Inherit: auto'); end
    % complex multipliers: R (32-bit) x a (16-bit)
    cm = find_system(core{c}, 'RegExp','on', opts{:}, 'BlockType','SubSystem', 'Name','^complex multiplier');
    for j = 1:numel(cm)
        pr = find_system(cm{j}, opts{:}, 'SearchDepth',1, 'BlockType','Product');
        sm = find_system(cm{j}, opts{:}, 'SearchDepth',1, 'BlockType','Sum');
        for i = 1:numel(pr), set_param(pr{i}, 'OutDataTypeStr','fixdt(1,32,29)', 'SaturateOnIntegerOverflow','on'); end
        for i = 1:numel(sm), set_param(sm{i}, 'OutDataTypeStr','fixdt(1,32,29)', 'SaturateOnIntegerOverflow','on'); end
    end
    % top level of the core: -1 gains, the two 3-input sums, squares, final sum
    gn = find_system(core{c}, opts{:}, 'SearchDepth',1, 'BlockType','Gain');
    for i = 1:numel(gn), set_param(gn{i}, 'OutDataTypeStr','fixdt(1,32,29)', 'SaturateOnIntegerOverflow','on'); end
    sm = find_system(core{c}, opts{:}, 'SearchDepth',1, 'BlockType','Sum');
    for i = 1:numel(sm)
        if strcmp(get_param(sm{i},'Name'),'Add2')                     % P = Re^2 + Im^2
            set_param(sm{i}, 'OutDataTypeStr','fixdt(0,40,34)', 'SaturateOnIntegerOverflow','on');
        else                                                          % sum of 3 complex terms
            set_param(sm{i}, 'OutDataTypeStr','fixdt(1,32,28)', 'SaturateOnIntegerOverflow','on');
        end
    end
    pr = find_system(core{c}, opts{:}, 'SearchDepth',1, 'BlockType','Product');   % squares
    for i = 1:numel(pr), set_param(pr{i}, 'OutDataTypeStr','fixdt(0,40,34)', 'SaturateOnIntegerOverflow','on'); end
end
cnt({'HFMA cores updated', numel(core)});

%% 5. Sum of the 16 band spectra (top level) and the HFMA P_theta outports
top = [mdl '/Subsystem'];
add = find_system(top, opts{:}, 'SearchDepth',1, 'BlockType','Sum');
for i = 1:numel(add)
    set_param(add{i}, 'OutDataTypeStr','fixdt(0,40,34)', 'SaturateOnIntegerOverflow','on');
end
cnt({'Top-level band-sum', numel(add)});

%% 6. Check: update the diagram
fprintf('\nUpdating diagram...\n');
try
    set_param(mdl, 'SimulationCommand', 'update');
    fprintf('OK - diagram updated with no errors.\n');
catch ME
    fprintf(2, 'Update diagram FAILED:\n%s\n', ME.message);
    if ~isempty(ME.cause), fprintf(2, 'Cause: %s\n', ME.cause{1}.message); end
end
fprintf('Save as a new name, e.g. HFMA_BUTTERWORTH_fixed.slx, then run the sweep.\n');