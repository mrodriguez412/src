%% ========================================================================
%  System identification of the drone linear.x dynamics
%  ------------------------------------------------------------------------
%  Fits ARX and FIR models mapping  /vrep/twistCommand.linear.x  (input u)
%  to  /vrep/localTwist.twist.linear.x  (output y), and prints the
%  coefficient strings ready to paste into predict.launch.py.
%
%  Runs on base MATLAB only -- the System Identification Toolbox is used
%  as an optional cross-check if it happens to be installed.
%
%  Upload alongside: vel.csv, vel2.csv, othervel.csv, otherbag2.csv
%% ========================================================================

clear; close all; clc;

%% ---------------------------- CONFIG ------------------------------------
% Bag 1 (18.7 s, 6 command transitions) -> TRAINING
TRAIN_CMD   = 'vel2.csv';        % command  (frame_id "n/a")
TRAIN_STATE = 'vel.csv';         % measured (frame_id "body")

% Bag 2 (10.5 s, 4 command transitions) -> VALIDATION (never fitted on)
TEST_CMD    = 'otherbag2.csv';
TEST_STATE  = 'othervel.csv';

% Sampling period of the identified model.
% MUST equal 1/rate in predict.launch.py. The plant rings at ~5 Hz and
% rises in ~0.15 s, so 50 Hz is a sensible choice; 10 Hz is too coarse.
Ts = 1/50;

COL = 4;                 % CSV column: 4=linear.x 5=linear.y 6=linear.z
                         %             7=angular.x 8=angular.y 9=angular.z

% The command is published at 10 Hz and held by the driver between
% publishes, so zero-order hold is the physically correct upsampling.
% Linear interpolation would invent a 0.1 s ramp at every transition --
% exactly where all the information lives.
CMD_INTERP = 'previous';

% Order sweep. nk >= 1 is REQUIRED: the node predicts y(k+1) from data up
% to time k, so a model with a u(k+1) term could not be implemented.
NA_LIST = 1:6;
NB_LIST = 1:6;
NK_LIST = 1:3;
FIR_NB  = [5 10 20 30 40 50];    % FIR = ARX with na = 0

%% ---------------------------- LOAD --------------------------------------
fprintf('=== Loading ===\n');
[tc1, u1raw] = load_twist_csv(TRAIN_CMD,   COL);
[ty1, y1raw] = load_twist_csv(TRAIN_STATE, COL);
[tc2, u2raw] = load_twist_csv(TEST_CMD,    COL);
[ty2, y2raw] = load_twist_csv(TEST_STATE,  COL);

[t1, u1, y1] = resample_pair(tc1, u1raw, ty1, y1raw, Ts, CMD_INTERP);
[t2, u2, y2] = resample_pair(tc2, u2raw, ty2, y2raw, Ts, CMD_INTERP);

fprintf('train: %4d samples, %.2f s   u in [%+.2f %+.2f]  y in [%+.2f %+.2f]\n', ...
        numel(t1), t1(end), min(u1), max(u1), min(y1), max(y1));
fprintf('test : %4d samples, %.2f s   u in [%+.2f %+.2f]  y in [%+.2f %+.2f]\n', ...
        numel(t2), t2(end), min(u2), max(u2), min(y2), max(y2));

check_excitation(u1, 'train');
check_excitation(u2, 'test ');

%% ------------------------- ORDER SWEEP ----------------------------------
fprintf('\n=== Order sweep (fitted on train, ranked on held-out test) ===\n');

R = struct('na',{},'nb',{},'nk',{},'th',{},'fit1',{},'fitsim',{},'stable',{});

for na = NA_LIST
    for nb = NB_LIST
        for nk = NK_LIST
            R(end+1) = fit_and_score(u1, y1, u2, y2, na, nb, nk); %#ok<SAGROW>
        end
    end
end
for nb = FIR_NB
    R(end+1) = fit_and_score(u1, y1, u2, y2, 0, nb, 1); %#ok<SAGROW>
end

% Rank by free-run SIMULATION fit on the test bag. One-step fit is a poor
% discriminator here: at 50 Hz, y(k) alone already predicts y(k+1) to ~99%,
% so ranking on it just picks the highest order.
[~, ord] = sort([R.fitsim], 'descend');
R = R(ord);

fprintf('\n%-4s %-4s %-4s %-6s %10s %10s %8s\n', ...
        'na','nb','nk','type','sim fit %','1-step %','stable');
fprintf('%s\n', repmat('-', 1, 52));
for k = 1:min(15, numel(R))
    r = R(k);
    if r.na == 0, ty = 'FIR'; else, ty = 'ARX'; end
    fprintf('%-4d %-4d %-4d %-6s %10.2f %10.2f %8s\n', ...
            r.na, r.nb, r.nk, ty, r.fitsim, r.fit1, bool2str(r.stable));
end

% --- Parsimonious selection -------------------------------------------
% The simulation fit plateaus: once the dynamics are captured, extra orders
% buy fractions of a percent, which is noise, not signal. So take the best
% stable fit, then step BACK to the simplest model within FIT_TOL of it.
FIT_TOL = 0.5;    % percentage points

topfit = [];
for k = 1:numel(R)
    if R(k).stable, topfit = R(k).fitsim; break; end
end
if isempty(topfit)
    error('No stable model found -- check the data.');
end

best = []; bestpar = Inf;
for k = 1:numel(R)
    if ~R(k).stable,                     continue; end
    if R(k).fitsim < topfit - FIT_TOL,   continue; end
    npar = R(k).na + R(k).nb;
    if npar < bestpar
        bestpar = npar; best = R(k);
    end
end

fprintf('\nBest stable simulation fit: %.2f %%\n', topfit);
fprintf('Simplest model within %.1f pp of it -> chosen (%d parameters).\n', ...
        FIT_TOL, bestpar);

fprintf('\n=== Selected model ===\n');
if best.na == 0
    fprintf('FIR   nb=%d nk=%d\n', best.nb, best.nk);
else
    fprintf('ARX   na=%d nb=%d nk=%d\n', best.na, best.nb, best.nk);
end
fprintf('simulation fit (test) : %6.2f %%\n', best.fitsim);
fprintf('one-step fit   (test) : %6.2f %%\n', best.fit1);

[A, B] = th2poly(best.th, best.na, best.nb, best.nk);
fprintf('A(q) = ');  fprintf('%+.6g ', A);  fprintf('\n');
fprintf('B(q) = ');  fprintf('%+.6g ', B);  fprintf('\n');
fprintf('DC gain = %.4f   (sum(B)/sum(A))\n', sum(B)/sum(A));
if best.na > 0
    p = roots(A);
    fprintf('poles   = '); fprintf('%.4f%+.4fi  ', [real(p) imag(p)]'); fprintf('\n');
    fprintf('|poles| = '); fprintf('%.4f  ', abs(p)); fprintf('\n');
end

%% -------------------- LAUNCH FILE COEFFICIENTS --------------------------
% Node arithmetic (model_prediction_rate.py, timer_cb):
%     pred = -sum(state_coef .* x) + sum(command_coef .* u)
% with x = [y(k-N+1) ... y(k)] oldest-first, and inverse_coef_list=True
% reversing the coefficient lists -> coefficients are written NEWEST-FIRST.
%
% Predicting y(k+1) from the ARX equation at t = k+1:
%     y(k+1) = -a1*y(k) - ... - ana*y(k-na+1)
%              + B(2)*u(k) + B(3)*u(k-1) + ...
% so:  state_coef   = A(2:end)     command_coef = B(2:end)

state_coef   = A(2:end);
command_coef = B(2:end);

fprintf('\n=== Paste into predict.launch.py ===\n');
% Single-quoted format strings: '' is a literal quote, " needs no escaping.
fprintf('{''~/rate'': %.1f},\n', 1/Ts);
fprintf('{''~/inverse_coef_list'': True},\n');
fprintf('{''~/command_type'': "geometry_msgs/Twist"},\n');
fprintf('{''~/command_field'': "linear.x"},\n');
fprintf('{''~/command_coef_csv'': "%s"},\n', csvstr(command_coef));
fprintf('{''~/state_type'': "geometry_msgs/TwistStamped"},\n');
fprintf('{''~/state_field'': "twist.linear.x"},\n');
fprintf('{''~/state_coef_csv'': "%s"},\n', csvstr(state_coef));

%% ------------------- VERIFY THE NODE ARITHMETIC -------------------------
% Re-implement exactly what the Python node does, including the list
% reversal and truncation, and check it reproduces the MATLAB predictor.
fprintf('\n=== Node arithmetic check ===\n');
yh_ref  = arx_predict1(u2, y2, best.th, best.na, best.nb, best.nk);
yh_node = simulate_node(u2, y2, command_coef, state_coef);
m = ~isnan(yh_ref) & ~isnan(yh_node);
if ~any(m)
    err = Inf;
else
    err = max(abs(yh_ref(m) - yh_node(m)));
end
fprintf('max |matlab_predictor - node_emulation| = %.3e\n', err);
if err < 1e-9
    fprintf('OK: the coefficient strings above are in the order the node expects.\n');
else
    fprintf('MISMATCH -- do not trust the launch file coefficients.\n');
end

%% --------------------------- PLOTS --------------------------------------
yh   = arx_predict1(u2, y2, best.th, best.na, best.nb, best.nk);
ysim = arx_sim(     u2, y2, best.th, best.na, best.nb, best.nk);

figure('Name','Validation','Position',[100 100 950 700]);

subplot(3,1,1);
stairs(t2, u2, 'k', 'LineWidth', 1.2); grid on;
ylabel('u  [m/s]'); title('Command (validation bag)');
ylim([min(u2) max(u2)] + 0.2*max(1e-3, max(u2)-min(u2))*[-1 1]);

subplot(3,1,2);
plot(t2, y2, 'b', 'LineWidth', 1.1); hold on;
plot(t2, yh, 'r--', 'LineWidth', 1.1); grid on;
ylabel('y  [m/s]'); legend('measured','1-step prediction','Location','best');
title(sprintf('One-step-ahead prediction  (fit %.2f %%)', best.fit1));

subplot(3,1,3);
plot(t2, y2, 'b', 'LineWidth', 1.1); hold on;
plot(t2, ysim, 'r--', 'LineWidth', 1.1); grid on;
xlabel('t  [s]'); ylabel('y  [m/s]');
legend('measured','free-run simulation','Location','best');
title(sprintf('Free-run simulation  (fit %.2f %%)', best.fitsim));

% Step response of the identified model
figure('Name','Step response','Position',[120 120 700 400]);
Nst = round(2/Ts);
ust = 0.5*ones(Nst,1);
yst = arx_sim(ust, zeros(Nst,1), best.th, best.na, best.nb, best.nk);
plot((0:Nst-1)*Ts, yst, 'LineWidth', 1.4); grid on;
xlabel('t  [s]'); ylabel('y  [m/s]');
title('Identified model: response to a 0.5 m/s step');
yline(sum(B)/sum(A)*0.5, 'k:', 'steady state');

%% ------------------ OPTIONAL TOOLBOX CROSS-CHECK ------------------------
if exist('arx') ~= 0 && exist('iddata') ~= 0 %#ok<EXIST>
    fprintf('\n=== System Identification Toolbox cross-check ===\n');
    try
        z1 = iddata(y1, u1, Ts);
        z2 = iddata(y2, u2, Ts);
        mdl = arx(z1, [best.na best.nb best.nk]);
        fprintf('toolbox A = '); fprintf('%+.6g ', mdl.A); fprintf('\n');
        fprintf('manual  A = '); fprintf('%+.6g ', A);     fprintf('\n');
        fprintf('toolbox B = '); fprintf('%+.6g ', mdl.B); fprintf('\n');
        fprintf('manual  B = '); fprintf('%+.6g ', B);     fprintf('\n');
        mA = mdl.A(:); mB = mdl.B(:); vA = A(:); vB = B(:);
        if numel(mA) == numel(vA) && numel(mB) == numel(vB)
            fprintf('max coefficient difference: %.3e\n', ...
                max([max(abs(mA - vA)); max(abs(mB - vB))]));
        else
            fprintf('(different polynomial lengths -- compare by eye)\n');
        end
        figure('Name','compare()'); compare(z2, mdl);
    catch ME
        fprintf('toolbox check skipped: %s\n', ME.message);
    end
else
    fprintf('\n(System Identification Toolbox not installed -- using the\n');
    fprintf(' built-in least-squares implementation, which computes the\n');
    fprintf(' same ARX estimate.)\n');
end

%% ======================= LOCAL FUNCTIONS ================================

function [t, v] = load_twist_csv(fname, col)
% Parse a `ros2 topic echo --csv` dump of a TwistStamped.
% Columns: sec, nsec, frame_id, lin.x, lin.y, lin.z, ang.x, ang.y, ang.z
    fid = fopen(fname, 'r');
    if fid < 0
        error('Cannot open %s. Upload it next to this script.', fname);
    end
    C = textscan(fid, '%f%f%s%f%f%f%f%f%f', 'Delimiter', ',');
    fclose(fid);
    n = min(cellfun(@numel, C));
    if n == 0
        error('%s parsed as empty -- check the column layout.', fname);
    end
    t = C{1}(1:n) + C{2}(1:n)*1e-9;     % seconds since epoch
    v = C{col}(1:n);
end

function [t, u, y] = resample_pair(tc, uc, ty, yv, Ts, cmd_interp)
% Put command and state on one common time grid over their overlap.
    % interp1 rejects duplicate sample points, and bags do contain them.
    [tc, ia] = unique(tc(:), 'stable'); uc = uc(ia);
    [ty, ib] = unique(ty(:), 'stable'); yv = yv(ib);
    [tc, ia] = sort(tc); uc = uc(ia);
    [ty, ib] = sort(ty); yv = yv(ib);

    t0 = max(tc(1),   ty(1));
    t1 = min(tc(end), ty(end));
    if t1 <= t0
        error('Command and state do not overlap in time.');
    end
    t = (0 : Ts : floor((t1-t0)/Ts)*Ts)';
    u = interp1(tc - t0, uc, t, cmd_interp);   % ZOH: the driver holds it
    y = interp1(ty - t0, yv, t, 'linear');     % smooth measured signal
    keep = ~isnan(u) & ~isnan(y);
    if nnz(keep) < 20
        error('Too few usable samples after resampling (%d).', nnz(keep));
    end
    t = t(keep); u = u(keep); y = y(keep);
    t = t - t(1);
end

function check_excitation(u, tag)
% Warn about the input-diversity requirement from Step 1.
    lv = unique(round(u, 3));
    ns = sum(diff(u) ~= 0);
    fprintf('  [%s] %d distinct command levels, %d transitions', tag, numel(lv), ns);
    if numel(lv) <= 3
        fprintf('   <-- WARNING: not diverse (Step 1 asks for many levels, incl. 0)');
    end
    fprintf('\n');
end

function r = fit_and_score(u1, y1, u2, y2, na, nb, nk)
% Fit on bag 1, score on bag 2.
    th = arx_ls(u1, y1, na, nb, nk);
    r.na = na; r.nb = nb; r.nk = nk; r.th = th;
    r.fit1   = fitpct(y2, arx_predict1(u2, y2, th, na, nb, nk));
    r.fitsim = fitpct(y2, arx_sim(     u2, y2, th, na, nb, nk));
    if na == 0
        r.stable = true;                       % FIR is always stable
    else
        r.stable = all(abs(roots([1; th(1:na)]')) < 1);
    end
end

function th = arx_ls(u, y, na, nb, nk)
% Least-squares ARX:  y(t) = -a1 y(t-1).. + b1 u(t-nk)..
% theta = [a1..ana b1..bnb]
    N    = numel(y);
    p    = max(na, nk + nb - 1);
    rows = (p+1):N;
    Phi  = zeros(numel(rows), na + nb);
    for i = 1:na
        Phi(:, i) = -y(rows - i);
    end
    for j = 1:nb
        Phi(:, na + j) = u(rows - (nk + j - 1));
    end
    th = Phi \ y(rows);
    th = th(:);
end

function yh = arx_predict1(u, y, th, na, nb, nk)
% One-step-ahead prediction using MEASURED past outputs (what the node does).
    N  = numel(y);
    p  = max(na, nk + nb - 1);
    yh = nan(N, 1);
    for t = (p+1):N
        acc = 0;
        for i = 1:na,  acc = acc - th(i)      * y(t - i);            end
        for j = 1:nb,  acc = acc + th(na + j) * u(t - (nk + j - 1)); end
        yh(t) = acc;
    end
end

function ys = arx_sim(u, y, th, na, nb, nk)
% Free-run simulation: feeds its own output back. This is the honest test
% of whether the model captured the dynamics.
    N  = numel(u);
    p  = max(na, nk + nb - 1);
    ys = nan(N, 1);
    ys(1:p) = y(1:p);
    for t = (p+1):N
        acc = 0;
        for i = 1:na,  acc = acc - th(i)      * ys(t - i);           end
        for j = 1:nb,  acc = acc + th(na + j) * u(t - (nk + j - 1)); end
        % An unstable candidate diverges; clamp so it scores badly instead
        % of poisoning the comparison with Inf/NaN.
        if ~isfinite(acc), acc = 1e6; end
        ys(t) = max(-1e6, min(1e6, acc));
    end
end

function yh = simulate_node(u, y, command_coef, state_coef)
% Bit-for-bit emulation of model_prediction_rate.py timer_cb, including
% inverse_coef_list=True (list reversal) and the sliding-window truncation.
    cc = flip(command_coef(:));     % node: command_coef.reverse()
    sc = flip(state_coef(:));       % node: state_coef.reverse()
    Nc = numel(cc); Ns = numel(sc);
    N  = numel(u);
    yh = nan(N, 1);
    for k = 1:N
        if k < max(Nc, Ns), continue; end
        xw = y(k-Ns+1 : k);         % self.x, oldest first
        uw = u(k-Nc+1 : k);         % self.u, oldest first
        yh(k) = -sum(sc .* xw) + sum(cc .* uw);
    end
    % node publishes the prediction for k+1 at step k -> shift to compare
    yh = [nan; yh(1:end-1)];
end

function [A, B] = th2poly(th, na, nb, nk)
% theta -> MATLAB polynomial convention.
% A(1)=1 and A(i+1) multiplies y(t-i);  B(i) multiplies u(t-i+1).
    A = [1, reshape(th(1:na), 1, [])];
    B = [zeros(1, nk), reshape(th(na+1:na+nb), 1, [])];
end

function f = fitpct(y, yh)
% MATLAB compare()-style NRMSE fit percentage.
    m = ~isnan(yh) & ~isnan(y);
    if nnz(m) < 2 || norm(y(m) - mean(y(m))) == 0
        f = -Inf; return;
    end
    f = 100 * (1 - norm(y(m) - yh(m)) / norm(y(m) - mean(y(m))));
    if ~isfinite(f), f = -Inf; end
end

function s = csvstr(v)
    s = strjoin(arrayfun(@(x) sprintf('%.8g', x), v(:)', 'UniformOutput', false), ',');
end

function s = bool2str(b)
    if b, s = 'yes'; else, s = 'NO'; end
end
