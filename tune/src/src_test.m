function n_fail = src_test(fs_in_list, fs_out_list, ch, nch)

%%
% src_test - test src executable objective audio quality parameters
%
% src_test(fs_in, fs_out)
%
% fs_in  - vector of rates in
% fs_out - vector of rates out
%
% A default in-out matrix is tested if the paremeters are omitted.
%

%%
% Copyright (c) 2016, Intel Corporation
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%   * Redistributions of source code must retain the above copyright
%     notice, this list of conditions and the following disclaimer.
%   * Redistributions in binary form must reproduce the above copyright
%     notice, this list of conditions and the following disclaimer in the
%     documentation and/or other materials provided with the distribution.
%   * Neither the name of the Intel Corporation nor the
%     names of its contributors may be used to endorse or promote products
%     derived from this software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
%
% Author: Seppo Ingalsuo <seppo.ingalsuo@linux.intel.com>
%

addpath('../m');
mkdir_check('plots');
mkdir_check('reports');

if nargin < 1
        fs_in_list = [8e3 11025 12e3 16e3 18900 22050 24e3 32e3 44100 48e3 ...
			  64e3 88.2e3 96e3 176400 192e3];
end

if nargin < 2
	fs_out_list = [8e3 11025 12e3 16e3 18900 22050 24e3 32e3 44100 48e3];
end

if nargin < 4
        ch = 0; % 0=alter tested channel during test, 1=L, 2=R, etc.
        nch = 2;
end

%% Generic test pass/fail criteria
t.g_db_tol = 0.1;
t.thdnf_db_max = -80;
t.dr_db_min = 100;
t.aap_db_max = -60; % Relax from THD+N due to tough requirement for fs/2 band end
t.aip_db_max = -60; % Relax from THD+N due to tough requirement for fs/2 band end

%% Defaults for test
t.bits_in = 32;
t.bits_out = 32;
t.ch = ch;
t.nch = nch;
t.full_test = 1;

%% Workaround for Octave hang if too many windows remain open
%  the plots are exported into plots directory in png format
%  and can be viewed from there.
t.close_plot_windows = 1;

%% Init for test loop
n_test = 7;
n_fsi = length(fs_in_list);
n_fso = length(fs_out_list);
r.fs_in_list = fs_in_list;
r.fs_out_list = fs_out_list;
r.g = zeros(n_fsi, n_fso);
r.dr = zeros(n_fsi, n_fso);
r.fr_db = zeros(n_fsi, n_fso);
r.fr_hz = zeros(n_fsi, n_fso, 2);
r.fr3db_hz = zeros(n_fsi, n_fso);
r.thdnf = zeros(n_fsi, n_fso);
r.aap = zeros(n_fsi, n_fso);
r.aip = zeros(n_fsi, n_fso);
r.pf = zeros(n_fsi, n_fso, n_test);
r.n_fail = 0;
%% Loop all modes to test
for b = 1:n_fso
        for a = 1:n_fsi
                v = -ones(n_test,1); % Set pass/fail test verdict to not executed
                tn = 1;
                t.fs1 = fs_in_list(a);
                t.fs2 = fs_out_list(b);
                v(1) = chirp_test(t);
                if v(1) ~= -1 && t.full_test == 1
                        %% Chirp was processed so this in/out Fs is supported
                        [v(2), r.g(a,b)] = g_test(t);
                        [v(3), r.fr_db(a,b), r.fr_hz(a,b,:), r.fr3db_hz(a,b)] = fr_test(t);
                        [v(4), r.thdnf(a,b)] = thdnf_test(t);
                        [v(5), r.dr(a,b)] = dr_test(t);
                        [v(6), r.aap(a,b)] = aap_test(t);
                        [v(7), r.aip(a,b)] = aip_test(t);
                end
                %% Done, store pass/fail
                r.pf(a, b, :) = v;
                idx = find(v > 0);
                r.n_fail = r.n_fail + sum(v(idx));
        end
end

%% Print table with test summary: Gain
fn = 'reports/g_src.txt';
print_val(fn, fs_in_list, fs_out_list, r.g, r.pf, 'Gain dB');

%% Print table with test summary: FR
fn = 'reports/fr_src.txt';
print_fr(fn, fs_in_list, fs_out_list, r.fr_db, r.fr_hz, r.pf);

%% Print table with test summary: FR
fn = 'reports/fr_src.txt';
print_val(fn, fs_in_list, fs_out_list, r.fr3db_hz/1e3, r.pf, ...
        'Frequency response -3 dB 0 - X kHz');

%% Print table with test summary: THD+N vs. frequency
fn = 'reports/thdnf_src.txt';
print_val(fn, fs_in_list, fs_out_list, r.thdnf, r.pf, ...
        'Worst-case THD+N vs. frequency');

%% Print table with test summary: DR
fn = 'reports/dr_src.txt';
print_val(fn, fs_in_list, fs_out_list, r.dr, r.pf, ...
        'Dynamic range dB (CCIR-RMS)');

%% Print table with test summary: AAP
fn = 'reports/aap_src.txt';
print_val(fn, fs_in_list, fs_out_list, r.aap, r.pf, ...
        'Attenuation of alias products dB');

%% Print table with test summary: AIP
fn = 'reports/aip_src.txt';
print_val(fn, fs_in_list, fs_out_list, r.aip, r.pf, ...
        'Attenuation of image products dB');


%% Print table with test summary: pass/fail
fn = 'reports/pf_src.txt';
print_pf(fn, fs_in_list, fs_out_list, r.pf);

fprintf('\n');
fprintf('Number of failed tests = %d\n', r.n_fail);

end


%%
%% Test execution with help of common functions
%%

function [fail, g_db] = g_test(t)

%% Reference: AES17 6.2.2 Gain
test = test_defaults_src(t);
prm = src_param(t.fs1, t.fs2, test.coef_bits);
test.fu = prm.c_pb*min(t.fs1,t.fs2);
test.g_db_tol = t.g_db_tol;
test.g_db_expect = 0;

%% Create input file
test = g_test_input(test);

%% Run test
test = test_run_src(test, t);

%% Measure
test.fs = t.fs2;
test = g_test_measure(test);

%% Get output parameters
fail = test.fail;
g_db = test.g_db;
delete_check(test.fn_in);
delete_check(test.fn_out);

end

function [fail, pm_range_db, range_hz, fr3db_hz] = fr_test(t)

%% Reference: AES17 6.2.3 Frequency response
test = test_defaults_src(t);
prm = src_param(t.fs1, t.fs2, test.coef_bits);

test.rp_max = prm.rp_tot;      % Max. ripple +/- dB allowed
test.f_lo = 20;                % For response reporting, measure from 20 Hz
test.f_hi = min(t.fs1,t.fs2)*prm.c_pb;   % to designed filter upper frequency
test.f_max = 0.99*min(t.fs1/2, t.fs2/2); % Measure up to min. Nyquist frequency

%% Create input file
test = fr_test_input(test);

%% Run test
test = test_run_src(test, t);

%% Measure
test.fs = t.fs2;
test = fr_test_measure(test);

fail = test.fail;
pm_range_db = test.rp;
range_hz = [test.f_lo test.f_hi];
fr3db_hz = test.fr3db_hz;
delete_check(test.fn_in);
delete_check(test.fn_out);

%% Print
src_test_result_print(t, 'Frequency response', 'FR', test.ph);

end

function [fail, thdnf] = thdnf_test(t)

%% Reference: AES17 6.3.2 THD+N ratio vs. frequency
test = test_defaults_src(t);

prm = src_param(t.fs1, t.fs2, test.coef_bits);
test.f_start = 20;
test.f_end = prm.c_pb*min(t.fs1,t.fs2);
test.fu = prm.c_pb*min(t.fs1,t.fs2);

%% Create input file
test = thdnf_test_input(test);

%% Run test
test = test_run_src(test, t);

%% Measure
test.fs = t.fs2;
test.thdnf_max = t.thdnf_db_max;
test = thdnf_test_measure(test);
thdnf = max(max(test.thdnf));
fail = test.fail;
delete_check(test.fn_in);
delete_check(test.fn_out);

%% Print
src_test_result_print(t, 'THD+N ratio vs. frequency', 'THDNF');

end

function [fail, dr_db] = dr_test(t)

%% Reference: AES17 6.4.1 Dynamic range
test = test_defaults_src(t);
test.dr_db_min = t.dr_db_min;

%% Create input file
test = dr_test_input(test);

%% Run test
test = test_run_src(test, t);

%% Measure
test.fs = t.fs2;
test = dr_test_measure(test);

%% Get output parameters
fail = test.fail;
dr_db = test.dr_db;
delete_check(test.fn_in);
delete_check(test.fn_out);

end


function [fail, aap_db] = aap_test(t)

%% Reference: AES17 6.6.6 Attenuation of alias products
test = test_defaults_src(t);
test.aap_max = t.aap_db_max;

if t.fs1 <= t.fs2
        %% Internal rate must be lower than input
        fail = -1;
        aap_db = NaN;
        return;
end

test.f_start = 0.5*t.fs1;
test.f_end = 0.5*t.fs2;

%% Create input file
test = aap_test_input(test);

%% Run test
test = test_run_src(test, t);

%% Measure
test.fs = t.fs2;
test = aap_test_measure(test);
aap_db = test.aap;
fail = test.fail;
delete_check(test.fn_in);
delete_check(test.fn_out);

%% Print
src_test_result_print(t, 'Attenuation of alias products', 'AAP');

end

function [fail, aip_db] = aip_test(t)

%% Reference: AES17 6.6.7 Attenuation of image products
test = test_defaults_src(t);
test.aip_max = t.aip_db_max;
if t.fs1 >= t.fs2
        %% Internal rate must be higher than input
        fail = -1;
        aip_db = NaN;
        return;
%%
end

%% Create input file
test.f_end = t.fs1/2;
test = aip_test_input(test);

%% Run test
test = test_run_src(test, t);

%% Measure
test.fs = t.fs2;
test = aip_test_measure(test);
aip_db = test.aip;
fail = test.fail;
delete_check(test.fn_in);
delete_check(test.fn_out);

%% Print
src_test_result_print(t, 'Attenuation of image products', 'AIP');

end

%% Some SRC test specific utility functions
%%
function fail = chirp_test(t)

fprintf('Spectrogram test %d -> %d ...\n', t.fs1, t.fs2);

if t.ch == 0
        t.ch = 1+round(rand(1,1)*(t.nch-1)); % Test random channel 1..Nch
end

fail = -1;
fn_in = 'chirp_test_in.txt';
fn_out = 'chirp_test_out.txt';
test.ex = './src_test';
delete_check(fn_in);
delete_check(fn_out);

cl = 2;
ac = 10^(-1/20);
tc = ((1:round(cl*t.fs1))-1)/t.fs1;
size(t)
x0 = ac*chirp(tc, 0, cl, t.fs1/2,'linear');
x = zeros(length(x0),t.nch);
sign = 1;
for c = 1:t.nch
        x(:,c) = dither_and_quantize(sign*x0, t.bits_in);
	sign = -sign;
end
write_ascii_data(x, fn_in);

test.arg = { num2str(t.bits_in), num2str(t.bits_out), num2str(t.fs1), num2str(t.fs2), fn_in, fn_out};
test = test_run(test);
scale_out = 1/2^(t.bits_out-1);
y = [];
if exist(fn_out)
        fprintf('Reading output data file %s...\n', fn_out);
	dir_fn_out = dir(fn_out);
	if dir_fn_out.bytes > 0
		out = load('-ascii',fn_out);
		y = out(t.ch:t.nch:end)*scale_out;
	end
end

if length(y) < 1
        return;
end

figure;
ns = 1024;
no = round(0.9*ns);  specgram(y, ns, t.fs2, kaiser(ns,27), no);
caxis([-150 0]);
colorbar('EastOutside');
fail = 0;

src_test_result_print(t, 'Chirp', 'chirp');

% Delete files unless e.g. debugging and need data to run
if t.full_test
	delete_check(fn_in);
	delete_check(fn_out);
end

end

function test = test_defaults_src(t)
test.bits_in = t.bits_in;
test.bits_out = t.bits_out;
test.nch = t.nch;
test.ch = t.ch;
test.fs = t.fs1;
test.mask_f = [];
test.mask_lo = [];
test.mask_hi = [];
test.coef_bits = 24; % No need to use actual word length in test
end

function test = test_run_src(test, t)
test.ex = './src_test';
test.arg = { num2str(t.bits_in), num2str(t.bits_out), num2str(t.fs1), num2str(t.fs2), test.fn_in, test.fn_out};
delete_check(test.fn_out);
test = test_run(test);
end

function src_test_result_print(t, testverbose, testacronym, ph)
tstr = sprintf('%s SRC %d, %d', testverbose, t.fs1, t.fs2);
if nargin > 3
        title(ph, tstr);
else
        title(tstr);
end
pfn = sprintf('plots/%s_src_%d_%d.png', testacronym, t.fs1, t.fs2);
print(pfn, '-dpng');
if t.close_plot_windows
	close all;
end
end

%% The next are results printing functions

function  print_val(fn, fs_in_list, fs_out_list, val, pf, valstr)
n_fsi = length(fs_in_list);
n_fso = length(fs_out_list);
fh = fopen(fn,'w');
fprintf(fh,'\n');
fprintf(fh,'SRC test result: %s\n', valstr);
fprintf(fh,'%8s, ', 'out \ in');
for a = 1:n_fsi-1
        fprintf(fh,'%8.1f, ', fs_in_list(a)/1e3);
end
fprintf(fh,'%8.1f', fs_in_list(n_fsi)/1e3);
fprintf(fh,'\n');
for b = 1:n_fso
        fprintf(fh,'%8.1f, ', fs_out_list(b)/1e3);
        for a = 1:n_fsi
                if pf(a,b,1) < 0
                        cstr = 'x';
                else
                        if isnan(val(a,b))
                                cstr = '-';
                        else
                                cstr = sprintf('%8.2f', val(a,b));
                        end
                end
                if a < n_fsi
                        fprintf(fh,'%8s, ', cstr);
                else
                        fprintf(fh,'%8s', cstr);
                end
        end
        fprintf(fh,'\n');
end
fclose(fh);
type(fn);
end

function print_fr(fn, fs_in_list, fs_out_list, fr_db, fr_hz, pf)
n_fsi = length(fs_in_list);
n_fso = length(fs_out_list);
fh = fopen(fn,'w');
fprintf(fh,'\n');
fprintf(fh,'SRC test result: Frequency response +/- X.XX dB (YY.Y kHz) \n');
fprintf(fh,'%8s, ', 'out \ in');
for a = 1:n_fsi-1
        fprintf(fh,'%12.1f, ', fs_in_list(a)/1e3);
end
fprintf(fh,'%12.1f', fs_in_list(n_fsi)/1e3);
fprintf(fh,'\n');
for b = 1:n_fso
        fprintf(fh,'%8.1f, ', fs_out_list(b)/1e3);
        for a = 1:n_fsi
                if pf(a,b,1) < 0
                        cstr = 'x';
                else
                        cstr = sprintf('%4.2f (%4.1f)', fr_db(a,b), fr_hz(a,b,2)/1e3);
                end
                if a < n_fsi
                        fprintf(fh,'%12s, ', cstr);
                else
                        fprintf(fh,'%12s', cstr);
                end
        end
        fprintf(fh,'\n');
end
fclose(fh);
type(fn);
end

function print_fr3db(fn, fs_in_list, fs_out_list, fr3db_hz, pf)
n_fsi = length(fs_in_list);
n_fso = length(fs_out_list);
fh = fopen(fn,'w');
fprintf(fh,'\n');
fprintf(fh,'SRC test result: Frequency response -3dB 0 - X.XX kHz\n');
fprintf(fh,'%8s, ', 'out \ in');
for a = 1:n_fsi-1
        fprintf(fh,'%8.1f, ', fs_in_list(a)/1e3);
end
fprintf(fh,'%8.1f', fs_in_list(n_fsi)/1e3);
fprintf(fh,'\n');
for b = 1:n_fso
        fprintf(fh,'%8.1f, ', fs_out_list(b)/1e3);
        for a = 1:n_fsi
                if pf(a,b,1) < 0
                        cstr = 'x';
                else
                        cstr = sprintf('%4.1f', fr3db_hz(a,b)/1e3);
                end
                if a < n_fsi
                        fprintf(fh,'%8s, ', cstr);
                else
                        fprintf(fh,'%8s', cstr);
                end
        end
        fprintf(fh,'\n');
end
fclose(fh);
type(fn);
end


function print_pf(fn, fs_in_list, fs_out_list, pf)
n_fsi = length(fs_in_list);
n_fso = length(fs_out_list);
fh = fopen(fn,'w');
fprintf(fh,'\n');
fprintf(fh,'SRC test result: Fails\n');
fprintf(fh,'%8s, ', 'out \ in');
for a = 1:n_fsi-1
        fprintf(fh,'%14.1f, ', fs_in_list(a)/1e3);
end
fprintf(fh,'%14.1f', fs_in_list(n_fsi)/1e3);
fprintf(fh,'\n');
spf = size(pf);
npf = spf(3);
for b = 1:n_fso
        fprintf(fh,'%8.1f, ', fs_out_list(b)/1e3);
        for a = 1:n_fsi
                if pf(a,b,1) < 0
                        cstr = 'x';
                else
                        cstr = sprintf('%d', pf(a,b,1));
                        for n=2:npf
                                if pf(a,b,n) < 0
                                        cstr = sprintf('%s/x', cstr);
                                else
                                        cstr = sprintf('%s/%d', cstr,pf(a,b,n));
                                end
                        end
                end
                if a < n_fsi
                        fprintf(fh,'%14s, ', cstr);
                else
                        fprintf(fh,'%14s', cstr);
                end
        end
        fprintf(fh,'\n');
end
fclose(fh);
type(fn);
end
