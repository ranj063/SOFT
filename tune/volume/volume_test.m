function n_fail = volume_test()

%%
% volume_test - test FIR EQ executable objective audio quality parameters
%
%

%%
% Copyright (c) 2017, Intel Corporation
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

comp = 'volume';
addpath('../m');
plots = 'plots';
reports = 'reports';
mkdir_check(plots);
mkdir_check(reports);

ch = [1 2];
nch = 2;
fs_list = [48e3];

%% Generic test pass/fail criteria
t.g_db_tol = 0.1;
t.thdnf_db_max = -100;
t.dr_db_min = 140;

%% Loop all modes to test
t.bits_in = 16;
t.bits_out = 16;
t.ch = ch;
t.nch = nch;

n_test = 2;
n_fs = length(fs_list);
r.fs_list = fs_list;
r.g = zeros(n_fs, 1);
r.dr = zeros(n_fs, 1);
r.thdnf = zeros(n_fs, 1);
r.pf = zeros(n_fs, n_test);
r.n_fail = 0;

for a = 1:n_fs
        v = -ones(n_test,1); % Set pass/fail test verdict to not executed
        tn = 1;
        t.fs1 = fs_list(a);
        t.fs2 = fs_list(a);
        [v(1) r.g(a)] = g_test(t);
        [v(2) r.thdnf(a)] = thdnf_test(t);
        %[v(2) r.dr(a)] = dr_test(t);
        %v(4) = fr_test(t);

        %% Done, store pass/fail
        r.pf(a, :) = v;
        idx = find(v > 0);
        r.n_fail = r.n_fail + sum(v(idx));

end

%% Print table with test summary: Gain
fn = sprintf('%s/g_%s.txt', reports, comp);
print_val(fn, fs_list, r.g, r.pf, 'Gain (dB)');

%% Print table with test summary: THD+N vs. frequency
fn = sprintf('%s/thdnf_%s.txt', reports, comp);
print_val(fn, fs_list, r.thdnf, r.pf, 'Worst-case THD+N vs. frequency');

%% Print table with test summary: DR
%fn = sprintf('%s/dr_%s.txt', reports, comp);
%print_val(fn, fs_list, r.dr, r.pf, 'Dynamic range (dB CCIR-RMS)');

fprintf('\n');
fprintf('Number of failed tests = %d\n', r.n_fail);

end


%%
%% Test execution with help of common functions
%%

function [fail, thdnf] = thdnf_test(t)

%% Reference: AES17 6.3.2 THD+N ratio vs. frequency
test = test_defaults_volume(t);
test.thdnf_max = t.thdnf_db_max;

test.f_start = 20;
test.f_end = test.fs * 0.41667; % 20 kHz @ 48 kHz
test.fu = test.fs * 0.41667; % 20 kHz @ 48 kHz

%% Create input file
test = thdnf_test_input(test);

%% Run test
test = test_run_volume(test, t);

%% Measure
test = thdnf_test_measure(test);

%% For EQ use the -20dBFS result and ignore possible -1 dBFS fail
thdnf = max(test.thdnf_low);
if thdnf > test.thdnf_max
        fail = 1;
else
        fail = 0;
end
delete_check(test.fn_in);
delete_check(test.fn_out);

%% Print
volume_test_result_print(t, 'THD+N ratio vs. frequency', 'THDNF');

end


function [fail, g_db] = g_test(t)

%% Reference: AES17 6.2.2 Gain
test = test_defaults_volume(t);
test.fu = test.fs * 0.41667; % 20 kHz @ 48 kHz
test.g_db_tol = t.g_db_tol;

%% Create input file
test = g_test_input(test);

%% Run test
test = test_run_volume(test, t);

%% EQ gain at test frequency
channel = 1;
test.g_db_expect = 0;

%% Measure
test = g_test_measure(test);

%% Get output parameters
fail = test.fail;
g_db = test.g_db;
delete_check(test.fn_in);
delete_check(test.fn_out);

end

function [fail, dr_db] = dr_test(t)

%% Reference: AES17 6.4.1 Dynamic range
test = test_defaults_volume(t);
test.dr_db_min = t.dr_db_min;

%% Create input file
test = dr_test_input(test);

%% Run test
test = test_run_volume(test, t);

%% Measure
test = dr_test_measure(test);

%% Get output parameters
fail = test.fail;
dr_db = test.dr_db;
delete_check(test.fn_in);
delete_check(test.fn_out);

end



function [fail, pm_range_db, range_hz, fr3db_hz] = fr_test(t)

%% Reference: AES17 6.2.3 Frequency response
test = test_defaults_volume(t);

test.f_lo = 20;
test.f_hi = 0.999*min(t.fs1/2,t.fs2/2);
test.f_max = 0.999*min(t.fs1/2, t.fs2/2);

%% Create input file
test = fr_test_input(test);

%% Run test
test = test_run_volume(test, t);

%% Measure
test = volume_mask(test, t, 0.5);
test = fr_test_measure(test);

fail = test.fail;
pm_range_db = test.rp;
range_hz = [test.f_lo test.f_hi];
fr3db_hz = test.fr3db_hz;
delete_check(test.fn_in);
delete_check(test.fn_out);

%% Print
volume_test_result_print(t, 'Frequency response', 'FR', test.ph, test.fh);

end

function test = volume_mask(test, t, tol)
%% Create mask from theoretical frequency response calculated from coefficients
%  and align mask to be relative 997 Hz response
j = 1;
for channel = t.ch
        [b, ~] = volume_decode_blob(t.blob, t.eq(channel));
        h = freqz(b, 1, test.f, test.fs);
        m = 20*log10(abs(h));
        i997 = find(test.f > 997, 1, 'first')-1;
        m = m - m(i997);
        test.mask_f = test.f;
        test.mask_lo(:,j) = m-tol;
        test.mask_hi(:,j) = m+tol;
        j = j+1;
end

end

function test = test_defaults_volume(t)
test.bits_in = t.bits_in;
test.bits_out = t.bits_out;
test.nch = t.nch;
test.ch = t.ch;
test.fs = t.fs1;
test.mask_f = [];
test.mask_lo = [];
test.mask_hi = [];
test.rp_max = [];
end

function test = test_run_volume(test, t)
test.ex = './volume_test';
test.arg = { num2str(t.bits_in), num2str(t.bits_out), num2str(test.fn_in), num2str(test.fn_out)};
delete_check(test.fn_out);
test = test_run(test);
end

function volume_test_result_print(t, testverbose, testacronym, ph, fh)
tstr = sprintf('%s Volume %d Hz', testverbose, t.fs1);
if nargin > 3
        nph = length(ph);
        for i=1:nph
                title(ph(i), tstr);
        end
else
        title(tstr);
end
if nargin > 4
        nfh = length(fh);
        for i=1:nfh
                figure(fh(i));
                pfn = sprintf('plots/%s_volume_%d_%d.png', ...
                        testacronym, t.fs1, i);
                print(pfn, '-dpng');
        end
else
        pfn = sprintf('plots/%s_volume_%d.png', testacronym, t.fs1);
        print(pfn, '-dpng');
end
end


%%
%% The next are results printing functions
%%

function  print_val(fn, fs_list, val, pf, valstr)
n_fs = length(fs_list);
fh = fopen(fn,'w');
fprintf(fh,'\n');
%fprintf(fh,'%s\n', valstr);
fprintf(fh,'%8s, ', 'Fs (kHz)');
fprintf(fh,'%s', valstr);
fprintf(fh,'\n');
for a = 1:n_fs
        fprintf(fh,'%8.1f, ', fs_list(a)/1e3);
        if pf(a,1) < 0
                cstr = 'x';
        else
                if isnan(val(a))
                        cstr = '-';
                else
                        cstr = sprintf('%8.2f', val(a));
                end
        end
        fprintf(fh,'%8s', cstr);
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
                        cstr = sprintf('%4.2f (%4.1f)', ...
                                fr_db(a,b), fr_hz(a,b,2)/1e3);
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
