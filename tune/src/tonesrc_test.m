function n_fail = tonesrc_test(fs_in_list, fs_out_list)

%%
% tonesrc_test - test tonesrc executable objective audio quality parameters
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
        fs_in_list = [8e3 16e3 24e3 32e3 44100 48e3 96e3 192e3];
end
if nargin < 2
        fs_out_list = fs_in_list;
end


%% Loop all modes to test
ch = 1;
nch = 2;
gtol_db = 0.1;
drlimit_db = 140;
n_fail = 0;
n_fsi = length(fs_in_list);
n_fso = length(fs_out_list);
dr = zeros(n_fsi, n_fso);
g = zeros(n_fsi, n_fso);
pf = zeros(n_fsi, n_fso);
for b = 1:n_fso
        for a = 1:n_fsi
                fs1 = fs_in_list(a);
                fs2 = fs_out_list(b);
                [pf(a,b), dr(a,b), g(a,b), missing] = dr_test(fs1, fs2, gtol_db, drlimit_db, ch, nch);
                n_fail = n_fail + pf(a,b);
                if missing
                        pf(a, b) = -1;
                end
        end
end

%% Print table with test summary: DR
fn = 'reports/dr_tonesrc.txt';
print_dr(fn, fs_in_list, fs_out_list, dr, pf);

%% Print table with test summary: Gain
fn = 'reports/g_tonesrc.txt';
print_g(fn, fs_in_list, fs_out_list, g, pf);

%% Print table with test summary: pass/fail
fn = 'reports/pf_tonesrc.txt';
print_pf(fn, fs_in_list, fs_out_list, pf);

fprintf('\n');
fprintf('Number of failed tests = %d\n', n_fail);

%% Delete src in/out text files
if n_fail == 0
        delete_check('tonesrc_out.txt');
end

end

function y = trim_sample(x, t_start, fs)
y = x(round(t_start*fs:end));
end

function [fail, dr_db, g_db, missing] = dr_test(fs1, fs2, gtol_db, ...
        mindr_db, ch, nch)

%% AES17 6.4.1 Dynamic range

%% Run SRC with sine input -60 dB relative to maximum input
fn_out = 'tonesrc_out.txt';
t = 3.0;
t_start = 1.0; % Ignore first 1.0s of SRC output notch filter to settle
f = 997;
a_db =-60; % As in AES17, though this low level hides quite a lot of aliasing
a = 10^(a_db/20);
bits = 32;
src_out = run_tonesrc(fs1, fs2, f, a, t, ch, nch);
if length(src_out) < 1
        fail = 0; dr_db = 0; g_db = 0; missing = 1;
        return;
else
        missing = 0;
end
src_out = src_out / 2^(bits-1);

%% Notch filter
y0n = stdnotch(src_out, f, fs2);

%% CCIR-RMS weighting filter
y0w = stdweight(y0n, fs2);

%% Trim sample by removing first 1s to let the notch to apply
i1 = round(t_start*fs2)+1;
y = src_out(i1:end);
y_n = y0w(i1:end);

%% Gain, SNR
level_in = a_db;
level_out = level_dbfs(y);
level_final = level_dbfs(y_n);
g_db = level_out-level_in;
dr_db = level_out-level_final-a_db;
fprintf('Gain = %4.1f dB, DR = %5.1f dB\n', g_db, dr_db);
if 0
        plot(y);
        hold on
        plot(y_n,'g');
        hold off
end

%% Check pass/fail
fail = 0;
if abs(g_db) > gtol_db
        fprintf('Failed gain %f dB (max %f dB)\n', g_db, gtol_db);
        fail = 1;
end
if dr_db < mindr_db
        fprintf('Failed DR %f dB (min %f dB)\n', dr_db, mindr_db);
        fail = 1;
end

end


function [out, in] = run_tonesrc(fs1, fs2, f, a, t, ch, nch)
ex = './tonesrc_test';
fn_in = 'tonesrc_in.txt';
fn_out = 'tonesrc_out.txt';
src_cmd = sprintf('%s %d %f %f %f %d', ex, fs1, f, a, t, fs2);
fprintf('%s\n', src_cmd);
if exist(ex) == 2
else
        error('Can''t find compiled executable %s', ex);
end
if (ch < 1)
        ch = 1+round(rand(1,1)*(nch-1));
end
delete_check(fn_out);
system(src_cmd);
out = [];
if exist(fn_out)
	dir_fn_out = dir(fn_out);
	if dir_fn_out.bytes > 0
		out = load(fn_out);
		out = out(ch:nch:end);
	end
end

if exist(fn_in)
        in = load(fn_in);
        in = in(ch:nch:end);
else
        in = [];
end

end



function  print_dr(fn, fs_in_list, fs_out_list, dr, pf);

n_fsi = length(fs_in_list);
n_fso = length(fs_out_list);
fh = fopen(fn,'w');
fprintf(fh,'\n');
fprintf(fh,'SRC test result: DR dB(CCIR-RMS)\n');
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
                        cstr = sprintf('%5.1f', dr(a,b));
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

function  print_g(fn, fs_in_list, fs_out_list, g, pf);

n_fsi = length(fs_in_list);
n_fso = length(fs_out_list);
fh = fopen(fn,'w');
fprintf(fh,'\n');
fprintf(fh,'SRC test result: Gain dB\n');
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
                        cstr = sprintf('%8.5f', g(a,b));
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
                        cstr = sprintf('%d', pf(a,b,:));
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
