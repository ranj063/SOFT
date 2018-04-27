function  test = multi_ch_tones(test)

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
% Author: Ranjani Sridharan <ranjani.sridharan@linux.intel.com>
%

%% Adjust tone lengt to integer number of samples
test.nt = round(test.tl*test.fs); % Make number of samples per tone
test.tl = test.nt/test.fs;        % an integer by adjusting tl.
test.nf = length(test.f); % Number of frequencies
test.na = length(test.a); % Number of amplitudes

%% Test start and end marker tones
[h1, mark_start] = sync_chirp(test.fs, 'up');
[h2, mark_end] = sync_chirp(test.fs, 'down');
test.mark_t = mark_start.t;
test.mark_a = mark_start.a;
test.mark_a_db = mark_start.a_db;
test.ts = mark_start.t;

%% Idle time to start and end
t_idle0 = 0.5;
n_idle = round(test.fs*t_idle0);
t_idle = n_idle/test.fs;
x = zeros(test.na*test.nt +mark_start.n +mark_end.n +2*n_idle, test.nch, 'int32');

%% Add markers
idx1 = n_idle+1;
idx2 = length(x)-n_idle-mark_end.n;
for ch=1:test.nch
        x(idx1:idx1+mark_start.n-1, ch) = dither_and_quantize(h1, test.bits);
        x(idx2:idx2+mark_end.n-1, ch) = dither_and_quantize(h2, test.bits);
%% Dither also idle parts
        x(1:n_idle, ch) = dither_and_quantize(zeros(n_idle,1), test.bits);
        x(end-n_idle+1:end, ch) = dither_and_quantize(zeros(n_idle,1), test.bits);
end

%% Add discrete frequencies sweep
n_r = round(test.fs*test.tr);
win = ones(test.nt,1);
win(1:n_r) = linspace(0,1,n_r);
win(end-n_r+1:end) = linspace(1,0,n_r);
for m=1:test.nch
        i0 = n_idle+mark_start.n;
        for n=1:test.na
                i1 = i0 + 1;
                i2 = i1+test.nt-1;
                if test.nf > 1
                        f = test.f(round(unifrnd(1, test.nf)));
                else
                        f = test.f;
                end
                if test.na > 1
                        a = test.a(n);
                else
                        a = test.a;
                end
                fprintf('Mixing %.0f Hz %.1f dBFS sine for channel no: %d...\n', 
                  f, 20*log10(a), m);
                s = multitone(test.fs, f, a, test.tl);
                x(i1:i2, m) = dither_and_quantize(s.*win, test.bits);
                i0 = i2;
        end
end

%% Output
fprintf('Writing output data file %s...\n', test.fn_in);
write_ascii_data(x, test.fn_in);
write_wav("output.wav", x, test.fs, test.bits);
fprintf('Done.\n')

end



