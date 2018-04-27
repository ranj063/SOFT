function [b_t, a_t] = eq_define_parametric_eq(peq, fs)

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

% Parametric types
PEQ_HP1 = 1; PEQ_HP2 = 2; PEQ_LP1 = 3; PEQ_LP2 = 4;
PEQ_LS1 = 5; PEQ_LS2 = 6; PEQ_HS1 = 7; PEQ_HS2 = 8;
PEQ_PN2 = 9; PEQ_LP4 = 10; PEQ_HP4 = 11;

%figure(100);
%fv = logspace(log10(20),log10(20e3),500);
%hold on;
sp = size(peq);
b_t = 1; a_t = 1;
for i=1:sp(1)
        type = peq(i,1);
        f = peq(i,2);
        g = peq(i,3);
        bw = peq(i,4);
        if f < fs/2
                switch peq(i,1)
                        case PEQ_HP1, [b0, a0] = butter(1, 2*f/fs, 'high');
                        case PEQ_HP2, [b0, a0] = butter(2, 2*f/fs, 'high');
                        case PEQ_HP4, [b0, a0] = butter(4, 2*f/fs, 'high');
                        case PEQ_LP1, [b0, a0] = butter(1, 2*f/fs);
                        case PEQ_LP2, [b0, a0] = butter(2, 2*f/fs);
                        case PEQ_LP4, [b0, a0] = butter(4, 2*f/fs);
                        case PEQ_LS1, [b0, a0] = low_shelf_1st(f, g, fs);
                        case PEQ_LS2, [b0, a0] = low_shelf_2nd(f, g, fs);
                        case PEQ_HS1, [b0, a0] = high_shelf_1st(f, g, fs);
                        case PEQ_HS2, [b0, a0] = high_shelf_2nd(f, g, fs);
                        case PEQ_PN2, [b0, a0] = peak_2nd(f, g, bw, fs);
                        otherwise
                                error('Unknown parametric EQ type');
                end
                %h = freqz(b0, a0, fv, fs);
                %semilogx(fv, 20*log10(abs(h)),'g--');
                b_t=conv(b_t, b0); a_t = conv(a_t, a0);
        end
end
%h = freqz(b_t, a_t, fv, fs);
%semilogx(fv, 20*log10(abs(h)),'b');
%hold off; grid on; axis([20 20e3 -1 12]);
%xlabel('Frequency (Hz)'); ylabel('Magnitude (dB)');
%print -dpng peq.png;
end

function [b, a] = low_shelf_1st(fhz, gdb, fs)
zw = 2*pi*fhz;
w = wmap(zw, fs);
glin = 10^(gdb/20);
bs = [1 glin*w];
as = [1 w];
[b, a] = my_bilinear(bs, as, fs);
end

function [b, a] = low_shelf_2nd(fhz, gdb, fs)
zw = 2*pi*fhz;
w = wmap(zw, fs);
glin = 10^(gdb/20);
bs = [1 w*sqrt(2*glin) glin*w^2];
as = [1 w*sqrt(2) w^2];
[b, a] = my_bilinear(bs, as, fs);
end

function [b, a] = high_shelf_1st(fhz, gdb, fs)
zw = 2*pi*fhz;
w = wmap(zw, fs);
glin = 10^(gdb/20);
bs = [glin w];
as = [1 w];
[b, a] = my_bilinear(bs, as, fs);
end

function [b, a] = high_shelf_2nd(fhz, gdb, fs)
zw = 2*pi*fhz;
w = wmap(zw, fs);
glin = 10^(gdb/20);
bs = [glin w*sqrt(2*glin) w^2];
as = [1 w*sqrt(2) w^2];
[b, a] = my_bilinear(bs, as, fs);
end

function [b, a] = peak_2nd(fhz, gdb, bw, fs)
zw = 2*pi*fhz;
w = wmap(zw, fs);
glin = 10^(gdb/20);
qn = 1/(bw^2*sqrt(glin));
qd = sqrt(glin)/bw^2;
bs = [1 w/qn w^2];
as = [1 w/qd w^2];
[b, a] = my_bilinear(bs, as, fs);
end

function [b, a] = my_bilinear(sb, sa, fs)
t = 1/fs;
[b, a] = bilinear(sb, sa, t);
end

function sw = wmap(w, fs)
t = 1/fs;
sw = 2/t*tan(w*t/2);
end
