function y = aweight(x, fs)

%% y = aweight(x, fs)
%
% Input
% x - input signal
% fs - sample rate
%
% Output
% y - weighted signal
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

%% Frequency grid for FIR design
np = 1000;
f = linspace(0, fs/2, 1000);

%% From https://en.wikipedia.org/wiki/A-weighting
RA = 12200^2*f.^4. ...
        / ((f.^2+20.6^2).*sqrt((f.^2+107.7^2).*(f.^2+739.9^2)).*(f.^2+12200^2));
A_db = 2.0+20*log10(RA);
m_fir2 = 10.^(A_db/20);
f_fir2 = 2*f/fs;
n_fir2 = 1000; % Seems sufficient for up to 192 kHz
bz = fir2(n_fir2, f_fir2, m_fir2);
y = filter(bz, 1, x);

if 0
        h = freqz(bz, 1, f, fs);
        my_A_db = 20*log10(abs(h));
        figure;
        plot(f, A_db, f, my_A_db);
        grid on; xlabel('Hz'); ylabel('dB');
end

end
