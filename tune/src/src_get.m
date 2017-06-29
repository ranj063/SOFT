function src = src_get(cnv)

% src_get - calculate coefficients for a src
%
% src = src_get(cnv);
%
% cnv - src parameters
% src - src result
%

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

if exist('OCTAVE_VERSION', 'builtin')
        use_firpm = 0;
else
        use_firpm = 0;
end

if abs(cnv.fs1-cnv.fs2) < 1
        %% Return minimum needed for scripts to work
        src.L=1;
        src.M=1;
        src.odm=1;
        src.idm=1;
        src.MOPS=0;
        src.c_pb = 0;
        src.c_sb = 0;
        src.fir_delay_size = 0;
        src.out_delay_size = 0;
        src.blk_in = 1;
        src.blk_out = 1;
        src.gain = 1;
        return
end

%% fractional SRC parameters
[L, M] = src_factor1_lm(cnv.fs1, cnv.fs2);
src.L = L;
src.M = M;
src.num_of_subfilters = L;
[src.idm src.odm] = src_find_l0m0(src.L, src.M);
min_fs = min(cnv.fs1,cnv.fs2);
src.f_pb = min_fs*cnv.c_pb;
src.f_sb = min_fs*cnv.c_sb;
src.c_pb = cnv.c_pb;
src.c_sb = cnv.c_sb;
src.fs1 = cnv.fs1;
src.fs2 = cnv.fs2;
src.rp = cnv.rp;
src.rs = cnv.rs;

%% FIR PM design
fs3 = src.L*cnv.fs1;
f = [src.f_pb src.f_sb];
a = [1 0];

% Kaiser fir is not equiripple, can allow more ripple in the passband
% lower freq.
kdev = [(10^(4*cnv.rp/20)-1)/(10^(4*cnv.rp/20)+1) 10^(-cnv.rs/20)];
[kn0, kw, kbeta, kftype] = kaiserord(f, a, kdev, fs3);
if use_firpm
        dev = [(10^(cnv.rp/20)-1)/(10^(cnv.rp/20)+1) 10^(-cnv.rs/20)];
        [n0,fo,ao,w] = firpmord(f,a,dev,fs3);
else
        n0 = kn0;
end
n0 = round(n0*0.9);

nfir_increment = src.L;
nsf0 = (n0+1)/nfir_increment;
if nsf0 > floor(nsf0)
        n = (floor(nsf0)+1)*nfir_increment-1;
else
        n = n0;
end

src.subfilter_length = (n+1)/src.num_of_subfilters;
src.filter_length = n+1;
nfir = floor(n/2)*2; % Make order even
f_sb = linspace(src.f_sb, fs3/2, 100);
stopband_not_ok = 1;
while stopband_not_ok
        if use_firpm
                if nfir > 2000
                        b = fir1(nfir, kw, kftype, kaiser(nfir+1, kbeta));
                else
                        b = firpm(nfir,fo,ao,w);
                end
        else
                b = fir1(nfir, kw, kftype, kaiser(nfir+1, kbeta));
        end
        h_sb = freqz(b, 1, f_sb, fs3);
        m_sb = 20*log10(abs(h_sb));
        delta = cnv.rs+max(m_sb);
        fprintf('Delta=%f dB, N=%d\n', delta, nfir);
        if delta < 0
                stopband_not_ok = 0;
        else
                src.filter_length = src.filter_length + nfir_increment;
                nfir = nfir + nfir_increment;
        end
end

src.subfilter_length = src.filter_length/src.num_of_subfilters;
src.b = zeros(src.filter_length,1);
src.gain = 1;
src.b(1:nfir+1) = b*src.L;
m = max(abs(b));
gmax = (32767/32768)/m;
maxshift = floor(log(gmax)/log(2));
src.b = src.b * 2^maxshift;
src.gain = 1/2^maxshift;
src.shift = maxshift;

%% Reorder coefficients
src.coefs2 = zeros(src.filter_length,1);
src.coefs = zeros(src.filter_length,1);

for n = 1:src.num_of_subfilters
        i11 = (n-1)*src.subfilter_length+1;
        i12 = i11+src.subfilter_length-1;
        i21 = (src.num_of_subfilters-n)*src.subfilter_length+1;
        i22 = i21+src.subfilter_length-1;
        src.coefs(i11:i12) = src.b(n:src.num_of_subfilters:end);
        src.coefs2(i21:i22) = src.b(n:src.num_of_subfilters:end);
end

src.halfband = 0;
src.blk_in = M;
src.blk_out = L;
src.MOPS = cnv.fs1/M*src.filter_length/1e6;
src.fir_delay_size = src.subfilter_length + ...
        (src.num_of_subfilters-1)*src.idm + src.blk_in;
src.out_delay_size = (src.num_of_subfilters-1)*src.odm + 1;

end
