function eq = loudness2_iir(fs, fn)

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

if nargin < 2
        fn = 1;
end
if nargin < 1
        fs = 48e3;
end

%% Derived from Fletcher-Munson curves for 80 and 60 phon
f = [ 20,21,22,24,25,27,28,30,32,34,36,38,40,43,45,48,51,54,57,60,64, ...
        68,72,76,81,85,90,96,102,108,114,121,128,136,144,153,162,171, ...
        182,192,204,216,229,243,257,273,289,306,324,344,364,386,409, ...
        434,460,487,516,547,580,614,651,690,731,775,821,870,922,977, ...
        1036,1098,1163,1233,1307,1385,1467,1555,1648,1747,1851,1962, ...
        2079,2203,2335,2474,2622,2779,2945,3121,3308,3505,3715,3937, ...
        4172,4421,4686,4966,5263,5577,5910,6264,6638,7035,7455,7901, ...
        8373,8873,9404,9966,10561,11193,11861,12570,13322,14118,14962, ...
        15856,16803,17808,18872,20000];

m = [ 0.00,-0.13,-0.27,-0.39,-0.52,-0.64,-0.77,-0.89,-1.02,-1.16,  ...
        -1.31,-1.46,-1.61,-1.76,-1.91,-2.07,-2.24,-2.43,-2.64,-2.85, ...
        -3.04,-3.21,-3.35,-3.48,-3.62,-3.78,-3.96,-4.16,-4.35,-4.54, ...
        -4.72,-4.90,-5.08,-5.26,-5.45,-5.64,-5.83,-6.02,-6.19,-6.37, ...
        -6.57,-6.77,-6.98,-7.19,-7.40,-7.58,-7.76,-7.92,-8.08,-8.25, ...
        -8.43,-8.60,-8.76,-8.92,-9.08,-9.23,-9.38,-9.54,-9.69,-9.84, ...
        -9.97,-10.09,-10.18,-10.26,-10.33,-10.38,-10.43,-10.48,-10.54, ...
        -10.61,-10.70,-10.78,-10.85,-10.91,-10.95,-10.98,-11.02, ...
        -11.05,-11.07,-11.10,-11.11,-11.11,-11.10,-11.10,-11.11, ...
        -11.14,-11.17,-11.20,-11.21,-11.22,-11.21,-11.20,-11.20, ...
        -11.21,-11.21,-11.20,-11.17,-11.11,-11.02,-10.91,-10.78, ...
        -10.63,-10.46,-10.25,-10.00,-9.72,-9.39,-9.02,-8.62,-8.19, ...
        -7.73,-7.25,-6.75,-6.25,-5.75,-5.28,-4.87,-4.54,-4.33,-4.30];

%% Design EQ
eq = eq_defaults();
eq.fs = fs;
eq.target_f = f;
eq.target_m_db = m;
eq.enable_fir = 0;
eq.enable_iir = 1;
if 1
        eq.peq = [ ...
                eq.PEQ_LS1 40 +2 NaN ; ...
                eq.PEQ_LS1 80 +3 NaN ; ...
                eq.PEQ_LS1 200 +3 NaN ; ...
                eq.PEQ_LS1 400 +3 NaN ; ...
                eq.PEQ_HS2 13000 +7 NaN ; ...
                ];
end
if 0
        eq.enable_fir = 1;
        eq.fir_length = 63;
        eq.fir_autoband = 0;
        eq.fmin_fir = 300;
        eq.fmax_fir = 20e3;
        eq.fir_minph = 1;
end

eq = eq_compute(eq);

%% Normalize loudness of EQ
eq = eq_norm(eq);

%% Plot
eq_plot_iirfir(eq, fn);

end
