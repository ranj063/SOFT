function eq = loudness1_iir(fs, fn)

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

%% Derived from Fletcher-Munson curves for 80 and 70 phon
f = [ 20,21,22,24,25,27,28,30,32,34,36,38,40,43,45,48,51,54,57,60,64, ...
        68,72,76,81,85,90,96,102,108,114,121,128,136,144,153,162,171,182, ...
        192,204,216,229,243,257,273,289,306,324,344,364,386,409,434,460, ...
        487,516,547,580,614,651,690,731,775,821,870,922,977,1036,1098, ...
        1163,1233,1307,1385,1467,1555,1648,1747,1851,1962,2079,2203,2335, ...
        2474,2622,2779,2945,3121,3308,3505,3715,3937,4172,4421,4686, ...
        4966,5263,5577,5910,6264,6638,7035,7455,7901,8373,8873,9404,9966,  ...
        10561,11193,11861,12570,13322,14118,14962,15856,16803,17808, ...
        18872,20000];

m = [ 0.00,-0.09,-0.18,-0.25,-0.31,-0.36,-0.41,-0.46,-0.51,-0.57,-0.65,  ...
        -0.72,-0.80,-0.88,-0.96,-1.04,-1.12,-1.21,-1.31,-1.42,-1.52, ...
        -1.63,-1.73,-1.82,-1.91,-1.99,-2.07,-2.15,-2.22,-2.30,-2.37, ...
        -2.45,-2.54,-2.63,-2.72,-2.82,-2.92,-3.02,-3.12,-3.23,-3.34, ...
        -3.44,-3.55,-3.65,-3.74,-3.82,-3.89,-3.96,-4.04,-4.13,-4.24, ...
        -4.34,-4.44,-4.52,-4.60,-4.67,-4.74,-4.82,-4.89,-4.97,-5.04, ...
        -5.10,-5.14,-5.18,-5.21,-5.24,-5.26,-5.29,-5.33,-5.38,-5.44, ...
        -5.49,-5.52,-5.53,-5.52,-5.50,-5.50,-5.52,-5.55,-5.59,-5.61, ...
        -5.62,-5.61,-5.60,-5.59,-5.59,-5.60,-5.60,-5.60,-5.60,-5.60, ...
        -5.60,-5.60,-5.60,-5.60,-5.60,-5.59,-5.57,-5.54,-5.50,-5.46, ...
        -5.40,-5.33,-5.23,-5.09,-4.92,-4.72,-4.51,-4.30,-4.09,-3.88, ...
        -3.68,-3.48,-3.29,-3.11,-2.92,-2.75,-2.57,-2.39,-2.20];

%% Design EQ
eq = eq_defaults();
eq.fs = fs;
eq.target_f = f;
eq.target_m_db = m;
eq.enable_fir = 0;
eq.enable_iir = 1;
if 1
        eq.peq = [ ...
                eq.PEQ_LS1 40 +1 NaN ; ...
                eq.PEQ_LS1 80 +1 NaN ; ...
                eq.PEQ_LS1 90 +2 NaN ; ...
                eq.PEQ_LS1 300 +2 NaN ; ...
                eq.PEQ_HS1 13000 +1.8 NaN ; ...
                eq.PEQ_HS1 12000 +1.8 NaN ; ...
                ];
end
eq = eq_compute(eq);

%% Normalize loudness of EQ
eq = eq_norm(eq);

%% Plot
eq_plot_iirfir(eq, fn);

end
