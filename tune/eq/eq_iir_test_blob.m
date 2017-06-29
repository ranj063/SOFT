%% Design effect EQs and bundle them to parameter block

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

blob_fn = 'eq_iir.blob';
endian = 'little';
fs = 48e3;

%% Design example EQs
eq_bb = bassboost_iir(fs, 1); % Design EQ
eq_ls1 = loudness1_iir(fs, 2); % Design EQ
eq_ls2 = loudness2_iir(fs, 3); % Design EQ

figure(4)
semilogx(eq_bb.f,eq_bb.tot_eq_db, eq_ls1.f,eq_ls1.tot_eq_db, eq_ls2.f,eq_ls2.tot_eq_db);
legend('Bass boost','Loudness1','Loudness2');
grid on;
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');

%% A passthru EQ with one tap
b_pass = [1 0 0];
a_pass = [1 0 0];

%% Quantize and pack filter coefficients plus shifts etc.
br_pass = eq_iir_blob_resp(b_pass, a_pass);
br_bb = eq_iir_blob_resp(eq_bb.b_p, eq_bb.a_p);
br_ls1 = eq_iir_blob_resp(eq_ls1.b_p, eq_ls1.a_p);
br_ls2 = eq_iir_blob_resp(eq_ls2.b_p, eq_ls2.a_p);

%% Build blob
platform_max_channels = 4; %  From Reef src/include/reef/stream.h
assign_response = zeros(1,platform_max_channels);
bs = eq_iir_blob_merge(platform_max_channels, 4, zeros(1,platform_max_channels), [br_pass br_bb br_ls1 br_ls2]);

%% Pack and write file
bp = eq_iir_blob_pack(bs, endian);
eq_blob_write(blob_fn, bp);
