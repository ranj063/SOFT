function [b, a, stream_max_channels, number_of_responses, assign_response] = ...
        eq_iir_decode_blob(blobfn, resp_n)

%% Decode an IIR EQ binary blob
%
% [b, a, stream_max_channels, number_of_responses, assign_response] = ...
%        eq_fir_decode_blob(blobfn, resp_n)
%
% blobfn - file name of EQ setup blob
% resp_n - index of response to decode
%
% b - numerator coefficients
% a - denominator coefficients
% stream_max_channnels - numbers of channels in blob
% assign response - vector of EQ indexes assigned to channels
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

if nargin < 2
        resp_n = 0;
end

fh = fopen(blobfn, 'rb');
blob = fread(fh, inf, 'uint32');
fclose(fh);

stream_max_channels = blob(1);
number_of_responses = blob(2);
assign_response = blob(3:2+stream_max_channels);

if resp_n > number_of_responses-1;
        error('Request of non-available response');
end

j = 3+stream_max_channels;
b = 1.0;
a = 1.0;
for i=1:number_of_responses
        %blob(j:j+1)
        section_length = blob(j)*7+2;
        if i-1 == resp_n
                nbiquads = blob(j);
                k = j+2;
                for j=1:nbiquads
                        %fprintf('%10d\n',blob(k:k+6));
                        ai = signedint(blob(k+1:-1:k),32)';
                        bi = signedint(blob(k+4:-1:k+2),32)';
                        shifti = signedint(blob(k+5),32);
                        gaini = signedint(blob(k+6),16);
                        b0 = bi/2^30;
                        a0 = [1 -(ai/2^30)];
                        gain = gaini/2^14;
                        b0 = b0 * 2^(-shifti) * gain;
                        b = conv(b, b0);
                        a = conv(a, a0);
                        k = k+7;
                end
        end
        j = j+section_length;
end

end

function y = signedint(x, bits)
y = x;
idx = find(x > 2^(bits-1)-1);
y(idx) = x(idx)-2^bits;
end
