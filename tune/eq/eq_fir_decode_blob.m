function [b, a, stream_max_channels, number_of_responses, assign_response] = ...
        eq_fir_decode_blob(blobfn, resp_n)

%% Decode a FIR EQ binary blob
%
% [b, a, stream_max_channels, number_of_responses, assign_response] = ...
%        eq_fir_decode_blob(blobfn, resp_n)
%
%
% blobfn - file name of EQ setup blob
% resp_n - index of response to decode
%
% b - FIR coefficients
% a - no recursive filter part so this is returned as 1
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
blob16 = fread(fh, inf, 'int16');
fclose(fh);

stream_max_channels = blob16(1);
number_of_responses = blob16(2);
assign_response = blob16(3:2+stream_max_channels);

if resp_n > number_of_responses-1;
        error('Request of non-available response');
end

b16 = blob16(3+stream_max_channels:end);
j = 1;
b = [];
for i=1:number_of_responses
        filter_length = b16(j);
        input_shift = b16(j+1);
        output_shift = b16(j+2);
        if i-1 == resp_n
                bi = b16(j+3:j+2+filter_length);
                b = 2^(-input_shift)*2^(-output_shift)*double(bi)/32768;
        end
        j = j+filter_length+3;
end
a = 1;

end
