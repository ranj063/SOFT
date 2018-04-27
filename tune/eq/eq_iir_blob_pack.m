function blob8 = eq_iir_blob_pack(bs, endian)

%% Pack equalizer struct to bytes
%
% blob8 = eq_iir_blob_pack(bs, endian)
% bs - blob struct
% endian - use 'little' or 'big'
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

%% Shift values for little/big endian
switch lower(endian)
        case 'little'
                sh = [0 -8 -16 -24];
        case 'big'
                sh = [-24 -16 -8 0];
        otherwise
                error('Unknown endiannes');
end

%% Pack as 8 bits, header
nbytes_head = (2+bs.platform_max_channels)*4;
nbytes_coef = length(bs.all_coefficients)*4;
nbytes = nbytes_head+nbytes_coef;
blob8 = uint8(zeros(1,nbytes));

j = 1;
blob8(j:j+3) = w2b(bs.platform_max_channels, sh); j=j+4;
blob8(j:j+3) = w2b(bs.number_of_responses_defined, sh); j=j+4;

for i=1:bs.platform_max_channels
        blob8(j:j+3) = w2b(bs.assign_response(i), sh); j=j+4;
end

%% Pack coefficients
for i=1:length(bs.all_coefficients)
        blob8(j:j+3) = w2b(bs.all_coefficients(i), sh); j=j+4;
end
printf('Blob size is %d bytes.\n', nbytes);


end

function bytes = w2b(word, sh)
bytes = uint8(zeros(1,4));
bytes(1) = bitand(bitshift(word, sh(1)), 255);
bytes(2) = bitand(bitshift(word, sh(2)), 255);
bytes(3) = bitand(bitshift(word, sh(3)), 255);
bytes(4) = bitand(bitshift(word, sh(4)), 255);
end
