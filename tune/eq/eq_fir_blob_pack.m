function blob8 = eq_fir_blob_pack(bs, endian)

%% Pack equalizer struct to bytes
%
% blob8 = eq_fir_blob_pack(bs, endian)
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

%% Endiannes of blob
switch lower(endian)
        case 'little'
                sh = [0 -8];
        case 'big'
                sh = [-8 0];
        otherwise
                error('Unknown endiannes');
end

%% Pack as 16 bits
nh16 = 2+bs.platform_max_channels;
h16 = zeros(1,nh16, 'int16');
h16(1) = bs.platform_max_channels;
h16(2) = bs.number_of_responses_defined;
for i=1:bs.platform_max_channels
        h16(2+i) = bs.assign_response(i);
end

%% Merge header and coefficients, make even number of int16 to make it
%  multiple of int32
nc16 = length(bs.all_coefficients);
nb16 = ceil((nh16+nc16)/2)*2;
blob16 = zeros(1,nb16, 'int16');
blob16(1:nh16) = h16;
blob16(nh16+1:nh16+nc16) = int16(bs.all_coefficients);

%% Pack as 8 bits
nb8 = length(blob16)*2;
blob8 = zeros(1, nb8, 'uint8');
j = 1;
for i = 1:length(blob16)
        blob8(j:j+1) = w2b(blob16(i), sh);
        j = j+2;
end
printf('Blob size is %d bytes.\n', nb8);

end


function bytes = w2b(word, sh)
bytes = uint8(zeros(1,2));
bytes(1) = bitand(bitshift(word, sh(1)), 255);
bytes(2) = bitand(bitshift(word, sh(2)), 255);
end
