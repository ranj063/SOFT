function src_export_table_2s(fs_in, fs_out, l_2s, m_2s, pb_2s, sb_2s, ctype, vtype, ppath, hdir)

% reef_src_export_table_2s - Export src setup table
%
% src_export_table_2s(fs_in, fs_out, l, m, pb, sb, ctype, vtype, hdir)
%
% The parameters are used to differentiate files for possibly many same
% conversion factor filters with possibly different characteristic.
%
% fs_in  - input sample rates
% fs_out - output sample rates
% l      - interpolation factors
% m      - decimation factors
% pb     - passband widths
% sb     - stopband start frequencies
% ctype  - coefficient quantization
% vtype  - C variable type
% ppath  - print directory prefix to header file name include
% hdir   - directory for header files
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

hfn = sprintf('src_%s_table.h', ctype);
fh = fopen(fullfile(hdir,hfn), 'w');

fprintf(fh, '/* SRC conversions */\n');
n_in = length(fs_in);
n_out = length(fs_out);
i=1;
all_modes = zeros(2*n_in*n_out,4);
for n=1:2
        for b=1:n_out
                for a=1:n_in
                        all_modes(i,:) = [ l_2s(n,a,b) m_2s(n,a,b) ...
                                pb_2s(n,a,b) sb_2s(n,a,b) ];
                        i=i+1;
                end
        end
end
unique_modes = unique(all_modes,'rows');
sm = size(unique_modes);
for i=1:sm(1)
        um_tmp = unique_modes(i,:);
        if isequal(um_tmp(1:2),[1 1]) || isequal(um_tmp(1:2),[0 0])
        else
                fprintf(fh, '#include <%ssrc_%s_%d_%d_%d_%d.h>\n', ...
                        ppath, ctype, um_tmp);
        end
end
fprintf(fh,'\n');

fprintf(fh, '/* SRC table */\n');
switch ctype
        case 'double'
                fprintf(fh, '%s fir_one = 1.0;\n', vtype);
                fprintf(fh, 'struct src_stage src_double_1_1_0_0 =  { 0, 0, 1, 1, 1, 1, 1, 0, 1.0, &fir_one };\n');
                fprintf(fh, 'struct src_stage src_double_0_0_0_0 =  { 0, 0, 0, 0, 0, 0, 0, 0, 0.0, &fir_one };\n');
        case 'int24'
                scale24 = 2^23;
                fprintf(fh, '%s fir_one = %d;\n', vtype, round(scale24*0.5));
                fprintf(fh, 'struct src_stage src_int24_1_1_0_0 =  { 0, 0, 1, 1, 1, 1, 1, 0, -1, &fir_one };\n');
                fprintf(fh, 'struct src_stage src_int24_0_0_0_0 =  { 0, 0, 0, 0, 0, 0, 0, 0,  0, &fir_one };\n');
        case 'int32'
                scale32 = 2^31;
                fprintf(fh, '%s fir_one = %d;\n', vtype, round(maxint32*0.5));
                fprintf(fh, 'struct src_stage src_int32_1_1_0_0 =  { 0, 0, 1, 1, 1, 1, 1, 0, -1, &fir_one };\n');
                fprintf(fh, 'struct src_stage src_int32_0_0_0_0 =  { 0, 0, 0, 0, 0, 0, 0, 0,  0, &fir_one };\n');
        otherwise
                error('Unknown coefficient type!');
end

fprintf(fh, 'int src_in_fs[%d] = {%d', n_in, fs_in(1));
for i=2:n_in
        fprintf(fh, ', %d', fs_in(i));
end
fprintf(fh, '};\n');

fprintf(fh, 'int src_out_fs[%d] = {%d', n_out, fs_out(1));
for i=2:n_out
        fprintf(fh, ', %d', fs_out(i));
end
fprintf(fh, '};\n');

for n = 1:2
        fprintf(fh, 'struct src_stage *src_table%d[%d][%d] = {\n', ...
                n, n_out, n_in);
        for b = 1:n_out
                fprintf(fh, '\t{');
                for a = 1:n_in
                        fprintf(fh, ' &src_%s_%d_%d_%d_%d', ...
                                ctype, l_2s(n,a,b), m_2s(n,a,b), ...
                                pb_2s(n,a,b), sb_2s(n,a,b));
                        if a < n_in
                                fprintf(fh, ',');
                        end
                end
                fprintf(fh, '}');
                if b < n_out
                        fprintf(fh, ',\n');
                else
                        fprintf(fh, '\n');
                end
        end
        fprintf(fh, '};\n');
end

fclose(fh);

end
