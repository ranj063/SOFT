function success=src_export_coef(src, ctype, vtype, hdir)

% reef_src_export_coef - Export FIR coefficients
%
% success=reef_src_export_coef(src, ctype, hdir)
%
% src   - src definition struct
% ctype - 'float','int32', or 'int24'
% vtype - 'float','int32_t'
% hdir  - directory for header files
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

if src.L == src.M
        success = 0;
else
        pbi =  round(src.c_pb*1e4);
        sbi =  round(src.c_sb*1e4);
        hfn = sprintf('%s/src_%s_%d_%d_%d_%d.h', hdir, ctype, src.L, src.M, ...
                pbi, sbi);
        vfn = sprintf('src_%s_%d_%d_%d_%d_fir', ctype, src.L, src.M, pbi, sbi);
        sfn = sprintf('src_%s_%d_%d_%d_%d', ctype, src.L, src.M, pbi, sbi);

        fprintf('Exporting %s ...\n', hfn);
        fh = fopen(hfn, 'w');

        switch ctype
                case 'double'
                        fprintf(fh, 'const %s %s[%d] = {\n', ...
                                vtype, vfn, src.filter_length);
                        fprintf(fh,'\t%16.9e', src.coefs(1));
                        for n=2:src.filter_length
                                fprintf(fh, ',\n');
                                fprintf(fh,'\t%16.9e', src.coefs(n));
                        end
                        fprintf(fh,'\n\n};');
                case 'int32'
                        fprintf(fh, 'const %s %s[%d] = {\n', ...
                                vtype, vfn, src.filter_length);
                        cint32 = round(2^31*src.coefs);
                        fprintf(fh,'\t%d', cint32(1));
                        for n=2:src.filter_length
                                fprintf(fh, ',\n');
                                fprintf(fh,'\t%d', cint32(n));
                        end
                        fprintf(fh,'\n\n};');
                case 'int24'
                        fprintf(fh, 'const %s %s[%d] = {\n', ...
                                vtype, vfn, src.filter_length);
                        cint24 = round(2^23*src.coefs);
                        fprintf(fh,'\t%d', cint24(1));
                        for n=2:src.filter_length
                                fprintf(fh, ',\n');
                                fprintf(fh,'\t%d', cint24(n));
                        end
                        fprintf(fh,'\n\n};');
                otherwise
                        error('Unknown C type %s !!!', ctype);
        end

        fprintf(fh, '\n');
        switch ctype
                case 'double'
                        fprintf(fh, 'struct src_stage %s = {%d, %d, %d, %d, %d, %d, %d, %d, %f, %s};\n', ...
                                sfn, src.idm, src.odm, src.num_of_subfilters, ...
                                src.subfilter_length, src.filter_length, ...
                                src.blk_in, src.blk_out, src.halfband, ...
                                src.gain, vfn);
                case 'int24'
                        fprintf(fh, 'struct src_stage %s = {%d, %d, %d, %d, %d, %d, %d, %d, %d, %s};\n', ...
                                sfn, src.idm, src.odm, src.num_of_subfilters, ...
                                src.subfilter_length, src.filter_length, ...
                                src.blk_in, src.blk_out, src.halfband, ...
                                src.shift, vfn);
                case 'int32'
                        fprintf(fh, 'struct src_stage %s = {%d, %d, %d, %d, %d, %d, %d, %d, %d, %s};\n', ...
                                sfn, src.idm, src.odm, src.num_of_subfilters, ...
                                src.subfilter_length, src.filter_length, ...
                                src.blk_in, src.blk_out, src.halfband, ...
                                src.shift, vfn);
        end
        %fprintf(fh, '\n');
        fclose(fh);
        success = 1;
end

end
