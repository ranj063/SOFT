function src_2stage(fs_in, fs_out, delete_in, delete_out)

% src_2stage - export src conversions for given fs_in and fs_out
%
% src_2tage(fs_in, fs_out)
%
% fs_in  - vector of input sample rates
% fs_out - vector of output sample rates
%
% If fs_in and and fs_out are omitted the default is used
% fs_in = [8e3 16e3 24e3 32e3 44.1e3 48e3 96e3 192e3];
%
% If fs_out is omitted the default is
% fs_out = fs_in
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

%% Default input rates
if nargin < 1
        fs_in = [8e3 11025 16e3 22050 24e3 32e3 44100 48e3 96e3 192e3];
end

%% Default output rates
if nargin < 2
        fs_out = [8e3 11025 16e3 22050 24e3 32e3 44100 48e3];
end

%% Default rate combinations to exclude
if nargin < 4
        i1 = [  8000  8000  8000 11025 11025 11025 11025 11025 11025 16000 16000 16000 ];
        o1 = [ 11025 22050 44100  8000 16000 22050 24000 32000 44100 11025 22050 44100 ];

        i2 = [ 22050 22050 22050 22050 22050 22050 24000 24000 24000 32000 32000 32000 ];
        o2 = [  8000 11025 16000 24000 32000 44100 11025 22050 44100 11025 22050 44100 ];

        i3 = [ 44100 44100 44100 44100 44100 44100 ];
        o3 = [  8000 11025 16000 22050 24000 32000 ];

        i4 = [ 96000 96000 96e3 96e3 96e3 96e3 96000 192e3 192e3 192e3 192e3 192e3 192e3 192e3 ];
        o4 = [ 11025 22050  8e3 16e3 24e3 32e3 44100  8e3 11025  16e3 22050  24e3  32e3 44100 ];

        rm_in  = [i1 i2 i3 i4];
        rm_out = [o1 o2 o3 o4];
end

hdir = mkdir_check('include');
rdir = mkdir_check('reports');

%% Find fractional conversion factors
nfsi = length(fs_in);
nfso = length(fs_out);
l_2s = zeros(2, nfsi, nfso);
m_2s = zeros(2, nfsi, nfso);
mops_2s = zeros(2, nfsi, nfso);
pb_1s = zeros(2,nfsi, nfso);
sb_2s = zeros(2,nfsi, nfso);
defs.fir_delay_size = 0;
defs.out_delay_size = 0;
defs.blk_in = 0;
defs.blk_out = 0;
defs.num_in_fs = nfsi;
defs.num_out_fs = nfso;
defs.stage1_times_max = 0;
defs.stage2_times_max = 0;
defs.stage_buf_size = 0;
h = 1;
for b = 1:nfso
        for a = 1:nfsi
                fs1 = fs_in(a);
                fs2 = fs_out(b);
                [l1, m1, l2, m2] = src_factor2_lm(fs1, fs2);
                fs3 = fs1*l1/m1;
                cnv1 = src_param(fs1, fs3);
                cnv2 = src_param(fs3, fs2);
                if (fs2 < fs1)
                        % When decimating 1st stage passband can be limited
                        % for wider transition band
                        f_pb = fs2*cnv2.c_pb;
                        cnv1.c_pb = f_pb/min(fs1,fs3);
                end
                if (fs2 > fs1)
                        % When interpolating 2nd stage passband can be limited
                        % for wider transition band
                        f_pb = fs1*cnv1.c_pb;
                        cnv2.c_pb = f_pb/min(fs2,fs3);
                end
                idx_in = find(rm_in == fs1);
                idx_out = find(rm_out(idx_in) == fs2);
                if length(idx_out) > 0 && length(idx_out) > 0
                else
                        src1 = src_get(cnv1);
                        src2 = src_get(cnv2);
                        k = gcd(src1.blk_out, src2.blk_in);
                        stage1_times = src2.blk_in/k;
                        stage2_times = stage1_times*src1.blk_out/src2.blk_in;
                        defs.stage1_times_max = max(defs.stage1_times_max, stage1_times);
                        defs.stage2_times_max = max(defs.stage2_times_max, stage2_times);
                        l_2s(:,a,b) = [src1.L src2.L];
                        m_2s(:,a,b) = [src1.M src2.M];
                        mops_2s(:,a,b) = [src1.MOPS src2.MOPS];
                        pb_2s(:,a,b) = [round(1e4*src1.c_pb) round(1e4*src2.c_pb)];
                        sb_2s(:,a,b) = [round(1e4*src1.c_sb) round(1e4*src2.c_sb)];
                        defs.fir_delay_size = max(defs.fir_delay_size, src1.fir_delay_size);
                        defs.out_delay_size = max(defs.out_delay_size, src1.out_delay_size);
                        defs.blk_in = max(defs.blk_in, src1.blk_in);
                        defs.blk_out = max(defs.blk_out, src1.blk_out);
                        defs.fir_delay_size = max(defs.fir_delay_size, src2.fir_delay_size);
                        defs.out_delay_size = max(defs.out_delay_size, src2.out_delay_size);
                        defs.blk_in = max(defs.blk_in, src2.blk_in);
                        defs.blk_out = max(defs.blk_out, src2.blk_out);
                        defs.stage_buf_size = max(defs.stage_buf_size, src1.blk_out*stage1_times);
                        src_export_coef(src1, 'int24', 'int32_t', hdir);
                        src_export_coef(src2, 'int24', 'int32_t', hdir);
                end
        end
end

%% Export modes table
src_export_table_2s(fs_in, fs_out, l_2s, m_2s, pb_2s, sb_2s, ...
        'int24', 'int32_t', 'reef/audio/coefficients/src/', hdir);
src_export_defines(defs, 'int24', hdir);

%% Print 2 stage conversion factors
fn = sprintf('%s/src_2stage.txt', rdir);
fh = fopen(fn,'w');
fprintf(fh,'\n');
fprintf(fh,'Two stage fractional SRC: Ratios\n');
fprintf(fh,'%8s, ', 'out \ in');
for a = 1:nfsi
        fprintf(fh,'%12.1f, ', fs_in(a)/1e3);
end
fprintf(fh,'\n');
for b = 1:nfso
        fprintf(fh,'%8.1f, ', fs_out(b)/1e3);
        for a = 1:nfsi
                cstr = print_ratios(l_2s, m_2s, a, b);
                fprintf(fh,'%12s, ', cstr);
        end
        fprintf(fh,'\n');
end
fprintf(fh,'\n');

%% Print 2 stage MOPS
fprintf(fh,'Dual stage fractional SRC: MOPS\n');
fprintf(fh,'%8s, ', 'out \ in');
for a = 1:nfsi
        fprintf(fh,'%8.1f, ', fs_in(a)/1e3);
end
fprintf(fh,'\n');
for b = 1:nfso
        fprintf(fh,'%8.1f, ', fs_out(b)/1e3);
        for a = 1:nfsi
                mops = sum(mops_2s(:,a,b));
                if sum(l_2s(:,a,b)) < eps
                        mops_str = 'x';
                else
                        mops_str = sprintf('%.2f', mops);
                end
                fprintf(fh,'%8s, ', mops_str);
        end
        fprintf(fh,'\n');
end
fprintf(fh,'\n');

%% Print 2 stage MOPS per stage
fprintf(fh,'Dual stage fractional SRC: MOPS per stage\n');
fprintf(fh,'%10s, ', 'out \ in');
for a = 1:nfsi
        fprintf(fh,'%10.1f, ', fs_in(a)/1e3);
end
fprintf(fh,'\n');
for b = 1:nfso
        fprintf(fh,'%10.1f, ', fs_out(b)/1e3);
        for a = 1:nfsi
                mops = mops_2s(:,a,b);
                if sum(l_2s(:,a,b)) < eps
                        mops_str = 'x';
                else
                        mops_str = sprintf('%.2f+%.2f', mops(1), mops(2));
                end
                fprintf(fh,'%10s, ', mops_str);
        end
        fprintf(fh,'\n');
end
fprintf(fh,'\n');
fclose(fh);
type(fn);

end

function d = mkdir_check(d)
if exist(d) ~= 7
        mkdir(d);
end
end

function cstr = print_ratios(l_2s, m_2s, a, b)
l1 = l_2s(1,a,b);
m1 = m_2s(1,a,b);
l2 = l_2s(2,a,b);
m2 = m_2s(2,a,b);
if l1+m2+l2+m2 == 0
        cstr = 'x';
else
        if m2 == 1
                if l2 == 1
                        cstr2 = '';
                else
                        cstr2 = sprintf('*%d', l2);
                end
        else
                cstr2 = sprintf('*%d/%d', l2, m2);
        end
        if m1 == 1
                cstr1 = sprintf('%d', l1);
        else
                cstr1 = sprintf('%d/%d', l1, m1);
        end
        cstr = sprintf('%s%s', cstr1, cstr2);
end
end
