#ifndef _COMMON_TEST_H
#define _COMMON_TEST_H

/*
 * Copyright (c) 2017, Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Intel Corporation nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Seppo Ingalsuo <seppo.ingalsuo@linux.intel.com>
 *         Liam Girdwood <liam.r.girdwood@linux.intel.com>
 *         Keyon Jie <yang.jie@linux.intel.com>
 */

#include <stdint.h>
#include <stddef.h>
#include <time.h>
#include <stdio.h>

#include <uapi/ipc.h>
#include <reef/reef.h>
#include <reef/audio/component.h>
#include <reef/audio/format.h>

/*
 * Static Buffer Convenience Constructors.
 */
#define SPIPE_BUFFER(bid, bsize) \
	{.comp.id = bid, .size = bsize}
#define SPIPE_COMP_CONNECT(source, sink) \
	{.source_id = source, .sink_id = sink}

/*
 * Static Component Convenience Constructors.
 */
#define SCONFIG {.frame_fmt = SOF_IPC_FRAME_S32_LE, .periods_sink = 2,\
	.periods_source = 2, .preload_count = 1}
#define SPIPE_COMP(cid, ctype, csize) \
	{.id = cid, .type = ctype, .hdr.size = sizeof(struct csize)}
#define SPIPE_HOST(scomp, hno_irq, hdmac, hchan, hconfig) \
	{.comp = scomp, .no_irq = hno_irq, .dmac_id = hdmac,\
	.dmac_chan = hchan, .dmac_config = hconfig,\
	.config = SCONFIG}
#define SPIPE_DAI(scomp, ddai_type, ddai_idx, ddmac, dchan, dconfig) \
	{.comp = scomp, .type = ddai_type, .index = ddai_idx, \
	.dmac_id = ddmac, .dmac_chan = dchan, .dmac_config = dconfig, \
	.config = SCONFIG}
#define SPIPE_VOL(scomp, vmin, vmax) \
	{.comp = scomp, .min_value = vmin, .max_value = vmax,\
	.config = SCONFIG}
#define SPIPE_GENERIC(scomp) {.comp = scomp, .config = SCONFIG}
#define SPIPE_FILEREAD(scomp, fn_in) {.comp = scomp, .config = SCONFIG, .fn = fn_in}
#define SPIPE_FILEWRITE(scomp, fn_out) {.comp = scomp, .config = SCONFIG, .fn = fn_out}

/*
 * Static Pipeline Convenience Constructor
 */
#define SPIPE_PIPE(pid, pcore, pcid, psid, pdeadline, ppriority, pframes) \
	{.pipeline_id = pid, .core = pcore, .deadline = pdeadline, \
        .priority = ppriority, .comp_id = pcid, .sched_id = psid, \
	.frames_per_sched = pframes}
#define SPIPE_PIPE_CONNECT(psource, bsource, bid, psink, bsink) \
	{.pipeline_source_id = psource, .comp_source_id = bsource, \
	.buffer_id = bid, .pipeline_sink_id = psink, .comp_sink_id = bsink}

/*
 * Static pipeline container and constructor
 */

#define SCOMP(ccomps) \
	{.comps = (struct sof_ipc_comp *)ccomps, .num_comps \
		= ARRAY_SIZE(ccomps)}

#define SPIPE(ncomp, sbuffer, sconnect) \
	{.scomps = ncomp, .num_scomps = ARRAY_SIZE(ncomp), \
	.buffer = sbuffer, .num_buffers = ARRAY_SIZE(sbuffer), \
	.connect = sconnect, .num_connections = ARRAY_SIZE(sconnect)}

#define FILEREAD_FN_MAXLENGTH 256
#define FILEWRITE_FN_MAXLENGTH 256

struct scomps {
	struct sof_ipc_comp *comps;
	uint32_t num_comps;
};

struct spipe {
	struct scomps *scomps;
	uint32_t num_scomps;
	struct sof_ipc_buffer *buffer;
	uint32_t num_buffers;
	struct sof_ipc_pipe_comp_connect *connect;
	uint32_t num_connections;
};

struct fileread_state {
	char fn[FILEREAD_FN_MAXLENGTH];
	FILE *fh;
	int reached_eof;
	int n;
};

struct fileread_comp_data {
	uint32_t period_bytes;
	uint32_t channels;
	uint32_t frame_bytes;
	uint32_t rate;
	struct fileread_state frs;
	int (*fileread_func)(struct comp_dev *dev, struct comp_buffer *sink,
		struct comp_buffer *source, uint32_t frames);

};

struct filewrite_state {
	char fn[FILEWRITE_FN_MAXLENGTH];
	FILE *fh;
	int write_fail;
	int n;
	int (*filewrite_func)(struct comp_dev *dev, struct comp_buffer *sink,
		struct comp_buffer *source, uint32_t frames);

};

struct filewrite_comp_data {
	uint32_t period_bytes;
	uint32_t channels;
	uint32_t frame_bytes;
	uint32_t rate;
	struct filewrite_state fws;
	int (*filewrite_func)(struct comp_dev *dev, struct comp_buffer *sink,
		struct comp_buffer *source, uint32_t frames);

};

/* Test bench component IPC */

struct sof_ipc_comp_fileread {
	struct sof_ipc_comp comp;
	struct sof_ipc_comp_config config;
	char *fn;
} __attribute__((packed));

struct sof_ipc_comp_filewrite {
	struct sof_ipc_comp comp;
	struct sof_ipc_comp_config config;
	char *fn;
} __attribute__((packed));

int scheduler_init(struct reef *reef);
void sys_comp_fileread_init(void);
void sys_comp_filewrite_init(void);

int test_bench_pipeline_setup(struct reef *reef,
	struct sof_ipc_pipe_new pipeline[], struct spipe spipe[]);

int test_bench_pipeline_params_start(struct ipc *ipc,
	int32_t fs, int nch, int deadline, int sched_id, int pipe_id);

int test_bench_pipeline_params(struct ipc *ipc, int32_t fs,
	int nch, int host_comp_id, int deadline, uint32_t pipe_id);

void test_bench_enable_trace(void);
void test_bench_disable_trace(void);


#endif
