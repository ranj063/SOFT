/* Simple test bench versions of SOF functions */

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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <reef/task.h>
#include <reef/alloc.h>
#include <reef/ipc.h>
#include <reef/dai.h>
#include <reef/dma.h>
#include <reef/work.h>
#include <reef/wait.h>
#include <reef/intel-ipc.h>
#include <reef/audio/pipeline.h>
#include "common_test.h"

/* Use simplified host test bench versions for features */
#define TB_HOST_ALLOC_FREE_MEMCPY 1
#define TB_TRACE 1
#define TB_SCHEDULE 1
#define TB_IPC 1
#define TB_TIMER 1
#define TB_WORK 1

int ipc_stream_send_position(struct comp_dev *cdev,
	struct sof_ipc_stream_posn *posn) {
	return 0;
}

struct dai *dai_get(uint32_t type, uint32_t index) {
	return NULL;
}

struct dma *dma_get(int dmac_id) {
	return NULL;
}

void work_cancel_default(struct work *work) {
	return;
}

void _trace_error_atomic(uint32_t event) {
	return;
}

#if TB_IPC
struct ipc *_ipc;
#endif

#if TB_SCHEDULE

struct schedule_data {
	spinlock_t lock;
	struct list_item list; /* list of tasks in priority queue */
	uint32_t clock;
};
static struct schedule_data *sch;
#endif

#if TB_TRACE
static int test_bench_trace = 1;
#endif

#if TB_TRACE

static char replace_blank(char x)
{
	char y = x;

	if ((y < '!') || (y > 'z')) {
			y = ' ';
	}
	return y;
}

void _trace_event(uint32_t event)
{
	char a, b, c;

	if (test_bench_trace > 0) {
		a = replace_blank((char) (event & 0xff));
		b = replace_blank((char) ((event >> 8) & 0xff));
		c = replace_blank((char) ((event >> 16) & 0xff));
		printf("Trace %10d %c%c%c\n", event, c, b, a);
	}
}

void _trace_error(uint32_t event)
{
	char a, b, c;

	if (test_bench_trace > 0) {
		a = replace_blank((char) (event & 0xff));
		b = replace_blank((char) ((event >> 8) & 0xff));
		c = replace_blank((char) ((event >> 16) & 0xff));
		printf("Error %10d %c%c%c\n", event, c, b, a);
	}
}

void test_bench_enable_trace(void)
{
	test_bench_trace = 1;
	printf("Trace print enabled\n");
}

void test_bench_disable_trace(void)
{
	test_bench_trace = 0;
	printf("Trace print disabled\n");
}
#endif

#if TB_HOST_ALLOC_FREE_MEMCPY

void *rmalloc(int zone, int module, size_t bytes)
{
	return malloc(bytes);
}

void *rzalloc(int zone, int module, size_t bytes)
{
	void *x;
	x = malloc(bytes);
	bzero(x, bytes);
	return x;
}

void rfree(void *ptr)
{
	free(ptr);
}

void *rballoc(int zone, int module, size_t bytes)
{
	return malloc(bytes);
}

void rbfree(void *ptr)
{
	free(ptr);
}

void *xthal_memcpy(void *dest, const void *src, size_t size)
{
	return memcpy(dest, src, size);
}

#endif

#if TB_SCHEDULE

void schedule_task_complete(struct task *task)
{

	list_item_del(&task->list);
	task->state = TASK_STATE_COMPLETED;
}

void schedule_task(struct task *task, uint64_t start, uint64_t deadline)
{

	task->deadline = deadline;
	list_item_prepend(&task->list, &sch->list);
	task->state = TASK_STATE_QUEUED;

	if (task->func)
		task->func(task->data);

	schedule_task_complete(task);
}

void schedule(void)
{
}

int scheduler_init(struct reef *reef)
{
	trace_pipe("ScI");

	sch = rzalloc(RZONE_SYS, RFLAGS_NONE, sizeof(*sch));
	list_init(&sch->list);
	spinlock_init(&sch->lock);

	return 0;
}

void schedule_task_idle(struct task *task, uint64_t deadline)
{
}

#endif

#if TB_IPC

int platform_ipc_init(struct ipc *ipc)
{
	struct intel_ipc_data *iipc;
	//uint32_t imrd;
	int i;

	_ipc = ipc;

	/* init ipc data */
	iipc = rzalloc(RZONE_SYS, RFLAGS_NONE, sizeof(struct intel_ipc_data));
	ipc_set_drvdata(_ipc, iipc);
	_ipc->dsp_msg = NULL;
	list_init(&ipc->empty_list);
	list_init(&ipc->msg_list);
	spinlock_init(&ipc->lock);

	for (i = 0; i < MSG_QUEUE_SIZE; i++)
		list_item_prepend(&ipc->message[i].list, &ipc->empty_list);

	/* allocate page table buffer */
	iipc->page_table = rballoc(RZONE_SYS, RFLAGS_NONE,
		HOST_PAGE_SIZE);
	if (iipc->page_table)
		bzero(iipc->page_table, HOST_PAGE_SIZE);

	/* PM */
	iipc->pm_prepare_D3 = 0;

	return 0;
}

int ipc_stream_send_xrun(struct comp_dev *cdev,
	struct sof_ipc_stream_posn *posn)
{
	return 0;
}

#endif

#if TB_WORK

void work_schedule_default(struct work *w, uint64_t timeout)
{
}

#endif

int test_bench_pipeline_setup(struct reef *reef,
	struct sof_ipc_pipe_new *pipeline, struct spipe *spipe)
{
	struct scomps *sc;
	int i, j;
	struct sof_ipc_comp *c;
	struct ipc *ipc;

	/* Init IPC */
	if (ipc_init(reef) < 0) {
		fprintf(stderr, "error: IPC init\n");
		return -EINVAL;
	}
	ipc = reef->ipc;

	/* Init scheduler */
	if (scheduler_init(reef) < 0) {
		fprintf(stderr, "error: scheduler init\n");
		return -EINVAL;
	}

	/* Init pipeline system */
	if (pipeline_init() < 0) {
		fprintf(stderr, "error: pipeline init\n");
		return -EINVAL;
	}


	/* Register components */
	sc = spipe->scomps;
	for (i = 0; i < (int) spipe->num_scomps; i++) {
		c = sc[i].comps;
		for (j = 0; j < (int) sc[i].num_comps; j++) {
			if (ipc_comp_new(ipc, c) < 0) {
				fprintf(stderr, "error: comp register\n");
				return -EINVAL;
			}
			c = (void *) c + c->hdr.size;
		}
	}

	/* Pipeline new must be after component registration since the
	 * scheduling component	must exist.
	 */
	if (ipc_pipeline_new(ipc, pipeline) < 0) {
		fprintf(stderr, "error: pipeline new\n");
		return -EINVAL;
	}

	/* Register buffers */
	for (i = 0; i < (int) spipe->num_buffers; i++) {
		if (ipc_buffer_new(ipc, &spipe->buffer[i]) < 0) {
			fprintf(stderr, "error: buffer new\n");
			return -EINVAL;
		}
	}

	/* Connect components */
	for (i = 0; i < (int) spipe->num_connections; i++) {
		if (ipc_comp_connect(ipc, &spipe->connect[i]) < 0) {
			fprintf(stderr, "error: comp connect\n");
			return -EINVAL;
		}
	}

	/* Pipeline complete */
	ipc_pipeline_complete(ipc, pipeline->comp_id);

	return 0;
}

int test_bench_pipeline_params_start(struct ipc *ipc,
	int32_t fs, int nch, int deadline, int sched_id, int pipe_id)
{
	struct ipc_comp_dev *pcm_dev;
	struct pipeline *p;
	struct comp_dev *cd;
	int ret;

	ret = test_bench_pipeline_params(ipc, fs, nch, sched_id, deadline,
		pipe_id);
	if (ret < 0) {
		printf("error: pipeline params\n");
		return -EINVAL;
	}

	/* Get IPC component device for pipeline */
	pcm_dev = ipc_get_comp(ipc, sched_id);
	if (pcm_dev == NULL) {
		printf("error: ipc get comp\n");
		return -EINVAL;
	}

	/* Point to pipeline */
	cd = pcm_dev->cd;
	p = pcm_dev->cd->pipeline;

	/* Component prepare */
	ret = pipeline_prepare(p, cd);

	/* Start the pipeline */
	ret = pipeline_cmd(p, cd, COMP_CMD_START, NULL);
	if (ret < 0)
		printf("Warning: Failed start pipeline command.\n");

	return ret;
}

int test_bench_pipeline_params(struct ipc *ipc, int32_t fs,
	int nch, int host_comp_id, int deadline, uint32_t pipe_id)
{
	int fs_period, ret = 0;
	struct ipc_comp_dev *pcm_dev;
	struct pipeline *p;
	struct comp_dev *cd;
	struct sof_ipc_pcm_params params;

	/* Compute period from sample rates */
	fs_period = (int) (0.9999 + fs * deadline / 1e6);

	params.comp_id = pipe_id;
	params.params.buffer_fmt = SOF_IPC_BUFFER_INTERLEAVED;
	params.params.frame_fmt = SOF_IPC_FRAME_S32_LE;
	params.params.direction = SOF_IPC_STREAM_PLAYBACK;
	params.params.rate = fs;
	params.params.channels = nch;
	switch (params.params.frame_fmt) {
	case(SOF_IPC_FRAME_S16_LE):
		params.params.sample_container_bytes = 2;
		params.params.sample_valid_bytes = 2;
		params.params.host_period_bytes = fs_period * nch *
			params.params.sample_container_bytes;
		break;
	case(SOF_IPC_FRAME_S24_4LE):
		params.params.sample_container_bytes = 4;
		params.params.sample_valid_bytes = 3;
		params.params.host_period_bytes = fs_period * nch *
			params.params.sample_container_bytes;
		break;
	case(SOF_IPC_FRAME_S32_LE):
		params.params.sample_container_bytes = 4;
		params.params.sample_valid_bytes = 4;
		params.params.host_period_bytes = fs_period * nch *
			params.params.sample_container_bytes;
		break;
	default:
		printf("error: invalid frame format\n");
		return -EINVAL;
	}

	/* Get IPC component device for pipeline*/
	pcm_dev = ipc_get_comp(ipc, host_comp_id);
	if (pcm_dev == NULL) {
		printf("error: ipc get comp\n");
		return -EINVAL;
	}

	/* Point to pipeline */
	cd = pcm_dev->cd;
	p = pcm_dev->cd->pipeline;

	/* Pipeline params */
	if (p == NULL) {
		printf("error: pipeline NULL\n");
		return -EINVAL;
	}

	ret = pipeline_params(p, cd, &params);
	if (ret < 0) {
		printf("error: pipeline_params\n");
	}

	return ret;
}
