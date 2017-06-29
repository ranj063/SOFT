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
 *         Keyon Jie <yang.jie@linux.intel.comel.com>
 */

#include "common_test.h"
#include <reef/ipc.h>
//#include <uapi/tone_abi.h>

/* Keep this 1 to apply tone parameters from command line. Otherwise
 * default tone parameters are used.
 */
#define SEND_COMP_CMD_TONE 0

/* Internal buffer size */
#define TEST_BENCH_DEADLINE 1000 /* 1 ms */
#define TEST_BENCH_NCH 2 /* Stereo */
#define TEST_BENCH_RATE_MAX 48000 /* Hz */
#define TEST_BENCH_PERIOD_FRAMES (TEST_BENCH_RATE_MAX*TEST_BENCH_DEADLINE/1000000)
#define TEST_BENCH_FRAME_SIZE (4 * TEST_BENCH_NCH) /* 4bytes x  NCH */
#define INT_PERIOD_SIZE (TEST_BENCH_PERIOD_FRAMES * TEST_BENCH_FRAME_SIZE)

/* main firmware context */
static struct reef reef;

/*
 * Test Pipeline
 *
 * Tone(0) --- B(3) ---> SRC(1) --B(4)---> Filewrite(2)
 */

/*
 * Components used in static pipeline
 */

#define TONE_ID 0
#define SRC_ID 1
#define FW_ID 2
#define PIPE_ID 5
#define SCHED_ID TONE_ID /* Component ID=0 schedules pipeline */

static struct sof_ipc_comp_tone tone_p0[] = {
	/* ID = 0 */
	SPIPE_GENERIC(SPIPE_COMP(TONE_ID, SOF_COMP_TONE, sof_ipc_comp_tone)),
};

static struct sof_ipc_comp_src src_p0[] = {
	SPIPE_GENERIC(SPIPE_COMP(SRC_ID, SOF_COMP_SRC, sof_ipc_comp_src)), /* ID = 1 */
};

static struct sof_ipc_comp_filewrite filewrite_p0[] = {
	SPIPE_FILEWRITE(SPIPE_COMP(FW_ID, SOF_COMP_FILEWRITE,
	sof_ipc_comp_filewrite),
	"tonesrc_out.txt"), /* ID = 2 */
};

static struct scomps pipe0_scomps[] = {
	SCOMP(tone_p0),
	SCOMP(src_p0),
	SCOMP(filewrite_p0),
};

/*
 * Buffers used in static pipeline
 */
static struct sof_ipc_buffer buffer0[] = {
	/* B3  Tone0 -> SRC1 */
	SPIPE_BUFFER(3, INT_PERIOD_SIZE * 5),

	/* B4  SRC1 -> Filewrite1 */
	SPIPE_BUFFER(4, INT_PERIOD_SIZE * 5),
};

/* pipeline 0 component/buffer connections */
static struct sof_ipc_pipe_comp_connect c_connect0[] = {
	SPIPE_COMP_CONNECT(0, 3), /* p(0): Tone(0) -> B(3) */
	SPIPE_COMP_CONNECT(3, 1), /* p(0): B(3) ->  SRC(1) */
	SPIPE_COMP_CONNECT(1, 4), /* p(0): SRC(1) -> B(4) */
	SPIPE_COMP_CONNECT(4, 2), /* p(0): B(4) ->  Filewrite(2) */
};

/* the static pipelines */
static struct spipe spipe[] = {
	SPIPE(pipe0_scomps, buffer0, c_connect0),
};

/* pipeline, ID=5 */
struct sof_ipc_pipe_new pipeline[] = {
	SPIPE_PIPE(0, 0, PIPE_ID, SCHED_ID, TEST_BENCH_DEADLINE, TASK_PRI_LOW, \
		TEST_BENCH_PERIOD_FRAMES),
};

void usage(char *executable);

#if 0
static int tone_cmd_helper(struct comp_dev *cd, struct sof_ipc_ctrl_data *cmd, int index, int nch, int32_t value)
{
	int i;
	struct sof_ipc_ctrl_value_comp *cdata = (struct sof_ipc_ctrl_value_comp *) cmd->data->data;


	cmd->index = index;
	for (i = 0; i < nch; i++) {
		cdata[i].index = i;
		cdata[i].svalue = value;
	}

	return comp_cmd(cd, COMP_CMD_SET_DATA, cmd);
}
#endif

int main(int argc, char **argv)
{
	struct ipc_comp_dev *pcm_dev;
	struct pipeline *p;
	struct comp_dev *cd;
	struct filewrite_comp_data *fwcd;
	clock_t tic, toc;
	double c_realtime, t_exec;
	int fs1, fs2, n_out, ret, deadline, nch = TEST_BENCH_NCH; /* stereo */
	float t_tone, t_inc, t;
#if SEND_COMP_CMD_TONE
	struct sof_ipc_ctrl_data *cmd;
	float a_tone, f_tone;
	int32_t frequency, amplitude;
#endif

	/* Handle command line */
	if (argc == 6) {
		fs1 = atof(argv[1]);
#if SEND_COMP_CMD_TONE
		f_tone = atof(argv[2]);
		a_tone = atof(argv[3]);
#endif
		t_tone = atof(argv[4]);
		fs2 = atof(argv[5]);
	} else {
		usage(argv[0]);
		return EXIT_SUCCESS;
	}

	/* Init components */
	sys_comp_init();
	sys_comp_tone_init();
	sys_comp_src_init();
	sys_comp_filewrite_init();

	/* Set SRC sink rate into IPC, src_new() will copy it */
	src_p0[0].sink_rate = fs2;
	deadline = pipeline[0].deadline;
	pipeline[0].frames_per_sched = (int) (0.9999 + fs2 * deadline / 1e6);

	/* Construct pipeline, pass parameters, start */
	if (test_bench_pipeline_setup(&reef, &pipeline[0], &spipe[0]) < 0)
		exit(EXIT_FAILURE);

	deadline = pipeline[0].deadline;
	if (test_bench_pipeline_params_start(reef.ipc, fs1, nch,
		deadline, SCHED_ID, PIPE_ID) < 0)
		exit(EXIT_FAILURE);

	/* Prepare for comp_cmd() */
	pcm_dev = ipc_get_comp(reef.ipc, TONE_ID);
	p = pcm_dev->cd->pipeline;
	cd = pcm_dev->cd;

#if SEND_COMP_CMD_TONE
	/* Frequency & amplitude command for steady tone*/
	frequency = (int32_t) (f_tone * 65536); /* Q18.14 */
	amplitude = (int32_t) (a_tone * 2147483647); /* Q1.31 */
	cmd = malloc(sizeof(struct sof_ipc_ctrl_data)
		+ sizeof(struct sof_abi_hdr)
		+TEST_BENCH_NCH * sizeof(struct sof_ipc_ctrl_value_comp));
	cmd->comp_id = TONE_ID;
	cmd->type = SOF_CTRL_TYPE_VALUE_COMP_SET;
	cmd->cmd = SOF_CTRL_CMD_ENUM;
	cmd->num_elems = TEST_BENCH_NCH;
	cmd->data->size = TEST_BENCH_NCH * sizeof(struct sof_ipc_ctrl_value_comp);
	cmd->data->abi = SOF_ABI_VERSION;
	cmd->data->magic = SOF_ABI_MAGIC;
	cmd->data->comp_abi = SOF_TONE_ABI_VERSION;
	//cdata = (struct sof_ipc_ctrl_value_comp *) cmd->data->data;

	/* Frequency */
	ret = tone_cmd_helper(cd, cmd, SOF_TONE_IDX_FREQUENCY, nch, frequency);
	if (ret < 0) {
		printf("Error: Tone frequency set failed!\n");
		exit(EXIT_FAILURE);
	}

	/* Amplitude */
	ret = tone_cmd_helper(cd, cmd, SOF_TONE_IDX_AMPLITUDE, nch, amplitude);
	if (ret < 0) {
		printf("Error: Tone amplitude set failed!\n");
		exit(EXIT_FAILURE);
	}
#endif
	/* Get filewrite state */
	pcm_dev = ipc_get_comp(reef.ipc, FW_ID);
	fwcd = comp_get_drvdata(pcm_dev->cd);

	/*
	 * Run copy until t_tone is reached
	 */
	pcm_dev = ipc_get_comp(reef.ipc, SCHED_ID);
	p = pcm_dev->cd->pipeline;
	cd = pcm_dev->cd;
	test_bench_disable_trace(); /* Reduce print output */
	tic = clock();

	t_inc = 1e-6 * deadline;
	t = t_inc;
	while (t < t_tone) {
		pipeline_schedule_copy(p, 0);
		t = t + t_inc;
	}

	/* Done! Next reset and free pipeline */
	toc = clock();
	test_bench_enable_trace();
	ret = pipeline_reset(p, cd);
	if (ret < 0) {
		printf("Failed pipeline_reset()\n");
		exit(EXIT_FAILURE);
	}
	pipeline_free(p);

	n_out = fwcd->fws.n;
	t_exec = (double) (toc - tic) / CLOCKS_PER_SEC;
	c_realtime = (double) n_out / nch / fs2 / t_exec;
	printf("wrote %d output samples,\n", n_out);
	printf("test execution time was %.2f us, %.2f x realtime.\n",
		1e3 * t_exec, c_realtime);

	return EXIT_SUCCESS;
}

void usage(char *executable)
{
	fprintf(stderr, "Usage: %s <tone rate> <frequency> <amplitude> <seconds> <out rate>\n",
		executable);
	fprintf(stderr, "E.g. \"%s 44100 997 0.5 1.0 48000\"\n", executable);
}
