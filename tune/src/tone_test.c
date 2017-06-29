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
 */

#include "common_test.h"
#include <reef/ipc.h>
#include <uapi/tone.h>

#define BYPASS_TONE_CMDS 0 /* Keep normally 0 */

/* Internal buffer size */
#define TEST_BENCH_DEADLINE 1000 /* 1 ms */
#define TEST_BENCH_NCH 2 /* Stereo */
#define TEST_BENCH_RATE_MAX 192000 /* Hz */
#define TEST_BENCH_PERIOD_FRAMES (TEST_BENCH_RATE_MAX*TEST_BENCH_DEADLINE/1000000)
#define TEST_BENCH_FRAME_SIZE (4 * TEST_BENCH_NCH) /* 4bytes x  NCH */
#define INT_PERIOD_SIZE (TEST_BENCH_PERIOD_FRAMES * TEST_BENCH_FRAME_SIZE)

/* main firmware context */
static struct reef reef;

/*
 * Test Pipeline (ID = 3)
 *
 * Tone(0) --- B(2) ---> Filewrite(1)
 */

#define TONE_ID 0
#define FW_ID 1
#define PIPE_ID 3
#define SCHED_ID TONE_ID /* Component ID=0 schedules pipeline */

/*
 * Components used in static pipeline
 */

static struct sof_ipc_comp_tone tone_p0[] = {
	SPIPE_GENERIC(SPIPE_COMP(TONE_ID, SOF_COMP_TONE, sof_ipc_comp_tone)),
};

static struct sof_ipc_comp_filewrite filewrite_p0[] = {
	SPIPE_FILEWRITE(SPIPE_COMP(FW_ID, SOF_COMP_FILEWRITE,
	sof_ipc_comp_filewrite), "tone_out.txt"),
};

static struct scomps scomps_p0[] = {
	SCOMP(tone_p0),
	SCOMP(filewrite_p0),
};

/*
 * Buffers used in static pipeline
 */
static struct sof_ipc_buffer buffers_p0[] = {
	/* B(2) */
	SPIPE_BUFFER(2, INT_PERIOD_SIZE * 2),
};

/* pipeline 0 component/buffer connections */
static struct sof_ipc_pipe_comp_connect connect_p0[] = {
	SPIPE_COMP_CONNECT(0, 2), /* p(0): Tone(0) -> B(2) */
	SPIPE_COMP_CONNECT(2, 1), /* p(0): B(2) ->  Filewrite(1) */
};

/* the static pipelines */
static struct spipe spipe[] = {
	SPIPE(scomps_p0, buffers_p0, connect_p0),
};

/* pipelines */
struct sof_ipc_pipe_new pipeline[] = {
	SPIPE_PIPE(0, 0, PIPE_ID, SCHED_ID, TEST_BENCH_DEADLINE, TASK_PRI_LOW, \
		TEST_BENCH_PERIOD_FRAMES),
};

void usage(char *executable);

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

int main(int argc, char **argv)
{
	struct ipc_comp_dev *pcm_dev;
	struct pipeline *p;
	struct comp_dev *cd;
	struct filewrite_comp_data *fwcd;
	clock_t tic, toc;
	double c_realtime, t_exec;
	int fs, n_out, ret, deadline;
	float t_tone, t, t_inc;
	int nch = TEST_BENCH_NCH; /* Stereo */
	int demo = 0;

	struct sof_ipc_ctrl_value_chan *switchdata;
	struct sof_ipc_ctrl_data *ctrl_cmd;
	struct sof_ipc_ctrl_data *cmd;
	float a_tone = 0.5;
	float f_tone = 440;
	int32_t frequency, amplitude;
	int i;

	/* Handle command line */
	switch (argc) {
	case 4:
		fs = atof(argv[1]);
		t_tone = atof(argv[2]);
		demo = atoi(argv[3]);
		break;
	case 5:
		fs = atof(argv[1]);
		t_tone = atof(argv[2]);
		f_tone = atof(argv[3]);
		a_tone = atof(argv[4]);
		break;
	default:
		usage(argv[0]);
		return EXIT_SUCCESS;
	}

	/* Init components */
	sys_comp_init();
	sys_comp_tone_init();
	sys_comp_filewrite_init();

	/* Set pipeline frames_per_sched */
	deadline = pipeline[0].deadline;
	pipeline[0].frames_per_sched = (int) (0.9999 + fs * deadline / 1e6);

	/* Construct pipeline, pass parameters, start */
	if (test_bench_pipeline_setup(&reef, &pipeline[0], &spipe[0]) < 0)
		exit(EXIT_FAILURE);

	if (test_bench_pipeline_params_start(reef.ipc, fs, nch,
		deadline, SCHED_ID, PIPE_ID) < 0)
		exit(EXIT_FAILURE);

	pcm_dev = ipc_get_comp(reef.ipc, TONE_ID);
	p = pcm_dev->cd->pipeline;
	cd = pcm_dev->cd;

#if BYPASS_TONE_CMDS == 0
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

	if (demo == 1) {
		/* Repeated beeps or tone sweep example. */
		tone_cmd_helper(cd, cmd, SOF_TONE_IDX_FREQUENCY, nch, 17145922); /* C4 note 261.626 Hz Q16.16 */
		tone_cmd_helper(cd, cmd, SOF_TONE_IDX_AMPLITUDE, nch, (int32_t) (a_tone * 2147483647)); /* Q1.31 */
		tone_cmd_helper(cd, cmd, SOF_TONE_IDX_FREQ_MULT, nch, 1137589835); /* Go up half-step Q2.30 1.0595 */
		tone_cmd_helper(cd, cmd, SOF_TONE_IDX_AMPL_MULT, nch, 956973408); /* Attenuated beeps Q2.30 -1 dB 0.89125 */
		tone_cmd_helper(cd, cmd, SOF_TONE_IDX_LENGTH, nch, 1200); /* 150 ms in 125 us steps */
		tone_cmd_helper(cd, cmd, SOF_TONE_IDX_PERIOD, nch, 1600); /* 200 ms in 125 us steps */
		tone_cmd_helper(cd, cmd, SOF_TONE_IDX_REPEATS, nch, 10); /* 10 beeps e.g. B4 .. C4 */
		tone_cmd_helper(cd, cmd, SOF_TONE_IDX_LIN_RAMP_STEP, nch, 10737418); /* Linear ramp step Q1.31 0.005 */
	}

	/* Unmute */
	ctrl_cmd = malloc(sizeof(struct sof_ipc_ctrl_data)
		+ sizeof(struct sof_abi_hdr)
		+ TEST_BENCH_NCH * sizeof(struct sof_ipc_ctrl_value_chan));
	ctrl_cmd->comp_id = TONE_ID;
	ctrl_cmd->type = SOF_CTRL_TYPE_VALUE_CHAN_SET;
	ctrl_cmd->cmd = SOF_CTRL_CMD_SWITCH;
	ctrl_cmd->num_elems = TEST_BENCH_NCH;
	switchdata = (struct sof_ipc_ctrl_value_chan *) ctrl_cmd->chanv;
	for (i = 0; i < TEST_BENCH_NCH; i++) {
		switchdata[i].channel = i;
		switchdata[i].value = 1;
	}
	ret = comp_cmd(cd, COMP_CMD_SET_VALUE, ctrl_cmd);
	free(ctrl_cmd);
	if (ret < 0) {
		printf("Warning: Failed unmute component command.\n");
		pipeline_free(p);
		exit(EXIT_FAILURE);
	}
#endif

	/* Get pointer to filewrite */
	pcm_dev = ipc_get_comp(reef.ipc, FW_ID);
	fwcd = comp_get_drvdata(pcm_dev->cd);

	/*
	 * Run copy until time limit is met
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
	c_realtime = (double) n_out / nch / fs / t_exec;
	printf("wrote %d output samples to file \"%s\",\n", n_out, filewrite_p0[0].fn);
	printf("test execution time was %.2f us, %.2f x realtime.\n", 1e3 * t_exec, c_realtime);

	return EXIT_SUCCESS;
}

void usage(char *executable)
{
	fprintf(stderr, "Usage: %s <sample rate> <seconds> <frequency> <amplitude>\n", executable);
	fprintf(stderr, "Demos: %s <sample rate> <seconds> <demo sound #1..N>\n", executable);
	fprintf(stderr, "E.g.\n\"%s 48000 1.0 997 0.5\"\n", executable);
	fprintf(stderr, "\"%s 48000 1.0 1\"\n", executable);
}
