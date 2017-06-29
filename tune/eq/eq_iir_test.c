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
#include <uapi/eq.h>
#include <reef/ipc.h>
#include <audio/eq_iir.h>

/* Internal buffer */
#define TEST_BENCH_DEADLINE 1000 /* 1 ms */
#define TEST_BENCH_NCH 2 /* Stereo */
#define TEST_BENCH_RATE_MAX 192000 /* Hz */
#define TEST_BENCH_PERIOD_FRAMES (TEST_BENCH_RATE_MAX*TEST_BENCH_DEADLINE/1000000)
#define TEST_BENCH_FRAME_SIZE (4 * TEST_BENCH_NCH) /* 4bytes x  NCH */
#define INT_PERIOD_SIZE (TEST_BENCH_PERIOD_FRAMES * TEST_BENCH_FRAME_SIZE)

/* main firmware context */
static struct reef reef;

/*
 * Test Pipeline (ID=5)
 *
 * Fileread(0) --- B1(3) ---> EQ(1) --- B2(4)---> Filewrite(2)
 */

#define FR_ID 0
#define EQ_ID 1
#define FW_ID 2
#define PIPE_ID 5
#define SCHED_ID FR_ID /* Component ID=0 schedules pipeline */

/*
 * Components used in static pipeline
 */

static struct sof_ipc_comp_fileread fileread_p0[] = {
	SPIPE_FILEREAD(SPIPE_COMP(FR_ID, SOF_COMP_FILEREAD,
	sof_ipc_comp_fileread), "eq_in.txt"),
};

static struct sof_ipc_comp_filewrite filewrite_p0[] = {
	SPIPE_FILEWRITE(
	SPIPE_COMP(FW_ID, SOF_COMP_FILEWRITE, sof_ipc_comp_filewrite),
	"eq_out.txt"),
};

static struct sof_ipc_comp_eq_iir eq_iir_p0[] = {
	SPIPE_GENERIC(
	SPIPE_COMP(EQ_ID, SOF_COMP_EQ_IIR, sof_ipc_comp_eq_iir)),
};

static struct scomps scomps_p0[] = {
	SCOMP(fileread_p0),
	SCOMP(eq_iir_p0),
	SCOMP(filewrite_p0),
};

/*
 * Buffers used in static pipeline 2.
 */
static struct sof_ipc_buffer buffers_p0[] = {
	SPIPE_BUFFER(3, INT_PERIOD_SIZE * 2), /* B(3) */
	SPIPE_BUFFER(4, INT_PERIOD_SIZE * 2), /* B(4) */
};

/* pipeline 0 component/buffer connections */
static struct sof_ipc_pipe_comp_connect connect_p0[] = {
	SPIPE_COMP_CONNECT(0, 3), /* p(0): Read(0) -> B1(3) */
	SPIPE_COMP_CONNECT(3, 1), /* p(0): B1(3) ->  EQ(1) */
	SPIPE_COMP_CONNECT(1, 4), /* p(0): EQ(1) -> B2(4) */
	SPIPE_COMP_CONNECT(4, 2), /* p(0): B2(4) ->  Write(2) */
};

/* the static pipelines */
static struct spipe spipe[] = {
	SPIPE(scomps_p0, buffers_p0, connect_p0),
};

/* pipelines */
struct sof_ipc_pipe_new pipeline[] = {
	SPIPE_PIPE(0, 0, PIPE_ID, SCHED_ID, TEST_BENCH_DEADLINE, TASK_PRI_LOW,
	TEST_BENCH_PERIOD_FRAMES),
};

void usage(char *executable);

int main(int argc, char **argv)
{
	struct ipc_comp_dev *pcm_dev;
	struct pipeline *p;
	struct comp_dev *cd;
	struct fileread_comp_data *frcd;
	struct filewrite_comp_data *fwcd;
	struct sof_ipc_ctrl_data *ctrl_cmd;
	struct sof_eq_iir_config *iir_config;
	struct sof_ipc_ctrl_value_comp *eqidxdata;
	struct sof_ipc_ctrl_value_chan *switchdata;
	clock_t tic, toc;
	double c_realtime, t_exec;
	int fs, n_in, n_out, ret = 0, bsize, deadline, i;
	int nch = TEST_BENCH_NCH; /* 2 channels test */
	uint8_t raw_blob[SOF_EQ_IIR_MAX_SIZE];
	int eq_idx[PLATFORM_MAX_CHANNELS];
	char *fn_blob;
	FILE *bfh;

	/* Set default EQ preset to 0 */
	for (i = 0; i < PLATFORM_MAX_CHANNELS; i++)
		eq_idx[i] = 0;

	/* Handle command line */
	if (argc == 7) {
		fs = atoi(argv[1]);
		eq_idx[0] = atoi(argv[2]);
		eq_idx[1] = atoi(argv[3]);
		fn_blob = argv[4];
		fileread_p0[0].fn = argv[5];
		filewrite_p0[0].fn = argv[6];

	} else {
		usage(argv[0]);
		return EXIT_SUCCESS;
	}

	/* Init components */
	sys_comp_init();
	sys_comp_fileread_init();
	sys_comp_filewrite_init();
	sys_comp_eq_iir_init();

	/* Set pipeline frames_per_sched */
	deadline = pipeline[0].deadline;
	pipeline[0].frames_per_sched = (int) (0.9999 + fs * deadline / 1e6);

	/* Construct pipeline, pass parameters, start */
	if (test_bench_pipeline_setup(&reef, pipeline, spipe) < 0)
		exit(EXIT_FAILURE);

	/* Get IIR EQ configuration */
	bfh = fopen(fn_blob, "rb");
	if (bfh == NULL) {
		fprintf(stderr, "Error: Can't open blob %s!\n", fn_blob);
		return EXIT_FAILURE;
	};
	bsize = fread(raw_blob, 1, SOF_EQ_IIR_MAX_SIZE, bfh);
	fclose(bfh);
	printf("Read %d bytes EQ blob\n", bsize);

	/* Fill IPC struct data */
	ctrl_cmd = malloc(sizeof(struct sof_ipc_ctrl_data) +bsize);
	ctrl_cmd->comp_id = EQ_ID;
	ctrl_cmd->type = SOF_CTRL_TYPE_DATA_SET;
	ctrl_cmd->cmd = SOF_CTRL_CMD_BINARY;
	memcpy(ctrl_cmd->data, raw_blob, bsize);

	iir_config = (struct sof_eq_iir_config *) ctrl_cmd->data->data;
	printf("Max channels = %d\n", iir_config->channels_in_config);
	printf("Number of responses = %d\n", iir_config->number_of_responses);
	printf("Default assign to channels = ");
	for (i = 0; i < nch; i++)
		printf("%d ", iir_config->data[i]);
	printf("\n");

	/* Send configuration IPC to IIR EQ */
	pcm_dev = ipc_get_comp(reef.ipc, EQ_ID);
	p = pcm_dev->cd->pipeline;
	cd = pcm_dev->cd;
	ret = comp_cmd(cd, COMP_CMD_SET_DATA, ctrl_cmd);
	free(ctrl_cmd);
	if (ret < 0) {
		printf("Error: Failed IIR configuration!\n");
		pipeline_free(p);
		exit(EXIT_FAILURE);
	}


	/* Activate EQ defined in the command line */
	ctrl_cmd = malloc(sizeof(struct sof_ipc_ctrl_data)
		+ sizeof(struct sof_abi_hdr)
		+TEST_BENCH_NCH * sizeof(struct sof_ipc_ctrl_value_comp));
	ctrl_cmd->comp_id = EQ_ID;
	ctrl_cmd->type = SOF_CTRL_TYPE_VALUE_COMP_SET;
	ctrl_cmd->cmd = SOF_CTRL_CMD_ENUM;
	ctrl_cmd->num_elems = TEST_BENCH_NCH;
	ctrl_cmd->data->type = SOF_CTRL_TYPE_VALUE_COMP_SET;
	ctrl_cmd->data->size
		= TEST_BENCH_NCH * sizeof(struct sof_ipc_ctrl_value_comp);
	ctrl_cmd->data->abi = SOF_ABI_VERSION;
	ctrl_cmd->data->magic = SOF_ABI_MAGIC;
	ctrl_cmd->data->comp_abi = SOF_EQ_FIR_ABI_VERSION;
	eqidxdata = (struct sof_ipc_ctrl_value_comp *) ctrl_cmd->data->data;
	for (i = 0; i < TEST_BENCH_NCH; i++) {
		eqidxdata[i].index = i;
		eqidxdata[i].svalue = eq_idx[i];
	}
	ret = comp_cmd(cd, COMP_CMD_SET_DATA, ctrl_cmd);
	free(ctrl_cmd);
	if (ret < 0) {
		printf("Error: Failed IIR response switch!\n");
		pipeline_free(p);
		exit(EXIT_FAILURE);
	}

	/* Send params and start IPC.
	 * Note that EQ must be configured before params and preload,
	 * otherwise the pipeline preload won't be completed that results to
	 * returned error from IIR preload. The next component filewrite won't
	 * then have it's func pointer set.
	 */
	if (test_bench_pipeline_params_start(reef.ipc, fs, nch,
		deadline, SCHED_ID, PIPE_ID) < 0)
		exit(EXIT_FAILURE);

	/* Unmute */
	ctrl_cmd = malloc(sizeof(struct sof_ipc_ctrl_data)
		+ sizeof(struct sof_abi_hdr)
		+ TEST_BENCH_NCH * sizeof(struct sof_ipc_ctrl_value_chan));
	ctrl_cmd->comp_id = EQ_ID;
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

	/* Get pointers to fileread and filewrite */
	pcm_dev = ipc_get_comp(reef.ipc, FW_ID);
	fwcd = comp_get_drvdata(pcm_dev->cd);
	pcm_dev = ipc_get_comp(reef.ipc, FR_ID);
	frcd = comp_get_drvdata(pcm_dev->cd);

	/*
	 * Run pipeline until EOF from fileread
	 */
	pcm_dev = ipc_get_comp(reef.ipc, SCHED_ID);
	p = pcm_dev->cd->pipeline;
	cd = pcm_dev->cd;
	test_bench_disable_trace(); /* Reduce print output */
	tic = clock();

	while (frcd->frs.reached_eof == 0)
		pipeline_schedule_copy(p, 0);

	/* Done! Next reset and free pipeline */
	toc = clock();
	test_bench_enable_trace();
	ret = pipeline_reset(p, cd);
	if (ret < 0) {
		printf("Failed pipeline_reset()\n");
		exit(EXIT_FAILURE);
	}
	pipeline_free(p);

	n_in = frcd->frs.n;
	n_out = fwcd->fws.n;
	t_exec = (double) (toc - tic) / CLOCKS_PER_SEC;
	c_realtime = (double) n_out / nch / fs / t_exec;
	printf("Read %d input samples,\n", n_in);
	printf("wrote %d output samples,\n", n_out);
	printf("test execution time was %.2f us, %.2f x realtime.\n",
		1e3 * t_exec, c_realtime);

	return EXIT_SUCCESS;
}

void usage(char *executable)
{
	fprintf(stderr, "Usage: %s <samplerate> <ch1 eq> <ch2 eq> ",
		executable);
	fprintf(stderr, "<blob> <input file> <output file>\n");
	fprintf(stderr, "E.g. \"%s 48000 1 1 eq.blob in.txt out.txt \"\n",
		executable);
}
