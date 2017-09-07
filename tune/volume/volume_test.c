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

/* Internal buffer */
#define TEST_BENCH_DEADLINE 1000 /* 1 ms */
#define TEST_BENCH_NCH 2 /* Stereo */
#define TEST_BENCH_RATE_MAX 48000 /* Hz */
#define TEST_BENCH_PERIOD_FRAMES (TEST_BENCH_RATE_MAX*TEST_BENCH_DEADLINE/1000000)
#define TEST_BENCH_FRAME_SIZE (4 * TEST_BENCH_NCH) /* 4bytes x  NCH */
#define INT_PERIOD_SIZE (TEST_BENCH_PERIOD_FRAMES * TEST_BENCH_FRAME_SIZE)

/* main firmware context */
static struct reef reef;

/*
 * Test Pipeline (ID=5)
 *
 * Fileread(0) ---B1(3)---> Volume(1) ---B2(4)---> Filewrite(2)
 */

#define FR_ID 0
#define VOLUME_ID 1
#define FW_ID 2
#define PIPE_ID 5
#define SCHED_ID FR_ID /* Component ID=0 schedules pipeline */

/*
 * Components used in static pipeline
 */

static struct sof_ipc_comp_fileread fileread_p0[] = {
	SPIPE_FILEREAD(SPIPE_COMP(FR_ID, SOF_COMP_FILEREAD,
	sof_ipc_comp_fileread), "src_in.txt"),
};

static struct sof_ipc_comp_filewrite filewrite_p0[] = {
	SPIPE_FILEWRITE(SPIPE_COMP(FW_ID, SOF_COMP_FILEWRITE,
	sof_ipc_comp_filewrite), "src_out.txt"),
};

static struct sof_ipc_comp_volume volume_p0[] = {
	SPIPE_VOL(SPIPE_COMP(VOLUME_ID, SOF_COMP_VOLUME, sof_ipc_comp_volume),
		0, 0xffffffff)
};

static struct scomps scomps_p0[] = {
	SCOMP(fileread_p0),
	SCOMP(volume_p0),
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
	SPIPE_COMP_CONNECT(0, 3), /* p(0): Fileread(0) -> B1(3) */
	SPIPE_COMP_CONNECT(3, 1), /* p(0): B1(3) ->  Volume(1) */
	SPIPE_COMP_CONNECT(1, 4), /* p(0): Volume(1) -> B24) */
	SPIPE_COMP_CONNECT(4, 2), /* p(0): B2(4) -> Filewrite(2) */
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

int main(int argc, char **argv)
{
	clock_t tic, toc;
	double c_realtime;
	double t_exec;
	int fs, n_in, n_out, ret;
	struct ipc_comp_dev *pcm_dev;
	struct pipeline *p;
	struct comp_dev *cd;
	int deadline;
	struct fileread_comp_data *frcd;
	struct filewrite_comp_data *fwcd;
	int nch = TEST_BENCH_NCH; /* 2 channels test */
	int bits_in, bits_out;

	/* Handle command line */
	if (argc > 2) {
		switch (argc) {
		case 5:
			bits_in = atoi(argv[1]);
			bits_out = atoi(argv[2]);
			fileread_p0[0].fn = argv[3];
			filewrite_p0[0].fn = argv[4];
			break;
		default:
			usage(argv[0]);
			return EXIT_SUCCESS;
		}
	} else {
		usage(argv[0]);
		return EXIT_SUCCESS;
	}

	/* Init components */
	sys_comp_init();
	sys_comp_fileread_init();
	sys_comp_filewrite_init();
	sys_comp_volume_init();

	/* Set SRC sink rate into IPC, src_new() will copy it */
	fs = 48000;
	deadline = pipeline[0].deadline;
	pipeline[0].frames_per_sched = (int) (0.9999 + fs * deadline / 1e6);

	/* Set fileread config */
	switch (bits_in) {
	case 16:
		fileread_p0->config.frame_fmt = SOF_IPC_FRAME_S16_LE;
		break;
	case 24:
		fileread_p0->config.frame_fmt = SOF_IPC_FRAME_S24_4LE;
		break;
	default:
		fileread_p0->config.frame_fmt = SOF_IPC_FRAME_S32_LE;
		break;
	}

	/* Set filewrite config */
	switch (bits_out) {
	case 16:
		filewrite_p0->config.frame_fmt = SOF_IPC_FRAME_S16_LE;
		break;
	case 24:
		filewrite_p0->config.frame_fmt = SOF_IPC_FRAME_S24_4LE;
		break;
	default:
		filewrite_p0->config.frame_fmt = SOF_IPC_FRAME_S32_LE;
		break;
	}


	/* Construct pipeline, pass parameters, start */
	if (test_bench_pipeline_setup(&reef, pipeline, spipe) < 0)
		exit(EXIT_FAILURE);

	/* Params is set to source fs1 */
	if (test_bench_pipeline_params_start(reef.ipc, fs, nch, deadline, SCHED_ID, PIPE_ID) < 0)
		exit(EXIT_FAILURE);

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
	printf("test execution time was %.2f us, %.2f x realtime.\n", 1e3 * t_exec, c_realtime);

	return EXIT_SUCCESS;
}

void usage(char *executable)
{
	fprintf(stderr, "Usage: %s <bits in> <bits out> <input.txt> <output.txt>>\n", executable);
}
