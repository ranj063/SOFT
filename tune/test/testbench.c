/*
 * Copyright (c) 2018, Intel Corporation
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
 *	   Ranjani Sridharan <ranjani.sridharan@linux.intel.com>
 */

#include <sof/ipc.h>
#include <sof/list.h>
#include <getopt.h>
#include <dlfcn.h>
#include "common_test.h"
#include "topology.h"
#include "trace.h"
#include "file.h"

#define TESTBENCH_NCH 2 /* Stereo */

/* main firmware context */
static struct sof sof;
static int fr_id; /* comp id for fileread */
static int fw_id; /* comp id for filewrite */
static int sched_id; /* comp id for scheduling comp */

int debug;

/*
 * Parse shared library from user input
 * Currently only handles volume comp
 */
static void parse_libraries(char *libs, void *handle)
{
	char *lib_token, *comp_token;
	char *token = strtok_r(libs, ",", &lib_token);
	char message[DEBUG_MSG_LEN];

	while (token) {
		char *token1 = strtok_r(token, "=", &comp_token);

		/* parse shared library for volume component */
		if (strcmp(token1, "vol") == 0) {
			while (token1) {
				token1 = strtok_r(NULL, "=", &comp_token);
				if (!token1)
					return;

				/* close shared library object */
				if (handle)
					dlclose(handle);

				/* open volume shared library object */
				handle = dlopen(token1, RTLD_LAZY);
				if (!handle) {
					fprintf(stderr, "error: %s\n",
						dlerror());
					exit(EXIT_FAILURE);
				}

				sprintf(message, "opening vol shared lib %s\n",
					token1);
				debug_print(message);
			}
		}
		token = strtok_r(NULL, ",", &lib_token);
	}
}

/* print usage for testbench */
static void print_usage(char *executable)
{
	printf("Usage: %s -i <input_file> -o <output_file> ", executable);
	printf("-t <tplg_file> -b <input_format> ");
	printf("-a <comp1=comp1_library,comp2=comp2_library>\n");
	printf("input_format should be S16_LE, S32_LE, S24_LE or FLOAT_LE\n");
	printf("Example Usage:\n");
	printf("%s -i in.txt -o out.txt -t test.tplg ", executable);
	printf("-b S16_LE -a vol=libsof_volume.so\n");
}

/* free components */
static void free_comps(void)
{
	struct list_item *clist;
	struct list_item *temp;
	struct ipc_comp_dev *icd = NULL;

	list_for_item_safe(clist, temp, &sof.ipc->comp_list) {
		icd = container_of(clist, struct ipc_comp_dev, list);
		switch (icd->type) {
		case COMP_TYPE_COMPONENT:
			comp_free(icd->cd);
			list_item_del(&icd->list);
			rfree(icd);
			break;
		case COMP_TYPE_BUFFER:
			rfree(icd->cb->addr);
			rfree(icd->cb);
			list_item_del(&icd->list);
			rfree(icd);
			break;
		default:
			rfree(icd->pipeline);
			list_item_del(&icd->list);
			rfree(icd);
			break;
		}
	}
}

int main(int argc, char **argv)
{
	struct ipc_comp_dev *pcm_dev;
	struct pipeline *p;
	struct sof_ipc_pipe_new *ipc_pipe;
	struct comp_dev *cd;
	struct file_comp_data *frcd, *fwcd;
	char *tplg_file = NULL, *input_file = NULL;
	char *output_file = NULL, *bits_in = "S32_LE";
	char pipeline[DEBUG_MSG_LEN];
	clock_t tic, toc;
	double c_realtime, t_exec;
	int fs, n_in, n_out, ret;
	int option = 0;

	/* volume component share library handle */
	void *vol_handle = NULL;

	/* TODO: create a shared library table for all components */
	/*set up default volume shared library */
	if (!vol_handle) {
		vol_handle = dlopen("libsof_volume.so", RTLD_LAZY);
		if (!vol_handle) {
			fprintf(stderr, "error: %s\n", dlerror());
			exit(EXIT_FAILURE);
		}
	}

	/* set up trace class definition table from trace header */
	setup_trace_table();

	/* command line arguments*/
	while ((option = getopt(argc, argv, "hdi:o:t:b:a:")) != -1) {
		switch (option) {
		/* input sample file */
		case 'i':
			input_file = strdup(optarg);
			break;

		/* output sample file */
		case 'o':
			output_file = strdup(optarg);
			break;

		/* topology file */
		case 't':
			tplg_file = strdup(optarg);
			break;

		/* input samples bit format */
		case 'b':
			bits_in = strdup(optarg);
			break;

		/* override default libraries */
		case 'a':
			parse_libraries(optarg, vol_handle);
			break;

		/* enable debug prints */
		case 'd':
			debug = 1;
			break;

		/* print usage */
		case 'h':
		default:
			print_usage(argv[0]);
			exit(EXIT_FAILURE);
		}
	}

	/* check args */
	if (!tplg_file || !input_file || !output_file) {
		print_usage(argv[0]);
		exit(EXIT_FAILURE);
	}

	/* initialize ipc and scheduler */
	if (tb_pipeline_setup(&sof) < 0) {
		fprintf(stderr, "error: pipeline init\n");
		exit(EXIT_FAILURE);
	}

	/* parse topology file and create pipeline */
	if (parse_topology(tplg_file, &sof, &fr_id, &fw_id, &sched_id, bits_in,
			   input_file, output_file, vol_handle, pipeline) < 0) {
		fprintf(stderr, "error: parsing topology\n");
		exit(EXIT_FAILURE);
	}

	/* Get pointers to fileread and filewrite */
	pcm_dev = ipc_get_comp(sof.ipc, fw_id);
	fwcd = comp_get_drvdata(pcm_dev->cd);
	pcm_dev = ipc_get_comp(sof.ipc, fr_id);
	frcd = comp_get_drvdata(pcm_dev->cd);

	/* Run pipeline until EOF from fileread */
	pcm_dev = ipc_get_comp(sof.ipc, sched_id);
	p = pcm_dev->cd->pipeline;
	ipc_pipe = &p->ipc_pipe;

	fs = ipc_pipe->deadline * ipc_pipe->frames_per_sched;

	/* set pipeline params and trigger start */
	if (tb_pipeline_start(sof.ipc, TESTBENCH_NCH, bits_in, ipc_pipe) < 0) {
		fprintf(stderr, "error: pipeline params\n");
		exit(EXIT_FAILURE);
	}

	cd = pcm_dev->cd;
	tb_enable_trace(false); /* reduce trace output */
	tic = clock();

	while (frcd->fs.reached_eof == 0)
		pipeline_schedule_copy(p, 0);

	if (!frcd->fs.reached_eof)
		printf("warning: possible pipeline xrun\n");

	/* reset and free pipeline */
	toc = clock();
	tb_enable_trace(true);
	ret = pipeline_reset(p, cd);
	if (ret < 0) {
		fprintf(stderr, "error: pipeline reset\n");
		exit(EXIT_FAILURE);
	}

	n_in = frcd->fs.n;
	n_out = fwcd->fs.n;
	t_exec = (double)(toc - tic) / CLOCKS_PER_SEC;
	c_realtime = (double)n_out / TESTBENCH_NCH / fs / t_exec;

	/* free all components/buffers in pipeline */
	free_comps();

	/* free trace class defs */
	free_trace_table();

	/* print test summary */
	printf("==========================================================\n");
	printf("		           Test Summary\n");
	printf("==========================================================\n");
	printf("Test Pipeline:\n");
	printf("%s\n", pipeline);
	printf("Input bit format: %s\n", bits_in);
	printf("Output written to file: \"%s\"\n", output_file);
	printf("Input sample count: %d\n", n_in);
	printf("Output sample count: %d\n", n_out);
	printf("Total execution time: %.2f us, %.2f x realtime\n",
	       1e3 * t_exec, c_realtime);

	/* free all other data */
	free(bits_in);
	free(input_file);
	free(tplg_file);
	free(output_file);

	/* close shared library object */
	if (vol_handle)
		dlclose(vol_handle);

	return EXIT_SUCCESS;
}
