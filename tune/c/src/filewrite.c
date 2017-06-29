/*
 * Copyright (c) 2016, Intel Corporation
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

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <reef/reef.h>
#include <reef/lock.h>
#include <reef/list.h>
#include <reef/stream.h>
#include <reef/work.h>
#include <reef/clock.h>
#include <reef/audio/component.h>
#include <reef/audio/format.h>
#include <reef/audio/pipeline.h>
#include "common_test.h"

static inline void circular_inc_wrap(int32_t **ptr, int32_t *end, size_t size)
{
	if (*ptr >= end)
		*ptr = (int32_t *) ((size_t) * ptr - size);
}

static inline void circular_inc_wrap_16(int16_t **ptr, int16_t *end, size_t size)
{
	if (*ptr >= end)
		*ptr = (int16_t *) ((size_t) * ptr - size);
}

static int filewrite_s32_default(struct comp_dev *dev,
	struct comp_buffer *sink, struct comp_buffer *source, uint32_t frames)
{
	struct filewrite_comp_data *cd = comp_get_drvdata(dev);
	int32_t *src = (int32_t *) source->r_ptr;
	int i, n, n_wrap_src, n_min, ret;
	int nch = dev->params.channels;
	int nwrite = 0;

	if (cd->fws.write_fail)
		return -EINVAL;

	n = frames * nch;

	while (n > 0) {
		n_wrap_src = (int32_t *) source->end_addr - src;
		n_min = (n < n_wrap_src) ? n : n_wrap_src;
		while (n_min > 0) {
			n -= nch;
			n_min -= nch;
			for (i = 0; i < nch; i++) {
				ret = fprintf(cd->fws.fh, "%d\n", *src);
				nwrite++;
				src++;
				if (ret < 0) {
					cd->fws.write_fail = 1;
					goto quit;
				}
			}
		}
		circular_inc_wrap(&src, source->end_addr, source->size);
	}

quit:
	cd->fws.n += nwrite;
	return nwrite;
}

static int filewrite_s16(struct comp_dev *dev,
	struct comp_buffer *sink, struct comp_buffer *source, uint32_t frames)
{
	struct filewrite_comp_data *cd = comp_get_drvdata(dev);
	int16_t *src = (int16_t *) source->r_ptr;
	int i, n, n_wrap_src, n_min, ret;
	int nch = dev->params.channels;
	int nwrite = 0;

	if (cd->fws.write_fail)
		return -EINVAL;

	n = frames * nch;

	while (n > 0) {
		n_wrap_src = (int16_t *) source->end_addr - src;
		n_min = (n < n_wrap_src) ? n : n_wrap_src;
		while (n_min > 0) {
			n -= nch;
			n_min -= nch;
			for (i = 0; i < nch; i++) {
				ret = fprintf(cd->fws.fh, "%d\n", *src);
				nwrite++;
				src++;
				if (ret < 0) {
					cd->fws.write_fail = 1;
					goto quit;
				}
			}
		}
		circular_inc_wrap_16(&src, source->end_addr, source->size);
	}


quit:
	cd->fws.n += nwrite;
	return nwrite;
}

static int filewrite_s24_default(struct comp_dev *dev,
	struct comp_buffer *sink, struct comp_buffer *source, uint32_t frames)
{
	struct filewrite_comp_data *cd = comp_get_drvdata(dev);
	int32_t *src = (int32_t *) source->r_ptr;
	int32_t se;
	int i, n, n_wrap_src, n_min, ret;
	int nch = dev->params.channels;
	int nwrite = 0;

	if (cd->fws.write_fail)
		return -EINVAL;

	n = frames * nch;

	while (n > 0) {
		n_wrap_src = (int32_t *) source->end_addr - src;
		n_min = (n < n_wrap_src) ? n : n_wrap_src;
		while (n_min > 0) {
			n -= nch;
			n_min -= nch;
			for (i = 0; i < nch; i++) {
				se = *src << 8;
				ret = fprintf(cd->fws.fh, "%d\n", se >> 8);
				nwrite++;
				src++;
				if (ret < 0) {
					cd->fws.write_fail = 1;
					goto quit;
				}
			}
		}
		circular_inc_wrap(&src, source->end_addr, source->size);
	}


quit:
	cd->fws.n += nwrite;
	return nwrite;
}

static struct comp_dev *filewrite_new(struct sof_ipc_comp *comp)
{
	struct comp_dev *dev;
	struct sof_ipc_comp_filewrite *filewrite;
	struct sof_ipc_comp_filewrite *ipc_filewrite
		= (struct sof_ipc_comp_filewrite *) comp;
	struct filewrite_comp_data *cd;

	dev = malloc(COMP_SIZE(struct sof_ipc_comp_filewrite));
	if (dev == NULL)
		return NULL;

	filewrite = (struct sof_ipc_comp_filewrite *) &dev->comp;
	memcpy(filewrite, ipc_filewrite, sizeof(struct sof_ipc_comp_filewrite));

	cd = malloc(sizeof(*cd));
	if (cd == NULL) {
		free(dev);
		return NULL;
	}

	comp_set_drvdata(dev, cd);
	cd->filewrite_func = filewrite_s32_default;

	strncpy(cd->fws.fn, ipc_filewrite->fn, FILEWRITE_FN_MAXLENGTH);
	cd->fws.fh = fopen(cd->fws.fn, "w");
	if (cd->fws.fh == NULL) {
		fprintf(stderr, "Error: File %s open for write failed.\n",
			cd->fws.fn);
		free(cd);
		free(dev);
		return NULL;
	}

	cd->fws.write_fail = 0;
	cd->fws.n = 0;
	return dev;
}

static void filewrite_free(struct comp_dev *dev)
{
	struct filewrite_data *td = comp_get_drvdata(dev);
	struct filewrite_comp_data *cd = comp_get_drvdata(dev);

	fclose(cd->fws.fh);
	free(td);
	free(dev);
}

/* set component audio stream parameters */
static int filewrite_params(struct comp_dev *dev)
{
	struct filewrite_comp_data *cd = comp_get_drvdata(dev);
	struct sof_ipc_comp_config *config = COMP_GET_CONFIG(dev);

	/* Need to compute this in non-dai endpoint */
	dev->frame_bytes =
		dev->params.sample_container_bytes * dev->params.channels;

	/* calculate period size based on config */
	cd->period_bytes = dev->frames * dev->frame_bytes;

	/* File to sink supports only S32_LE/S16_LE/S24_4LE PCM formats */
	if ((config->frame_fmt != SOF_IPC_FRAME_S32_LE)
		&& (config->frame_fmt != SOF_IPC_FRAME_S24_4LE)
		&& (config->frame_fmt != SOF_IPC_FRAME_S16_LE))
		return -EINVAL;

	return 0;
}

static int fw_cmd(struct comp_dev *dev, struct sof_ipc_ctrl_data *cdata)
{
	return -EINVAL;
}

/* used to pass standard and bespoke commands (with data) to component */
static int filewrite_cmd(struct comp_dev *dev, int cmd, void *data)
{
	struct sof_ipc_ctrl_data *cdata = data;
	int ret = 0;

	switch (cmd) {
	case COMP_CMD_SET_DATA:
		ret = fw_cmd(dev, cdata);
		break;
	case COMP_CMD_START:
	case COMP_CMD_STOP:
	case COMP_CMD_PAUSE:
	case COMP_CMD_RELEASE:
	default:
		ret = comp_set_state(dev, cmd);
		break;
	}

	return ret;
}

/* copy and process stream data from source to sink buffers
 * returns the number of bytes copied
 */
static int filewrite_copy(struct comp_dev *dev)
{
	struct comp_buffer *sink = NULL;
	struct comp_buffer *source;
	struct filewrite_comp_data *cd = comp_get_drvdata(dev);
	int ret = 0;

	/* Get filewrite component source buffer */
	source = list_first_item(&dev->bsource_list, struct comp_buffer,
		sink_list);

	/* Test that source has enough free frames */
	if (source->avail >= cd->period_bytes) {
		/* Write PCM samples into file */
		ret = cd->filewrite_func(dev, sink, source, dev->frames);
		if (ret < 0) {
			printf("error: filewrite fail\n");
			return ret;
		}
		comp_update_buffer_consume(source, ret *
			dev->params.sample_container_bytes);
	}

	return ret;
}

static int filewrite_prepare(struct comp_dev *dev)
{
	struct sof_ipc_comp_config *config = COMP_GET_CONFIG(dev);
	struct comp_buffer *source_buffer;
	struct filewrite_comp_data *cd = comp_get_drvdata(dev);
	int ret = 0;

	/* fileread component sink buffer */
	source_buffer = list_first_item(&dev->bsource_list, struct comp_buffer,
		sink_list);

	switch (config->frame_fmt) {
	case(SOF_IPC_FRAME_S16_LE):
		/* set downstream buffer size */
		ret = buffer_set_size(source_buffer, dev->frames * 2 *
			config->periods_source * dev->params.channels);
		if (ret < 0) {
			printf("error: fileread buffer size set\n");
			return ret;
		}

		buffer_reset_pos(source_buffer);

		/* set fileread function */
		cd->filewrite_func = filewrite_s16;
		break;
	case(SOF_IPC_FRAME_S24_4LE):
		/* set downstream buffer size */
		ret = buffer_set_size(source_buffer, dev->frames * 4 *
			config->periods_source * dev->params.channels);
		if (ret < 0) {
			printf("error: fileread buffer size set\n");
			return ret;
		}

		buffer_reset_pos(source_buffer);

		/* set fileread function */
		cd->filewrite_func = filewrite_s24_default;
		break;
	case(SOF_IPC_FRAME_S32_LE):
		/* set downstream buffer size */
		ret = buffer_set_size(source_buffer, dev->frames * 4 *
			config->periods_source * dev->params.channels);
		if (ret < 0) {
			printf("error: fileread buffer size set\n");
			return ret;
		}

		buffer_reset_pos(source_buffer);
		break;
	default:
		return -EINVAL;
	}
	dev->state = COMP_STATE_PREPARE;

	return 0;
}

static int filewrite_reset(struct comp_dev *dev)
{
	dev->state = COMP_STATE_INIT;

	return 0;
}

struct comp_driver comp_filewrite = {
	.type = SOF_COMP_FILEWRITE,
	.ops =
	{
		.new = filewrite_new,
		.free = filewrite_free,
		.params = filewrite_params,
		.cmd = filewrite_cmd,
		.copy = filewrite_copy,
		.prepare = filewrite_prepare,
		.reset = filewrite_reset,
	},
};

void sys_comp_filewrite_init(void)
{
	comp_register(&comp_filewrite);
}
