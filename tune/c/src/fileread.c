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
#include <inttypes.h>
#include <reef/reef.h>
#include <reef/lock.h>
#include <reef/list.h>
#include <reef/stream.h>
#include <reef/work.h>
#include <reef/clock.h>
#include <reef/audio/component.h>
#include <reef/audio/format.h>
#include <reef/audio/pipeline.h>
#include <uapi/ipc.h>
#include "common_test.h"

#define SHRINK_SINK 0

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

static int fileread_s32_default(struct comp_dev *dev, struct comp_buffer *sink,
        struct comp_buffer *source, uint32_t frames)
{
        struct fileread_comp_data *cd = comp_get_drvdata(dev);
        int32_t *dest = (int32_t *) sink->w_ptr;
        int i, n, n_wrap_dest, n_min, ret;
        int nch = dev->params.channels;
        int nread = 0;

        n = frames * nch;
        while (n > 0) {
                n_wrap_dest = (int32_t *) sink->end_addr - dest;
                n_min = (n < n_wrap_dest) ? n : n_wrap_dest;
                while (n_min > 0) {
                        n -= nch;
                        n_min -= nch;
                        for (i = 0; i < nch; i++) {
                                ret = fscanf(cd->frs.fh, "%d", dest);
                                nread++;
                                dest++;
                                if (ret == EOF)
                                        goto read_eof;
                        }
                }
                circular_inc_wrap(&dest, sink->end_addr, sink->size);
        }

        cd->frs.n += nread;
        return nread;

read_eof:
        cd->frs.n += nread;
        cd->frs.reached_eof = 1;
        return nread;
}

static int fileread_s16(struct comp_dev *dev, struct comp_buffer *sink,
	struct comp_buffer *source, uint32_t frames)
{
	struct fileread_comp_data *cd = comp_get_drvdata(dev);
        int16_t *dest = (int16_t *) sink->w_ptr;
        int i, n, n_wrap_dest, n_min, ret;
        int nch = dev->params.channels;
        int nread = 0;

        n = frames * nch;
        while (n > 0) {
                n_wrap_dest = (int16_t *) sink->end_addr - dest;
                n_min = (n < n_wrap_dest) ? n : n_wrap_dest;
                while (n_min > 0) {
                        n -= nch;
                        n_min -= nch;
                        for (i = 0; i < nch; i++) {
                                ret = fscanf(cd->frs.fh, "%hd", dest);
                                nread++;
                                dest++;
                                if (ret == EOF)
                                        goto read_eof;
			}
		}
                circular_inc_wrap_16(&dest, sink->end_addr, sink->size);
	}

        cd->frs.n += nread;
        return nread;

read_eof:
        cd->frs.n += nread;
        cd->frs.reached_eof = 1;
        return nread;
}

static int fileread_s24_default(struct comp_dev *dev, struct comp_buffer *sink,
	struct comp_buffer *source, uint32_t frames)
{
	struct fileread_comp_data *cd = comp_get_drvdata(dev);
	int32_t *dest = (int32_t *) sink->w_ptr;
	int32_t sample;
	int i, n, n_wrap_dest, n_min, ret;
	int nch = dev->params.channels;
	int nread = 0;

	n = frames * nch;
	while (n > 0) {
		n_wrap_dest = (int32_t *) sink->end_addr - dest;
		n_min = (n < n_wrap_dest) ? n : n_wrap_dest;
		while (n_min > 0) {
			n -= nch;
			n_min -= nch;
			for (i = 0; i < nch; i++) {
				ret = fscanf(cd->frs.fh, "%d", &sample);
				*dest = sample & 0x00ffffff;  /* Mask bits */
				nread++;
				dest++;
				if (ret == EOF)
					goto read_eof;
			}
		}
		circular_inc_wrap(&dest, sink->end_addr, sink->size);
	}

	cd->frs.n += nread;
	return nread;

read_eof:
	cd->frs.n += nread;
	cd->frs.reached_eof = 1;
	return nread;
}

static struct comp_dev *fileread_new(struct sof_ipc_comp *comp)
{
	struct comp_dev *dev;
	struct sof_ipc_comp_fileread *fileread;
	struct sof_ipc_comp_fileread *ipc_fileread
		= (struct sof_ipc_comp_fileread *) comp;
	struct fileread_comp_data *cd;

	dev = malloc(COMP_SIZE(struct sof_ipc_comp_fileread));
	if (dev == NULL)
		return NULL;

	fileread = (struct sof_ipc_comp_fileread *) &dev->comp;
	memcpy(fileread, ipc_fileread, sizeof(struct sof_ipc_comp_fileread));

	cd = malloc(sizeof(*cd));
	if (cd == NULL) {
		free(dev);
		return NULL;
	}

	comp_set_drvdata(dev, cd);
	cd->fileread_func = fileread_s32_default;

	/* Get filename from IPC and open file */
	strncpy(cd->frs.fn, ipc_fileread->fn, FILEREAD_FN_MAXLENGTH);
	cd->frs.fh = fopen(cd->frs.fn, "r");
	if (cd->frs.fh == NULL) {
		fprintf(stderr, "Error: File %s open for read failed.\n",
			cd->frs.fn);
		free(cd);
		free(dev);
		return NULL;
	}
	cd->frs.reached_eof = 0;
	cd->frs.n = 0;

	return dev;
}

static void fileread_free(struct comp_dev *dev)
{
	struct fileread_data *td = comp_get_drvdata(dev);
	struct fileread_comp_data *cd = comp_get_drvdata(dev);

	fclose(cd->frs.fh);
	free(td);
	free(dev);
}

/* set component audio stream parameters */
static int fileread_params(struct comp_dev *dev)
{
	struct fileread_comp_data *cd = comp_get_drvdata(dev);
	struct sof_ipc_comp_config *config = COMP_GET_CONFIG(dev);

	/* Need to compute this in non-host endpoint */
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

static int fr_cmd(struct comp_dev *dev, struct sof_ipc_ctrl_data *cdata)
{
	return -EINVAL;
}

/* used to pass standard and bespoke commands (with data) to component */
static int fileread_cmd(struct comp_dev *dev, int cmd, void *data)
{
	struct sof_ipc_ctrl_data *cdata = data;
	int ret = 0;

	switch (cmd) {
	case COMP_CMD_SET_DATA:
		ret = fr_cmd(dev, cdata);
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
static int fileread_copy(struct comp_dev *dev)
{
	struct comp_buffer *sink;
	struct comp_buffer *source = NULL;
	struct fileread_comp_data *cd = comp_get_drvdata(dev);
	int ret = 0;

	/* fileread component sink buffer */
	sink = list_first_item(&dev->bsink_list, struct comp_buffer,
		source_list);

	/* Test that sink has enough free frames */
	if (sink->free >= cd->period_bytes && !cd->frs.reached_eof) {
		/* Read PCM samples from file */
		ret = cd->fileread_func(dev, sink, source, dev->frames);
		if (ret > 0)
			comp_update_buffer_produce(sink, ret *
			dev->params.sample_container_bytes);
	}

	return ret;
}

static int fileread_prepare(struct comp_dev *dev)
{
	struct sof_ipc_comp_config *config = COMP_GET_CONFIG(dev);
	struct comp_buffer *sink_buffer;
	struct fileread_comp_data *cd = comp_get_drvdata(dev);
	int ret = 0;

	/* fileread component sink buffer */
	sink_buffer = list_first_item(&dev->bsink_list, struct comp_buffer,
		source_list);

	switch(config->frame_fmt) {
	case(SOF_IPC_FRAME_S16_LE):
#if SHRINK_SINK == 1
		/* set downstream buffer size */
		ret = buffer_set_size(sink_buffer, dev->frames * 2 *
			config->periods_sink * dev->params.channels);
		if (ret < 0) {
			printf("error: fileread buffer size set\n");
			return ret;
		}
#endif
		buffer_reset_pos(sink_buffer);

		/* set fileread function */
		cd->fileread_func = fileread_s16;
		break;
	case(SOF_IPC_FRAME_S24_4LE):
#if SHRINK_SINK == 1
		/* set downstream buffer size */
		ret = buffer_set_size(sink_buffer, dev->frames * 4 *
			config->periods_sink * dev->params.channels);
		if (ret < 0) {
			printf("error: fileread buffer size set\n");
			return ret;
		}
#endif
		buffer_reset_pos(sink_buffer);
		/* set fileread function */
		cd->fileread_func = fileread_s24_default;
		break;
	case(SOF_IPC_FRAME_S32_LE):
#if SHRINK_SINK == 1
		/* set downstream buffer size */
		ret = buffer_set_size(sink_buffer, dev->frames * 4 *
			config->periods_sink * dev->params.channels);
		if (ret < 0) {
			printf("error: fileread buffer size set\n");
			return ret;
		}
#endif
		buffer_reset_pos(sink_buffer);
		break;
	default:
		return -EINVAL;
	}

	dev->state = COMP_STATE_PREPARE;

	return ret;
}

static int fileread_reset(struct comp_dev *dev)
{
	dev->state = COMP_STATE_INIT;

	return 0;
}

struct comp_driver comp_fileread = {
	.type = SOF_COMP_FILEREAD,
	.ops =
	{
		.new = fileread_new,
		.free = fileread_free,
		.params = fileread_params,
		.cmd = fileread_cmd,
		.copy = fileread_copy,
		.prepare = fileread_prepare,
		.reset = fileread_reset,
	},
};

void sys_comp_fileread_init(void)
{
	comp_register(&comp_fileread);
}
