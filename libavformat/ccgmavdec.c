/*
 * CasparCG MAV demuxer
 * Copyright (c) 2019 Krzysztof Pyrkosz
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#define _FILE_OFFSET_BITS  64

#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "rawdec.h"
#include "internal.h"

#if CONFIG_CCGMAV_DEMUXER

#define CCGMAV_HEADER_SIZE 52

typedef struct {
    uint8_t  version;
    uint32_t width;
    uint32_t height;
    double   fps;
    uint8_t  field_mode;
    char     video_fourcc[4];
    char     audio_fourcc[4];
    int      audio_channels;
    int      sample_rate;
    FILE*    data_file;
    int      status;
    uint32_t audioBufSize;
    uint32_t videoBufSize;

} CCGMAVContext;

static int ccgmav_probe(const AVProbeData *p)
{
    if (p->buf_size <= 4)
        return 0;
    if (!memcmp(p->buf, "OMAV", 4)) {
        return AVPROBE_SCORE_MAX;
    }
    return 0;
}

static int ccgmav_read_header(AVFormatContext *s)
{
    CCGMAVContext *context = s->priv_data;
    AVIOContext *pb = s->pb;
    AVStream *ast;
    AVStream *vst;
    unsigned char header[CCGMAV_HEADER_SIZE];
    char mav_filename[1024];

    if (avio_read(pb, header, CCGMAV_HEADER_SIZE) != CCGMAV_HEADER_SIZE)
        return AVERROR(EIO);

    context->version = header[4];
    context->width = *(uint32_t*)&header[8];
    context->height = *(uint32_t*)&header[12];
    context->fps = *(double*)&header[16];
    context->field_mode = header[24];
    context->video_fourcc[0] = header[40];
    context->video_fourcc[1] = header[41];
    context->video_fourcc[2] = header[42];
    context->video_fourcc[3] = header[43];
    context->audio_fourcc[0] = header[44];
    context->audio_fourcc[1] = header[45];
    context->audio_fourcc[2] = header[46];
    context->audio_fourcc[3] = header[47];
    context->audio_channels = 8; //*(int*)&header[48]; <- bug - CasparCG now uses 8 channels, but MAV always writes 2
    context->sample_rate = 48000;
    context->status = 0;

    ast = avformat_new_stream(s, NULL);
    avpriv_set_pts_info(ast, 64, 1, context->sample_rate);
    ast->codecpar->codec_id = AV_CODEC_ID_PCM_S32LE;
    ast->codecpar->bits_per_coded_sample = 64;
    ast->codecpar->bit_rate = context->audio_channels * context->sample_rate * 4;
    ast->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
    ast->codecpar->channels    = context->audio_channels;
    ast->codecpar->channel_layout = context->audio_channels == 1 ? AV_CH_LAYOUT_MONO : AV_CH_LAYOUT_STEREO;
    ast->codecpar->sample_rate = context->sample_rate;

    vst = avformat_new_stream(s, NULL);
    avpriv_set_pts_info(vst, 60, 1, context->fps);
    vst->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    vst->codecpar->codec_id   = AV_CODEC_ID_MJPEG;
    vst->codecpar->width  = context->width;
    vst->codecpar->height = context->height;

    strcpy(mav_filename, s->filename);
    mav_filename[strlen(mav_filename) - 3] = 'm';
    mav_filename[strlen(mav_filename) - 2] = 'a';
    mav_filename[strlen(mav_filename) - 1] = 'v';
    context->data_file = fopen(mav_filename, "rb");

    return 0;
}

static int ccgmav_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    CCGMAVContext *context = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = 0;
    long long position;
    long long next_position;

    if (context->status == 0) {
        if (avio_read(pb, (void*)&position, 8) != 8) {
            if (avio_size(pb) == avio_tell(pb)) {
                return AVERROR_EOF;
            }
            return AVERROR(EIO);
        }

        if (avio_read(pb, (void*)&next_position, 8) != 8) {
            if (avio_size(pb) == avio_tell(pb)) {
                fseek(context->data_file, 0, SEEK_END);
                next_position = ftell(context->data_file);
            } else {
                return AVERROR(EIO);
            }
        } else {
            avio_seek(pb, -8, SEEK_CUR);
        }

        fseek(context->data_file, position, SEEK_SET);

        if (fread(&context->audioBufSize, 1, 4, context->data_file) != 4)
            return AVERROR(EIO);

        context->videoBufSize = next_position-position-context->audioBufSize-4;

        if ((ret = av_new_packet(pkt, context->audioBufSize)) < 0)
            return ret;
        if ((ret = fread(&pkt->data[0], 1, context->audioBufSize, context->data_file)) != context->audioBufSize)
            return ret < 0 ? ret : AVERROR(EIO);
        pkt->stream_index = s->streams[0]->index;

        context->status = 1;
    } else {
        if ((ret = av_new_packet(pkt, context->videoBufSize)) < 0)
            return ret;
        if ((ret = fread(&pkt->data[0], 1, context->videoBufSize, context->data_file)) != context->videoBufSize)
            return ret < 0 ? ret : AVERROR(EIO);
        pkt->stream_index = s->streams[1]->index;

        context->status = 0;
    }

    return ret;
}

AVInputFormat ff_ccgmav_demuxer = {
    .name           = "ccgmav",
    .long_name      = NULL_IF_CONFIG_SMALL("CasparCG MAV"),
    .priv_data_size = sizeof(CCGMAVContext),
    .read_probe     = ccgmav_probe,
    .read_header    = ccgmav_read_header,
    .read_packet    = ccgmav_read_packet,
};
#endif
