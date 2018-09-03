#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdbool.h>
#include <errno.h>

#define LOG_TAG "ia_tunneling_hal_test"

#include <cutils/log.h>
#include "tunnel.h"
#include "conversion_routines.h"
#include "iaxxx-system-identifiers.h"

/*
 * To dump 4 MIC data, the source end points are
 * 0x4020 0x40A0 0x4120 0x41A0
 */

#define DOA_TUNNEL_SRC (IAXXX_SYSID_PLUGIN_1_OUT_EP_2)
                        // Source end point for DOA meta data
#define VQ_TUNNEL_SRC (IAXXX_SYSID_PLUGIN_0_OUT_EP_0)
                        // Source end point for VQ confidence meta data
#define VP_PARAM_TUNNEL_SRC (IAXXX_SYSID_PLUGIN_1_OUT_EP_3)
                         // Source end point for VP Parameter data

#define MIC1_TUNNEL_SRC  (IAXXX_SYSID_CHANNEL_RX_0_EP_0)
#define MIC2_TUNNEL_SRC  (IAXXX_SYSID_CHANNEL_RX_2_EP_0)
#define MIC3_TUNNEL_SRC  (IAXXX_SYSID_CHANNEL_RX_4_EP_0)
#define MIC4_TUNNEL_SRC  (IAXXX_SYSID_CHANNEL_RX_6_EP_0)
#define AEC_REF1_TUNNEL_SRC (IAXXX_SYSID_CHANNEL_RX_8_EP_0)
#define AEC_REF2_TUNNEL_SRC (IAXXX_SYSID_CHANNEL_RX_9_EP_0)


#define MAX_TUNNELS 32
#define TUNNELS_THAT_NEED_SYNC 10
#define BUF_SIZE 8192
#define DOA_OUTPUT_FILE             "/data/data/doa_tunnel_output"
#define VQ_CONFIDENCE_OUTPUT_FILE   "/data/data/vq_conf_tunnel_output"
#define OUTPUT_FILE                 "/data/data/tnl_op"
#define UNPARSED_OUTPUT_FILE        "/data/data/unparsed_output"
#define VP_PARAM_DUMP_FILE          "/data/data/param_dump"

#define SALIENCE    "SALIENCE"
#define BEARING     "BEARING"
#define TNL_SYNC_MODE  0
#define TNL_ASYNC_MODE	1
#define TNL_ENC_AFLOAT  1
#define TNL_ENC_Q15     0xF

struct raf_format_type {
    uint16_t frameSizeInBytes; /*!< Frame length in bytes */
    uint8_t encoding;   /*!< Encoding */
    uint8_t sampleRate; /*!< Sample rate */
};

struct raf_frame_type {
    uint64_t timeStamp; /*!< Timestamp of the frame */
    uint32_t seqNo;     /*!< Optional sequence number of the frame */

    struct raf_format_type format; /*!< Format information for the frame */
    uint32_t data[0];   /*!< Start of the variable size payload.
                           It must start at 128 bit aligned address for all the frames */
};

volatile int capturing = 1;

void sigint_handler(int sig __unused) {
    ALOGE("Interrupted, setting the exit condition");
    capturing = 0;
}

/*
 *                  Buf idx                                                      Content
 * 0                                                                sdeNumDirections (1x1)
 * [1, 4, 7,... (sdeNumDirections *3-2)]                            Bearing/Direction of Dominant Sources (sdeNumDirections x1)
 * [2, 5, 8,... (sdeNumDirections *3-1)]                            Salience for the Dominant Sources (sdeNumDirections x1)
 * [3, 6, 9,... (sdeNumDirections *3)]                              RMS Estimate: Level information per-source in dB FS
 *
 * [3*sdeNumDirections +1 : 3*sdeNumDirections +1+sdeNumAngles]     SDE Frame Response (24x1)
 * [2*sdeNumDirections +1+sdeNumAngles+1]                           Source Class (1x1)
 *                                                                      0 - Ambient / Quiet
 *                                                                      1 - Speech
 *                                                                      2 - Echo
 *                                                                      3 - Music
 *                                                                      4 - Wanted / Un-wanted Noises
 * [(3*sdeNumDirections)+sdeNumAngles+2 :                           SourceClassConfidence (4x1)
 *          (3*sdeNumDirections)+sdeNumAngles+5 ]
 * [(3*sdeNumDirections)+sdeNumAngles+6 ]                           SNR Estimate in dB. And estimate of Signal to Noise Ratio (SNR)
 *                                                                  of the input signal, on a per frame basis
 * [ (3*sdeNumDirections)+nAngles+7 ]                               rxVad. Status of VAD on AEC Ref
 *                                                                      0 - OFF
 *                                                                      1 - ON
 * [ (3*sdeNumDirections)+nAngles+8:                                VP param ids and param vals
 *     (3*sdeNumDirections)+nAngles+22 ]
 */
void parse_doa_meta_data(FILE *out_fp, unsigned char *buf_itr) {
    float bearing[3], salience[3], rms_estimate[3], num_directions, source_class;
    // Frame response is for every 15 degrees, for 360 degrees we will have 24 values
    const int sde_frame_response = 24;
    const int num_of_src_class_conf = 4;
    const int num_of_vp_params = 15;
    float frame_response[sde_frame_response];
    float src_class_conf[num_of_src_class_conf];
    float snr_estimate_in_db;
    float rx_vad;
    int i;
    float param_ids[num_of_vp_params];
    float param_vals[num_of_vp_params];

    if (NULL == buf_itr || NULL == out_fp) {
        ALOGE("%s: Buffer or file pointer is NULL", __func__);
        return;
    }

    kst_float_to_IEEE_float((void *)&num_directions, buf_itr);
    buf_itr += 4;

    for (i = 0; i < num_directions; i++) {
        kst_float_to_IEEE_float((void *)&bearing[i], buf_itr);
        buf_itr += 4;

        kst_float_to_IEEE_float((void *)&salience[i], buf_itr);
        buf_itr +=4;

        kst_float_to_IEEE_float((void *)&rms_estimate[i], buf_itr);
        buf_itr +=4;
    }

    for (i = 0; i < sde_frame_response; i++) {
        kst_float_to_IEEE_float((void *)&frame_response[i], buf_itr);
        buf_itr += 4;
    }

    kst_float_to_IEEE_float((void *)&source_class, buf_itr);
    buf_itr += 4;

    for (i = 0; i < num_of_src_class_conf; i++) {
        kst_float_to_IEEE_float((void *)&src_class_conf[i], buf_itr);
        buf_itr += 4;
    }

    kst_float_to_IEEE_float((void *)&snr_estimate_in_db, buf_itr);
    buf_itr += 4;

    kst_float_to_IEEE_float((void *)&rx_vad, buf_itr);
    buf_itr += 4;

    for (i = 0; i < num_of_vp_params; i++) {
        kst_float_to_IEEE_float((void *)&param_ids[i], buf_itr);
        buf_itr += 4;

        kst_float_to_IEEE_float((void *)&param_vals[i], buf_itr);
        buf_itr += 4;
    }

    for (i = 0; i < num_directions; i++) {
        fprintf(out_fp, " bearing %d = %f:", i, bearing[i]);
    }

    for (i = 0; i < num_directions; i++) {
        fprintf(out_fp, " Salience  %d = %f:", i, salience[i]);
    }

    for (i = 0; i < num_directions; i++) {
        fprintf(out_fp, " RMS Estimate  %d = %f:", i, rms_estimate[i]);
    }

    for (i = 0; i < sde_frame_response; i++) {
        fprintf(out_fp, " SDE Frame Response %d = %f:", i, frame_response[i]);
    }

    fprintf(out_fp, " Source Class = %f:", source_class);

    for (i = 0; i < num_of_src_class_conf; i++) {
        fprintf(out_fp, " Source Class Confidence %d = %f:", i, src_class_conf[i]);
    }

    fprintf(out_fp, " SNR Estimate in dB = %f:", snr_estimate_in_db);

    fprintf(out_fp, " rxVad = %f:", rx_vad);

    for (i = 0; i < num_of_vp_params; i++) {
        fprintf(out_fp, " Param ID 0x%X = %f:", (unsigned int)param_ids[i], param_vals[i]);
    }

    fprintf(out_fp, "\n");
    fflush(out_fp);
}

/*
 * Buf idx                  Content
 *   0                     Number of keywords
 * [ 1 : numKwSlots ]      CS out Confidence level for the KWs present in that particular slot
 * [ numKwSlots + 1 ]      Number of events (numEvents)
 * [ numKwSlots + 2 ]      KW slot Index (AEC Ref instance KW_DETECT_EVENT0)
 * [ numKwSlots + 3 ]      Start Frame Sequence Number (AEC Ref instance START_FRM_EVENT1)
 * [ numKwSlots + 4 ]      End Frame Sequence Number (AEC Ref instance END_FRM_EVENT2)
 * [ numKwSlots + 5 ]      KW slot Index (AEC Ref instance TRUE_KW_EVENT3)
 * [ numKwSlots + 6 ]      KW slot Index (AEC Ref instance FA_KW_EVENT4)
 * [ numKwSlots + 7 ]      Peak confidence level (AEC Ref instance PEAK_CONF_LEVEL_EVENT5)
 * [ numKwSlots + 8 ]      Number of keywords (same as numKwsCsout)
 * [ numKwSlots + 9 :
 *   numKwSlots + 9 +
 *   numKwSlots - 1]       AEC Ref Confidence level for the KWs present in that particular slot
 */
void parse_vq_meta_data(FILE *out_fp, unsigned char *buf_itr) {
    float num_kws, confidence, temp;
    int i;

    if (NULL == buf_itr || NULL == out_fp) {
        ALOGE("%s: Buffer or file pointer is NULL", __func__);
        return;
    }

    kst_float_to_IEEE_float((void *)&num_kws, buf_itr);
    buf_itr += 4;
    fprintf(out_fp, "Num of Keywords = %f: ", num_kws);

    for (i = 0; i < (int) num_kws; i++) {
        kst_float_to_IEEE_float((void *)&confidence, buf_itr);
        buf_itr += 4;

        fprintf(out_fp, "CSOUT conf slot %d = %f: ", i, confidence);
        confidence = 0.0f;
    }

    kst_float_to_IEEE_float((void *)&temp, buf_itr);
    buf_itr += 4;
    fprintf(out_fp, "Num of events = %f: ", temp);
    temp = 0.0f;

    kst_float_to_IEEE_float((void *)&temp, buf_itr);
    buf_itr += 4;
    fprintf(out_fp, "AECREF KW_DETECT_EVENT0 = %f: ", temp);
    temp = 0.0f;

    kst_float_to_IEEE_float((void *)&temp, buf_itr);
    buf_itr += 4;
    fprintf(out_fp, "AECREF START_FRM_EVENT1 = %f: ", temp);
    temp = 0.0f;

    kst_float_to_IEEE_float((void *)&temp, buf_itr);
    buf_itr += 4;
    fprintf(out_fp, "AECREF END_FRM_EVENT2 = %f: ", temp);
    temp = 0.0f;

    kst_float_to_IEEE_float((void *)&temp, buf_itr);
    buf_itr += 4;
    fprintf(out_fp, "AECREF TRUE_KW_EVENT3 = %f: ", temp);
    temp = 0.0f;

    kst_float_to_IEEE_float((void *)&temp, buf_itr);
    buf_itr += 4;
    fprintf(out_fp, "AECREF FA_KW_EVENT4 = %f: ", temp);
    temp = 0.0f;

    kst_float_to_IEEE_float((void *)&temp, buf_itr);
    buf_itr += 4;
    fprintf(out_fp, "AECREF PEAK_CONF_LEVEL_EVENT5 = %f: ", temp);
    temp = 0.0f;

    kst_float_to_IEEE_float((void *)&num_kws, buf_itr);
    buf_itr += 4;
    fprintf(out_fp, "Num of Keywords = %f: ", num_kws);

    for (i = 0; i < (int) num_kws; i++) {
        kst_float_to_IEEE_float((void *)&confidence, buf_itr);
        buf_itr += 4;

        fprintf(out_fp, "AECREF conf slot %d = %f: ", i, confidence);
        confidence = 0.0f;
    }

    fprintf(out_fp, "\n");
}

void parse_audio_tunnel_data(FILE *out_fp, unsigned char *buf_itr, int frame_sz_in_bytes) {
    char q16_buf[BUF_SIZE]; // This can be smaller but by how much?
    int frameSizeInWords = (frame_sz_in_bytes + 3) >> 2;

    if (NULL == buf_itr || NULL == out_fp) {
        ALOGE("%s: Buffer or file pointer is NULL", __func__);
        return;
    }

    kst_float_to_q15_vector(q16_buf, buf_itr, frameSizeInWords);

    fwrite(q16_buf, (frameSizeInWords * 2), 1, out_fp);
}

/*
 * Buffer Index             Content
 *      [0]                 ParamCount - No of updated parameters in that particular frame
 * [1, 3, 5, ...]           VP runtime parameter ID's
 * [2, 4, 6, ...]           VP runtime parameter value's
 */
void parse_param_data(FILE *out_fp, unsigned char *buf_itr) {
    float param_id = 0.0f, param_val = 0.0f;
    int i = 0;
    float num_of_params = 0.0f;

    if (NULL == buf_itr || NULL == out_fp) {
        ALOGE("%s: Buffer or file pointer is NULL", __func__);
        return;
    }

    kst_float_to_IEEE_float((void *)&num_of_params, buf_itr);
    buf_itr += 4;

    fprintf(out_fp, "PARAM ID\tPARAM VALUE\n");
    for (i = 0; i < num_of_params; i++) {
        kst_float_to_IEEE_float((void *)&param_id, buf_itr);
        buf_itr += 4;

        kst_float_to_IEEE_float((void *)&param_val, buf_itr);
        buf_itr += 4;

        fprintf(out_fp, "0x%X\t\t%f\n", (unsigned int)param_id, param_val);
    }
}

int main(int argc, char *argv[]) {
    struct ia_tunneling_hal *thdl = NULL;
    int err = 0;
    FILE *out_fp[MAX_TUNNELS] = { NULL };
    FILE *unp_out_fp = NULL;
    int bytes_avail = 0, bytes_rem = 0;
    int bytes_read = 0;
    void *buf = NULL;
    // The magic number is ROME in ASCII reversed. So we are looking for EMOR in the byte stream
    const unsigned char magic_num[4]  = {0x45, 0x4D, 0x4F, 0x52};
    int i = 0;
    bool valid_frame = true;
    int num_of_tunnels = 0;
    int timer_signal = 0;
    int tunnel_src[MAX_TUNNELS] = { 0 };
    int tunnel_mode[MAX_TUNNELS] = { 0 };
    int tunnel_encode[MAX_TUNNELS];
    int lastSeqNum[MAX_TUNNELS] = { 0 };
    int notFirstFrame[MAX_TUNNELS] = { 0 };
    int frameDropCount[MAX_TUNNELS] = { 0 };
    uint64_t tunnel_time_stamps[MAX_TUNNELS] = { 0 };
    bool is_initial_align_reached = true;
    unsigned char *frame_start, *buf_itr;
    // Minimum bytes required is the magic number + tunnel id + reserved and crc + raf struct
    int min_bytes_req = 4 + 2 + 6 + sizeof(struct raf_frame_type);
    int instance;

    if (argc < 5) {
        ALOGE("USAGE: %s <instance number> <Number of tunnels> <Time in seconds> <Source End pt 1> <tnl mode> <encode fmt> <Source End pt 2> <tnl mode> <encode fmt>...", argv[0]);
        return -EINVAL;
    }

    instance = strtol(argv[1], NULL, 0);
    ALOGD("instance %d", instance);

    num_of_tunnels = strtol(argv[2], NULL, 0);
    ALOGD("Number of tunnels %d", num_of_tunnels);

    timer_signal= strtol(argv[3], NULL, 0);
    ALOGD("tunnel out timer based req %d", timer_signal);

    if (argc != (num_of_tunnels * 3 + 4)) {
        ALOGE("USAGE: %s <instance number> <Number of tunnels> <Sync Tunnel req> <Source End pt 1> <tnl mode> <encode fmt> <Source End pt 2> <tnl mode> <encode fmt>...", argv[0]);
        return -EINVAL;
    }

    for (i = 0; i < num_of_tunnels; i++) {
        tunnel_src[i] = strtol(argv[i*3+4], NULL, 0);
        tunnel_mode[i] = strtol(argv[i*3+5], NULL, 0);
        tunnel_encode[i] = strtol(argv[i*3+6], NULL, 0);
        ALOGD("Tunnel source 0x%x Tunnel mode %d Tunnel encode %d", tunnel_src[i], tunnel_mode[i], tunnel_encode[i]);
    }

    thdl = ia_start_tunneling(0);
    if (NULL == thdl) {
        ALOGE("Failed to start tunneling");
        goto exit;
    }

    for (i = 0; i < num_of_tunnels; i++) {
        err = ia_enable_tunneling_source(thdl, tunnel_src[i], tunnel_mode[i], tunnel_encode[i]);
        if (0 != err) {
            ALOGE("Failed to enable tunneling for src_id %u mode %u encode %u", tunnel_src[i], tunnel_mode[i], tunnel_encode[i]);
            goto exit;
        }
    }

    buf = malloc(BUF_SIZE * 2);
    if (NULL == buf) {
        ALOGE("Failed to allocate memory to read buffer");
        goto exit;
    }

    unp_out_fp = fopen(UNPARSED_OUTPUT_FILE, "wb");
    if (NULL == unp_out_fp) {
        ALOGE("Failed to open the file %s", UNPARSED_OUTPUT_FILE);
        goto exit;
    }

    signal(SIGINT, sigint_handler);

    if (num_of_tunnels && timer_signal) {
        signal(SIGALRM, sigint_handler);
        alarm(timer_signal);
    }

    unsigned short int tunnel_id;
    unsigned short int tunl_src;
    while (1) {
read_again:
        if (0 == capturing) {
            ALOGE("Time to bail from here");
            break;
        }

        if (0 != bytes_avail) {
            if (bytes_avail < 0) {
                bytes_rem = 0;
            } else {
                bytes_rem = bytes_avail;
                ALOGD("bytes_avail is %d", bytes_rem);
                memcpy(buf, buf_itr, bytes_rem);
            }
        } else {
			bytes_rem = 0;
		}

        // Ensure that we read BUF_SIZE always otherwise the kernel read will hang
        bytes_avail = ia_read_tunnel_data (thdl,
                                           (void *)((unsigned char *)buf + bytes_rem),
                                           BUF_SIZE);
        if (bytes_avail <= 0) {
            ALOGE("Failed to read data from the tunnel");
            break;
        }

        fwrite((void *)((unsigned char *)buf + bytes_rem), bytes_avail, 1, unp_out_fp);
        fflush(unp_out_fp);

        bytes_avail += bytes_rem; // update the available bytes with the previous reminder if any
        ALOGD("bytes_avail is after read %d", bytes_avail);
        buf_itr = (unsigned char *)buf;

        do {
            // Check for MagicNumber 0x454D4F52
            if (buf_itr[0] != magic_num[0] || buf_itr[1] != magic_num[1] ||
                    buf_itr[2] != magic_num[2] || buf_itr[3] != magic_num[3]) {
                  ALOGE("Could not find the magic number, reading again");
                  ALOGE("buf_itr[0] %x buf_itr[1] %x buf_itr[2] %x buf_itr[3] %x ",
                            buf_itr[0], buf_itr[1], buf_itr[2], buf_itr[3]);
                  goto exit;
            }
            ALOGD("bytes_avail is after magic %d: prev :%d", bytes_avail, bytes_avail + 540);
            // Bookmark the start of the frame
            frame_start = buf_itr;

            // Skip the magic number
            buf_itr += 4;
            bytes_avail -= 4;

            // Read the tunnelID
            tunnel_id = ((unsigned char) (buf_itr[0]) |
                         (unsigned char) (buf_itr[1]) << 8);

            // Skip tunnelID
            buf_itr += 2;
            bytes_avail -= 2;

            tunl_src = ((unsigned char) (buf_itr[0]) |
                         (unsigned char) (buf_itr[1]) << 8);

            // Skip src id field and CRC - 6 bytes in total
            buf_itr += 6;
            bytes_avail -= 6;

            valid_frame = true;
            if (tunnel_id > MAX_TUNNELS) {
                ALOGE("Invalid tunnel id %d", tunnel_id);
                valid_frame = false;
            }

            struct raf_frame_type rft;
            memcpy(&rft, buf_itr, sizeof(struct raf_frame_type));
            if (true == valid_frame) {
                if (NULL == out_fp[tunnel_id]) {
                    char filename[256];

                    if (DOA_TUNNEL_SRC == tunl_src) {
                        snprintf(filename, 256, "%s_%d.txt", DOA_OUTPUT_FILE, instance);
                    } else if (VQ_TUNNEL_SRC == tunl_src) {
                        snprintf(filename, 256, "%s_%d.txt", VQ_CONFIDENCE_OUTPUT_FILE, instance);
                    } else if (VP_PARAM_TUNNEL_SRC == tunl_src) {
                        snprintf(filename, 256, "%s_%d.txt", VP_PARAM_DUMP_FILE, instance);
                    } else {
                        snprintf(filename, 256, "%sid%d-src0x%x-enc0x%x_client%d.pcm", OUTPUT_FILE, tunnel_id, tunl_src, rft.format.encoding, instance);
                    }
                    // Open the file to dump
                    out_fp[tunnel_id] = fopen(filename, "wb");
                    if (NULL == out_fp[tunnel_id]) {
                        ALOGE("ERROR: Failed to open the file %s", filename);
                        goto exit;
                    }
                }
            }

            ALOGD("Tunnel id %d timestamp %llu", tunnel_id, rft.timeStamp);
            tunnel_time_stamps[tunnel_id] = rft.timeStamp;

            // Skip the raf_frame_type
            buf_itr += sizeof(struct raf_frame_type);
            bytes_avail -= sizeof(struct raf_frame_type);

            if (bytes_avail < rft.format.frameSizeInBytes) {
                ALOGD("Incomplete frame received bytes_avail %d framesize %d", bytes_avail, rft.format.frameSizeInBytes);
                buf_itr = frame_start;
                bytes_avail += min_bytes_req;
                goto read_again;
            }

            if (true == valid_frame) {
                if (DOA_TUNNEL_SRC == tunl_src) {
                    parse_doa_meta_data(out_fp[tunnel_id], (void*) buf_itr);
                } else if (VQ_TUNNEL_SRC == tunl_src) {
                    parse_vq_meta_data(out_fp[tunnel_id], (void*) buf_itr);
                } else if (VP_PARAM_TUNNEL_SRC == tunl_src) {
                    parse_param_data(out_fp[tunnel_id], (void*) buf_itr);
                } else {
                    ALOGD("@@@Tunnel id %d encoding %d", tunnel_id, rft.format.encoding);
                    if (TNL_ENC_AFLOAT == rft.format.encoding) {
                        parse_audio_tunnel_data(out_fp[tunnel_id], buf_itr, rft.format.frameSizeInBytes);
                    } else {
                        fwrite(buf_itr, rft.format.frameSizeInBytes, 1, out_fp[tunnel_id]);
                    }
                }
            }

            /* Calculate the frame drop count */
            if (notFirstFrame[tunnel_id]) {
                   frameDropCount[tunnel_id] += (rft.seqNo - lastSeqNum[tunnel_id] - 1);
            }
            lastSeqNum[tunnel_id] = rft.seqNo;
            notFirstFrame[tunnel_id] = 1;
            // Skip the data
            buf_itr += rft.format.frameSizeInBytes;
            bytes_avail -= rft.format.frameSizeInBytes;
            /* For debugging the tunnel read errors or wrong magic numbers or bus errors*/
			bytes_read += rft.format.frameSizeInBytes + min_bytes_req;
        } while (bytes_avail > min_bytes_req);
    }

exit:
    for (i = 0; i < MAX_TUNNELS; i++) {
        if (notFirstFrame[i]) {
			ALOGE("drop count tunnel id %u: %u", i, frameDropCount[i]);
        }
    }
    ALOGE("bytes_read so far %d", bytes_read);
    if (buf) {
        free(buf);
        buf = NULL;
    }

    if (unp_out_fp) {
        fflush(unp_out_fp);
        fclose(unp_out_fp);
    }

    for (i = 0; i < MAX_TUNNELS; i++) {
        if (out_fp[i])
            fclose(out_fp[i]);
    }

    for (i = 0; i < num_of_tunnels; i++) {
        err = ia_disable_tunneling_source(thdl, tunnel_src[i], tunnel_mode[i], tunnel_encode[i]);
        if (0 != err) {
            ALOGE("Failed to disable tunneling for tunl_id %u src_id %u", i, tunnel_src[i]);
        }
    }

    err = ia_stop_tunneling(thdl);
    if (0 != err) {
        ALOGE("Failed to stop tunneling");
    }

    return 0;
}
