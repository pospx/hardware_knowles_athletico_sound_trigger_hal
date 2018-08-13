#ifndef _CVQ_IOCTL_H
#define _CVQ_IOCTL_H

/* KW model files info */
#define IA_INIT_PARAMS		_IO('T', 0x011)
#define IA_CVQ_START		_IO('T', 0x012)
#define IA_LOAD_KEYWORDS	_IO('T', 0x013)
#define IA_UNLOAD_KEYWORDS	_IO('T', 0x014)
#define IA_CVQ_STOP		_IO('T', 0x015)
#define IA_GET_KW_ID		_IO('T', 0x016)
#define IA_IS_PERFMODE          _IO('T', 0x017)

/* Voice Id has two models, hence total models is 4 (1 OEM + 1 VoiceId + 1 user)
*/
#define MAX_KW_MODELS 5

struct ia_kw_priv {
	uint64_t kw_buff_addr;

	/* Actual length in bytes after adding wdb headers and padding bytes */
	uint32_t kw_size;
	/* Length in bytes without the padding bytes and the wdb headers */
	uint32_t wdb_size;

};

struct ia_kw_info {
	struct ia_kw_priv kw[MAX_KW_MODELS];
	uint32_t kw_count;
};

enum ia_cvq_rate {
	IA_8KHZ = 0,
	IA_16KHZ = 1,
	IA_24KHZ = 2,
	IA_48KHZ = 4,
};

enum ia_vq_mode {
	IA_VS_MODE = 0,
	IA_CVQ_MODE,
};

enum ia_kw_preserve {
	IA_IGNORE_KW = 0,
	IA_PRESERVE_KW,
};

enum ia_format {
	IA_FORMAT_Q11 = 1,
	IA_FORMAT_Q15 = 2,
};

enum ia_frame_size {
	IA_1MS_FRAME = 1,
	IA_2MS_FRAME = 2,
	IA_8MS_FRAME = 8,
	IA_10MS_FRAME = 10,
	IA_15MS_FRAME = 15,
	IA_16MS_FRAME = 16,
};

enum ia_vad_mode {
    IA_NO_VAD = 0,
    IA_MIC_VAD,
};

struct ia_cvq_params {
    uint8_t rate;
    uint8_t mode;
    uint8_t format;
    uint8_t frame_size;
    uint8_t kw_preserve;
    uint8_t vad;
    uint8_t preset;
};

typedef enum _ia_perf_mode_e {
        IA_NON_PERF_MODE = 0,
        IA_I2S_PERF_MODE,
}ia_perf_mode_e;

static inline bool isvalid_frame_size(uint8_t frame_size)
{
	bool valid = false;
	switch (frame_size) {
	case IA_1MS_FRAME:
	case IA_2MS_FRAME:
	case IA_8MS_FRAME:
	case IA_10MS_FRAME:
	case IA_15MS_FRAME:
	case IA_16MS_FRAME:
		valid = true;
		break;
	}

	return valid;
}

static inline bool isvalid_rate(uint8_t rate)
{
	bool valid = false;
	switch (rate) {
	case IA_8KHZ:
	case IA_16KHZ:
	case IA_24KHZ:
	case IA_48KHZ:
		valid = true;
		break;
	}

	return valid;
}

static inline bool isvalid_format(uint8_t format)
{
	if (format != IA_FORMAT_Q11 && format != IA_FORMAT_Q15)
		return false;
	return true;
}

static inline bool isvalid_mode(uint8_t mode)
{
	if (mode != IA_VS_MODE && mode != IA_CVQ_MODE)
		return false;

	return true;
}

static inline bool isvalid_kw_option(uint8_t option)
{
	if (option != IA_IGNORE_KW && option != IA_PRESERVE_KW)
		return false;

	return true;
}

int start_cvq(void);
int stop_cvq(void);
int init_params();
int write_model(unsigned char *data, int length);
int flush_model(void);
int get_event(struct iaxxx_get_event *ge);
int unload_all_models();
int setup_mic_routes();
int enable_mic_route(int enable);
int set_sensor_route(bool enable);

#endif /* _CVQ_IOCTL_H */
