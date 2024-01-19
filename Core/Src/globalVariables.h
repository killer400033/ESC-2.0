#include <stdint.h>

enum AdcReadMap {
	ignore_i1,
	ignore_i2,
	ignore_i3,
};

typedef struct TrigState {
	uint32_t encoder_out; // TIM1
	uint16_t adc_out[3]; // ADC1 and ADC2
	uint32_t cycle_cnt;
	enum AdcReadMap adc_read_map;
} TrigState;

extern TrigState saved_state;
extern char _msg[100];
extern uint32_t curr_cycle_cnt;
extern enum AdcReadMap curr_adc_read_map;
extern uint32_t pid_loop_overrun;
