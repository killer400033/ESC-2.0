#include <stdint.h>

// These are the saved values captured every PWM cycle
struct TrigState {
	uint32_t encoder_out; // TIM1
	uint16_t adc_out[3]; // ADC1 and ADC2
	uint32_t cycle_cnt;
};

extern struct TrigState _saved_state;
extern struct TrigState saved_state;
extern char _msg[80];
extern uint32_t curr_cycle_cnt;
