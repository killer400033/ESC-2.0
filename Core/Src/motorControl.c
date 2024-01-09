#include "motorControl.h"
#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "constants.h"
#include "globalVariables.h"

extern void fixedToFloat(uint32_t *input, float *output);

// Variables
uint8_t printPending = TRUE;

typedef struct PIDState {
	float error;
	float integral;
} PIDState;

float _P = 1.0;
float _I = 1.0;

float set_iq = 1.0;
float set_id = 0;

// Function Declarations
void calculateSVM(float _vq, float _vd);
void printData(void);
void doPIDLoop(float *_vq, float *_vd, PIDState *prev_q, PIDState *prev_d);
float calculatePID(PIDState *prev_state, float curr_error, float time);

void mainLoop(void) {
	float _vq = 0;
	float _vd = 0;
	PIDState prev_q = {.error=0, .integral=0};
	PIDState prev_d = {.error=0, .integral=0};
	// Main Running Loop
  while (1) {

  	// Updating Motor Output
  	calculateSVM(_vq, _vd);

  	// Analyzing Motor Inputs
  	if (pid_loop_overrun) {
  		doPIDLoop(&_vq, &_vd, &prev_q, &prev_d);
  		pid_loop_overrun--;
  	}

    // USART printing
    if (LL_DMA_IsActiveFlag_TC1(DMA1) && printPending) {
    	printData();
    }
  }
}

void doPIDLoop(float *_vq, float *_vd, PIDState *prev_q, PIDState *prev_d) {
	float _ia;
	float _ib;
	float _i1;
	float _i2;
	float _i3;
	float _iq;
	float _id;
	float time;

	TrigState curr_saved_state = saved_state; // This makes sure saved_state doesn't change through loop
	curr_cycle_cnt -= curr_saved_state.cycle_cnt; // Make sure PID frequency is atleast 2x slower than PWM frequency
	uint32_t raw_output[2];
	float trig_output[2];

	LL_CORDIC_WriteData(CORDIC, ((uint32_t)(((float)(curr_saved_state.encoder_out % MAGNETIC_AGL_ENCODER_CNT)) * ENCODER_TO_ANGLE)) | 0x7FFF0000);

	_i1 = curr_saved_state.adc_out[1] * ADC_SCALING;
	_i2 = curr_saved_state.adc_out[2] * ADC_SCALING;
	_i3 = curr_saved_state.adc_out[3] * ADC_SCALING;

	// Clarke Transform
	if (curr_saved_state.adc_read_map == ignore_i1) {
		_ia = -_i2 -_i3;
		_ib = ONEONSQRT3*_i2 - ONEONSQRT3*_i3;
	}
	else if (curr_saved_state.adc_read_map == ignore_i2) {
		_ia = _i1;
		_ib = -ONEONSQRT3*_i1 - 2*ONEONSQRT3*_i3;
	}
	else {
		_ia = _i1;
		_ib = ONEONSQRT3*_i1 + 2*ONEONSQRT3*_i2;
	}

	raw_output[0] = LL_CORDIC_ReadData(CORDIC) & 0xFFFF;
	raw_output[1] = LL_CORDIC_ReadData(CORDIC) >> 16;
	fixedToFloat(raw_output, trig_output);

	// Park Transform (d axis aligns with a axis)
	_id = trig_output[1] * _ia + trig_output[0] * _ib;
	_iq = -trig_output[0] * _ia + trig_output[1] * _ib;

	time = curr_saved_state.cycle_cnt * (2 * PWM_PERIOD) / CLK;
	*_vq = calculatePID(prev_q, (set_iq - _iq), time);
	*_vd = calculatePID(prev_d, (set_id - _id), time);
}

float calculatePID(PIDState *prev_state, float curr_error, float time) {
	float proportional = _P * curr_error;
	float integral = prev_state->integral + _I*time*0.5*(curr_error + prev_state->error);
	float output = proportional + integral;

	prev_state->integral = integral;
	prev_state->error = curr_error;

	return output;
}

void calculateSVM(float _vq, float _vd) {
	float _va;
	float _vb;
	float _v1;
	float _v2;
	float _v3;
	float _max;
	float _min;
	float _mid;
	float trig_output[2];
	uint32_t raw_output[2];

	// CORDIC code
	LL_CORDIC_WriteData(CORDIC, ((uint32_t)(((float)(LL_TIM_GetCounter(TIM3) % MAGNETIC_AGL_ENCODER_CNT)) * ENCODER_TO_ANGLE)) | 0x7FFF0000);
	// This stalls the CPU memory bus until the cordic calculation is complete, so if for some reason we need fast interrupts later, this must be changed to polling
	raw_output[0] = LL_CORDIC_ReadData(CORDIC) & 0xFFFF;
	raw_output[1] = LL_CORDIC_ReadData(CORDIC) >> 16;
	fixedToFloat(raw_output, trig_output);

	// Example Timer code

	// Inverse Park Transform (d axis aligns with a axis)
	_va = trig_output[1] * _vd - trig_output[0] * _vq;
	_vb = trig_output[0] * _vd + trig_output[1] * _vq;

	// Inverse Clarke Transform
	_v1 = _va * ONEONSQRT3; // Multiplying by 1/sqrt(3) scales the vectors such that when |(va, vb)| = 1, the SVM is saturated
	_v2 = (-_va*0.5 + _vb*SQRT3ON2) * ONEONSQRT3;
	_v3 = (-_va*0.5 - _vb*SQRT3ON2) * ONEONSQRT3;

	// Space Vector Modulation
	// Finds the min and max value in v1, v2, v3
	if (_v1 > _v2) {
		_min = _v2;
		_max = _v1;
		curr_adc_read_map = ignore_i1;
	}
	else {
		_min = _v1;
		_max = _v2;
		curr_adc_read_map = ignore_i2;
	}

	if (_v3 > _max) {
		_max = _v3;
		curr_adc_read_map = ignore_i3;
	}
	else if (_v3 < _min) {
		_min = _v3;
	}

	// Converts the sinusoidal waveform into SVM between 0 and 1
	_mid = (_min + _max) / 2.0;
	_v1 += 0.5 - _mid;
	_v2 += 0.5 - _mid;
	_v3 += 0.5 - _mid;

	// Clamp v1, v2, v3 between 0 to 1. This automatically applies Overmodulation into a trapezoidal waveform
	_v1 = (_v1 < 0) ? 0 : ((_v1 > 1) ? 1 : _v1);
	_v2 = (_v2 < 0) ? 0 : ((_v2 > 1) ? 1 : _v2);
	_v3 = (_v3 < 0) ? 0 : ((_v3 > 1) ? 1 : _v3);

	LL_TIM_OC_SetCompareCH1(TIM1, (float)PWM_PERIOD*_v1);
	LL_TIM_OC_SetCompareCH2(TIM1, (float)PWM_PERIOD*_v2);
	LL_TIM_OC_SetCompareCH3(TIM1, (float)PWM_PERIOD*_v3);
}

void printData(void) {
	snprintf(_msg, 100, "encoder: %lu, ADC: %u - %u - %u, Cycle: %lu, Overrun: %lu\r\n", LL_TIM_GetCounter(TIM3),
			saved_state.adc_out[0], saved_state.adc_out[1], saved_state.adc_out[2], saved_state.cycle_cnt, pid_loop_overrun);
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 100);
  LL_DMA_ClearFlag_TC1(DMA1);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}
