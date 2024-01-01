#include "helperFunctions.h"
#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "dmaVariables.h"
#include "constants.h"

extern void fixedToFloat(uint32_t *input, float *output);

// Function Declarations
void calculateSVM(float _vq, float _vd);
void printData(void);
void calculatePID(void);

// Variables
uint8_t printPending = TRUE;

void mainLoop(void) {
	snprintf(_msg, 80, "HELLO WORLD");
	// Main Running Loop
  while (1) {

  	// Running Motor Loop
  	calculateSVM(0.2, 0);

    // USART printing
    if (LL_DMA_IsActiveFlag_TC1(DMA1) && printPending) {
    	printData();
    }
  }
}

void calculatePID(void) {

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
	uint32_t raw_output[2];
	float trig_output[2];

	// CORDIC code
	LL_CORDIC_WriteData(CORDIC, ((uint32_t)(((float)(LL_TIM_GetCounter(TIM3) % MAGNETIC_AGL_ENCODER_CNT)) * ENCODER_TO_ANGLE)) | 0x7FFF0000);
	// This stalls the CPU memory bus until the cordic calculation is complete, so if for some reason we need fast interrupts later, this must be changed to polling
	raw_output[0] = LL_CORDIC_ReadData(CORDIC) & 0xFFFF;
	raw_output[1] = LL_CORDIC_ReadData(CORDIC) >> 16;
	fixedToFloat(raw_output, trig_output);

	// Example Timer code

	// Inverse Park Transform (q axis aligns with d axis)
	_va = trig_output[1] * _vd - trig_output[0] * _vq;
	_vb = trig_output[0] * _vd - trig_output[1] * _vq;

	// Inverse Clarke Transform
	_v1 = _va * SVM_COMPEN;
	_v2 = (-_va*0.5 + _vb*SQRT3ON2) * SVM_COMPEN;
	_v3 = (-_va*0.5 - _vb*SQRT3ON2) * SVM_COMPEN;

	// Space Vector Modulation
	// Finds the min and max value in v1, v2, v3
	if (_v1 > _v2) {
		_min = _v2;
		_max = _v1;
	}
	else {
		_min = _v1;
		_max = _v2;
	}

	if (_v3 > _max) {
		_max = _v3;
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
	snprintf(_msg, 80, "encoder: %lu, ADC: %u - %u - %u, Cycle: %lu\r\n", LL_TIM_GetCounter(TIM3), saved_state.adc_out[0], saved_state.adc_out[1], saved_state.adc_out[2], saved_state.cycle_cnt);
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 80);
  LL_DMA_ClearFlag_TC1(DMA1);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}
