// Variables
#define POLE_CNT 2
#define ENCODER_RES 2400
#define CLK 170000000
#define PWM_PERIOD 4250
#define ADC_SCALING 10.0 / 4096.0;

// Constants
#define TRUE 1
#define FALSE 0
#define SQRT3ON2 0.86602540378
#define ONEONSQRT3 0.57735026919 // This scales _va and _vb by 1/sqrt(3) such that when |(vq, vd)| = 1, the SVM is saturated
#define MAGNETIC_AGL_ENCODER_CNT (ENCODER_RES * 2 / POLE_CNT)
#define ENCODER_TO_ANGLE (65536.0/(float)MAGNETIC_AGL_ENCODER_CNT)
