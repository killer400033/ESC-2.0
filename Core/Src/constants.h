#define TRUE 1
#define FALSE 0

#define SQRT3ON2 0.86602540378
// This scales _va and _vb by 1/sqrt(3) such that when |(vq, vd)| = 1, the SVM is saturated
#define SVM_COMPEN 0.57735026919
#define POLE_CNT 14
#define ENCODER_RES 2400
#define MAGNETIC_AGL_ENCODER_CNT (ENCODER_RES * 2 / POLE_CNT)
#define ENCODER_TO_ANGLE (65536.0/(float)MAGNETIC_AGL_ENCODER_CNT)
#define PWM_PERIOD 4250
