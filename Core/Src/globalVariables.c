#include "globalVariables.h"

TrigState saved_state = {0};
char _msg[100] = {0};
uint32_t curr_cycle_cnt = 0;
enum AdcReadMap curr_adc_read_map;
uint32_t pid_loop_overrun = 0;
