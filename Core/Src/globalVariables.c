#include "globalVariables.h"

TrigState _saved_state = {0};
TrigState saved_state = {0};
char _msg[100] = {0};
uint32_t curr_cycle_cnt = 0;
enum AdcReadMap curr_adc_read_map;
