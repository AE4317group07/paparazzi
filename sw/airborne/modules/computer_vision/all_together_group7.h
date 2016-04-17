/**
 * @file modules/computer_vision/all_together_group7.h
 */

#ifndef ALL_TOGETHER_H
#define ALL_TOGETHER_H

#include <stdint.h>

extern struct opticflow_t opticflow;                      //< Opticflow calculations
extern struct opticflow_result_t opticflow_result;

extern uint8_t color_lum_min_o;
extern uint8_t color_lum_max_o;

extern uint8_t color_cb_min_o;
extern uint8_t color_cb_max_o;

extern uint8_t color_cr_min_o;
extern uint8_t color_cr_max_o;

extern uint8_t color_lum_min_b;
extern uint8_t color_lum_max_b;

extern uint8_t color_cb_min_b;
extern uint8_t color_cb_max_b;

extern uint8_t color_cr_min_b;
extern uint8_t color_cr_max_b;

extern int obstacle_region;
extern uint16_t tro;
extern uint16_t trb;

// Module functions
extern void all_together_init(void);

#endif /* ALL_TOGETHER_H */
