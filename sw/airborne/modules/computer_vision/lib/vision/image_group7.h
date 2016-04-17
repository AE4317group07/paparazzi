
/**
 * @file modules/computer_vision/lib/vision/image_group7.h
 */

#ifndef _CV_LIB_VISION_IMAGE_GROUP7_H
#define _CV_LIB_VISION_IMAGE_GROUP7_H

#include "std.h"
#include <stdint.h>
#include <sys/time.h>
#include "modules/computer_vision/lib/vision/image.h"

/* Image function */

int16_t  image_yuv422_colorfilter(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M,
 uint8_t y_m2, uint8_t y_M2, uint8_t u_m2, uint8_t u_M2, uint8_t v_m2, uint8_t v_M2, int thres, int trb);
#endif
