#ifndef OPTICAL_FLOW_CALCULATOR_H
#define OPTICAL_FLOW_CALCULATOR_H

#include "std.h"
#include "opticflow/inter_thread_data.h"
#include "lib/vision/image_group7.h"
#include "lib/v4l/v4l2.h"

struct opticflow_t {
  bool_t got_first_img;             ///< If we got a image to work with
  float prev_phi;                   ///< Phi from the previous image frame
  float prev_theta;                 ///< Theta from the previous image frame
  struct image_t img_gray;          ///< Current gray image frame
  struct image_t prev_img_gray;     ///< Previous gray image frame
  struct timeval prev_timestamp;    ///< Timestamp of the previous frame, used for FPS calculation

  uint8_t max_track_corners;        ///< Maximum amount of corners Lucas Kanade should track
  uint16_t window_size;             ///< Window size of the Lucas Kanade calculation (needs to be even)
  uint8_t subpixel_factor;          ///< The amount of subpixels per pixel
  uint8_t max_iterations;           ///< The maximum amount of iterations the Lucas Kanade algorithm should do
  uint8_t threshold_vec;            ///< The threshold in x, y subpixels which the algorithm should stop

  bool_t fast9_adaptive;            ///< Whether the FAST9 threshold should be adaptive
  uint8_t fast9_threshold;          ///< FAST9 corner detection threshold
  uint16_t fast9_min_distance;      ///< Minimum distance in pixels between corners
};

//Declaration of functions
void opticflow_calc_init(struct opticflow_t *opticflow, uint16_t w, uint16_t h);
struct point_t compute_FOE(struct image_t* img,struct flow_t *vectors,uint16_t tracked_cnt, uint8_t subpixel_factor);
float *compute_TTC(struct flow_t *vectors,uint16_t tracked_cnt, uint8_t subpixel_factor, struct point_t foe);

#endif /* OPTICAL_FLOW_CALCULATOR_H */


