/******************************************************************************************************
*
*  Authors: Ricardo Almeida, Pedro Dias, Diogo Martins, Afonso Mendes
*  Date: 16/04/2016
*  Contains the function to initialize and the main function of the module used for obstacle avoidance
*  Two different avoidance strategies are implemented (optical flow and advanced color filter)
*  The used strategy can be chosen with in the RUN_COLOR define
*
******************************************************************************************************/

/**
 * @file modules/computer_vision/all_together.c
 */

// Include headers
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/all_together_group7.h"
#include "modules/computer_vision/optical_flow_calculator_group7.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/lib/vision/image_group7.h"
#include "opticflow/inter_thread_data.h"
#include "lib/vision/lucas_kanade.h"
#include "lib/vision/fast_rosten.h"

#include "subsystems/datalink/telemetry.h"
#include <stdio.h>
#include <std.h>
#include "state.h"

// Filter color Settings
uint8_t color_lum_min_o=0;
uint8_t color_lum_max_o=131;
uint8_t color_cb_min_o=93;
uint8_t color_cb_max_o=255;
uint8_t color_cr_min_o=134;
uint8_t color_cr_max_o=255;

uint8_t color_lum_min_b=20;
uint8_t color_lum_max_b=39;
uint8_t color_cb_min_b=0;
uint8_t color_cb_max_b=213;
uint8_t color_cr_min_b=24;
uint8_t color_cr_max_b=174;

// Result
int obstacle_region = 0;

// Color threshold values
uint16_t tro = 1311;
uint16_t trb = 2220;

//DEFINES SECTION

// Camera parameters 
#ifndef OPTICFLOW_FOV_W
#define OPTICFLOW_FOV_W 0.89360857702
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FOV_W)

#ifndef OPTICFLOW_FOV_H
#define OPTICFLOW_FOV_H 0.67020643276
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FOV_H)

#ifndef OPTICFLOW_FX
#define OPTICFLOW_FX 343.1211
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FX)

#ifndef OPTICFLOW_FY
#define OPTICFLOW_FY 348.5053
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FY)

// Shows in VLC the FAST9 detected corners
#ifndef OPTICFLOW_SHOW_CORNERS
#define OPTICFLOW_SHOW_CORNERS 0
#endif
PRINT_CONFIG_VAR(OPTICFLOW_SHOW_CORNERS)

// Shows in VLC the Luckas-Kannade flow results
#ifndef OPTICFLOW_SHOW_FLOW
#define OPTICFLOW_SHOW_FLOW 0
#endif
PRINT_CONFIG_VAR(OPTICFLOW_SHOW_FLOW)

// TTC threshold value
#ifndef TTC_THRESHOLD
#define TTC_THRESHOLD 2
#endif
PRINT_CONFIG_VAR(TTC_THRESHOLD)

#ifndef WIDTH
#define WIDTH 270
#endif
PRINT_CONFIG_VAR(WIDTH)

// Selects which strategy to run, 1 if want to run color
#ifndef RUN_COLOR
#define RUN_COLOR 1
#endif
PRINT_CONFIG_VAR(RUN_COLOR)

// Used structures variables
struct opticflow_t opticflow;                  
struct opticflow_result_t opticflow_result;
static struct opticflow_state_t opticflow_state; 

// Function
bool_t optical_flow_func(struct image_t* img);
bool_t optical_flow_func(struct image_t* img)
{
  // Convert image to grayscale
  image_to_grayscale(img, &opticflow.img_gray);

  // Copy to previous image if not set
  if (!opticflow.got_first_img) {
    image_copy(&opticflow.img_gray, &opticflow.prev_img_gray);
    opticflow.got_first_img = TRUE;
  }

  // *************************************************************************************
  // Corner detection
  // *************************************************************************************

  //printf("\n \n FAST9 Threshold: %d \n \n",opticflow.fast9_threshold);

  struct point_t *corners = fast9_detect(img, opticflow.fast9_threshold, opticflow.fast9_min_distance,
	                                 img->w/4, img->h/4, &opticflow_result.corner_cnt);

  // Adaptive threshold
  if (opticflow.fast9_adaptive) {

    // Decrease and increase the threshold based on previous values
    if (opticflow_result.corner_cnt < 40 && opticflow.fast9_threshold > 5) {
      opticflow.fast9_threshold--;
    } else if (opticflow_result.corner_cnt > 50 && opticflow.fast9_threshold < 60) {
      opticflow.fast9_threshold++;
    }
  }

// Shows the VLC stream of the found corner features
#if OPTICFLOW_SHOW_CORNERS
  image_show_points(img, corners, opticflow_result.corner_cnt);
#endif

  // Check if we found some corners to track
  if (opticflow_result.corner_cnt < 1) {
    free(corners);
    image_copy(&opticflow.img_gray, &opticflow.prev_img_gray);
    return FALSE;
  }

  // *************************************************************************************
  // Corner Tracking
  // *************************************************************************************

  // Execute a Lucas Kanade optical flow

  opticflow_result.tracked_cnt = opticflow_result.corner_cnt;

  struct flow_t *vectors = opticFlowLK(&opticflow.img_gray, &opticflow.prev_img_gray, corners, &opticflow_result.tracked_cnt, opticflow.window_size / 2, opticflow.subpixel_factor, opticflow.max_iterations,
  opticflow.threshold_vec, opticflow.max_track_corners);

// Shows in VLC the optical flow vectors
#if OPTICFLOW_SHOW_FLOW
  image_show_flow(img, vectors, opticflow_result.tracked_cnt, opticflow.subpixel_factor);
#endif

  // Get the state to do flow derotation
  opticflow_state.phi = stateGetNedToBodyEulers_f()->phi;
  opticflow_state.theta = stateGetNedToBodyEulers_f()->theta;

  // Flow Derotation
  float diff_flow_x = (opticflow_state.phi - opticflow.prev_phi) * img->w / OPTICFLOW_FOV_W;
  float diff_flow_y = (opticflow_state.theta - opticflow.prev_theta) * img->h / OPTICFLOW_FOV_H;
  opticflow_result.flow_x = opticflow_result.flow_x - diff_flow_x * opticflow.subpixel_factor;
  opticflow_result.flow_y = opticflow_result.flow_y - diff_flow_y * opticflow.subpixel_factor;
  opticflow.prev_phi = opticflow_state.phi;
  opticflow.prev_theta = opticflow_state.theta;

  //Calculates the Focus of Expansion
  struct point_t foe = compute_FOE(img,vectors,opticflow_result.tracked_cnt, opticflow.subpixel_factor);

  //Computes TTC using distances from Focus of Expansion
  float *TTC = compute_TTC(vectors,opticflow_result.tracked_cnt, opticflow.subpixel_factor,foe);

  // Gets the minimum TTC obtained 
  float min = TTC[0];
  uint8_t ind_min = 0;
  for(int i=1;i<opticflow_result.tracked_cnt;i++){
     if(TTC[i]<min){
	ind_min = i;
	min = TTC[i];
     }
  }
  printf("\n min TTC: %f \n",min);

  //Searches for the area in the image where the obtacle was detected or ignore result if it is above the threshold
  if(min < TTC_THRESHOLD){

     uint16_t pos_x;
     pos_x = vectors[ind_min].pos.x/opticflow.subpixel_factor;

     if(pos_x < WIDTH/3)
	obstacle_region = 1;
     else if(pos_x < 2*WIDTH/3)
	obstacle_region = 2;
     else
	obstacle_region = 3;

   /*
     //Shows where the point is in the stream
     struct point_t tt_point;
     tt_point.x = pos_x;
     tt_point.y = vectors[ind_min].pos.y/opticflow.subpixel_factor;
     image_show_points(img, &tt_point,1);
*/
  }
  else
     obstacle_region = 0;
  
  DOWNLINK_SEND_COLORFILTER(DefaultChannel, DefaultDevice, &obstacle_region);

  return FALSE;
}

bool_t colorfilter_func(struct image_t* img);
bool_t colorfilter_func(struct image_t* img)
{
  // Color filter 
  obstacle_region = image_yuv422_colorfilter(img,img,
      color_lum_min_o ,color_lum_max_o ,
      color_cb_min_o ,color_cb_max_o ,
      color_cr_min_o ,color_cr_max_o ,
      color_lum_min_b ,color_lum_max_b ,
      color_cb_min_b ,color_cb_max_b ,
      color_cr_min_b ,color_cr_max_b ,
      tro, trb
      );
  DOWNLINK_SEND_COLORFILTER(DefaultChannel, DefaultDevice, &obstacle_region);
return FALSE;
}


void all_together_init(void)
{
  // Initialize optical flow parameters
  opticflow_calc_init(&opticflow, 320, 240);


  // Decides which strategy to run based on RUN_COLOR
  if(RUN_COLOR)
     cv_add(colorfilter_func);
  else
     cv_add(optical_flow_func);

}
