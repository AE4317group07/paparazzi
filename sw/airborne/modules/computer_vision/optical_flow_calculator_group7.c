/******************************************************************************************************
*
* Authors: Ricardo Almeida, Pedro Dias, Diogo Martins, Afonso Mendes
* Date: 16/04/2016
* Contains the definitions of auxiliary functions used in the all_together.c file
*
******************************************************************************************************/

#include "std.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Own Header
#include "optical_flow_calculator_group7.h"

// Computer Vision
#include "lib/vision/image_group7.h"
#include "lib/vision/lucas_kanade.h"
#include "lib/vision/fast_rosten.h"

//Set the default values used in FAST corner detection and Lucas-Kannade optical flow function
#ifndef OPTICFLOW_MAX_TRACK_CORNERS
#define OPTICFLOW_MAX_TRACK_CORNERS 15
#endif
PRINT_CONFIG_VAR(OPTICFLOW_MAX_TRACK_CORNERS)

#ifndef OPTICFLOW_WINDOW_SIZE
#define OPTICFLOW_WINDOW_SIZE 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_WINDOW_SIZE)

#ifndef OPTICFLOW_SUBPIXEL_FACTOR
#define OPTICFLOW_SUBPIXEL_FACTOR 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_SUBPIXEL_FACTOR)

#ifndef OPTICFLOW_MAX_ITERATIONS
#define OPTICFLOW_MAX_ITERATIONS 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_MAX_ITERATIONS)

#ifndef OPTICFLOW_THRESHOLD_VEC
#define OPTICFLOW_THRESHOLD_VEC 2
#endif
PRINT_CONFIG_VAR(OPTICFLOW_THRESHOLD_VEC)

#ifndef OPTICFLOW_FAST9_ADAPTIVE
#define OPTICFLOW_FAST9_ADAPTIVE TRUE
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_ADAPTIVE)

#ifndef OPTICFLOW_FAST9_THRESHOLD
#define OPTICFLOW_FAST9_THRESHOLD 30
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_THRESHOLD)

#ifndef OPTICFLOW_FAST9_MIN_DISTANCE
#define OPTICFLOW_FAST9_MIN_DISTANCE 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_MIN_DISTANCE)

/* Functions only used here */

static int cmp_pos_x(const void *a, const void *b);
static int cmp_pos_y(const void *a, const void *b);


/**
 * Initialize the opticflow calculator
 * @param[out] *opticflow The new optical flow calculator
 * @param[in] *w The image width
 * @param[in] *h The image height
 */
void opticflow_calc_init(struct opticflow_t *opticflow, uint16_t w, uint16_t h)
{

  /* Create the image buffers */
  image_create(&opticflow->img_gray, w, h, IMAGE_GRAYSCALE);
  image_create(&opticflow->prev_img_gray, w, h, IMAGE_GRAYSCALE);


  /* Set the previous values */
  opticflow->got_first_img = FALSE;
  opticflow->prev_phi = 0.0;
  opticflow->prev_theta = 0.0;

  /* Set the default values */
  opticflow->max_track_corners = OPTICFLOW_MAX_TRACK_CORNERS;
  opticflow->window_size = OPTICFLOW_WINDOW_SIZE;
  opticflow->subpixel_factor = OPTICFLOW_SUBPIXEL_FACTOR;
  opticflow->max_iterations = OPTICFLOW_MAX_ITERATIONS;
  opticflow->threshold_vec = OPTICFLOW_THRESHOLD_VEC;

  opticflow->fast9_adaptive = OPTICFLOW_FAST9_ADAPTIVE;
  opticflow->fast9_threshold = OPTICFLOW_FAST9_THRESHOLD;
  opticflow->fast9_min_distance = OPTICFLOW_FAST9_MIN_DISTANCE;
}

/**
 * Computes the FOE (Focus of expansion)
 * @param[out] *point_t Focus of expansion position
 * @param[in] *image Last taken image
 * @param[in] *vectors Optic flow vectors
 * @param[in] *tracked_cnt Number of tracked corners
 * @param[in] *subpixel_factor Used factor of subpixels
 */
struct point_t compute_FOE(struct image_t* img,struct flow_t *vectors,uint16_t tracked_cnt, uint8_t subpixel_factor)
{
   // Variables initialization
   static struct point_t foe; 
   foe.x = 135;
   foe.y = 135;
   int i =0;


   // x coordinate of FoE
   // sort optical flow horizontal values 
   qsort(vectors, tracked_cnt, sizeof(struct flow_t), cmp_pos_x);

   // Finds where the vectors change to the opposite direction (estimate of FOE x coordinate)
   int16_t pos_before = vectors[0].pos.x;
   for(i=1;i<tracked_cnt;i++){
      if(vectors[i].flow_x > 0){
         foe.x = (pos_before + vectors[i].pos.x)/2/subpixel_factor;
         break;
      }
      else
      { 
         pos_before = vectors[i].pos.x;
      }
   }


   // y coordinate of FoE
   // sort optical flow vertical values 
   qsort(vectors, tracked_cnt, sizeof(struct flow_t), cmp_pos_y);
   

    // Finds where the vectors change to the opposite direction (estimate of FOE y coordinate)
   pos_before = vectors[0].pos.y;
   for(i=1;i<tracked_cnt;i++){
      if(vectors[i].flow_y > 0){
         foe.y = (pos_before + vectors[i].pos.y)/2/subpixel_factor;
         break;
      }
      else
      {
        pos_before = vectors[i].pos.y;
      }
   }

   return foe;

}


/**
 * Computes the TTC (Time to contact)
 * @param[out] *TTC
 * @param[in] *vectors optical flow vectors
 * @param[in] *tracked_cnt Number of tracked corners
 * @param[in] *subpixel_factor Used factor of subpixels
 * @param[in] *foe Position of FOE
 */
float *compute_TTC(struct flow_t *vectors,uint16_t tracked_cnt, uint8_t subpixel_factor, struct point_t foe)
{

   // Initialization of internally used variables
   float TTC[tracked_cnt];
   int x2_x[tracked_cnt];
   int x2_y[tracked_cnt];
   int x1_x[tracked_cnt];
   int x1_y[tracked_cnt];
   int x1_dist, flow_dist;
  
   // Loop performing the calculation of TTC for all the optical flow vector positions
   for(int i=0;i<tracked_cnt;i++){
      
      x2_x[i] = vectors[i].pos.x/subpixel_factor - foe.x;
      x2_y[i] = vectors[i].pos.y/subpixel_factor - foe.y;

      x1_x[i] = x2_x[i] - vectors[i].flow_x/subpixel_factor;
      x1_y[i] = x2_y[i] - vectors[i].flow_y/subpixel_factor;

      x1_dist = sqrt(x1_x[i]*x1_x[i] + x1_y[i]*x1_y[i]);
      flow_dist = sqrt(vectors[i].flow_x/subpixel_factor*vectors[i].flow_x/subpixel_factor + vectors[i].flow_y/subpixel_factor*vectors[i].flow_y/subpixel_factor);
      TTC[i] = x1_dist/(flow_dist*3.5);   // 3.5 = FPS
   }

   return TTC;

}

/**
 * Compares the x postion of two points
 * @param[out] *Returns the boolean value of the comparison
 * @param[in] *structure containing the value to compare
 * @param[in] *structure containing the value to compare
 */
static int cmp_pos_x(const void *a, const void *b)
{
  const struct flow_t *a_p = (const struct flow_t *)a;
  const struct flow_t *b_p = (const struct flow_t *)b;
  return (a_p->pos.x  - b_p->pos.x);
}

/**
 * Compares the y postion of two points
 * @param[out] *Returns the boolean value of the comparison
 * @param[in] *structure containing the value to compare
 * @param[in] *structure containing the value to compare
 */
static int cmp_pos_y(const void *a, const void *b)
{
  const struct flow_t *a_p = (const struct flow_t *)a;
  const struct flow_t *b_p = (const struct flow_t *)b;
  return (a_p->pos.y  - b_p->pos.y);
}
