/**
 * @file modules/computer_vision/lib/vision/image_group7.c
 */
#include "subsystems/datalink/telemetry.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/lib/vision/image_group7.h"


#include <stdio.h>
#include <std.h>
#include "image.h"
#include <stdlib.h>
#include <string.h>

/**
 * Filter colors in an YUV422 image
 * @param[in] *input The input image to filter
 * @param[out] *output The filtered output image
 * @param[in] y_m The Y minimum value
 * @param[in] y_M The Y maximum value
 * @param[in] u_m The U minimum value
 * @param[in] u_M The U maximum value
 * @param[in] v_m The V minimum value
 * @param[in] v_M The V maximum value
 * @return The amount of filtered pixels
 */
int16_t  image_yuv422_colorfilter(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M, 
 uint8_t y_m2, uint8_t y_M2, uint8_t u_m2, uint8_t u_M2, uint8_t v_m2, uint8_t v_M2, int thres, int thrb)
{
  uint16_t cnt = 0;
  uint16_t pix_r = 0;
  uint16_t pix_c = 0;
  uint16_t pix_l = 0;
  uint16_t pix_r2 = 0;
  uint16_t pix_c2 = 0;
  uint16_t pix_l2 = 0;
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf; 
  float uter =0;
  float dter =0;

  // Copy the creation timestamp (stays the same)
  memcpy(&output->ts, &input->ts, sizeof(struct timeval));

  uter = output->w/3;
  dter = output->w*2/3;

  float w1 = output->w/8;
  float w4 = output->w*7/8;
  float h1 = output->h/4;
  float h4 = output->h*3/4;
  bool isOrange= FALSE;

  // Go trough all the pixels

 for (uint16_t y = 0; y < output->h; y++) {
	for (uint16_t x = 0; x < output->w; x += 2) {
	

                // Check if the color is inside the specified values
		      if ((dest[1] >= y_m) && (dest[1] <= y_M)  && (dest[0] >= u_m) && (dest[0] <= u_M) && (dest[2] >= v_m) && (dest[2] <= v_M)) {
		
			// UYVY
			dest[0] = 64;        // U
			dest[1] = source[1];  // Y
			dest[2] = 255;        // V
			dest[3] = source[3];  // Y
	
		       	if(x<uter) pix_l++;
		   
		       	else if(x>uter && x<dter) pix_c++;

			else pix_r++;
			
			isOrange= TRUE;

		      }




		if((x>w1) && (x<w4) && (y>h1) && (y <h4) && isOrange == FALSE){
		    
		      // Check if the color is inside the specified values
		      if ((dest[1] >= y_m2) && (dest[1] <= y_M2)  && (dest[0] >= u_m2) && (dest[0] <= u_M2) && (dest[2] >= v_m2) && (dest[2] <= v_M2)) {
		
			// UYVY
			dest[0] = 167;        // U
			dest[1] = 177;  // Y
			dest[2] = 1;        // V
			dest[3] = 177;  // Y
	
		       	if(x<uter) pix_l2++;
		   
		       	else if(x>uter && x<dter) pix_c2++;

			else pix_r2++;

		      
		      }

		}
             // Go to the next 2 pixels
	      dest += 4;
	      source += 4;
              isOrange= FALSE;
	}
}
      

      if(pix_l > thres || pix_c > thres || pix_r>thres){
  	if(pix_l > pix_c && pix_l> pix_r) cnt = 1;
  	if(pix_r > pix_l && pix_r> pix_c) cnt = 3;
  	if(pix_c > pix_l && pix_c> pix_r) cnt = 2;
	return cnt;
	
      }
      else if(pix_l2 > thrb|| pix_c2 > thrb || pix_r2>thrb){
  	if(pix_l2 > pix_c2 && pix_l2> pix_r2) cnt = 1;
  	if(pix_r2 > pix_l2 && pix_r2> pix_c2) cnt = 3;
  	if(pix_c2 > pix_l2 && pix_c2> pix_r2) cnt = 2;
      }
      
      
      else return 0;


  return cnt;
}
