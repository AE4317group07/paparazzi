#include "modules/color_avoider_group7/color_avoider_group7.h"
#include "modules/computer_vision/all_together_group7.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include <time.h>
#include <stdlib.h>

int safeToGoForwards=0;
int32_t incrementForAvoidance;

void color_avoider_init() {
	// Initialise the variables of the colorfilter to accept orange and black
	color_lum_min_o=0;
	color_lum_max_o=131;
	color_cb_min_o=93;
	color_cb_max_o=255;
	color_cr_min_o=134;
	color_cr_max_o=255;

	color_lum_min_b=20;
	color_lum_max_b=39;
	color_cb_min_b=0;
	color_cb_max_b=213;
	color_cr_min_b=24;
	color_cr_max_b=174;
	// Initialise random values
	srand(time(NULL));
}
void color_avoider_periodic() {
	// Check the amount of color. If this is above a threshold
	// you want to change the drone heading
	safeToGoForwards = obstacle_region;
}

/**
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(int32_t *heading, int32_t increment)
{
  *heading = *heading + increment;
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(*heading); // HEADING HAS INT32_ANGLE_FRAC....
  return FALSE;
}
uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters){
	  struct EnuCoor_i new_coor;
	  struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	  // Calculate the sine and cosine of the heading the drone is keeping
	  float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));

	  // Now determine where to place the waypoint you want to go to
	  new_coor.x = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	  new_coor.y = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	  new_coor.z = pos->z; // Keep the height the same

	  // Set the waypoint to the calculated position
	  waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);

	  return FALSE;
}

