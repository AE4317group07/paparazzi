#ifndef COLOR_AVOIDER_H
#define COLOR_AVOIDER_H
#include <inttypes.h>

int safeToGoForwards;
extern int32_t incrementForAvoidance;
extern void color_avoider_init(void);
extern void color_avoider_periodic(void);
extern uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters);
extern uint8_t increase_nav_heading(int32_t *heading, int32_t increment);
extern uint8_t chooseRandomIncrementAvoidance(void);

#endif
