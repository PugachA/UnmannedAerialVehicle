#ifndef _NAV_H_
#define _NAV_H_

#include "stm32f4xx_hal.h"
#include "math.h"
#include "Wp.h"

//#include "cmsis_os.h"

class Nav
{
  private:

	const double DEG2RAD = 0.0174533;
	const double LAT2Y = 6363535;
	const double LON2X = 3580000;

	Wp active_wp;
	Wp plane_position;
	double plane_track;
	double plane_abs_speed;

	float getCourseToWp();
	void updateXYcoordForWp();
	
  public:
	Nav(void);
    float getDistanceToActiveWp();
    float getOmegaTurnToWp();
    void updateActiveWp(Wp wp);
    void updatePlanePos(double lat, double lon, double alt, double course, double speed);
			
};
#endif
