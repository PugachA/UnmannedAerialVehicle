#ifndef _NAV_H_
#define _NAV_H_

#include "stm32f4xx_hal.h"
#include "math.h"
#include "Wp.h"
#include "../../Gps/Gps.h"

//#include "cmsis_os.h"

class Nav
{
  private:

	Wp active_wp;
	Wp plane_position;
	double plane_course;

	float getWpDistance();
	float getWpCourse();
	void calcXYcoordForWp();
	
  public:
    Nav(); //constructor
    float getOmegaTurnToWp();
    void updateActiveWp(Wp &wp);
    void updatePlanePos(double lat, double lon, double alt, double course);
			
};
#endif
