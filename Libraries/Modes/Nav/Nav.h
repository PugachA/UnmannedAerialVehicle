#ifndef _NAV_H_
#define _NAV_H_

#include "stm32f4xx_hal.h"
#include "math.h"
#include "Wp.h"

//#include "cmsis_os.h"

class Nav
{
  private:

	Wp active_wp;

	float getWpDistance();
	float getWpCourse();
	void calcWpXYcoord();
	


  public:
    Nav(); //constructor
    float getOmegaTurnToWp(double current_course);
    void updateActiveWp(Wp &wp);
			
};
#endif
