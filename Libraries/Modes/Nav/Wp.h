#ifndef _WP_H_
#define _WP_H_

#include "stm32f4xx_hal.h"
#include "math.h"
//#include "cmsis_os.h"

class Wp
{
  private:
	double wp_lat; //deg
	double wp_lon; //deg
	double wp_alt; //meters

	double wp_x = 0; //meters
	double wp_y = 0; //meters

	uint8_t wp_is_active = 0;
	uint8_t wp_is_home = 0;
	uint8_t wp_is_last = 0;
	
  public:
	Wp(void);
    Wp(double lat, double lon, double alt); //constructor

    void setWpCoord(double lat, double lon, double alt);
    double getWpLat();
    double getWpLon();
    double getWpAlt();

    void setWpXCoord(double X);
    double getWpXCoord();
    void setWpYCoord(double Y);
    double getWpYCoord();

    void setWpAsHome();
    void setWpAsLast();
    uint8_t isLast();
    uint8_t isHome();
    bool operator != (Wp wp1);
};
#endif
