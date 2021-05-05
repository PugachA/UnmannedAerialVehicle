#include "Nav.h"
Nav::Nav() //constructor
{

}
float Nav::getDistanceToActiveWp()
{
	float difX = active_wp.getWpXCoord()-plane_position.getWpXCoord();
	float difY = active_wp.getWpYCoord()-plane_position.getWpYCoord();

	return sqrt( difX*difX + difY*difY );
}
//переменная текущего курса есть в классе: this->plane_course
//координаты самолета брать отсюда this->plane_position.get...
//координаты точки отсюда active_wp.getWpXCoord() active_wp.getWpYCoord()
float Nav::getCourseToWp()
{

}
void Nav::updateXYcoordForWp()
{
	float difLat = active_wp.getWpLat()-plane_position.getWpLat();
	float difLon = active_wp.getWpLon()-plane_position.getWpLon();

	active_wp.setWpXCoord( difLon*DEG2RAD*LON2X );
	active_wp.setWpYCoord( difLat*DEG2RAD*LAT2Y );
}
void Nav::updateActiveWp(Wp &wp)
{
	if(this->active_wp.getWpLat() != wp.getWpLat() && this->active_wp.getWpLon() != wp.getWpLon() )
		this->active_wp = wp;
	updateXYcoordForWp();
}
float Nav::getOmegaTurnToWp()
{
	return 0.0;
}
void Nav::updatePlanePos(double lat, double lon, double alt, double course)
{
	this->plane_position.setWpCoord(lat, lon, alt);
	this->plane_course = course;
}
