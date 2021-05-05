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
float Nav::getCourseToWp()//переменная текущего курса есть в классе
{

}
void Nav::calcXYcoordForWp()
{
	active_wp.setWpXCoord(active_wp.getWpLon()*0.05); //хз как будет делаться перевод в линейные написал заглушку
	active_wp.setWpYCoord(active_wp.getWpLat()*0.05); //хз как будет делаться перевод в линейные написал заглушку
}
void Nav::updateActiveWp(Wp &wp)
{
	this->active_wp = wp;
}
float Nav::getOmegaTurnToWp()
{

}
void Nav::updatePlanePos(double lat, double lon, double alt, double course)
{
	this->plane_position.setWpCoord(lat, lon, alt);
	this->plane_course = course;
}
