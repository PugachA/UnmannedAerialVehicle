#include "Nav.h"

Nav::Nav() //constructor
{

}
float Nav::getWpDistance()
{

}
float Nav::getWpCourse()
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
float Nav::getOmegaTurnToWp(double current_course)
{

}
