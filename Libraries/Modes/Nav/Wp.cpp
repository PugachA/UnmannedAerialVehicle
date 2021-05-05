#include "Wp.h"

Wp::Wp(double lat, double lon, double alt) //constructor
{
  	this->wp_lat = lat;
  	this->wp_lon = lon;
  	this->wp_alt = alt;
}
void Wp::setWpCoord(double lat, double lon, double alt) //constructor
{
  	this->wp_lat = lat;
  	this->wp_lon = lon;
  	this->wp_alt = alt;
}
void Wp::setWpXCoord(double X)
{
	this->wp_x = X;
}
double Wp::getWpXCoord()
{
	return this->wp_x;
}
void Wp::setWpYCoord(double Y)
{
	this->wp_y = Y;
}
double Wp::getWpYCoord()
{
	return this->wp_y;
}
void Wp::setWpAsHome()
{

}
void Wp::setWpAsLast()
{

}
double Wp::getWpLat()
{
	return this->wp_lat;
}
double Wp::getWpLon()
{
	return this->wp_lon;
}
