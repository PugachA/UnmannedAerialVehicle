#include "Nav.h"
Nav::Nav() //constructor
{
	k_omega_turn = 0.3;
}
float Nav::getDistanceToActiveWp()
{
	float difX = active_wp.getWpXCoord()-plane_position.getWpXCoord();
	float difY = active_wp.getWpYCoord()-plane_position.getWpYCoord();

	return sqrt( difX*difX + difY*difY );
}
//переменная текущего аутевого угла есть в классе: this->plane_track
//координаты самолета брать отсюда this->plane_position.get...
//координаты точки отсюда active_wp.getWpXCoord() active_wp.getWpYCoord()
float Nav::getDeltaPsiToWp()
{
	double speed_x = 0.0;
	double speed_y = 0.0;
	double vect_prod = 0.0;
	double scalar_prod = 0.0;
	double abs_dist_to_wp = 0.0;
	double d_psi = 0.0;

	// координаты вектора текущей скорости в ортодромической СК
	if (this->plane_track <= PI/2)
	{
		speed_x = this->plane_abs_speed * sin(this->plane_track);
		speed_y = this->plane_abs_speed * cos(this->plane_track);
	}
	else if ((this->plane_track > PI/2) && (this->plane_track <= PI))
	{
		speed_x = this->plane_abs_speed * sin(PI - this->plane_track);
		speed_y = -this->plane_abs_speed * cos(PI - this->plane_track);
	}
	else if ((this->plane_track > PI) && (this->plane_track <= 3*PI/2))
	{
		speed_x = -this->plane_abs_speed * sin(this->plane_track - PI);
		speed_y = -this->plane_abs_speed * cos(this->plane_track - PI);
	}
	else if ((this->plane_track > 3*PI/2) && (this->plane_track <= 2*PI))
	{
		speed_x = -this->plane_abs_speed * sin(2*PI - this->plane_track);
		speed_y = this->plane_abs_speed * cos(2*PI - this->plane_track);
	}

	vect_prod = speed_x * active_wp.getWpYCoord() - speed_y * active_wp.getWpXCoord(); //вертикальная компонента векторного произведения вектора на цель и скорости
	scalar_prod = speed_x * active_wp.getWpXCoord() + speed_y * active_wp.getWpYCoord(); // скалярное произведение векторов в плоскости местного горизонта

	abs_dist_to_wp = getDistanceToActiveWp();

	//расчет угла на цель через модуль векторного произведения, для однозначного определения координатной четверти исп sin и cos (векторное и скалярное пр)
	if ((vect_prod >= 0) && (scalar_prod >= 0))
	{
		d_psi = asin(vect_prod / (this->plane_abs_speed * abs_dist_to_wp));
	}
	else
		if ((vect_prod > 0) && (scalar_prod < 0))
		{
			d_psi = PI - asin(vect_prod / (this->plane_abs_speed * abs_dist_to_wp));
		}
		else
			if ((vect_prod < 0) && (scalar_prod < 0))
			{
				d_psi = -PI - asin(vect_prod / (this->plane_abs_speed * abs_dist_to_wp));
			}
			else
				if ((vect_prod < 0) && (scalar_prod > 0))
				{
					d_psi = asin(vect_prod / (this->plane_abs_speed * abs_dist_to_wp));
				}

	return (float)(d_psi*RAD2DEG); //функция возвращает разницу между текущим путевым углом и направлением на цель, в градусах от -180 до +180;
}
void Nav::updateXYcoordForWp()
{
	//Перевод в ортодромические координаты, инициализация СК в точке, где находится самолет

	float difLat = active_wp.getWpLat()-plane_position.getWpLat();
	float difLon = active_wp.getWpLon()-plane_position.getWpLon();

	active_wp.setWpXCoord( difLon*DEG2RAD*LON2X );
	active_wp.setWpYCoord( difLat*DEG2RAD*LAT2Y );
}
void Nav::updateActiveWp(Wp wp)
{
	if(this->active_wp != wp)
		this->active_wp = wp;
	updateXYcoordForWp();
}
float Nav::getOmegaTurnToWp()
{
	float d_psi = getDeltaPsiToWp();

	return (k_omega_turn * d_psi); //deg/s
}
void Nav::updatePlanePos(double lat, double lon, double alt, double track, double speed)
{
	this->plane_position.setWpCoord(lat, lon, alt);
	this->plane_track = track*DEG2RAD;
	this->plane_abs_speed = speed;
}

float Nav::getActiveWpAlt()
{
	return this->active_wp.getWpAlt();
}
Wp Nav::getActiveWp()
{
	return this->active_wp;
}
