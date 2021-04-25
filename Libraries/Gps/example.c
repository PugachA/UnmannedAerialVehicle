#include "Gps/minmea.h"


char nmea_line[80] = {0,};
minmea_sentence_rmc frame;
float latitude = 0;
float longitude = 0;
float speed = 0;

while(1)
{
	sprintf(nmea_line, "$GNRMC,190535.00,A,5546.28108,N,03740.64391,E,0.077,,040319,,,A*62");
	//sprintf(nmea_line, "$GNRMC,190527.80,V,,,,,,,040319,,,N*6C");

	if (minmea_parse_rmc(&frame, nmea_line))
	{
		if(&frame.valid)
		{
			latitude = minmea_tocoord(&frame.latitude);
			longitude = minmea_tocoord(&frame.longitude);
			speed = minmea_tofloat(&frame.speed);
		}
	}
}
