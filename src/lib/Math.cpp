#include "Math.h"

const double Math::PI = 3.14159265;

Math::Math() {

}

Math::~Math() {
	
}

double Math::square(double value) {
	return value*value;
}

double Math::radianToDegree(double radian) {
	double degree = radian * 180.0 / PI;

	while(degree < 0)
		degree += 360.0;
	
	while(degree > 360)
		degree -= 360;

	return degree;
}

int Math::discretiseDegree(double degree) {
	double value = 0.0;
	
	if(degree >= 22.5 && degree < 67.5)
		return 45;
	
	if(degree >= 67.5 && degree < 112.5)
		return 90;
	
	if(degree >= 112.5 && degree < 157.5)
		return 135;
		
	if(degree >= 157.5 && degree < 205.5)
		return 180;
	
	if(degree >= 205.5 && degree < 247.5)
		return 225;
	
	if(degree >= 247.5 && degree < 292.5)
		return 270;
	
	if(degree >= 292.5 && degree < 337.5)
		return 315;
	
	return 0;
	
	return value;
}
