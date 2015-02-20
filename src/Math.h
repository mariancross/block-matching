/**
  * @class Math
  * @author Maria Santamaria
  * @date January 2014
  */
  
#ifndef MATH_H
#define MATH_H

#include <vector>

class Math {
	private:	
	
	public:
		static const double PI;
		
	protected:
	
	public:
		Math();
		~Math();
		
		static double square(double value);
		
		static double radianToDegree(double radian);
		
		static int discretiseDegree(double degree);
};

#endif
