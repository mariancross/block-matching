/**
  * @class BlockMatching
  * @author Maria Santamaria
  * @date June 2012
  */

#ifndef TUPLE_H
#define TUPLE_H

#include <utility>  
#include "Block.h"
#include "Point2D.h"

class Tuple {
	private:
		Block block;
		Point2D mv;
		int searchPoints;
		
	protected:
	
	public:
		Tuple();
		Tuple(const Block& block, const Point2D& mv, int searchPoints);
		~Tuple();
		
		void setBlock(const Block& block);
		void setMotionVector(const Point2D& mv);
		void setSearchPoints(int searchPoints);
		void set(const Block& block, const Point2D& mv, int searchPoints);
		
		Block getBlock() const;
		Point2D getMotionVector() const;
		int getSearchPoints() const;
};

#endif

