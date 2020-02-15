/**
  * @class SearchWindow
  * @author Maria Santamaria
  * @date January 2014
  */

#ifndef SEARCH_WINDOW_H
#define SEARCH_WINDOW_H

#include <utility>
#include "BDM.h"
#include "Point2D.h"

class SearchWindow {
	private:
		Point2D start;
		Point2D end;
		
	protected:
	
	public:
		SearchWindow();
		SearchWindow(const Point2D& start, const Point2D& end);
		~SearchWindow();
		
		void set(const Point2D& start, const Point2D& end);
		void setStart(const Point2D& start);
		void setEnd(const Point2D& end);
		
		Point2D getStart();
		Point2D getEnd();
};

#endif
