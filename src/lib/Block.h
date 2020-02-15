/**
  * @class Block
  * @author Maria Santamaria
  * @date January 2014
  */

#ifndef BLOCK_H
#define BLOCK_H

#include <utility>
#include <opencv2/core/core.hpp>
#include "Point2D.h"
#include "SearchWindow.h"

class Block {
	private:
		Point2D position;
		int size;
		SearchWindow window;
		
	protected:
	
	public:
		Block();
		Block(const Point2D& position, int size, const SearchWindow& window);
		~Block();
		
		void setPosition(const Point2D& position);
		void setSize(int size);
		void setWindow(const SearchWindow& window);
		void set(const Point2D& position, int size, const SearchWindow& window);
		
		Point2D getPosition() const;
		int getSize() const;
		SearchWindow getWindow() const;
};

#endif

