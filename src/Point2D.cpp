#include "Point2D.h"

Point2D::Point2D(int x, int y) {
	this->x = x;
	this->y = y;
}

Point2D::~Point2D(){ 
	
}

int Point2D::getX() const {
	return x;
}

int Point2D::getY() const {
	return y;
}

void Point2D::setX(int x) {
	this->x = x;
}

void Point2D::setY(int y) {
	this->y = y;
}

void Point2D::set(int x, int y) {
	this->x = x;
	this->y = y;
}

bool Point2D::operator<(const Point2D& other) const {
	if(x < other.getX()) return true;
	if(x > other.getY()) return false;

	if(y < other.getY()) return true;
	if(y > other.getY()) return false;

	return false;
}

