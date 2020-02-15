#include "Tuple.h"
using namespace std;

Tuple::Tuple() {
	searchPoints = 0;
}

Tuple::Tuple(const Block& block, const Point2D& mv, int searchPoints) {
	this->block = block;
	this->mv = mv;
	this->searchPoints = searchPoints;
}

Tuple::~Tuple() {
	
}

void Tuple::setBlock(const Block& block) {
	this->block = block;
}

void Tuple::setMotionVector(const Point2D& mv) {
	this->mv = mv;
}

void Tuple::setSearchPoints(int searchPoints) {
	this->searchPoints = searchPoints;
}

void Tuple::set(const Block& block, const Point2D& mv, int searchPoints) {
	this->block = block;
	this->mv = mv;
	this->searchPoints = searchPoints;
}
		
Block Tuple::getBlock() const {
	return block;
}

Point2D Tuple::getMotionVector() const {
	return mv;
}

int Tuple::getSearchPoints() const {
	return searchPoints;
}

