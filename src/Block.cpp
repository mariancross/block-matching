#include "Block.h"
using namespace std;
using namespace cv;

Block::Block() {
	size = 0;
}

Block::Block(const Point2D& position, int size, const SearchWindow& window) {
	this->position = position;
	this->size = size;
	this->window = window;
}

Block::~Block() {
	
}

void Block::setPosition(const Point2D& position) {
	this->position = position;
}

void Block::setSize(int size) {
	this->size = size;
}

void Block::setWindow(const SearchWindow& window) {
	this->window = window;
}

void Block::set(const Point2D& position, int size, const SearchWindow& window) {
	this->position = position;
	this->size = size;
	this->window = window;
}

Point2D Block::getPosition() const {
	return position;
}

int Block::getSize() const {
	return size;
}

SearchWindow Block::getWindow() const {
	return window;
}

