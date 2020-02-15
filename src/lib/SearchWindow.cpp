#include "SearchWindow.h"
using namespace std;

SearchWindow::SearchWindow() {

}

SearchWindow::SearchWindow(const Point2D& start, const Point2D& end) {
	this->start = start;
	this->end = end;
}

SearchWindow::~SearchWindow() {
	
}

void SearchWindow::set(const Point2D& start, const Point2D& end) {
	this->start = start;
	this->end = end;
}

void SearchWindow::setStart(const Point2D& start) {
	this->start = start;
}

void SearchWindow::setEnd(const Point2D& end) {
	this->end = end;
}

Point2D SearchWindow::getStart() {
	return start;
}

Point2D SearchWindow::getEnd() {
	return end;
}
