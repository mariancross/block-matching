#include "Node.h"

Node::Node() {
	edges = 0;
	edgesParent = 0;
}

Node::Node(const Block& block, int edges) {
	this->block = block;
	this->edges = edges;
	this->edgesParent = 0;
}

Node::~Node() {

}

void Node::set(const Block& block, int edges) {
	this->block = block;
	this->edges = edges;
}

void Node::setBlock(const Block& block) {
	this->block = block;
}

void Node::setEdges(int edges) {
	this->edges = edges;
}

void Node::setEdgesParent(int edgesParent) {
	this->edgesParent = edgesParent;
}

Block Node::getBlock() {
	return block;
}

int Node::getEdges() {
	return edges;
}

int Node::getEdgesParent() {
	return edgesParent;
}
