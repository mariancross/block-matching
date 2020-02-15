/**
  * @class Node
  * @author Maria Santamaria
  * @date January 2013
  */

#ifndef NODE_H
#define NODE_H

#include "Block.h"

class Node {
	private:
		Block block;
		int edges;
		int edgesParent;
		
	protected:
	
	public:
		Node();
		Node(const Block& block, int edges);
		~Node();
		
		void set(const Block& block, int edges);
		void setBlock(const Block& block);
		void setEdges(int edges);
		void setEdgesParent(int edgesParent);
				
		Block getBlock();
		int getEdges();
		int getEdgesParent();
};

#endif

