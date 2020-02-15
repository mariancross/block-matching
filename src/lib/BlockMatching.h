/**
  * @class BlockMatching
  * @author Maria Santamaria
  * @date January 2014
  */

#ifndef BLOCK_MATCHING_H
#define BLOCK_MATCHING_H

#include <cfloat>
#include <iostream>
#include <map>
#include <queue>
#include <utility>
#include <vector>
#include <opencv2/core/core.hpp>  
#include "BDM.h"
#include "Block.h"
#include "Edges.h"
#include "Math.h"
#include "Node.h"
#include "Point2D.h"
#include "Tuple.h"

class BlockMatching {
	private:
		int blockID;

		double T;
		double threshold;

		std::vector<Tuple> output;	
		
		std::vector< std::vector <int> > controlMatrix;

		std::vector<int> dxSquare;
		std::vector<int> dySquare;

		std::vector<int> dxCross;
		std::vector<int> dyCross;

		std::vector<int> dxDiamond;
		std::vector<int> dyDiamond;

		std::vector<int> dxHexagonal;
		std::vector<int> dyHexagonal;

		cv::Mat gradientDir;

		Block buildBlock(int x, int y, int displacement, int blockSize, int width, int height);
		Node buildNode(int x, int y, int displacement, int blockSize, int width, int height, const cv::Mat& edgeMap);

		// Count edges within a block
		double countEdges(const Block& block, const cv::Mat& edgeMap);

		// Fixed block size
		void fixedBsSearch(const cv::Mat& currentFrame, const cv::Mat& referenceFrame, int displacement, int blockSize, Tuple(BlockMatching::*algorithm)(const Block&, const cv::Mat&, const cv::Mat&));

		// Variable block size
		void variableBsSearch(const cv::Mat& currentFrame, const cv::Mat& referenceFrame, int displacement, int blockSize, Tuple(BlockMatching::*algorithm)(const Block&, const cv::Mat&, const cv::Mat&));

		// Breathe-first search
		void bfs(const cv::Mat& currentFrame, const cv::Mat& referenceFrame, const cv::Mat& edgeMap, int x, int y, int displacement, int blockSize, Tuple(BlockMatching::*algorithm)(const Block&, const cv::Mat&, const cv::Mat&));

	protected:

	public:
		static const int VARIABLE_BLOCK = 0, NO_VARIABLE_BLOCK = 1;

		BlockMatching();
		BlockMatching(int width, int height);

		~BlockMatching();

		void initialiseVars();
		void setGradientDir(const cv::Mat& currentFrame);

		// Full-search algorithm
		Tuple fs(const Block& block, const cv::Mat& currentFrame, const cv::Mat& referenceFrame);

		// Three-step search
		Tuple tss(const Block& block, const cv::Mat& currentFrame, const cv::Mat& referenceFrame);

		// Four-step search
		Tuple fss(const Block& block, const cv::Mat& currentFrame, const cv::Mat& referenceFrame);

		// Diamond search
		Tuple ds(const Block& block, const cv::Mat& currentFrame, const cv::Mat& referenceFrame);

		// Hexagonal based search
		Tuple hexbs(const Block& block, const cv::Mat& currentFrame, const cv::Mat& referenceFrame);

		// Multi-directional gradient descent search
		Tuple mdgds(const Block& block, const cv::Mat& currentFrame, const cv::Mat& referenceFrame);

		// Fast directional gradient descent search
		Tuple fdgds(const Block& block, const cv::Mat& currentFrame, const cv::Mat& referenceFrame);

		// Adaptive rood pattern search
		Tuple arps(const Block& block, const cv::Mat& currentFrame, const cv::Mat& referenceFrame);

		// Tunnelling-based search
		Tuple tbs(const Block& block, const cv::Mat& currentFrame, const cv::Mat& referenceFrame);

		// Estimate motion for a frame
		std::vector<Tuple> estimateMotion(const cv::Mat& currentFrame, const cv::Mat& referenceFrame, int displacement, int blockSize, int blockType, Tuple(BlockMatching::*algorithm)(const Block&, const cv::Mat&, const cv::Mat&));

		// Motion compensation
		cv::Mat compensation(const cv::Mat& referenceFrame, const std::vector<Tuple>& tuples);		
		
		// Draw block splitting
		cv::Mat drawBlocks(const cv::Mat& image, const std::vector<Tuple>& tuples);

		void setThreshold(int threshold);

};

#endif
