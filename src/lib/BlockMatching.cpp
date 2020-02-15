#include "BlockMatching.h"
using namespace std;

BlockMatching::BlockMatching() {
	blockID = -1;
	threshold = 0.5;
	T = 0.00781;
}

BlockMatching::BlockMatching(int width, int height) {
	blockID = -1;
	threshold = 0.5;
	T = 0.00781;

	int squareIdx[] = {0, 0, 1, 1, 1, 0, -1, -1, -1};
	int squareIdy[] = {0, -1, -1, 0, 1, 1, 1, 0, -1};	

	dxSquare = vector<int>(squareIdx, squareIdx + sizeof(squareIdx) / sizeof(int));
	dySquare = vector<int>(squareIdy, squareIdy + sizeof(squareIdy) / sizeof(int));

	int crossIdx[] = {0, 0, 1, 0, -1};
	int crossIdy[] = {0, -1, 0, 1, 0};

	dxCross = vector<int> (crossIdx, crossIdx + sizeof(crossIdx) / sizeof(int));
	dyCross = vector<int> (crossIdy, crossIdy + sizeof(crossIdy) / sizeof(int));

	int diamondIdx[] = {0, 0, 1, 2, 1, 0, -1, -2, -1};
	int diamondIdy[] = {0, -2, -1, 0, 1, 2, 1, 0, -1};

	dxDiamond = vector<int> (diamondIdx, diamondIdx + sizeof(diamondIdx) / sizeof(int));
	dyDiamond = vector<int> (diamondIdy, diamondIdy + sizeof(diamondIdy) / sizeof(int));

	int hexagonalIdx[] = {0, 1, 2, 1, -1, -2, -1};
	int hexagonalIdy[] = {0, -2, 0, 2, 2, 0, -2};
	
	dxHexagonal = vector<int> (hexagonalIdx, hexagonalIdx + sizeof(hexagonalIdx) / sizeof(int));
	dyHexagonal = vector<int> (hexagonalIdy, hexagonalIdy + sizeof(hexagonalIdy) / sizeof(int));

	controlMatrix = vector< vector<int> >(height, vector<int>(width, -1));
}

BlockMatching::~BlockMatching() {

}

void BlockMatching::initialiseVars() {
	blockID = -1;

	for(unsigned int i = 0 ; i < controlMatrix.size() ; i++)
		fill(controlMatrix[i].begin(), controlMatrix[i].end(), -1);
}

void BlockMatching::setGradientDir(const cv::Mat& currentFrame) {
	gradientDir = Edges::direction(currentFrame);
}

Block BlockMatching::buildBlock(int x, int y, int displacement, int blockSize, int width, int height) {
	Point2D point(x,y);
	Point2D start (max(0, x - displacement), max(0, y - displacement));
	Point2D end(min(width - blockSize, x + displacement), min(height - blockSize, y + displacement));

	SearchWindow window(start, end);

	Block block(point, blockSize, window);
	return block;
}

Node BlockMatching::buildNode(int x, int y, int displacement, int blockSize, int width, int height, const cv::Mat& edgeMap) {
	Block block = buildBlock(x, y, displacement, blockSize, width, height);
	
	int count = 0;
	
	for( int dy = y; dy < y + blockSize ; dy++ )
		for( int dx = x ; dx < x + blockSize ; dx++ )
			if(edgeMap.at<uchar>(dy,dx) > 0)
				count++;

	Node node(block, count);	
	return node;
}

Tuple BlockMatching::fs(const Block& block, const cv::Mat& currentFrame, const cv::Mat& referenceFrame) {
	int searchPoints = 0, blockSize = block.getSize();
	double minDistortion = DBL_MAX, distortion = 0.0;
	
	SearchWindow window = block.getWindow();
	Point2D wstart = window.getStart(), wend = window.getEnd();
	Point2D startPoint = block.getPosition(), endPoint, tmpPoint;
	
	for(int y = wstart.getY(); y <= wend.getY(); y++)  {
		for(int x = wstart.getX(); x <= wend.getX(); x++) {
			tmpPoint.set(x,y);		
			distortion = BDM::SAD(currentFrame, referenceFrame, startPoint, tmpPoint, blockSize);
			searchPoints++;

			if(distortion < minDistortion) {
				minDistortion = distortion;
				endPoint = tmpPoint;
			}	
		}
	}
	
	Point2D mv(endPoint.getX() - startPoint.getX(), endPoint.getY() - startPoint.getY());
	Tuple t(block, mv, searchPoints);

	return t;
}

Tuple BlockMatching::tss(const Block& block, const cv::Mat& currentFrame, const cv::Mat& referenceFrame) {
	blockID++;

	int searchPoints = 1, blockSize = block.getSize();
	double distortion = 0.0;

	SearchWindow window = block.getWindow();
	Point2D wstart = window.getStart(), wend = window.getEnd();
	Point2D startPoint = block.getPosition(), endPoint = block.getPosition(), tmpPoint;

	double minDistortion = BDM::SAD(currentFrame, referenceFrame, startPoint, endPoint, blockSize);
	controlMatrix[endPoint.getY()][endPoint.getX()] = blockID;

	for(int step = 4; step >= 1; step /= 2) {
		int centreX = endPoint.getX(), centreY = endPoint.getY(); 

		for(unsigned int i = 1; i < dxSquare.size(); i++) {
			int x = centreX + dxSquare[i] * step;
			int y = centreY + dySquare[i] * step;

			if(x < wstart.getX() || y < wstart.getY() || x > wend.getX() || y > wend.getY() || controlMatrix[y][x] == blockID)
				continue;

			tmpPoint.set(x,y);
			distortion = BDM::SAD(currentFrame, referenceFrame, startPoint, tmpPoint, blockSize);
			controlMatrix[y][x] = blockID;
			searchPoints++;

			if(distortion < minDistortion) {
				minDistortion = distortion;
				endPoint = tmpPoint;
			}
		}
	}

	Point2D mv(endPoint.getX() - startPoint.getX(), endPoint.getY() - startPoint.getY());
	Tuple t(block, mv, searchPoints);

	return t;
}

Tuple BlockMatching::fss(const Block& block, const cv::Mat& currentFrame, const cv::Mat& referenceFrame) {
	blockID++;

	int searchPoints = 1, step = 2, blockSize = block.getSize();
	double distortion = 0.0;

	SearchWindow window = block.getWindow();
	Point2D wstart = window.getStart(), wend = window.getEnd();
	Point2D startPoint = block.getPosition(), endPoint = block.getPosition(), tmpPoint;

	double minDistortion = BDM::SAD(currentFrame, referenceFrame, startPoint, endPoint, blockSize);
	controlMatrix[endPoint.getY()][endPoint.getX()] = blockID;

	while(true) {
		int idx = 0, idy = 0;
		int centreX = endPoint.getX(), centreY = endPoint.getY();

		for(unsigned int i = 1; i < dxSquare.size(); i++) {
			int x = centreX + dxSquare[i] * step;
			int y = centreY + dySquare[i] * step;

			if(x < wstart.getX() || y < wstart.getY() || x > wend.getX() || y > wend.getY() || controlMatrix[y][x] == blockID)
				continue;

			tmpPoint.set(x,y);
			distortion = BDM::SAD(currentFrame, referenceFrame, startPoint, tmpPoint, blockSize);
			controlMatrix[y][x] = blockID;
			searchPoints++;
			
			if(distortion < minDistortion) {
				minDistortion = distortion;
				endPoint = tmpPoint;
				
				idx = dxSquare[i];
				idy = dySquare[i];
			}
		}

		if(step == 1)
			break;	

		if(idx == 0 && idy == 0)
			step = 1;	
	}

	Point2D mv(endPoint.getX() - startPoint.getX(), endPoint.getY() - startPoint.getY());
	Tuple t(block, mv, searchPoints);

	return t;
}

Tuple BlockMatching::ds(const Block& block, const cv::Mat& currentFrame, const cv::Mat& referenceFrame) {
	blockID++;

	int searchPoints = 1, step = 2, blockSize = block.getSize();
	double distortion = 0.0;
	
	SearchWindow window = block.getWindow();
	Point2D wstart = window.getStart(), wend = window.getEnd();
	Point2D startPoint = block.getPosition(), endPoint = block.getPosition(), tmpPoint;

	vector<int> dx = dxDiamond;
	vector<int> dy = dyDiamond;
	
	double minDistortion = BDM::SAD(currentFrame, referenceFrame, startPoint, endPoint, blockSize);
	controlMatrix[endPoint.getY()][endPoint.getX()] = blockID;
	
	while(true) {
		int idx = 0, idy = 0;
		int centreX = endPoint.getX(), centreY = endPoint.getY();
		
		for(unsigned int i = 1; i < dx.size(); i++) {
			int x = centreX + dxSquare[i] * step;
			int y = centreY + dySquare[i] * step;

			if(x < wstart.getX() || y < wstart.getY() || x > wend.getX() || y > wend.getY() || controlMatrix[y][x] == blockID)
				continue;

			tmpPoint.set(x,y);
			distortion = BDM::SAD(currentFrame, referenceFrame, startPoint, tmpPoint, blockSize);
			controlMatrix[y][x] = blockID;
			searchPoints++;

			if( distortion < minDistortion ) {
				minDistortion = distortion;
				endPoint = tmpPoint;
				
				idx = dx[i];
				idy = dy[i];
			}
		}
		
		if(step == 1)
			break;
			
		if(idx == 0 && idy == 0) {
			step = 1;
			
			dx = dxCross;
			dy = dyCross;
		}		
	}
	
	Point2D mv(endPoint.getX() - startPoint.getX(), endPoint.getY() - startPoint.getY());
	Tuple t(block, mv, searchPoints);

	return t;
}

Tuple BlockMatching::hexbs(const Block& block, const cv::Mat& currentFrame, const cv::Mat& referenceFrame) {
	blockID++;

	int searchPoints = 1, step = 2, blockSize = block.getSize();
	double distortion = 0.0;
	
	SearchWindow window = block.getWindow();
	Point2D wstart = window.getStart(), wend = window.getEnd();
	Point2D startPoint = block.getPosition(), endPoint = block.getPosition(), tmpPoint;

	vector<int> dx = dxHexagonal;
	vector<int> dy = dyHexagonal;
	
	double minDistortion = BDM::SAD(currentFrame, referenceFrame, startPoint, endPoint, blockSize);
	controlMatrix[endPoint.getY()][endPoint.getX()] = blockID;

	while(true) {
		int idx = 0, idy = 0;
		int centreX = endPoint.getX(), centreY = endPoint.getY();
		
		for(unsigned int i = 1; i < dx.size(); i++) {
			int x = centreX + dxSquare[i] * step;
			int y = centreY + dySquare[i] * step;

			if(x < wstart.getX() || y < wstart.getY() || x > wend.getX() || y > wend.getY() || controlMatrix[y][x] == blockID)
				continue;

			tmpPoint.set(x,y);
			distortion = BDM::SAD(currentFrame, referenceFrame, startPoint, tmpPoint, blockSize);
			controlMatrix[y][x] = blockID;
			searchPoints++;

			if( distortion < minDistortion ) {
				minDistortion = distortion;
				endPoint = tmpPoint;
				
				idx = dx[i];
				idy = dy[i];
			}
		}

		if(step == 1)
			break;
			
		if(idx == 0 && idy == 0) {
			step = 1;
			
			dx = dxCross;
			dy = dyCross;
		}	
	}	
	
	Point2D mv(endPoint.getX() - startPoint.getX(), endPoint.getY() - startPoint.getY());
	Tuple t(block, mv, searchPoints);

	return t;
}

Tuple BlockMatching::mdgds(const Block& block, const cv::Mat& currentFrame, const cv::Mat& referenceFrame) {
	blockID++;

	int improvements = 0, searchPoints = 1, blockSize = block.getSize();
	double centreDistortion = 0.0, directionalDistortion = 0.0, distortion = 0.0;

	SearchWindow window = block.getWindow();
	Point2D wstart = window.getStart(), wend = window.getEnd();
	Point2D startPoint = block.getPosition(), endPoint = block.getPosition();
	
	double minDistortion = BDM::SAD(currentFrame, referenceFrame, startPoint, endPoint, blockSize);
	controlMatrix[endPoint.getY()][endPoint.getX()] = blockID;

	do {
		improvements = 0;
		centreDistortion = minDistortion;

		int centreX = endPoint.getX(), centreY = endPoint.getY();
		
		for(unsigned int i = 1; i < dxSquare.size(); i++) {
			int x = centreX, y = centreY;

			distortion = centreDistortion;
			directionalDistortion = 0.0;

			do {
				directionalDistortion = distortion;

				x += dxSquare[i];
				y += dySquare[i];

				if(x < wstart.getX() || y < wstart.getY() || x > wend.getX() || y > wend.getY() || controlMatrix[y][x] == blockID)
					break;

				distortion = BDM::SAD(currentFrame, referenceFrame, startPoint, Point2D(x,y), blockSize);
				controlMatrix[y][x] = blockID;
				searchPoints++;
			} while(distortion <= directionalDistortion);

			if(directionalDistortion < minDistortion) {
				minDistortion = directionalDistortion;
				endPoint.set(x - dxSquare[i], y - dySquare[i]);

				improvements++;
			}
		}
	} while(improvements > 0);

	Point2D mv(endPoint.getX() - startPoint.getX(), endPoint.getY() - startPoint.getY());
	Tuple t(block, mv, searchPoints);

	return t;
}

Tuple BlockMatching::fdgds(const Block& block, const cv::Mat& currentFrame, const cv::Mat& referenceFrame) {
	blockID++;

	int improvements = 0, searchPoints = 1, blockSize = block.getSize();
	double centreDistortion = 0.0, directionalDistortion = 0.0, distortion = 0.0;

	SearchWindow window = block.getWindow();
	Point2D wstart = window.getStart(), wend = window.getEnd();
	Point2D startPoint = block.getPosition(), endPoint = block.getPosition();
	
	double minDistortion = BDM::SAD(currentFrame, referenceFrame, startPoint, endPoint, blockSize);
	controlMatrix[endPoint.getY()][endPoint.getX()] = blockID;

	do {
		improvements = 0;
		centreDistortion = minDistortion;

		int centreX = endPoint.getX(), centreY = endPoint.getY();
		
		for(unsigned int i = 1; i < dxSquare.size(); i++) {
			int x = centreX, y = centreY;

			distortion = centreDistortion;
			directionalDistortion = 0.0;

			do {
				directionalDistortion = distortion;

				x += dxSquare[i];
				y += dySquare[i];

				if(x < wstart.getX() || y < wstart.getY() || x > wend.getX() || y > wend.getY() || controlMatrix[y][x] == blockID)
					break;

				distortion = BDM::SAD(currentFrame, referenceFrame, startPoint, Point2D(x,y), blockSize);
				controlMatrix[y][x] = blockID;
				searchPoints++;
			} while(distortion <= directionalDistortion);

			if(directionalDistortion < minDistortion) {
				double rdr = directionalDistortion / minDistortion;

				minDistortion = directionalDistortion;
				endPoint.set(x - dxSquare[i], y - dySquare[i]);

				improvements++;
				
				if(rdr < threshold)
					break;
			}
		}
	} while(improvements > 0);

	Point2D mv(endPoint.getX() - startPoint.getX(), endPoint.getY() - startPoint.getY());
	Tuple t(block, mv, searchPoints);

	return t;
}

Tuple BlockMatching::arps(const Block& block, const cv::Mat& currentFrame, const cv::Mat& referenceFrame) {
	blockID++;

	int searchPoints = 1, blockSize = block.getSize(), patternSize = 2, improvements = 0;
	double distortion = 0.0;

	SearchWindow window = block.getWindow();
	Point2D wstart = window.getStart(), wend = window.getEnd();
	Point2D startPoint = block.getPosition(), endPoint = block.getPosition(), tmpPoint;

	double minDistortion = BDM::SAD(currentFrame, referenceFrame, startPoint, endPoint, blockSize);
	controlMatrix[endPoint.getY()][endPoint.getX()] = blockID;

	vector<int> dx = dxCross;
	vector<int> dy = dyCross;

	if(startPoint.getX() != 0) {
		Point2D predMV = output[blockID - 1].getMotionVector();
		patternSize = max(abs(predMV.getX()), abs(predMV.getY()));
		
		dx.push_back(predMV.getX());
		dy.push_back(predMV.getY());
	}

	for(int i = 1; i < dx.size(); i++) {
		int x, y;

		if(i != dx.size() - 1 || startPoint.getX() == 0) {
			x = startPoint.getX() + dx[i] * patternSize;
			y = startPoint.getY() + dy[i] * patternSize;
		}
		else {
			x = startPoint.getX() + dx[i];
			y = startPoint.getY() + dy[i];
		}

		if(x < wstart.getX() || y < wstart.getY() || x > wend.getX() || y > wend.getY() || controlMatrix[y][x] == blockID)
			continue;

		tmpPoint.set(x,y);
		distortion = BDM::SAD(currentFrame, referenceFrame, startPoint, tmpPoint, blockSize);
		controlMatrix[y][x] = blockID;
		searchPoints++;

		if( distortion < minDistortion ) {
			minDistortion = distortion;
			endPoint = tmpPoint;
		}
	}

	do {
		improvements = 0;

		int centreX = endPoint.getX(), centreY = endPoint.getY();

		for(int i = 1; i < dxCross.size(); i++) {
			int x = centreX + dxCross[i];
			int y = centreY + dyCross[i];

			if(x < wstart.getX() || y < wstart.getY() || x > wend.getX() || y > wend.getY() || controlMatrix[y][x] == blockID)
				continue;

			tmpPoint.set(x,y);
			distortion = BDM::SAD(currentFrame, referenceFrame, startPoint, tmpPoint, blockSize);
			controlMatrix[y][x] = blockID;
			searchPoints++;

			if( distortion < minDistortion ) {
				minDistortion = distortion;
				endPoint = tmpPoint;

				improvements++;
			}

		}
	} while(improvements > 0);

	Point2D mv(endPoint.getX() - startPoint.getX(), endPoint.getY() - startPoint.getY());
	Tuple t(block, mv, searchPoints);

	return t;
}

Tuple BlockMatching::tbs(const Block& block, const cv::Mat& currentFrame, const cv::Mat& referenceFrame) {
	blockID++;

	int searchPoints = 1, step = 2, blockSize = block.getSize();
	double distortion = 0.0;

	SearchWindow window = block.getWindow();
	Point2D wstart = window.getStart(), wend = window.getEnd();
	Point2D startPoint = block.getPosition();
	Point2D endPoint1 = block.getPosition(), endPoint2 = block.getPosition(), tmpPoint;

	double minDistortion = BDM::SAD(currentFrame, referenceFrame, startPoint, startPoint, blockSize);
	double dist1 = minDistortion, dist2 = DBL_MAX;
	controlMatrix[endPoint1.getY()][endPoint1.getX()] = blockID;

	int improvements = 0;

	double direction = 0.0;
	Point2D point = block.getPosition();

	for(int y = point.getY(); y < point.getY() + blockSize ; y++)
		for(int x = point.getX(); x < point.getX() + blockSize; x++)
			direction += gradientDir.at<uchar>(y,x);

	direction /= Math::square((double)blockSize);
	direction -= 180.0;

	while(direction < 0)
		direction += 360.0;

	while(direction > 360)
		direction -= 360.0;

	int dx = 0, dy = 0;

	if(direction >= 45 && direction < 135) { // 90
		dx = 0;	dy = -1;
	}
	else if(direction >= 135 && direction < 225) { // 180
		dx = -1; dy = 0;
	}
	else if(direction >= 225 && direction < 315) {  // 270
		dx = 0;
		dy = 1;
	}
	else { // 0
		dx = 1;
		dy = 0;
	}

	int fails = 0;
	int xx = endPoint2.getX(), yy = endPoint2.getY();

	while(fails < 4) {
		xx += dx;
		yy += dy;

		fails++;

		if(xx < wstart.getX() || yy < wstart.getY() || xx > wend.getX() || yy > wend.getY() )
			continue;

		tmpPoint.set(xx,yy);
		distortion = BDM::SAD(currentFrame, referenceFrame, startPoint, tmpPoint, blockSize);
		searchPoints++;

		if(distortion < dist2) {
			dist2 = distortion;
			endPoint2 = tmpPoint;
		}
	}

	do {
		improvements = 0;
		int centreX = endPoint2.getX(), centreY = endPoint2.getY();
		
		for(unsigned int i = 1; i < dxSquare.size(); i++) {
			int x = centreX + dxSquare[1];
			int y = centreY + dxSquare[1];

			if(x < wstart.getX() || y < wstart.getY() || x > wend.getX() || y > wend.getY())
				continue;

			tmpPoint.set(x,y);
			distortion = BDM::SAD(currentFrame, referenceFrame, startPoint, tmpPoint, blockSize);
			searchPoints++;

			if(distortion < dist2) {
				dist2 = distortion;
				endPoint2 = tmpPoint;

				improvements++;
			}
		}
	} while(improvements > 0);

	do {
		improvements = 0;
		int centreX = endPoint1.getX(), centreY = endPoint1.getY();
		
		for(unsigned int i = 1; i < dxSquare.size(); i++) {
			int x = centreX + dxSquare[1];
			int y = centreY + dxSquare[1];

			if(x < wstart.getX() || y < wstart.getY() || x > wend.getX() || y > wend.getY() || controlMatrix[y][x] == blockID)
				continue;

			tmpPoint.set(x,y);
			distortion = BDM::SAD(currentFrame, referenceFrame, startPoint, tmpPoint, blockSize);
			controlMatrix[tmpPoint.getY()][tmpPoint.getX()] = blockID;
			searchPoints++;

			if(distortion < dist1) {
				dist1 = distortion;
				endPoint1 = tmpPoint;

				improvements++;
			}
		}
	} while(improvements > 0);

	Point2D mv;
	
	if(dist1 < dist2)
		mv.set(endPoint1.getX() - startPoint.getX(), endPoint1.getY() - startPoint.getY());
	else
		mv.set(endPoint2.getX() - startPoint.getX(), endPoint2.getY() - startPoint.getY());

	Tuple t(block, mv, searchPoints);	
	return t;
}

void BlockMatching::fixedBsSearch(const cv::Mat& currentFrame, const cv::Mat& referenceFrame, int displacement, int blockSize, Tuple(BlockMatching::*algorithm)(const Block&, const cv::Mat&, const cv::Mat&)) {
	int width = currentFrame.cols;
	int height = currentFrame.rows;

	// No splitting
	if(width % blockSize == 0 && height % blockSize == 0) {
		int lastX = width - blockSize;
		int lastY = height - blockSize;
		
		for(int y = 0; y <= lastY; y += blockSize) {
			for(int x = 0; x <= lastX; x += blockSize) {
				Block block = buildBlock(x, y, displacement, blockSize, width, height);
				Tuple t = (this->*algorithm)(block, currentFrame, referenceFrame);
				output.push_back(t);
			}
		}
	}
	// Split cols
	else if(width % blockSize != 0 && height % blockSize == 0) {
		int lastX = (width / blockSize) * blockSize;
		int lastY = height - blockSize;

		for(int y = 0; y <= lastY; y += blockSize) {
			for(int x = 0; x < lastX; x += blockSize) {
				Block block = buildBlock(x, y, displacement, blockSize, width, height);
				Tuple t = (this->*algorithm)(block, currentFrame, referenceFrame);
				output.push_back(t);
			}
		}

		int iniX = lastX;
		int newWidth = width - lastX;
		int bs = blockSize;
		
		while(newWidth % bs != 0 || height % bs != 0)
			bs /= 2;
		
		lastX = width - bs;

		for(int y = 0; y <= lastY; y += bs) {
			for(int x = iniX ; x <= lastX; x += bs) {
				Block block = buildBlock(x, y, displacement, bs, width, height);
				Tuple t = (this->*algorithm)(block, currentFrame, referenceFrame);
				output.push_back(t);
			}
		}
	}
	// Split rows
	else if(width % blockSize == 0 && height % blockSize != 0) {
		int lastX = width - blockSize;
		int lastY = (height / blockSize) * blockSize;

		for(int y = 0; y < lastY; y += blockSize) {
			for(int x = 0; x <= lastX; x += blockSize) {
				Block block = buildBlock(x, y, displacement, blockSize, width, height);
				Tuple t = (this->*algorithm)(block, currentFrame, referenceFrame);
				output.push_back(t);
			}
		}
		
		int iniY = lastY;
		int newHeight = height - lastY;
		int bs = blockSize;
		
		while(width % bs != 0 || newHeight % bs != 0)
			bs /= 2;
	 
		lastY = height - bs;

		for(int y = iniY; y <= lastY; y += bs) {
			for(int x = 0; x <= lastX; x += bs) {
				Block block = buildBlock(x, y, displacement, bs, width, height);
				Tuple t = (this->*algorithm)(block, currentFrame, referenceFrame);
				output.push_back(t);
			}
		}
	}	
	// Split both: cols and rows
	else {
		int lastX = (width / blockSize ) * blockSize;
		int lastY = (height / blockSize) * blockSize;
		
		for(int y = 0; y < lastY; y += blockSize) {
			for(int x = 0; x < lastX; x += blockSize) {
				Block block = buildBlock(x, y, displacement, blockSize, width, height);
				Tuple t = (this->*algorithm)(block, currentFrame, referenceFrame);
				output.push_back(t);
			}
		}
		
		int newWidth = width - lastX;
		int newHeight = height - lastY;
		int bs = blockSize;
		
		while(width % bs != 0 || newWidth % bs != 0 || height % bs != 0 || newHeight % bs != 0)
			bs /= 2;	

		int iniX = lastX, iniY = lastY;	

		lastX = width - bs;
		lastY = height - bs;	
		
		// Splitting cols
		for(int y = 0; y <= lastY; y += bs) {
			for(int x = iniX; x <= lastX; x += bs) {
				Block block = buildBlock(x, y, displacement, bs, width, height);
				Tuple t = (this->*algorithm)(block, currentFrame, referenceFrame);
				output.push_back(t);
			}
		}
		
		// Splitting rows
		for(int y = iniY; y <= lastY; y += bs) {
			for(int x = 0; x < iniX; x += bs) {
				Block block = buildBlock(x, y, displacement, bs, width, height);
				Tuple t = (this->*algorithm)(block, currentFrame, referenceFrame);
				output.push_back(t);
			}
		}
	}
}

void BlockMatching::bfs(const cv::Mat& currentFrame, const cv::Mat& referenceFrame, const cv::Mat& edgeMap, int x, int y, int displacement, int blockSize, Tuple(BlockMatching::*algorithm)(const Block&, const cv::Mat&, const cv::Mat&)) {
	Node root = buildNode(x, y, displacement, blockSize, currentFrame.cols, currentFrame.rows, edgeMap);
	queue<Node> q;
	q.push(root);

	int it = -1;

	while(!q.empty()) {
		it++;

		Node node = q.front();
		q.pop();

		Block block = node.getBlock();
		//if(((it == 0 && node.getEdges() >= T * Math::square(block.getSize())) || (it > 0 && node.getEdges() >= node.getEdgesParent() / 4)) && block.getSize() > 8) {
		if(node.getEdges() >= 1 && block.getSize() > 8) {
			int bs = block.getSize() / 2;
			Point2D p = block.getPosition();

			int x[] = {p.getX(), p.getX() + bs, p.getX(), p.getX() + bs};
			int y[] = {p.getY(), p.getY(), p.getY() + bs, p.getY() + bs};

			for(int i = 0; i < 4; i++) {
				Node n = buildNode (x[i], y[i], displacement, bs, currentFrame.cols, currentFrame.rows, edgeMap);
				n.setEdgesParent(node.getEdges());
				q.push(n);
			}
		}
		else {
			Tuple t = (this->*algorithm)(block, currentFrame, referenceFrame);
			output.push_back(t);
		}
	}
}

void BlockMatching::variableBsSearch(const cv::Mat& currentFrame, const cv::Mat& referenceFrame, int displacement, int blockSize, Tuple(BlockMatching::*algorithm)(const Block&, const cv::Mat&, const cv::Mat&)) {
	cv::Mat difImg = abs(currentFrame - referenceFrame);
	cv::Mat edgeMap = Edges::sobel(difImg);

	int width = currentFrame.cols;
	int height = currentFrame.rows;

	// No splitting
	if(width % blockSize == 0 && height % blockSize == 0) {
		int lastX = width - blockSize;
		int lastY = height - blockSize;
		
		for(int y = 0; y <= lastY; y += blockSize)
			for(int x = 0; x <= lastX; x += blockSize)
				bfs(currentFrame, referenceFrame, edgeMap, x, y, displacement, blockSize, algorithm);
	}
	// Split cols
	else if(width % blockSize != 0 && height % blockSize == 0) {
		int lastX = (width / blockSize) * blockSize;
		int lastY = height - blockSize;

		for(int y = 0; y <= lastY; y += blockSize)
			for(int x = 0; x < lastX; x += blockSize)
				bfs(currentFrame, referenceFrame, edgeMap, x, y, displacement, blockSize, algorithm);

		int iniX = lastX;
		int newWidth = width - lastX;
		int bs = blockSize;
		
		while(newWidth % bs != 0 || height % bs != 0)
			bs /= 2;
		
		lastX = width - bs;

		for(int y = 0; y <= lastY; y += bs)
			for(int x = iniX ; x <= lastX; x += bs)
				bfs(currentFrame, referenceFrame, edgeMap, x, y, displacement, bs, algorithm);
	}
	// Split rows
	else if(width % blockSize == 0 && height % blockSize != 0) {
		int lastX = width - blockSize;
		int lastY = (height / blockSize) * blockSize;

		for(int y = 0; y < lastY; y += blockSize)
			for(int x = 0; x <= lastX; x += blockSize)
				bfs(currentFrame, referenceFrame, edgeMap, x, y, displacement, blockSize, algorithm);
		
		int iniY = lastY;
		int newHeight = height - lastY;
		int bs = blockSize;
		
		while(width % bs != 0 || newHeight % bs != 0)
			bs /= 2;
	 
		lastY = height - bs;

		for(int y = iniY; y <= lastY; y += bs)
			for(int x = 0; x <= lastX; x += bs)
				bfs(currentFrame, referenceFrame, edgeMap, x, y, displacement, bs, algorithm);
	}	
	// Split both: cols and rows
	else {
		int lastX = (width / blockSize ) * blockSize;
		int lastY = (height / blockSize) * blockSize;
		
		for(int y = 0; y < lastY; y += blockSize)
			for(int x = 0; x < lastX; x += blockSize)
				bfs(currentFrame, referenceFrame, edgeMap, x, y, displacement, blockSize, algorithm);
		
		int newWidth = width - lastX;
		int newHeight = height - lastY;
		int bs = blockSize;
		
		while(width % bs != 0 || newWidth % bs != 0 || height % bs != 0 || newHeight % bs != 0)
			bs /= 2;	

		int iniX = lastX, iniY = lastY;	

		lastX = width - bs;
		lastY = height - bs;	
		
		// Splitting cols
		for(int y = 0; y <= lastY; y += bs)
			for(int x = iniX; x <= lastX; x += bs)
				bfs(currentFrame, referenceFrame, edgeMap, x, y, displacement, bs, algorithm);
		
		// Splitting rows
		for(int y = iniY; y <= lastY; y += bs)
			for(int x = 0; x < iniX; x += bs)
				bfs(currentFrame, referenceFrame, edgeMap, x, y, displacement, bs, algorithm);
	}
}

vector<Tuple> BlockMatching::estimateMotion(const cv::Mat& currentFrame, const cv::Mat& referenceFrame, int displacement, int blockSize, int blockType, Tuple(BlockMatching::*algorithm)(const Block&, const cv::Mat&, const cv::Mat&)) {
	output.clear();

	if(blockType == NO_VARIABLE_BLOCK)
		fixedBsSearch(currentFrame, referenceFrame, displacement, blockSize, algorithm);
	else if(blockType == VARIABLE_BLOCK)
		variableBsSearch(currentFrame, referenceFrame, displacement, blockSize, algorithm);

	return output;
}

cv::Mat BlockMatching::compensation(const cv::Mat& referenceFrame, const vector<Tuple>& tuples) {
	cv::Mat compensatedFrame(referenceFrame.rows, referenceFrame.cols, CV_8UC1);
	
	for( unsigned int i = 0 ; i < tuples.size() ; i++ ) {
		Block block = tuples[i].getBlock();

		Point2D start = block.getPosition();
		Point2D mv = tuples[i].getMotionVector();
		
		for( int y = start.getY() ; y < start.getY() + block.getSize() ; y++ )
			for( int x = start.getX() ; x < start.getX() + block.getSize() ; x++ )
				compensatedFrame.at<uchar>(y,x) = referenceFrame.at<uchar>(y + mv.getY(), x + mv.getX());
	}
		
	return compensatedFrame;
}

cv::Mat BlockMatching::drawBlocks(const cv::Mat& image, const vector<Tuple>& tuples) {
	cv::Mat blocksFrame;
	cvtColor(image, blocksFrame, cv::COLOR_GRAY2RGB);

	for(unsigned int i = 0; i < tuples.size(); i++) {
		Block block = tuples[i].getBlock();
		Point2D point = block.getPosition();
		
		for(int x = point.getX(); x < point.getX() + block.getSize(); x++) {
			blocksFrame.at<cv::Vec3b>(point.getY(), x)[1] = 255;
			
			if(point.getY() + block.getSize() < image.rows)
				blocksFrame.at<cv::Vec3b>(point.getY() + block.getSize(), x)[1] = 255;
		}
		
		for(int y = point.getY(); y < point.getY() + block.getSize(); y++) {
			blocksFrame.at<cv::Vec3b>(y, point.getX())[1] = 255;
			
			if(point.getX() + block.getSize() < image.cols)
				blocksFrame.at<cv::Vec3b>(y, point.getX() + block.getSize())[1] = 255;
		}
	}
	
	return blocksFrame;
}

void BlockMatching::setThreshold(int threshold) {
	this->threshold = threshold;
}
