#include <algorithm>
#include <cmath>
#include <string>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "BlockMatching.h"
#include "File.h"
#include "Quality.h"

double avgSearchPoints(const std::vector<Tuple>& tuples)
{
  double output = 0.0;
  
  for (unsigned int i = 0; i < tuples.size(); i++)
    output += tuples[i].getSearchPoints();
  
  output /= (double)tuples.size();
  return output;
}

void run(const std::vector<std::string>& entryList, int displacement, int blockSize, int blockType, Tuple(BlockMatching::*algorithm)(const Block&, const cv::Mat&, const cv::Mat&)) {
  unsigned int it = 1;

  cv::Mat currentFrame, referenceFrame;
  currentFrame = cv::imread(entryList[it].c_str(), cv::ImreadModes::IMREAD_GRAYSCALE); 

  referenceFrame = cv::imread(entryList[it - 1].c_str(), cv::ImreadModes::IMREAD_GRAYSCALE);

  BlockMatching bm(currentFrame.cols, currentFrame.rows);
  
  do {
    if(it > 1) {
      referenceFrame = currentFrame;
      currentFrame = cv::imread(entryList[it].c_str(), cv::ImreadModes::IMREAD_GRAYSCALE); 

      bm.initialiseVars();
    }
      
    //bm.setGradientDir(currentFrame);
    std::vector<Tuple> tuples = bm.estimateMotion(currentFrame, referenceFrame, displacement, blockSize, blockType, algorithm);
    cv::Mat compensatedFrame = bm.compensation(referenceFrame, tuples);

    // psnr
    double psnr = Quality::PSNR(currentFrame, compensatedFrame);

    // mssim
    cv::Scalar mssim = Quality::MSSIM(currentFrame, compensatedFrame);

    // search points
    double nspTotal = avgSearchPoints(tuples);

    // Print results
    std::cout << ( it ) << "\t\t" << psnr << "\t\t" << mssim[0] << "\t\t" << nspTotal << std::endl;

    it++;
    /*
    cv::Mat bloques = bm.drawBlocks(currentFrame, tuples);
    cv::imwrite("bloques.jpg", bloques);
    */
  }
  while(it <= entryList.size() - 1);
}

int main(int argc, char* argv[])
{
  std::string algorithm = std::string(argv[1]);
  int displacement = atoi(argv[2]);
  int blockSize = atoi(argv[3]);  
  std::string variableSize = std::string(argv[4]);

  int blockType;
  
  if( variableSize == "variable" )
    blockType = BlockMatching::VARIABLE_BLOCK;
  else
    blockType = BlockMatching::NO_VARIABLE_BLOCK;
  
  std::string path = std::string(argv[5]);

  std::vector< std::string > images = File::getEntryList(path);
  sort(images.begin(), images.end());  
  
  if(algorithm == "fs")
    run(images, displacement, blockSize, blockType, &BlockMatching::fs);
  else if(algorithm == "tss")
    run(images, displacement, blockSize, blockType, &BlockMatching::tss);
  else if(algorithm == "fss")
    run(images, displacement, blockSize, blockType, &BlockMatching::fss);
  else if(algorithm == "ds")
    run(images, displacement, blockSize, blockType, &BlockMatching::ds);
  else if(algorithm == "hexbs")
    run(images, displacement, blockSize, blockType, &BlockMatching::hexbs);
  else if(algorithm == "mdgds")
    run(images, displacement, blockSize, blockType, &BlockMatching::mdgds);
  else if(algorithm == "fdgds")
    run(images, displacement, blockSize, blockType, &BlockMatching::fdgds);
  else if(algorithm == "arps")
    run(images, displacement, blockSize, blockType, &BlockMatching::arps);
  else if(algorithm == "tbs")
    run(images, displacement, blockSize, blockType, &BlockMatching::tbs);
  
  return 0;
}
