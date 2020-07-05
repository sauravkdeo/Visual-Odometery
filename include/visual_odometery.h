#include <camera.h>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/plot.hpp>
#include <vector>

#ifndef _INCLUDE_VISUAL_ODOMETERY_H_
#define _INCLUDE_VISUAL_ODOMETERY_H_

class Visual_Odometery {
 private:
  Camera* cam;
  const int MAX_FEATURES = 500;
  int numGoodMatches{75};
  const float GOOD_MATCH_PERCENT = 0.15f;
  std::string path;
  cv::Mat1b im1, im2;
  cv::Mat3b cim1, cim2;
  cv::Mat3b h_merged;

  std::vector<cv::KeyPoint> keypoints1, keypoints2;
  std::vector<cv::Point2f> selected_points1, selected_points2;

  cv::Mat descriptors1, descriptors2;
  cv::Ptr<cv::Feature2D> orb;
  std::vector<cv::DMatch> matches;
  cv::Ptr<cv::DescriptorMatcher> matcher;
  cv::Mat essentialMat, mask;
  cv::Mat T;
  cv::Mat T_o;
  cv::Mat temp_T;
  cv::Mat R;
  cv::Mat R_o;
  cv::Mat delt;
  bool first;
  std::vector<double> xData, yData;
  cv::Ptr<cv::plot::Plot2d> plot_;
  cv::Mat display;

 public:
  Visual_Odometery(std::string);
  Visual_Odometery(std::string, Camera*);
  void plot_graph();
  void SfMAlgorithm();
  void ReadCameraModel(std::string path_);
  void detectORBandMatch();
  void UndistortImage(cv::Mat3b&, cv::Mat3b);
  void findFundamentalMatrix();
  void findEssentialMatrix();
  void estimateAndPlotPose();
};

#endif
