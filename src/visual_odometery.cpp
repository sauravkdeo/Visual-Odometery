#include <visual_odometery.h>

#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/plot.hpp>
#include <vector>

Visual_Odometery::Visual_Odometery(std::string path_, Camera* cam_)
    : path(path_), cam(cam_), first(true) {
  orb = cv::ORB::create(MAX_FEATURES);
}
Visual_Odometery::Visual_Odometery(std::string path_)
    : path(path_), cam(nullptr), first(true) {
  orb = cv::ORB::create(MAX_FEATURES);
}

void Visual_Odometery::UndistortImage(cv::Mat3b& src, cv::Mat3b dstn) {}

void Visual_Odometery::SfMAlgorithm() {
  std::vector<cv::String> fn;
  cv::glob(path, fn, true);  // recurse

  for (size_t k = 50; k < fn.size(); ++k) {
    im1 = cv::imread(fn[k]);
    im2 = cv::imread(fn[k + 1]);
    if (im1.empty() or im2.empty()) continue;

    cv::cvtColor(im1, cim1, cv::COLOR_BayerGR2BGR);
    cv::cvtColor(im2, cim2, cv::COLOR_BayerGR2BGR);
    UndistortImage(cim1, cim1);
    UndistortImage(cim2, cim2);
    detectORBandMatch();
    estimateAndPlotPose();

    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);

    //    cv::hconcat(cim1, cim2, h_merged);
    cv::resize(h_merged, h_merged, cv::Size(), 0.2, 0.75);

    //    cv::resize(h_merged, h_merged, cv::Size(), cam->fx, cam->fy);
    cv::imshow("Display window", h_merged);  // Show our image inside it.
    cv::waitKey(1);
  }
}

void Visual_Odometery::estimateAndPlotPose() {
  selected_points1.clear();
  selected_points2.clear();
  for (unsigned int i = 0; i < matches.size(); i++) {
    // queryidx -> left image
    selected_points1.push_back(keypoints1[matches[i].queryIdx].pt);
    // trainidx -> right image
    selected_points2.push_back(keypoints2[matches[i].trainIdx].pt);
  }
  //
  //  std::cout << selected_points1.size() << " " << selected_points2.size()
  //            << std::endl;
  essentialMat = cv::findEssentialMat(selected_points1, selected_points2);
  cv::recoverPose(essentialMat, selected_points1, selected_points2, R, T);
  if (first) {
    std::cout << "chech";
    first = false;
    T_o = T;
    R_o = R;
  } else {
    //    if (T.at<double>(0, 0) < 0) {
    //      T = -T;
    //    }
    //    if (R_o.at<double>(0, 0) < 0) {
    //      R_o = -R_o;
    //    }
    //    std::cout << "Old Val " << temp_T << T_o << std::endl;
    temp_R = R_o * R;
    R_o = temp_R;
    delt = R_o * T;
    temp_T = T_o + delt;
    T_o = temp_T;
  }
  //  std::cout << delt << " " << T_o << std::endl << std::endl;
  xData.push_back(T_o.at<double>(0, 0));
  yData.push_back(T_o.at<double>(0, 1));
  plot_graph();

  //    essentialMat = cv::findEssentialMat(selected_points2, selected_points1,
  //                                        cam->fx, cv::Point2d{cam->cx,
  //                                        cam->cy}, cv::RANSAC, 0.999, 1.0,
  //                                        mask);
  //  //
  //  cv::recoverPose(essentialMat, selected_points1, selected_points2, R, T,
  //                  cam->fx, cv::Point2d{cam->cx, cam->cy}, mask);
}

// void Visual_Odometery::print(cv::Mat mat_) {}
// void Visual_Odometery::findFundamentalMatrix() {}
//
// void Visual_Odometery::findEssentialMatrix() {}

void Visual_Odometery::plot_graph() {
  plot_ = cv::plot::Plot2d::create(xData, yData);
  plot_->render(display);
  cv::imshow("Plot", display);
  cv::waitKey(1);
}

void Visual_Odometery::detectORBandMatch() {
  orb->detectAndCompute(cim1, cv::Mat(), keypoints1, descriptors1);
  orb->detectAndCompute(cim2, cv::Mat(), keypoints2, descriptors2);
  matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
  matcher->match(descriptors1, descriptors2, matches, cv::Mat());

  std::sort(matches.begin(), matches.end());

  int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
  matches.erase(matches.begin() + numGoodMatches, matches.end());

  cv::drawMatches(cim1, keypoints1, cim2, keypoints2, matches, h_merged);
}

void Visual_Odometery::ReadCameraModel(std::string path = {"dataset/./model"}) {
  //	intrinsics_path = models_dir + "/stereo_narrow_left.txt"
  //	    lut_path = models_dir + "/stereo_narrow_left_distortion_lut.bin"
  //
  //	    intrinsics = np.loadtxt(intrinsics_path)
  //	    # Intrinsics
  //	    fx = intrinsics[0,0]
  //	    fy = intrinsics[0,1]
  //	    cx = intrinsics[0,2]
  //	    cy = intrinsics[0,3]
  //	    # 4x4 matrix that transforms x-forward coordinate frame at camera
  // origin and image frame for specific lens 	    G_camera_image =
  // intrinsics[1:5,0:4] 	    # LUT for undistortion 	    # LUT
  // consists of (u,v) pair for each pixel) 	    lut = np.fromfile(lut_path,
  // np.double) lut = lut.reshape([2, lut.size//2]) 	    LUT =
  // lut.transpose()
  //
  //	    return fx, fy, cx, cy, G_camera_image, LUT
}
