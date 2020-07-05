#include <fstream>  // std::ifstream
#include <string>
#include <vector>

#ifndef INCLUDE_CAMERA_H_
#define INCLUDE_CAMERA_H_

class Camera {
 private:
  std::string intrinsics_path{""};
  std::string lut_path{""};
  std::ifstream file;

 public:
  double fx{1};
  double fy{1};
  double cx{1};
  double cy{1};
  double G_camera_image[4][4];
  std::vector<std::vector<double>> K_matrix;
  double LUT{2};
  Camera(std::string, std::string);
  void updateKMatrix();
  void readcamProperty();
};

#endif /* INCLUDE_CAMERA_H_ */
