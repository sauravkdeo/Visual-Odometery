#include <camera.h>

#include <fstream>   // std::ifstream
#include <iostream>  // std::cout

Camera::Camera(std::string path1, std::string path2)
    : intrinsics_path(path1), lut_path(path2), file(intrinsics_path) {
  readcamProperty();
}
void Camera::readcamProperty() {
  std::cout << "asa";

  if (!file.is_open()) {
    return;
  }
  double k_array[4];
  for (int i = 0; i != 3; ++i) {
    for (int j = 0; j != 5; ++j) {
      if (i == 0) {
        file >> k_array[j];
      } else {
        file >> G_camera_image[i - 1][j];
      }
    }
    file.close();
    fx = k_array[0];
    fy = k_array[1];
    cx = k_array[2];
    cy = k_array[3];
    updateKMatrix();
  }
}
void Camera::updateKMatrix() {
  K_matrix = {{fx, 0, cx}, {0, fy, cy}, {0, 0, 1}};
}
