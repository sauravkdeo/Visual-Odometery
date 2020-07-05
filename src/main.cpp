#include <camera.h>
#include <visual_odometery.h>

#include <iostream>

int main() {
  std::string img_path = "dataset/stereo/centre/*.png";

  std::string intrinsics_path = "./dataset/model/stereo_narrow_left.txt";
  std::string lut_path =
      "./dataset/model/stereo_narrow_left_distortion_lut.bin";
  Camera cam(intrinsics_path, lut_path);
  Visual_Odometery vo(img_path);
  vo.SfMAlgorithm();

  return 0;
}
