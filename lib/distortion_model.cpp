
#include <opencv2/imgproc/imgproc.hpp>

#include "distortion_model.h"

namespace Distortion {

  using namespace cv;
  using namespace std;

  void DistortionModel::undistortImage( const Mat &distorted, Mat &undistorted,
      const Mat &Knew, const Size& new_size)
  {
    Size size = new_size.area() != 0 ? new_size : distorted.size();

    Mat map1, map2;
    initUndistortRectifyMap(Mat(cv::Matx33d::eye()), Knew, size, CV_16SC2, map1, map2 );
    remap(distorted, undistorted, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
  }

}

