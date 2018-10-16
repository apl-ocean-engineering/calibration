#pragma once

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

namespace camera_calibration {

class InputQueue {
public:
  InputQueue( const std::vector<fs::path> &files );
  InputQueue( const std::vector<std::string> &files );


  bool nextFrame( cv::Mat &mat );
  std::string frameName( void );

protected:

  std::vector<fs::path> _files;
  int _idx;

};

}
