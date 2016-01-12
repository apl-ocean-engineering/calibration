#ifndef __INPUT_QUEUE_H__
#define __INPUT_QUEUE_H__

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <vector>
#include <string>

#include <opencv2/core.hpp>

namespace CameraCalibration {

class InputQueue {
public:
  InputQueue( const std::vector<fs::path> &files );

  cv::Mat nextFrame( void );
  std::string frameName( void );

protected:

  const std::vector<fs::path> &_files;
  int _idx;

};

}


#endif
