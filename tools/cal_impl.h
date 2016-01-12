#ifndef __CAL_IMPL_H__
#define __CAL_IMPL_H__

#include <vector>

#include <opencv2/core.hpp>

#include <tclap/CmdLine.h>

#include <glog/logging.h>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "AplCam/board/board.h"
#include "AplCam/distortion/distortion_model.h"

#include "detection_io.h"
#include "input_queue.h"

using namespace std;
using namespace AplCam;
using namespace Distortion;
using namespace CameraCalibration;


class CalOpts {
public:
  CalOpts();
  ~CalOpts() {;}

  vector< fs::path > inFiles;

  fs::path boardPath;

  bool doDetect;
  fs::path detectionOutput, drawDetectionPath;

  bool doCalibrate;
  DistortionModel::DistortionModelType_t distortionModel;

  bool parseOpts( int argc, char **argv );

protected:

  void doParseCmdLine( TCLAP::CmdLine &cmd, int argc, char **argv );
  bool validateOpts();

};




class Cal {
  public:
    Cal( CalOpts &opts );
    ~Cal();

    int run( void );

    // Sub-commands
    void doDetect( void );
    void doCalibrate( void );

  protected:

    void drawDetection( const cv::Mat &img, Detection *detection );

    // Eager-load
    Board *board( void );
    DetectionIO *detectionIO( void );
    DistortionModel *model( void );

  private:

    CalOpts _opts;
    InputQueue _inputQueue;
    Board *_board;
    DetectionIO *_detectionIO;
    DistortionModel *_model;
};



#endif
