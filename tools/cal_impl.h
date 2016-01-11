#ifndef __CAL_IMPL_H__
#define __CAL_IMPL_H__

#include <vector>

#include <opencv2/core.hpp>

#include <tclap/CmdLine.h>

#include <glog/logging.h>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "board.h"
#include "detection_io.h"

using namespace std;
using namespace AplCam;
using namespace CameraCalibration;

class CalOpts {
public:
  CalOpts();
  ~CalOpts() {;}

  vector< fs::path > inFiles;
  fs::path detectionOutput, boardPath;
  bool doDetect;

  bool parseOpts( int argc, char **argv );

protected:

  void doParseCmdLine( TCLAP::CmdLine &cmd, int argc, char **argv );

  bool validateOpts();

};


class InputQueue {
public:
  InputQueue( const vector<fs::path> &files );

  cv::Mat nextFrame( void );
  std::string frameName( void );

protected:

  const vector<fs::path> &_files;
  int _idx;

};

class Cal {
  public:
    Cal( CalOpts &opts );
    ~Cal();

    int run( void );

    // Sub-commands
    void doDetect( void );


  protected:

    // Eager-load
    Board *board( void );
    DetectionIO *detectionIO( void );


  private:

   CalOpts _opts;
   InputQueue _inputQueue;
   Board *_board;
   DetectionIO *_detectionIO;
};



#endif
