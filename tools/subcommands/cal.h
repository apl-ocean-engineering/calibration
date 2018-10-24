#pragma once

#include <vector>
#include <string>

#include <boost/filesystem/path.hpp>

#include <CLI/CLI.hpp>

#include "libg3logger/g3logger.h"

#include "AplCam/detection_db.h"
#include "AplCam/board/board.h"

namespace calibration {

  namespace fs = boost::filesystem;

  struct CalOptions {
    std::string databaseName;
    std::string outputPath;
    std::string distortionModel;
  };


  class Cal {
  public:

    static CLI::App *SetupSubcommand( CLI::App &app );
    static void Run( CalOptions const &opts );

    Cal( CalOptions const &opts );
    ~Cal();

    void run( void );

  private:

    // AplCam::Board *loadBoard( const std::string &board );
    //
    // void processFrame( const cv::Mat &img, const std::string &tag );

    CalOptions const &_opts;

    // std::unique_ptr< AplCam::Board > _board;
    std::unique_ptr< AplCam::InMemoryDetectionDb > _db;

  };


}
