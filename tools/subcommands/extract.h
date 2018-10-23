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

  struct ExtractOptions {
    std::vector< std::string > inFiles;
    std::string boardFile;
    std::string databaseName;

    std::string annotationDir;
  };


  class Extract {
  public:

    static void SetupSubcommand( CLI::App &app );
    static void Run( ExtractOptions const &opts );

    Extract( ExtractOptions const &opts );
    ~Extract();

    void run( void );

  private:

    AplCam::Board *loadBoard( const std::string &board );

    void processFrame( const cv::Mat &img, const std::string &tag );

    ExtractOptions const &_opts;

    std::unique_ptr< AplCam::Board > _board;
    std::unique_ptr< AplCam::InMemoryDetectionDb > _db;

  };


}
