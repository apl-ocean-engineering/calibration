#pragma once

#include <vector>
#include <string>

#include <boost/filesystem/path.hpp>

#include <CLI/CLI.hpp>

#include "active_object/shared_queue.h"

#include "libg3logger/g3logger.h"

#include "AplCam/detection_db.h"
#include "AplCam/board/board.h"

namespace calibration {

  namespace fs = boost::filesystem;

  struct ExtractOptions {
    ExtractOptions()
      : parallelism(1) {;}

    std::vector< std::string > inFiles;
    std::string boardFile;
    std::string databaseName;

    std::string annotationDir;

    size_t parallelism;
  };


  class Extract {
  public:

    static CLI::App *SetupSubcommand( CLI::App &app );
    static void Run( ExtractOptions const &opts );

    Extract( ExtractOptions const &opts );
    ~Extract();

    void run( void );

  private:

    AplCam::Board *loadBoard( const std::string &board );

    void processFrame( const cv::Mat &img, const std::string &tag );

    void processFramesInQueue();

    ExtractOptions const &_opts;

    std::unique_ptr< AplCam::Board > _board;
    std::unique_ptr< AplCam::InMemoryDetectionDb > _db;


    //== Member variables for handling parallel processing ==

    struct QueueWork {
      QueueWork()
        : mat(), name("") {;}

      QueueWork( const cv::Mat &m, const std::string &n )
        : mat(m), name(n) {;}

      cv::Mat mat;
      std::string name;
    };

    std::condition_variable _threadStart;
    std::mutex _threadStartMutex;

    bool _threadDone;

    active_object::shared_queue< QueueWork > _workqueue;

  };


}
