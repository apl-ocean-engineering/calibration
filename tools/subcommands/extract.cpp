
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


#include "AplCam/detection/detection.h"

#include "extract.h"
#include "input_queue.h"


namespace calibration {
  using namespace AplCam;

  CLI::App *Extract::SetupSubcommand( CLI::App &app ) {
    // Create the option and subcommand objects.
    auto opt = std::make_shared<ExtractOptions>();
    auto sub = app.add_subcommand("extract", "extract fiducial marks");

    sub->add_option("infiles", opt->inFiles, "Input files (movies or images)")->required();
    sub->add_option("-b,--board", opt->boardFile, "Name of calibration board")->required();
    sub->add_option("-d,--database", opt->databaseName, "Database file");
    sub->add_option("--annotate", opt->annotationDir, "Directory for annotated images");

    sub->add_option("-j,--jobs", opt->parallelism, "Number of threads (specify 0 for number of cores)");

    sub->callback([opt]() { Extract::Run(*opt); });

    return sub;
  }

  void Extract::Run( const ExtractOptions &opts ) {
    Extract extract(opts);

    LOG(INFO) << "Database name: " << opts.databaseName;
    extract.run();
  }

  //======================================================================

  Extract::Extract( const ExtractOptions &opts )
    : _opts(opts),
      _board( loadBoard( opts.boardFile ) ),
      _db( new JsonDetectionDb( opts.databaseName ) )
  {
    CHECK( _board != nullptr ) << "Unable to load board from " << opts.boardFile;
    CHECK( _db != nullptr ) << "Unable to open database \"" << opts.databaseName << "\"";
    LOG_IF(DEBUG, !_opts.databaseName.empty() ) << "Saving data to " << _db->filename() << " : " << opts.databaseName;
  }

  Extract::~Extract()
  {
    ;
  }


  Board *Extract::loadBoard( const std::string &boardFile ) {
    return Board::load( boardFile, fs::path( boardFile ).stem().string() );
  }


  void Extract::run() {
    LOG(DEBUG) << "In Extract::run";

    camera_calibration::InputQueue queue( _opts.inFiles );
    int imgWidth = 0, imgHeight = 0;

    cv::Mat img;

    if( _opts.parallelism == 1 ) {
      while( queue.nextFrame(img) ) {
        if( imgWidth == 0 ) {
          imgWidth = img.size().width;
          imgHeight = img.size().height;
        }

        LOG(WARNING) << "Loading " << queue.frameName();
        if( img.empty() ) {
          LOG(WARNING) << "Read empty mat";
          continue;
        }

        processFrame( img, queue.frameName() );
      }
    } else {
      //== Multi-threaded version ==
      int parallelism = _opts.parallelism;

      if( parallelism == 0 ) {
        parallelism = std::thread::hardware_concurrency();
        LOG(INFO) << "Auto-detected " << parallelism << " cores";
      }
      CHECK( parallelism != 0 ) << "Unable to automatically detect the number of cores";

      // Prequeue a few jobs
      const size_t prequeue = 2 * parallelism;
      size_t pq = 0;
      while( queue.nextFrame(img) && pq < prequeue ) {
        if( imgWidth == 0 ) {
          imgWidth = img.size().width;
          imgHeight = img.size().height;
        }

        _workqueue.push( QueueWork( img, queue.frameName() ) );
        ++pq;
      }

      // Spin up threads
      std::vector< std::thread > threads(parallelism);
      _threadDone = false;

      LOG(INFO) << "Spawning " << parallelism << " extraction threads";
      //std::fill( threads.begin(), threads.end(), std::thread( std::bind( &Extract::processFramesInQueue, this ) ) );
      for( size_t i = 0; i < threads.size(); ++i ) {
        threads[i] = std::thread( std::bind( &Extract::processFramesInQueue, this ) );
      }

      CHECK( threads.size() > 0 ) << "Funny, I didn't spawn any threads...";

      // LOG(DEBUG) << "Notify all";
      // _threadStart.notify_all();

      while( queue.nextFrame(img) ) {
        _workqueue.push( QueueWork( img, queue.frameName() ) );
      }


      _threadDone = true;

      LOG(INFO) << "Reaping all threads";
      for( auto &t : threads ) {
        t.join();
      }


    }

    // Set db meta
    _db->setMeta( cv::Size(imgWidth, imgHeight) );

    if( !_db->filename().empty() ) _db->save();
  }


  void Extract::processFrame( const cv::Mat &img, const std::string &tag ) {
    //Mat mask = Mat::ones( img.size(), CV_8UC1 );
    //prepFrame( frame, mask );

    std::shared_ptr<Detection> detection( _board->detectPattern( img ) );

    _db->insert( tag, detection );

    if( !_opts.annotationDir.empty() ) {
      fs::path imgPath( _opts.annotationDir );
      imgPath /= tag + ".jpg";
      LOG(INFO) << "Annotating image to " << imgPath;

      cv::Mat annotateImg( img );
      detection->draw( annotateImg );

      cv::imwrite( imgPath.string(), annotateImg );
    }

  }

  void Extract::processFramesInQueue() {

    LOG(DEBUG) << "Processing in thread " << std::this_thread::get_id();
    // {
    //   std::unique_lock<std::mutex> lock(_threadStartMutex);
    //   _threadStart.wait( lock );
    // }

    //LOG(DEBUG) << " ... processing in thread";

    QueueWork work;

    while( true ) {
      if( _workqueue.wait_for_pop( work, std::chrono::seconds(1) ) ) {
        processFrame( work.mat, work.name );
      } else {
        if( _threadDone ) break;
      }
    }

    LOG(DEBUG) << "Exiting thread";

  }


}
