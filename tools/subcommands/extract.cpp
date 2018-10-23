
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


#include "AplCam/detection/detection.h"

#include "extract.h"
#include "input_queue.h"


namespace calibration {
  using namespace AplCam;

  void Extract::SetupSubcommand( CLI::App &app ) {
    // Create the option and subcommand objects.
    auto opt = std::make_shared<ExtractOptions>();
    auto sub = app.add_subcommand("extract", "extract fiducial marks");

    sub->add_option("infiles", opt->inFiles, "Input files (movies or images)");
    sub->add_option("-b,--board", opt->boardFile, "Name of calibration board");
    sub->add_option("-d,--database", opt->databaseName, "Name of database");
    sub->add_option("--annotate", opt->annotationDir, "Directory for annotated images");

    sub->set_callback([opt]() { Extract::Run(*opt); });
  }

  void Extract::Run( ExtractOptions const &opts ) {


    Extract extract(opts);
    extract.run();
  }

  //======================================================================

  Extract::Extract( ExtractOptions const &opts )
    : _opts(opts),
      _board( loadBoard( _opts.boardFile ) ),
      _db( new InMemoryDetectionDb( _opts.databaseName ) )
    {
      CHECK( _board != nullptr ) << "Unable to load board from " << opts.boardFile;
      CHECK( _db != nullptr ) << "Unable to open database \"" << opts.databaseName << "\"";
    }

    Extract::~Extract()
    {
      ;
    }

  void Extract::run() {
    LOG(WARNING) << "In Extract::run";

    camera_calibration::InputQueue queue( _opts.inFiles );


    cv::Mat img;
    while( queue.nextFrame(img) ) {
      LOG(WARNING) << "Loading " << queue.frameName();
      if( img.empty() ) {
        LOG(WARNING) << "Read empty mat";
        continue;
      }

      processFrame( img, queue.frameName() );
    }

    _db->sync();
  }


  Board *Extract::loadBoard( const std::string &boardFile ) {
    return Board::load( boardFile, fs::path( boardFile ).stem().string() );
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

}
