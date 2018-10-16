
#include <opencv2/core.hpp>

#include "extract.h"

#include "input_queue.h"


namespace calibration {

  void Extract::SetupSubcommand( CLI::App &app ) {
    // Create the option and subcommand objects.
    auto opt = std::make_shared<ExtractOptions>();
    auto sub = app.add_subcommand("extract", "extract fiducial marks");

    sub->add_option("infiles", opt->inFiles, "Input files (movies or images)");
    sub->add_option("-b,--board", opt->boardName, "Name of calibration board");
    sub->add_option("-d,--database", opt->databaseName, "Name of database");


    sub->set_callback([opt]() { Extract::Run(*opt); });
  }

  void Extract::Run( ExtractOptions const &opts ) {
    Extract extract(opts);
    extract.run();
  }

  //======================================================================

  Extract::Extract( ExtractOptions const &opts )
    : _opts(opts)
    {;}

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



    }

  }

}
