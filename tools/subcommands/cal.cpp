
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


#include "AplCam/detection/detection.h"
#include "AplCam/distortion/distortion_model.h"

#include "cal.h"
#include "input_queue.h"


namespace calibration {
  using namespace AplCam;
  using namespace Distortion;

  CLI::App *Cal::SetupSubcommand( CLI::App &app ) {
    // Create the option and subcommand objects.
    auto opt = std::make_shared<CalOptions>();
    auto sub = app.add_subcommand("calibrate", "Calibrate based on database of fiducial marks");

    sub->add_option("-d,--database", opt->databaseName, "Name of database")->required();
    sub->add_option("-o,--output", opt->outputPath, "Name of JSON file for calibration");
    sub->add_option("-m,--model", opt->distortionModel, "Name of distortion model")->required()->check( DistortionModel::ValidateModelName );

    sub->set_callback([opt]() { Cal::Run(*opt); });

    return sub;
  }

  void Cal::Run( CalOptions const &opts ) {
    Cal Cal(opts);
    Cal.run();
  }

  //======================================================================

  Cal::Cal( CalOptions const &opts )
    : _opts(opts),
      _db( new InMemoryDetectionDb( _opts.databaseName ) )
    {
      CHECK( _db != nullptr ) << "Unable to open database \"" << opts.databaseName << "\"";
    }

    Cal::~Cal()
    {
      ;
    }

  void Cal::run() {
    LOG(WARNING) << "In Cal::run";

    std::shared_ptr<DistortionModel> model( DistortionModel::MakeDistortionModel( _opts.distortionModel ) );
    CHECK( (bool)model ) << "Unable to create distortion model " << _opts.distortionModel;

    ImagePointsVecVec imagePoints;
    ObjectPointsVecVec objectPoints;

    for( auto const &detection : _db->map() ) {
      ImagePointsVec ptsVec;
      std::copy( detection.second->points.begin(), detection.second->points.end(), std::back_inserter( ptsVec ) );
      imagePoints.push_back(ptsVec);

      ObjectPointsVec objVec;
      std::copy( detection.second->corners.begin(), detection.second->corners.end(), std::back_inserter( objVec ) );
      objectPoints.push_back(objVec);
    }

    CalibrationResult result;

    // Need to get this from database meta
    cv::Size imageSize( 1920,1080 );
    int flags = 0;

    model->calibrate( objectPoints, imagePoints, imageSize, result, flags );

  }

}
