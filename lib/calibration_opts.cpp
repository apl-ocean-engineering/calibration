
#include "calibration_opts.h"

CalibrationOpts::CalibrationOpts()
  : calibrationDb(),
  calibrationFile(),
  detectionDb(),
  saveBoardPoses(),
  dataDir("../data"),
  boardName(),
  cameraName(),
  calibType( DistortionModel::CALIBRATION_NONE ),
  overwriteDb( false ),
  huberLoss( false ),
  fixSkew( true )
{;}

bool CalibrationOpts::parseOpts( int argc, char **argv )
{

  try {
    TCLAP::CmdLine cmd("calibration", ' ', "0.1" );
    doParseCmdLine( cmd, argc, argv );
  } catch( TCLAP::ArgException &e ) {
    LOG( ERROR ) << "error: " << e.error() << " for arg " << e.argId();
    return false;
  }

  return validateOpts();
}

void CalibrationOpts::doParseCmdLine( TCLAP::CmdLine &cmd, int argc, char **argv )
{

  TCLAP::ValueArg< std::string > calibrationDbArg( "Z", "calibration-db", "Calibration Db", false, "", "db name", cmd );
  TCLAP::ValueArg< std::string > calibrationFileArg( "z", "calibration-file", "Calibration Db", false, "", "db name", cmd );
  TCLAP::ValueArg< std::string > dataDirArg( "d", "data-dir", "Data dir", false, "data/", "dir", cmd );
  TCLAP::ValueArg< std::string > detectionDbArg( "D", "detection-db", "Detection Db", false, "", "db name", cmd );
  TCLAP::ValueArg< std::string > boardNameArg( "b", "board-name", "Board name", true, "", "board name", cmd );
  TCLAP::ValueArg< std::string > cameraNameArg("c", "camera-name", "Camera name", true, "", "camera name", cmd );
  TCLAP::ValueArg< std::string > calibTypeArg("m", "calibration-model", "Calibration mode", true, "", "{radial|angular}", cmd );

  TCLAP::SwitchArg overWriteDbArg("y", "overwrite-db", "Overwriting existing entries in calibration db", cmd, false );
  TCLAP::SwitchArg saveBoardPosesArg("S", "save-board-poses", "Save board poses back to detection db", cmd, false );
  TCLAP::SwitchArg doValidateArg("V", "dont-validate", "Don't validate data", cmd, true );
  TCLAP::SwitchArg huberLossArg("H", "huber-loss", "Use Huber loss function", cmd, false );

  cmd.parse( argc, argv );

  calibrationDb = calibrationDbArg.getValue();
  calibrationFile = calibrationFileArg.getValue();
  dataDir = dataDirArg.getValue();
  detectionDb = detectionDbArg.getValue();
  boardName = boardNameArg.getValue();
  cameraName = cameraNameArg.getValue();
  overwriteDb = overWriteDbArg.getValue();
  saveBoardPoses = saveBoardPosesArg.getValue();
  doValidate = doValidateArg.getValue();
  huberLoss  = huberLossArg.getValue();

  calibType = DistortionModel::ParseDistortionModel( calibTypeArg.getValue() );
}



//static struct option long_options[] = {
//  { "data-directory", true, NULL, 'd' },
//  { "board", true, NULL, 'b' },
//  { "camera", true, NULL, 'c' },
//  { "calibation-model", true, NULL, 'm' },
//  { "fix-skew", false, NULL, 'k'},
//  { "save-board-poses", required_argument, NULL, 'S' },
//  { "calibration-db", required_argument, NULL, 'Z' },
//  { "detection-db", required_argument, NULL, 'D' },
//  { "help", false, NULL, '?' },
//  { 0, 0, 0, 0 }
//};


//if( argc < 2 )
//{
//  help();
//  return false;
//}

//int indexPtr;
//int optVal;
//string c;

//// The '+' option ensures it stops on the first non-conforming option. Required for the
////   cmd opt1 opt2 opt3 verb verb_opt1 files ...
//// pattern I'm using
//while( (optVal = getopt_long( argc, argv, "+yZ:RSrb:c:d:km:?", long_options, &indexPtr )) != -1 ) {
//  switch( optVal ) {
//    case 'Z':
//      calibrationDb = optarg;
//      break;
//    case 'd':
//      dataDir = optarg;
//      break;
//    case 'D':
//      detectionDb = optarg;
//      break;
//    case 'b':
//      boardName = optarg;
//      break;
//    case 'c':
//      cameraName = optarg;
//      break;
//    case 'k':
//      //calibFlags |= PinholeCamera::CALIB_FIX_SKEW;
//      LOG(INFO) << "Skew is always fixed." << endl;
//      break;
//    case 'y':
//      overwriteDb = true;
//      break;
//    case 'S':
//      saveBoardPoses = optarg;
//      break;
//    case 'm':
//      calibType = DistortionModel::ParseCalibrationType( optarg );
//      break;
//    case '?':
//      help();
//      break;
//    default:
//      return false;

//  }
//}


bool CalibrationOpts::validateOpts( void )
{
  if( !calibrationDb.empty() ) {
    if( !calibrationFile.empty() ) {
      LOG(ERROR) << "Can't set both calibration file and calibration db";
      return false;
    }
  }

  if( boardName.empty() ) {
    LOG(ERROR) << "Board name not set";
    return false;
  }

  if( cameraName.empty() ) { LOG(ERROR) << "Camera name not set";
    return false;
  }

  if( calibType == DistortionModel::CALIBRATION_NONE ) {
    LOG(ERROR) << "Calibration type not specified";
    return false;
  }

  //   if( calibrationFile.empty() ) calibrationFile = cameraPath( mkCameraFileName() );

  //   return true;
  // }

  // The super will auto-fill calibrationFile if not set
if( !calibrationDb.empty() ) calibrationFile.clear();

return validate();
}
