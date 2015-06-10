#include <cctype>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <getopt.h>

#include <iostream>

#include <tclap/CmdLine.h>
#include <glog/logging.h>

#include "file_utils.h"
#include "board.h"
#include "detection.h"
#include "detection_set.h"
#include "image.h"

#include "distortion_model.h"
#include "camera_factory.h"
using namespace Distortion;

#include "calibration_db.h"
#include "calibrator.h"
using namespace AplCam;

using namespace cv;
using namespace std;

using kyotocabinet::HashDB;
using kyotocabinet::DB;

class Options {

  public:

    Options( )
    {;}

    typedef enum { MODE_REDUCE, MODE_EACH } Mode_t;

    string calibrationDb, outputFile;
    Mode_t mode;

    bool parseOpts( int argc, char **argv )
    {
      try {
        TCLAP::CmdLine cmd("calibration_db_dump", ' ', "0.1" );

        TCLAP::ValueArg< std::string > calibrationDbArg("c", "calibration-db", "CalibrationSet db", true, "", "CalibrationSet db file", cmd );
        TCLAP::ValueArg< std::string > outputFileArg("o", "output-file", "Output file", false, "", "Output file", cmd );

        TCLAP::ValueArg< std::string > modeArg("m", "mode", "Mode", true, "", "{reduce|each}", cmd );

        cmd.parse( argc, argv );

        calibrationDb = calibrationDbArg.getValue();
        outputFile    = outputFileArg.getValue();

        string modeStr( modeArg.getValue() );
        if( modeStr.compare( "reduce" ) == 0 ) {
          mode = MODE_REDUCE;
        } else if( modeStr.compare( "each" ) == 0 ) {
          mode = MODE_EACH;
        } else {
          LOG(ERROR) << "Don't understand mode \"" << modeStr << "\"";
          throw TCLAP::ArgException("");
        }


      } catch( TCLAP::ArgException &e ) {
        LOG( ERROR ) << "error: " << e.error() << " for arg " << e.argId();
      }

      return validateOpts();
    }


    bool checkForDb( const string &db, const string &name, string &msg )
    {
      if( db.empty() ) {
        msg = string("Must specify ") + name + " db";
        return false;
      }

      if( !file_exists( db ) ) {
        msg = string("Can't find ") + name + " db " + db;
        return false;
      }

      return true;
    }

    virtual bool validateOpts( void  )
    {
      if( !file_exists( calibrationDb ) ) {
        LOG(ERROR) << "Can't find calibration db \"" << calibrationDb << "\"";
        return false;
      }

      return true;
    }

};


// Inefficient
static double mean( const vector<double> &v ) 
{
  double total;
  for( size_t i = 0;  i < v.size(); ++i ) total += v[i];

  return total/v.size();
}

static double stdDev( const vector<double> &v, double mean )
{
  if( v.size() < 2 ) return 0.0;

  double total;

  for( size_t i = 0;  i < v.size(); ++i ) {
    double d = (v[i] - mean);
    total += d*d;
  }

  return sqrt( total/v.size() );
}

static double stdDev( const vector<double> &v )
{ 
  return stdDev( v, mean(v) );
}



// Technically speaking I should write a CalibrationResult unserializer
// Rather than tracking rms and totaltime separately.
class Calibrations {
  public:
    Calibrations( void )
      : _rms(), _totalTime(), _bad(0), _count(0)
    {;}

    virtual ~Calibrations()
    {
      for( size_t i = 0; i < _cameras.size(); ++i ) delete _cameras[i];
    }

    void add( const string &rec )
    {
      // Quick and dirty
      FileStorage fs( rec, FileStorage::READ | FileStorage::MEMORY );
      add( fs );
    }

    void add( FileStorage &fs )
    {
      _count++;

      if( !fs["success"].empty() && ((int)fs["success"] == 1) ) {

        if( !fs["rms"].empty() ) _rms.push_back( fs["rms"] );
        if( !fs["totalTime"].empty() ) _totalTime.push_back( fs["totalTime"] );

        _cameras.push_back( CameraFactory::Unserialize( fs ) );
      } else {
        ++_bad;
      }
    }

    string reduceString( void )
    {
      stringstream strm;

      strm << _count <<  " "  << (_count-_bad) << " " << mean( _rms ) << " " << stdDev( _rms );
      if( _totalTime.size() > 0 )
        strm << " " << mean( _totalTime ) << " " << stdDev( _totalTime );
      return strm.str();
    }


    void eachString( ostream &strm )
    {
      for( size_t i = 0; i < _cameras.size(); ++i ) {

        strm << i << " "
             << _cameras[i]->fx() << " " << _cameras[i]->fy() << " "
             << _cameras[i]->cx() << " " << _cameras[i]->cy() << " ";


        Mat coeff = _cameras[i]->distortionCoeffs();
        for( int j = 0; j < coeff.rows * coeff.cols; ++j ) {
          double *ptr = coeff.ptr<double>();
          strm << ptr[j] << " ";
        }

        strm << endl;

      }
    }

  protected:
    vector< DistortionModel * > _cameras;
    vector< double > _rms;
    vector< double > _totalTime;

    int _bad, _count;

};




class CalibrationSet {
  public:
    CalibrationSet()
    {;}

    virtual size_t size( void ) const = 0;
    virtual bool add( const string &key, const string &value ) = 0;
    virtual void dumpReduced( ostream &strm ) = 0;

    virtual void dumpEach( ostream &strm ) = 0;
};

class AllCalibrationSet : public CalibrationSet {
  public:
    AllCalibrationSet()
      : _datum(), _set( false )
    {;}

    virtual size_t size( void ) const { return _set ? 1 : 0; }

    virtual bool add( const string &key, const string &value )
    {
      if( key.compare("all") != 0 ) return false;
      if( _set ) return false;


      FileStorage fs( value, FileStorage::READ | FileStorage::MEMORY );
      fs["numImages"] >> _numImages;

      _datum.add( fs );
      _set = true;

      return true;
    }

    virtual void dumpReduced( ostream &strm ) 
    {
      strm << "0 " << _datum.reduceString() << endl;
      strm << _numImages << " " << _datum.reduceString() << endl;
    }

    virtual void dumpEach( ostream &strm )
    {
      if( _set ) _datum.eachString( strm );
    }

  protected: 
    Calibrations _datum;
    bool _set;
    int _numImages;
};

class RandomCalibrationSet : public CalibrationSet {
  public:
    RandomCalibrationSet()
      : CalibrationSet(),_data()
    {;}

    virtual size_t size( void ) const { return _data.size(); }

    virtual bool add( const string &key, const string &value )
    {
      if( key.compare(0,6,"random") != 0 ) return false;

      unsigned int count;

      if( sscanf( key.c_str(), "random(%u)", &count ) == 1 ) {
        _data[count].add( value );
        return true;
      }
      return false;
    }

    virtual void dumpReduced( ostream &strm ) 
    {
      for( map< unsigned int, Calibrations >::iterator itr = _data.begin(); itr != _data.end(); ++itr ) {
        strm << itr->first <<  " " << itr->second.reduceString() << endl;
      }
    }

    virtual void dumpEach( ostream &strm )
    {
      for( map< unsigned int, Calibrations >::iterator itr = _data.begin(); itr != _data.end(); ++itr ) {
        strm << "#" << endl;
        strm << "# " << "random_" << itr->first << endl;
        strm << "#" << endl;
        itr->second.eachString( strm );
      }
    }


  protected: 
    map< unsigned int, Calibrations > _data;
};


class IntervalCalibrationSet : public RandomCalibrationSet {
  public:
    IntervalCalibrationSet()
      : RandomCalibrationSet()
    {;}

    const int roundTo = 25;

    virtual bool add( const string &key, const string &value )
    {
      if( key.compare(0,8,"interval") != 0 ) return false;

      unsigned int interval, start, end;
      // Cheat
      int c  = scanf( key.c_str(), "interval(%u,%u,%u", &start, &interval, &end );
      if( c == 3 ) {
        int count =  floor((end - start)/interval );


        int c = round( (float)count / roundTo ) * roundTo;

        _data[c].add( value );

        return true;
      }
      return false;
    }

    virtual void dumpEach( ostream &strm )
    {
      for( map< unsigned int, Calibrations >::iterator itr = _data.begin(); itr != _data.end(); ++itr ) {
        strm << "#" << endl;
        strm << "# " << "interavl_" << itr->first << endl;
        strm << "#" << endl;
        itr->second.eachString( strm );
      }
    }
};


class DumpMain {
  public:
    DumpMain( Options &opt_ ) :
      opts( opt_ )
  {;}

    int run( void )
    {

      LOG(INFO) << "Opening calibration db: " << opts.calibrationDb;
      if( calibrations.open( opts.calibrationDb ) == false ) {
        LOG(ERROR) << "Error opening calibrations db: " << calibrations.error().name();
        return -1;
      }

      DB::Cursor *cur = calibrations.cursor();
      cur->jump();

      LOG(INFO) << "Loading calibrations.";

      string key, value;
      while( cur->get( &key, &value, true ) ) {


        bool result;
        if( key.compare( 0, 3, "all" ) == 0 )
          result = allCals.add( key, value );
        else if( key.compare( 0, 6, "random" ) == 0 )
          result = randomCals.add( key, value );
        else if( key.compare( 0, 8, "interval" ) == 0 )
          result = intervalCals.add( key, value );

        if( !result ) LOG(ERROR) << "Failed to add the calibration with key " << key;

      }


      LOG(INFO) << "Have " << allCals.size() << " all calibrations.";
      LOG(INFO) << "Have " << randomCals.size() << " random calibrations.";
      LOG(INFO) << "Have " << intervalCals.size() << " interval calibrations.";

      LOG(INFO) << "Preparing output.";

std::streambuf *buf;
      std::ofstream of;

      if( opts.outputFile.empty() ) {
            buf = std::cout.rdbuf();
      } else {
            of.open( opts.outputFile );
                buf = of.rdbuf();
      }

      ostream out(buf);

      switch( opts.mode ) {
        case Options::MODE_REDUCE: 
          return doReduce( out );
          break;
        case Options::MODE_EACH:
          return doEach( out );
          break;
      }

      delete cur;

      return 0;
    }




    int doReduce( ostream &out )
    {
      

      out << "# num_points num_reps num_good_reps rms_mean rms_stddev time_mean time_stddev" << endl;
      out << endl;
      if( allCals.size() > 0 ) {
        out << "# all" << endl;
        allCals.dumpReduced( out );
      }

      if( randomCals.size() > 0 ) {
        out << endl;
        out << "# random" << endl;
        randomCals.dumpReduced( out );
      }

      if( intervalCals.size() > 0 ) {
        out << endl;
        out << "# interval" << endl;
        intervalCals.dumpReduced( out );
      }

      return 0;
    }


    int doEach( ostream &out )
    {
      allCals.dumpEach( out );
      randomCals.dumpEach( out );
      intervalCals.dumpEach( out );

      return 0;
    }



  protected:

    Options &opts;

    CalibrationDb calibrations;

    AllCalibrationSet allCals;
    RandomCalibrationSet randomCals;
    IntervalCalibrationSet intervalCals;



};






int main( int argc, char** argv )
{
  google::InitGoogleLogging("video_calibration_permutation");
  FLAGS_logtostderr = 1;

  Options opts;
  if( !opts.parseOpts( argc, argv ) )  exit(-1);

  DumpMain main( opts );

  return main.run();
}
