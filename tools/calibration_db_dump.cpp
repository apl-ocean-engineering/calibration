#include <cctype>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <getopt.h>

#include <iostream>

#include "file_utils.h"
#include "board.h"
#include "detection.h"
#include "detection_set.h"
#include "image.h"

#include "distortion_model.h"
#include "camera_factory.h"
using namespace Distortion;

#include "calibration_db.h"
#include "calibration_opts_common.h"
#include "calibrator.h"
using namespace AplCam;

#include "video_splitters/video_splitter_opts.h"
#include "video_splitters/video_splitters.h"
using namespace AplCam::VideoSplitters;


using namespace cv;
using namespace std;

using kyotocabinet::HashDB;
using kyotocabinet::DB;

class Options {

  public:

    Options( )
      : resultsDb(), saveTo()
    {;}

    string resultsDb, saveTo;

    //== Option parsing and help ==
    string help( void )
    {
      stringstream strm;

      return strm.str();
    }


    bool parseOpts( int argc, char **argv, string &msg )
    {
      stringstream msgstrm;

      static struct option long_options[] = {
        { "results-db", required_argument, NULL, 'R' },
        { "save-to", required_argument, NULL, 'S' },
        { "help", false, NULL, '?' },
        { 0, 0, 0, 0 }
      };


      int indexPtr;
      int optVal;
      string c;

      // The '+' option ensures it stops on the first non-conforming option. Required for the
      //   cmd opt1 opt2 opt3 verb verb_opt1 files ...
      // pattern I'm using
      while( (optVal = getopt_long( argc, argv, "R:S:?", long_options, &indexPtr )) != -1 ) {
        switch( optVal ) {
          case 'R':
            resultsDb = optarg;
            break;
          case 'S':
            saveTo = optarg;
            break;
          case '?': 
            cout << help() << endl;;
            return false;
            break;
          default:
            return false;

        }
      }

      if( !validate( msg ) ) return false;

      return true;
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

    virtual bool validate( string &msg )
    {

      if( !checkForDb( resultsDb, "results", msg ) ) return false;

      return true;
    }

};


class Datum {
  public:
    Datum( void )
      : _rms()
    {;}

    void add( const string &rec )
    {
      // Quick and dirty
      FileStorage fs( rec, FileStorage::READ | FileStorage::MEMORY );
      if( !fs["rms"].empty() ) _rms.push_back( fs["rms"] );
    }

    string toString( void )
    {
      stringstream strm;

      strm << _rms.size() <<  " " << mean( _rms ) << " " << stdDev( _rms );

      return strm.str();
    }

    // Inefficient
    double mean( const vector<double> &v ) const
    {
      double total;
      for( size_t i = 0;  i < v.size(); ++i ) total += v[i];

      return total/v.size();
    }

    double stdDev( const vector<double> &v ) const 
    {
      if( v.size() < 2 ) return 0.0;

      double m = mean( v );
      double total;

      for( size_t i = 0;  i < v.size(); ++i ) {
        double d = (v[i] - m);
        total += d*d;
      }

      return sqrt( total/v.size() );
    }

  protected:
    vector< double > _rms;

};

class Calibration {
  public:
    Calibration()
    {;}

    virtual bool add( const string &key, const string &value ) = 0;
    virtual void dump( ostream &strm ) = 0;
};

class AllCalibration : public Calibration {
  public:
    AllCalibration()
      : _datum(), _set( false )
    {;}

    virtual bool add( const string &key, const string &value )
    {
      if( key.compare("all") != 0 ) return false;
      if( _set ) return false;

      _datum.add( value );
      _set = true;

      return true;
    }

    virtual void dump( ostream &strm ) 
    {
      strm << "all " << _datum.toString() << endl;
    }

  protected: 
    Datum _datum;
    bool _set;
};

class RandomCalibration : public Calibration {
  public:
    RandomCalibration()
      : _data()
    {;}

    virtual bool add( const string &key, const string &value )
    {
      if( key.compare(0,6,"random") != 0 ) return false;

      unsigned int count;
      // Cheat
      if( sscanf( key.c_str(), "random(%u)", &count ) == 1 ) {
        _data[count].add( value );
        return true;
      }
      return false;
    }

    virtual void dump( ostream &strm ) 
    {
      for( map< unsigned int, Datum >::iterator itr = _data.begin(); itr != _data.end(); ++itr ) {
      strm << itr->first << itr->second.toString() << endl;
      }
    }

  protected: 
    map< unsigned int, Datum > _data;
    bool _set;
};






int main( int argc, char** argv )
{

  Options opts;

  string optsMsg;
  if( !opts.parseOpts( argc, argv, optsMsg ) ) {
    cout << optsMsg << endl;
    exit(-1);
  }

  kyotocabinet::HashDB results;
  if( !results.open( opts.resultsDb ) ) {
    cerr << "Error opening results db: " << results.error().name() << endl;
    exit(-1);
  }

  DB::Cursor *cur = results.cursor();
  cur->jump();


  AllCalibration allCal;
  RandomCalibration randomCal;

  string key, value;
  while( cur->get( &key, &value, true ) ) {

    bool result;
    if( key.compare( 0, 3, "all" ) == 0 )
      result = allCal.add( key, value );
    else if( key.compare( 0, 6, "random" ) == 0 )
      result = randomCal.add( key, value );


    if( !result ) cerr << "Failed to add the calibration with key " << key << endl;

  }


  ofstream foo;
  if( !opts.saveTo.empty() ) {
    foo.open( opts.saveTo );
    cout.rdbuf( foo.rdbuf() );
  }

  cout << endl;
  cout << "# all" << endl;
  allCal.dump( cout );

  cout << endl;
  cout << "# random" << endl;
  randomCal.dump( cout );


  delete cur;

  return 0;
}
