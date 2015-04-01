
#include "video_splitters/video_splitter_opts.h"

namespace AplCam {
  namespace VideoSplitters {

    //-- RandomSplitterOpts --
    struct option RandomSplitterOpts::long_options[] = {
      { "count", required_argument, NULL, 'c' },
      { 0, 0, 0, 0}
    };

    bool RandomSplitterOpts::parseOpts( int argc, char **argv, string &msg )
    {
      char optVal;
      int indexPtr;
      while( (optVal = getopt_long( argc, argv, "c:", long_options, &indexPtr )) != -1 ) {
        switch( optVal ) {
          case 'c':
            count = atol( optarg );
            break;
          default:
            return false;
        }
      }

      if( count < 0 ) {
        msg = "Must specify count with -c option";
        return false;
      }

      return true;
    }


    //-- IntervalSplitterOpts --
    struct option IntervalSplitterOpts::long_options[] = {
      { "offset", required_argument, NULL, 'o' },
      { "interval", required_argument, NULL, 'i' },
      { 0, 0, 0, 0}
    };

    bool IntervalSplitterOpts::parseOpts( int argc, char **argv, string &msg )
    {
      char optVal;
      int indexPtr;
      while( (optVal = getopt_long( argc, argv, "i:o:", long_options, &indexPtr )) != -1 ) {
        switch( optVal ) {
          case 'i':
            interval = atoi( optarg );
            break;
          case 'o':
            offset = atoi( optarg );
            break;
          default:
            return false;
        }
      }

      if( interval < 0 || offset < 0 ) {
        msg = "Invalid interval or offset specified";
        return false;
      }

      return true;
    }

  }
}
