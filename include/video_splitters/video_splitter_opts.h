
#ifndef __VIDEO_SPLITTER_OPTS_H__
#define __VIDEO_SPLITTER_OPTS_H__

#include <unistd.h>
#include <getopt.h>

#include <string>

namespace AplCam {

  namespace VideoSplitters {

    using std::string;

    struct RandomSplitterOpts {
      RandomSplitterOpts()
        : count(-1) {;}

      long int count;

      static struct option long_options[]; 
      bool parseOpts( int argc, char **argv, string &msg );
    };

    struct IntervalSplitterOpts {
      IntervalSplitterOpts()
        : offset(0), interval(1) {;}

      int offset, interval;

      static struct option long_options[];
      bool parseOpts( int argc, char **argv, string &msg );
    };

  }
}

#endif
