
#ifndef __VIDEO_SPLITTER_OPTS_H__
#define __VIDEO_SPLITTER_OPTS_H__

#include <limits.h>
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
        : start(0), end(INT_MAX), interval(1) {;}

      int start, end, interval;

      static struct option long_options[];
      bool parseOpts( int argc, char **argv, string &msg );
    };

  }
}

#endif
