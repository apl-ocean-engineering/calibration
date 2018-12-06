
#include <CLI/CLI.hpp>

#include "libg3logger/g3logger.h"

#include "subcommands/extract.h"
#include "subcommands/cal.h"

using namespace calibration;


struct GlobalOpts {
  GlobalOpts()
    : verbosity(0)
    {;}

  int verbosity;

};

class TopLevelApp {
public:
  TopLevelApp()
    : _logger("cal"),
      _verbosity(0)
  {;}


  void processGlobalOpts( GlobalOpts const &opts ) {
    //setVerbosity( opts.verbosity );
  }

  void incrementVerbosity( int count ) {
    ++_verbosity;

    LOG( WARNING ) << "Setting verbosity to " << _verbosity;
    switch(_verbosity) {
    case 0:
      _logger.stderrHandle->call( &ColorStderrSink::setThreshold, WARNING );
      break;
    case 1:
      _logger.stderrHandle->call( &ColorStderrSink::setThreshold, INFO );
      break;
    case 2:
      _logger.stderrHandle->call( &ColorStderrSink::setThreshold, DEBUG );
      break;
    }
  }

private:

  libg3logger::G3Logger _logger;

  int _verbosity;
};



int main( int argc, char** argv )
{
  using namespace std::placeholders;

  TopLevelApp cal;
  CLI::App app{"Image calibration toolbox"};

  //=== Global options ===
  GlobalOpts opts;
  app.add_flag_function("-v", std::bind( &TopLevelApp::incrementVerbosity, &cal, _1 ), "Additional output (use -vv for even more!)");

  app.set_callback( std::bind( &TopLevelApp::processGlobalOpts, &cal, std::ref(opts) ) );

  // Add subcommands
  {
    auto sub = Extract::SetupSubcommand( app );
    sub->add_flag_function("-v", std::bind( &TopLevelApp::incrementVerbosity, &cal, _1 ), "Additional output (use -vv for even more!)");
  }

  {
    auto sub = Cal::SetupSubcommand( app );
    sub->add_flag_function("-v", std::bind( &TopLevelApp::incrementVerbosity, &cal, _1 ), "Additional output (use -vv for even more!)");
  }


  CLI11_PARSE(app, argc, argv);
}
