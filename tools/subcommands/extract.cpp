
#include "extract.h"

namespace calibration {

  void Extract::SetupSubcommand( CLI::App &app ) {
    // Create the option and subcommand objects.
    auto opt = std::make_shared<ExtractOptions>();
    auto sub = app.add_subcommand("extract", "extract fiducial marks");

    sub->add_option("infiles", opt->inFiles, "Input files (movies or images)");

    // Add options to sub, binding them to opt.
    // sub->add_option("-f,--file", opt->file, "File name")->required();
    // sub->add_flag("--with-foo", opt->with_foo, "Counter");

    // Set the run function as callback to be called when this subcommand is issued.
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

  }

}
