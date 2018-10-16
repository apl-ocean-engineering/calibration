#pragma once

#include <vector>
#include <string>

#include <CLI/CLI.hpp>

#include "libg3logger/g3logger.h"

namespace calibration {

  struct ExtractOptions {
    std::vector< std::string > inFiles;
    std::string boardName;
    std::string databaseName;
  };


  class Extract {
  public:

    static void SetupSubcommand( CLI::App &app );
    static void Run( ExtractOptions const &opts );

    Extract( ExtractOptions const &opts );

    void run( void );

  private:

    ExtractOptions const &_opts;

  };


}
