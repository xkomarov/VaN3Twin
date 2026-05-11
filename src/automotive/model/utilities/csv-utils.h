/* ============================================================================
 * Research Project: Data communication in the environment of
intelligent cars
 * Author: Kirill Komarov
 * Date: 2026
 * 
 * Description:
 * This file contains source code developed (or modified) as part of the 
 * research for the paper: "Data communication in the environment of
intelligent cars".
 * 
 * DISCLAIMER & ACKNOWLEDGEMENT:
 * Please note that this file contains or may contain code fragments, 
 * algorithms, or architectural solutions that were previously implemented 
 * in the "VaN3Twin" project https://github.com/DriveX-devs/VaN3Twin.git.
 * 
 * The borrowed code has been adapted and is used strictly for academic 
 * and research purposes. All rights to the original code segments belong 
 * to their respective original authors.
 * ============================================================================ */
#ifndef CSV_UTILS_H
#define CSV_UTILS_H

#include <string>
#include <fstream>
#include <unistd.h>

namespace ns3 {
  template <typename... Args>
  void
  writeDataToCSV (std::string filepath, std::string header, Args... args)
  {
    std::ofstream csv_out;
    bool first_element = true;

    if (access (filepath.c_str (), F_OK) != -1)
      {
        // The file already exists
        csv_out.open (filepath, std::ofstream::out | std::ofstream::app);
      }
    else
      {
        // The file does not exist yet
        csv_out.open (filepath);
        csv_out << header << std::endl;
      }

    // Lambda to print an element with a preceding comma if it is not the first element
    auto print_csv_arg = [&first_element, &csv_out] (const auto &arg) {
      if (!first_element)
        {
          csv_out << ",";
        }
      else
        {
          first_element = false;
        }
      csv_out << arg;
    };

    // Fold expression to save to the CSV file all the specified data
    (..., print_csv_arg (args));

    csv_out << std::endl;
  }
}

#endif // CSV_UTILS_H