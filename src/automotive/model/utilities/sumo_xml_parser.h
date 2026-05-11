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
#ifndef SUMO_XML_PARSER_H
#define SUMO_XML_PARSER_H

#include <libxml/parser.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>
#include <libxml/tree.h>
#include <fstream>
#include <string>
#include <vector>

namespace ns3 {

  int XML_rou_count_vehicles(xmlDocPtr doc);
  int XML_rou_count_pedestrians(xmlDocPtr doc);
  std::vector<std::tuple<std::string, float, float>> XML_poli_count_stations(std::ifstream &file);
}

#endif
