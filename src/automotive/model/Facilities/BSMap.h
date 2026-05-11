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
#ifndef BSFAMAP_H
#define BSFAMAP_H

#include "ns3/BSContainer.h"
#include "unordered_map"

namespace ns3
{
  class BSMap: public Object
  {
    public:
      BSMap();

      void add(Ptr<BSContainer> bscontainer);
      void remove(StationID_t station_id);

      Ptr<BSContainer> get(StationID_t station_id);
      Ptr<BSContainer> get(StationID_t station_id, bool &found);
    private:
      std::unordered_map<StationID_t,Ptr<BSContainer>> m_internal_bsvector;
  };
}

#endif // BSMAP_H
