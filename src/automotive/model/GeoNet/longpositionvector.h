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
#ifndef LONGPOSITIONVECTOR_H
#define LONGPOSITIONVECTOR_H
#include <stdint.h>
#include "ns3/gn-address.h"

namespace ns3 {

  typedef struct _longPositionVector{
    GNAddress GnAddress; //! Address representation based on ipv6address
    uint32_t TST; //! Time at which lat and long were acquired by GeoAdhoc router
    int32_t latitude;
    int32_t longitude;
    bool positionAccuracy : 1;
    int16_t speed :15;
    uint16_t heading;
  }GNlpv_t;

}

#endif // LONGPOSITIONVECTOR_H
