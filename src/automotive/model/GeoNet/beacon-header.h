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
#ifndef BEACONHEADER_H
#define BEACONHEADER_H

#include <stdint.h>
#include <string>
#include "ns3/header.h"
#include "ns3/gn-address.h"
#include "ns3/mac48-address.h"
#include "ns3/geonet.h"
#include "ns3/basic-header.h"
#include "ns3/common-header.h"
#include "ns3/longpositionvector.h"

namespace ns3
{
  class BeaconHeader : public Header
  {
    public:
      BeaconHeader();
      ~BeaconHeader();
      static TypeId GetTypeId (void);
      virtual TypeId GetInstanceTypeId (void) const;
      virtual void Print (std::ostream &os) const;
      virtual uint32_t GetSerializedSize (void) const;
      virtual void Serialize (Buffer::Iterator start) const;
      virtual uint32_t Deserialize (Buffer::Iterator start);

      //Setters
      void SetLongPositionV(GNlpv_t longPositionVector) {m_sourcePV = longPositionVector;}


      //Getters
      GNlpv_t GetLongPositionV(void) const {return m_sourcePV;}

    private:
      GNlpv_t m_sourcePV;  //!Source long position vector
      uint8_t m_reserved; //! aux variable for reading reserved fields
  };
}


#endif // BEACONHEADER_H
