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
#ifndef BASIC_HEADER_H
#define BASIC_HEADER_H
#include <stdint.h>
#include <string>
#include "ns3/header.h"
#include "ns3/gn-address.h"
#include "ns3/mac48-address.h"

namespace ns3 {

  class GNBasicHeader : public Header
  {
    public:
      GNBasicHeader();
      ~GNBasicHeader();
      static TypeId GetTypeId (void);
      virtual TypeId GetInstanceTypeId (void) const;
      virtual void Print (std::ostream &os) const;
      virtual uint32_t GetSerializedSize (void) const;
      virtual void Serialize (Buffer::Iterator start) const;
      virtual uint32_t Deserialize (Buffer::Iterator start);

      //Setters
      void SetVersion(uint8_t version) {m_version = version;}
      void SetNextHeader(uint8_t NH) {m_nextHeader = NH;}
      void SetLifeTime(uint8_t LT) {m_lifeTime = LT;}
      void SetRemainingHL(uint8_t RHL){m_remainingHopLimit = RHL;}

      //Getters
      uint8_t GetVersion() {return m_version;}
      uint8_t GetNextHeader() {return m_nextHeader;}
      uint8_t GetLifeTime() {return m_lifeTime;}
      uint8_t GetRemainingHL(){return m_remainingHopLimit;}


    private:
      uint8_t m_version : 4;
      uint8_t m_nextHeader : 4;
      uint8_t m_reserved;
      uint8_t m_lifeTime;
      uint8_t m_remainingHopLimit;

  };

}
#endif // BASIC_HEADER_H
