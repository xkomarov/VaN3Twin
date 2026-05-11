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
#ifndef BTPHEADER_H
#define BTPHEADER_H

#include <stdint.h>
#include <string>
#include "ns3/header.h"

namespace ns3
{
  class btpHeader : public Header
  {
    public:
      btpHeader();
      ~btpHeader();

      /**
       * \param port the destination port for this BTPHeader
       */
      void SetDestinationPort (uint16_t port);
      /**
       * \param port The source port for this BTPHeader
       */
      void SetSourcePort (uint16_t port);
      /**
       * \param port the destination port for this BTPHeader
       */
      void SetDestinationPortInfo (uint16_t portInfo);
      /**
       * \return The source port for this BTPHeader
       */
      uint16_t GetSourcePort (void) const;
      /**
       * \return the destination port for this BTPHeader
       */
      uint16_t GetDestinationPort (void) const;
      /**
       * \return the destination port for this BTPHeader
       */
      uint16_t GetDestinationPortInfo (void) const;

      static TypeId GetTypeId (void);
      virtual TypeId GetInstanceTypeId (void) const;
      virtual void Print (std::ostream &os) const;
      virtual uint32_t GetSerializedSize (void) const;
      virtual void Serialize (Buffer::Iterator start) const;
      virtual uint32_t Deserialize (Buffer::Iterator start);

    private:
      uint16_t m_destinationPort; //!< Destination port
      uint16_t m_source_destInfo; //!< Source port/Destination port info
  };

}
#endif // BTPHEADER_H
