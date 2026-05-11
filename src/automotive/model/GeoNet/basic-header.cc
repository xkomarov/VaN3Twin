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
#include "basic-header.h"
#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/address-utils.h"

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE ("GNBasicHeader");
  GNBasicHeader::GNBasicHeader()
  {
    NS_LOG_FUNCTION (this);
  }

  GNBasicHeader::~GNBasicHeader()
  {
    NS_LOG_FUNCTION (this);
  }

  TypeId
  GNBasicHeader::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::GNBasicHeader")
      .SetParent<Header> ()
      .SetGroupName ("Automotive")
      .AddConstructor<GNBasicHeader> ();
    return tid;
  }

  TypeId
  GNBasicHeader::GetInstanceTypeId (void) const
  {
    return GetTypeId ();
  }

  uint32_t
  GNBasicHeader::GetSerializedSize (void) const
  {
    return 4;
  }

  void
  GNBasicHeader::Print (std::ostream &os) const
  {
    os << m_version;
  }

  void
  GNBasicHeader::Serialize (Buffer::Iterator start) const
  {
    //ETSI EN 302 636-4-1 [9.8.4]
    Buffer::Iterator i = start;
    //Basic Header aux variables
    uint8_t version_NH = 0;
    version_NH = (m_version << 4) | (m_nextHeader);

    //Basic Header     ETSI EN 302 636-4-1 [9.6]
    i.WriteU8 (version_NH);
    i.WriteU8 (0x00); //! Reserved
    i.WriteU8 (m_lifeTime);
    i.WriteU8 (m_remainingHopLimit);
  }

  uint32_t
  GNBasicHeader::Deserialize (Buffer::Iterator start)
  {
    Buffer::Iterator i = start;
    //Basic Header
    uint8_t version_NH = 0;

    version_NH = i.ReadU8 ();
    m_version = version_NH >> 4;
    m_nextHeader = version_NH & 0x0f;
    m_reserved = i.ReadU8 ();
    m_lifeTime = i.ReadU8 ();
    m_remainingHopLimit = i.ReadU8 ();

    return GetSerializedSize ();
  }

}
