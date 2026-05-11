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
#include "tsb-header.h"
#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/address-utils.h"

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE ("TSBheader");
  NS_OBJECT_ENSURE_REGISTERED (TSBheader);

  TSBheader::TSBheader()
  {
    NS_LOG_FUNCTION (this);
  }

  TSBheader::~TSBheader()
  {
    NS_LOG_FUNCTION (this);
  }

  TypeId
  TSBheader::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::TSBheader")
      .SetParent<Header> ()
      .SetGroupName ("Automotive")
      .AddConstructor<TSBheader> ()
    ;
    return tid;
  }

  TypeId
  TSBheader::GetInstanceTypeId (void) const
  {
    return GetTypeId ();
  }

  uint32_t
  TSBheader::GetSerializedSize (void) const
  {
    return 40;
  }

  void
  TSBheader::Print (std::ostream &os) const
  {
    os << m_sourcePV.GnAddress;
  }

  void
  TSBheader::Serialize (Buffer::Iterator start) const
  {
    //ETSI EN 302 636-4-1 [9.8.3]
    Buffer::Iterator i = start;
    //Source long position vector
    uint16_t pai_speed = 0;
    pai_speed = (m_sourcePV.positionAccuracy << 15) | ((m_sourcePV.speed)>>1); //Speed >> 1 so that sign isnt lost

    //Sequence number
    i.WriteHtonU16 (m_seqNumber);

    //Reserved
    i.WriteHtonU16 (0x00);

    //Source long position vector
    WriteTo (i,m_sourcePV.GnAddress.ConvertTo ());
    i.WriteHtonU32 (m_sourcePV.TST);
    i.WriteHtonU32 (m_sourcePV.latitude);
    i.WriteHtonU32 (m_sourcePV.longitude);
    i.WriteHtonU16 (pai_speed);
    i.WriteHtonU16 (m_sourcePV.heading);
  }

  uint32_t
  TSBheader::Deserialize (Buffer::Iterator start)
  {
    Buffer::Iterator i = start;

    //Sequence number
    m_seqNumber = i.ReadNtohU16 ();

    //Source long position vector
    uint8_t addr[8];
    i.Read (addr,8);
    m_sourcePV.GnAddress.Set (addr);
    m_sourcePV.TST = i.ReadNtohU32 ();
    m_sourcePV.latitude = i.ReadNtohU32 ();
    m_sourcePV.longitude = i.ReadNtohU32 ();
    uint16_t pai_speed = 0;
    pai_speed = i.ReadU16 ();
    m_sourcePV.positionAccuracy = pai_speed >> 15;
    m_sourcePV.speed = pai_speed & 0x7fff;
    m_sourcePV.heading = i.ReadNtohU16 ();

    return GetSerializedSize ();
  }

}
