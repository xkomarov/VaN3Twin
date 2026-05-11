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
/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * Created by:
 *  Diego Gasco, Politecnico di Torino (diego.gasco@polito.it, diego.gasco99@gmail.com)
*/

#include "ns3/double.h"
#include "ns3/timestamp-tag.h"

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (TimestampTag);

TypeId
TimestampTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TimestampTag")
    .SetParent<Tag> ()
    .SetGroupName ("Wifi")
    .AddConstructor<TimestampTag> ()
    .AddAttribute ("Timestamp", "The Timestamp of the last packet received",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&TimestampTag::Get),
                   MakeDoubleChecker<double> ())
  ;
  return tid;
}

TypeId
TimestampTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

TimestampTag::TimestampTag ()
  : m_timestamp (0)
{
}

uint32_t
TimestampTag::GetSerializedSize (void) const
{
  return sizeof (double);
}

void
TimestampTag::Serialize (TagBuffer i) const
{
  i.WriteDouble (m_timestamp);
}

void
TimestampTag::Deserialize (TagBuffer i)
{
  m_timestamp = i.ReadDouble ();
}

void
TimestampTag::Print (std::ostream &os) const
{
  os << "Timestamp=" << m_timestamp;
}

void
TimestampTag::Set (double timestamp)
{
  m_timestamp = timestamp;
}

double
TimestampTag::Get (void) const
{
  return m_timestamp;
}

}
