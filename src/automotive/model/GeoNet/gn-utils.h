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
#ifndef GN_UTILS_H
#define GN_UTILS_H

#include "ns3/packet-socket-address.h"

#define GN_ETHERTYPE 0x8947

namespace ns3
{
  // This function returns a Packet Socket address, to be used in the sockets created for GeoNetworking,
  // given an interface index and a physical address. The correct Ethertype is automatically inserted.
  PacketSocketAddress getGNAddress (uint32_t ifindex, Address physicalAddress);

  // This function return a MAC-48 address, given any MAC-48 or EUI-64 ("mac64") identifier
  // In the second case, a MAC-48 address is calculated starting from the EUI-64 ("mac64") identifier, for usage inside GeoNetworking
  Mac48Address getGNMac48(Address generic_eui);
}

#endif // GN_UTILS_H

