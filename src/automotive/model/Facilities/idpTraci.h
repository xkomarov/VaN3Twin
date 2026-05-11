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
#ifndef IDPTRACI_H
#define IDPTRACI_H

#include "idp.h"
#include "ns3/traci-client.h"
#include <unordered_map>

namespace ns3 {


class IDPTraCI : public IDP
{
public:

  
  IDPTraCI ();
  IDPTraCI (Ptr<TraciClient> traci_client, std::string node_id, std::string tls_id = "");

  void
  setProperties (Ptr<TraciClient> traci_client, std::string node_id)
  {
    m_traci_client = traci_client;
    m_id = node_id;
  }
  void
  setTargetTls (std::string tls_id)
  {
    m_tls_id = tls_id;
  }
  void
  setTargetTlsList (std::vector<std::string> tls_ids)
  {
    m_tls_ids = tls_ids;
  }

  
  std::vector<SPATEM_mandatory_data_t> getSPATEMMandatoryData ();

  
  
  IDP_position_latlon_t getPosition ();
  
  IDP_position_cartesian_t getPositionXY ();
  
  IDP_position_cartesian_t getXY (double lon, double lat);
  
  double getCartesianDist (double lon1, double lat1, double lon2, double lat2);

private:
  std::string m_id;
  std::string m_tls_id;
  std::vector<std::string> m_tls_ids;
  Ptr<TraciClient> m_traci_client;
  std::unordered_map<std::string, std::string> m_tls_prev_states;
  std::unordered_map<std::string, uint8_t> m_tls_revisions;
  
};
} 

#endif 
