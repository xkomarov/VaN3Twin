#ifndef IDPTRACI_H
#define IDPTRACI_H

#include "idp.h"
#include "ns3/traci-client.h"
#include <unordered_map>

namespace ns3 {

/**
 * \ingroup automotive
 *
 * \brief This class is used to represent a IDP object that gathers information from TraCI.
 *
 */
class IDPTraCI : public IDP
{
public:

  /**
     * @brief Constructor
     *
     * This constructor initializes the IDPTraCI object.
     * @param traci_client The TraCI client object.
     * @param node_id The node ID of the SUMO vehicle.
     */
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

  /**
   * @brief This function returns the mandatory data of the SPATEM message for all traffic lights.
   * @return The mandatory data of the SPATEM message for all traffic lights.
   */
  std::vector<SPATEM_mandatory_data_t> getSPATEMMandatoryData ();

  // Added for GeoNet functionalities
  /**
     * @brief This function returns the vehicle's position in lat/lon coordinates.
     * @return
     */
  IDP_position_latlon_t getPosition ();
  /**
     * @brief This function returns the vehicle's position in cartesian coordinates.
     * @return
     */
  IDP_position_cartesian_t getPositionXY ();
  /**
     * @brief This function converts the vehicle's position in lat/lon coordinates to cartesian coordinates.
     * @param lon The vehicle's longitude.
     * @param lat The vehicle's latitude.
     * @return
     */
  IDP_position_cartesian_t getXY (double lon, double lat);
  /**
     * @brief This function returns the distance between two points in lat/lon coordinates.
     * @param lon1
     * @param lat1
     * @param lon2
     * @param lat2
     * @return
     */
  double getCartesianDist (double lon1, double lat1, double lon2, double lat2);

private:
  std::string m_id;
  std::string m_tls_id;
  std::vector<std::string> m_tls_ids;
  Ptr<TraciClient> m_traci_client;
  std::unordered_map<std::string, std::string> m_tls_prev_states;
  std::unordered_map<std::string, uint8_t> m_tls_revisions;
  
};
} // namespace ns3

#endif // IDPTRACI_H
