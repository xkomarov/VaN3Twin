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
#ifndef VRUDPGPSTRACECLIENT_H
#define VRUDPGPSTRACECLIENT_H

#include "VRUdp.h"
#include "ns3/gps-tc.h"

namespace ns3 {
  class VRUDPGPSTraceClient : public VRUdp
  {
  public:
    VRUDPGPSTraceClient(Ptr<GPSTraceClient>,std::string);
    VRUDPGPSTraceClient();

    VAM_mandatory_data_t getVAMMandatoryData() override;

    libsumo::TraCIPosition getPedPositionValue() override {
      libsumo::TraCIPosition pos;
      pos.x = m_gps_trace_client->getX();
      pos.y = m_gps_trace_client->getY();
      pos.z = 0.0;
      return pos;
    }
    double getPedSpeedValue() override {return m_gps_trace_client->getSpeedms ();}
    double getPedHeadingValue() override {return m_gps_trace_client->getHeadingdeg ();}

    std::vector<distance_t> get_min_distance(Ptr<LDM> LDM) override {return std::vector<distance_t>();}

    // Added for GeoNet functionalities
    VRUdp_position_latlon_t getPedPosition() override;
    VRUdp::VRUDP_position_cartesian_t getXY(double lon, double lat) override;
    double getCartesianDist (double lon1, double lat1, double lon2, double lat2){return 0.0;}

    private:
      std::string m_id;
      Ptr<GPSTraceClient> m_gps_trace_client;
  };
}

#endif // VRUDPGPSTRACECLIENT_H
