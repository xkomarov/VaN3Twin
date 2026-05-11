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
#ifndef VDPOPENCDA_H
#define VDPOPENCDA_H

#include "vdp.h"
#include "ns3/OpenCDAClient.h"



namespace ns3 {
  class VDPOpenCDA : public VDP
  {
  public:
    VDPOpenCDA(Ptr<OpenCDAClient>, std::string);
    VDPOpenCDA();
    CAM_mandatory_data_t getCAMMandatoryData();
    CPM_mandatory_data_t getCPMMandatoryData();
    // std::vector<SPATEM_mandatory_data_t> getSPATEMMandatoryData();

    double getSpeedValue() {return m_opencda_client->getSpeed (m_id);}
    double getTravelledDistance() {return 0;}
    double getHeadingValue() {return m_opencda_client->getHeading (m_id);}

    VDP_position_latlon_t getPosition();
    VDP_position_cartesian_t getPositionXY();
    VDP_position_cartesian_t getXY(double lon, double lat);
    double getCartesianDist (double lon1, double lat1, double lon2, double lat2);

    VDPDataItem<uint8_t> getAccelerationControl() {return VDPDataItem<uint8_t>();}
    VDPDataItem<int> getLanePosition();
    VDPDataItem<VDPValueConfidence<int,int>> getSteeringWheelAngle() {return VDPDataItem<VDPValueConfidence<int,int>>();}
    VDPDataItem<VDPValueConfidence<int,int>> getLateralAcceleration() {return VDPDataItem<VDPValueConfidence<int,int>>();}
    VDPDataItem<VDPValueConfidence<int,int>> getVerticalAcceleration() {return VDPDataItem<VDPValueConfidence<int,int>>();}
    VDPDataItem<int> getPerformanceClass() {return VDPDataItem<int>();}
    VDPDataItem<VDP_CEN_DSRC_tolling_zone_t> getCenDsrcTollingZone() {return VDPDataItem<VDP_CEN_DSRC_tolling_zone_t>();}

    VDPDataItem<unsigned int> getVehicleRole() {return VDPDataItem<unsigned int>();}
    VDPDataItem<uint8_t> getExteriorLights() {return VDPDataItem<uint8_t>();}
    VDPDataItem<VDP_PublicTransportContainerData_t> getPublicTransportContainerData() {return VDPDataItem<VDP_PublicTransportContainerData_t>();}
    VDPDataItem<VDP_SpecialTransportContainerData_t> getSpecialTransportContainerData() {return VDPDataItem<VDP_SpecialTransportContainerData_t>();}
    VDPDataItem<int> getDangerousGoodsBasicType() {return VDPDataItem<int>();}
    VDPDataItem<VDP_RoadWorksContainerBasicData_t> getRoadWorksContainerBasicData_t() {return VDPDataItem<VDP_RoadWorksContainerBasicData_t>();}
    VDPDataItem<uint8_t> getRescueContainerLightBarSirenInUse() {return VDPDataItem<uint8_t>();}
    VDPDataItem<VDP_EmergencyContainerData_t> getEmergencyContainerData() {return VDPDataItem<VDP_EmergencyContainerData_t>();}
    VDPDataItem<VDP_SafetyCarContainerData_t> getSafetyCarContainerData() {return VDPDataItem<VDP_SafetyCarContainerData_t>();}

    private:
      std::string m_string_id;
      int m_id;
      Ptr<OpenCDAClient> m_opencda_client;

      int m_length;
      int m_width;

  };

}

#endif // VDPOPENCDA_H
