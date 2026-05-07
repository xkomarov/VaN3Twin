#ifndef VDPGPSTRACECLIENT_H
#define VDPGPSTRACECLIENT_H

#include "vdp.h"
#include "ns3/gps-tc.h"

namespace ns3 {
  class VDPGPSTraceClient : public VDP
  {
  public:
    VDPGPSTraceClient(Ptr<GPSTraceClient>,std::string);
    VDPGPSTraceClient();

    CAM_mandatory_data_t getCAMMandatoryData();
    CPM_mandatory_data_t getCPMMandatoryData();
    // std::vector<SPATEM_mandatory_data_t> getSPATEMMandatoryData();

    double getSpeedValue() {return m_gps_trace_client->getSpeedms ();}
    double getTravelledDistance() {return m_gps_trace_client->getTravelledDistance ();}
    double getHeadingValue() {return m_gps_trace_client->getHeadingdeg ();}

    // Added for GeoNet functionalities
    VDP_position_latlon_t getPosition();
    VDP_position_cartesian_t getPositionXY();
    VDP_position_cartesian_t getXY(double lon, double lat);
    double getCartesianDist (double lon1, double lat1, double lon2, double lat2){return 0.0;}

    VDPDataItem<uint8_t> getAccelerationControl() {return VDPDataItem<uint8_t>();}
    VDPDataItem<int> getLanePosition() {return VDPDataItem<int>();}
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
      std::string m_id;
      Ptr<GPSTraceClient> m_gps_trace_client;
  };
}

#endif // VDPGPSTRACECLIENT_H
