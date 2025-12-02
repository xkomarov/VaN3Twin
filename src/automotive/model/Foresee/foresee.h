//
// Created by diego on 01/12/25.
//

#ifndef NS3_FORESEE_H
#define NS3_FORESEE_H

#include "ns3/core-module.h"
#include "ns3/LDM.h"

namespace ns3
{
class foresee
{
public:
  foresee() = default;
  ~foresee() = default;
  void WrapperFORESEEMobilityModel();
  void FORESEEMobilityModel();
  void setLDM (Ptr<LDM> ldm) {m_LDM = ldm;};
  void setTraciAPI (Ptr<TraciClient> traci) {m_traci = traci;};
  void setNumberOfLanes () {
    int lanes = m_traci->TraCIAPI::edge.getLaneNumber (m_traci->TraCIAPI::vehicle.getRoadID (m_vehicle_id));
    m_num_lanes = lanes;
  }
  void setVDP (VDP* vdp) {m_vdp = vdp;};
  void setDesiredSpeed (double speed) {m_desired_speed = speed;};
  void setVehicleID (std::string vehicleID) {m_vehicle_id = vehicleID;};
private:
  std::string m_vehicle_id;
  Ptr<LDM> m_LDM;
  Ptr<TraciClient> m_traci;
  VDP* m_vdp;
  int m_FORESEE_check_ms = 1000;
  int m_max_reception_mcs = 1000;
  double m_desired_speed = 0;
  double m_delta_ls = 0.5;
  double m_delta_ds = 0.5;
  double m_offset = 0.3;
  int m_num_lanes = 0;
};
}


#endif //NS3_FORESEE_H
