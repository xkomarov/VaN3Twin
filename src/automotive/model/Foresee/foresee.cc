//
// Created by diego on 01/12/25.
//

#include "foresee.h"

namespace ns3
{
void
foresee::WrapperFORESEEMobilityModel()
{
  if (m_num_lanes == 0)
    {
      NS_FATAL_ERROR ("Set a number of lanes greater than 0 to use FORESEE Mobility Model.");
    }
  if (m_LDM == nullptr)
    {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs the LDM of the vehicle.");
    }
  if (m_traci == nullptr)
    {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs TraCI.");
    }
  if (m_desired_speed == 0)
    {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs a Desired Speed greater than 0.");
    }
  Simulator::Schedule (Seconds(m_start_time), &foresee::FORESEEMobilityModel, this);
}

void
foresee::setNumberOfLanes ()
{
  int lanes = m_traci->TraCIAPI::edge.getLaneNumber (m_traci->TraCIAPI::vehicle.getRoadID (m_vehicle_id));
  m_num_lanes = lanes;
}

void
foresee::FORESEEMobilityModel ()
{
  std::vector<LDM::returnedVehicleData_t> vehicles;
  bool res = m_LDM->getAllCVs (vehicles);
  if (res == false)
    {
      // TODO automatically move vehicle
      Simulator::Schedule (MilliSeconds(m_FORESEE_check_ms), &foresee::FORESEEMobilityModel, this);
      return;
    }
  std::unordered_map<long, std::vector<double>> speeds_per_lane;
  std::unordered_map<long, std::vector<std::string>> veh_per_lane;
  double my_heading = m_vdp->getHeadingValue();
  double my_x = m_vdp->getPositionXY().x;
  double my_y = m_vdp->getPositionXY().y;
  VDPDataItem<int> my_lane = m_vdp->getLanePosition();
  for(auto it = vehicles.begin(); it != vehicles.end(); ++it)
    {
      if (it->vehData.heading != my_heading) continue; // Not the same direction
      auto pos = m_traci->simulation.convertLonLattoXY (it->vehData.lon, it->vehData.lat);
      double x = pos.x;
      // Filter out vehicles behind the ego
      if (my_heading == 90 && x < my_x) continue;
      if (my_heading == 270 && x > my_x) continue;
      OptionalDataItem<long> lane = it->vehData.lanePosition;
      if (lane.isAvailable())
        {
          speeds_per_lane[lane.getData()].push_back (it->vehData.speed_ms);
          veh_per_lane[lane.getData()].push_back (std::to_string (it->vehData.stationID));
        }
    }

  bool right_has_veh = false, left_has_veh = false;
  bool can_turn_right = false, can_turn_left = false;
  if (my_lane.getData() == 1)
    {
      // Left most lane (more internal)
      // Can only turn to right
      right_has_veh = !speeds_per_lane[my_lane.getData()+1].empty();
      can_turn_right = true;
      can_turn_left = false;
    }
  else if (my_lane.getData() == m_num_lanes)
    {
      // Right most lane (more external)
      // Can only turn to left
      left_has_veh = !speeds_per_lane[my_lane.getData()-1].empty();
      can_turn_left = true;
      can_turn_right = false;
    }
  else
    {
      // Can turn both to left and right
      right_has_veh = !speeds_per_lane[my_lane.getData()+1].empty();
      left_has_veh = !speeds_per_lane[my_lane.getData()-1].empty();
      can_turn_left = true;
      can_turn_right = true;
    }
  // Check for my lane
  bool mine_has_veh = !speeds_per_lane[my_lane.getData()].empty();

  // Control for minimum speeds registered
  double min_speed_mine, min_speed_left, min_speed_right;
  if (mine_has_veh)
    min_speed_mine = *std::min_element(speeds_per_lane[my_lane.getData()].begin(), speeds_per_lane[my_lane.getData()].end());
  else
    min_speed_mine = m_desired_speed;  // assume ego is driving at desired speed

  bool right_criterion = false;
  bool left_criterion = false;
  // Check left incentive criterion
  if (can_turn_left && left_has_veh)
    {
      min_speed_left = *std::min_element(speeds_per_lane[my_lane.getData()-1].begin(), speeds_per_lane[my_lane.getData()-1].end());
      if (std::abs(min_speed_left - min_speed_mine) > m_delta_ls)
        {
          if (min_speed_left > min_speed_mine)
            {
              left_criterion = true;
            }
          else
            {
              double DSth_left = min_speed_mine * (1 - m_offset);
              if (m_desired_speed > DSth_left + m_delta_ds)
                {
                  left_criterion = true;
                }
            }
        }
    }

  // Check right incentive criterion
  if (can_turn_right && right_has_veh)
    {
      min_speed_right = *std::min_element(speeds_per_lane[my_lane.getData()+1].begin(), speeds_per_lane[my_lane.getData()+1].end());
      if (std::abs(min_speed_right - min_speed_mine) > m_delta_ls)
        {
          if (min_speed_right > min_speed_mine)
            {
              right_criterion = true;
            }
          else
            {
              double DSth_right = min_speed_right * (1 - m_offset);
              if (m_desired_speed < DSth_right - m_delta_ds)
                {
                  right_criterion = true;
                }
            }
        }
    }

  bool could_change_lane;
  // {-1=right, 1=left}
  uint8_t lc_direction = 0;
  if (left_criterion) lc_direction = 1;
  else if (right_criterion) lc_direction = -1;
  bool startManeuver = false;
  if (lc_direction != 0)
    {
      could_change_lane = m_traci->vehicle.couldChangeLane (m_vehicle_id, lc_direction);
      if (could_change_lane)
        {
          if (left_criterion) m_traci->vehicle.changeLane (m_vehicle_id, my_lane.getData()-1, m_time_to_lc);
          else m_traci->vehicle.changeLane (m_vehicle_id, my_lane.getData()+1, m_time_to_lc);
        }
      else
        {
          // Check the coordination avoidance range
          bool found = false;
          // Take the four roles, target, ahead ego, ahead target
          std::string RV, HVAhead, RVAhead;
          for (auto it = (*m_lc_data_structure).begin(); it != (*m_lc_data_structure).end(); ++it)
            {
              auto v = it->second;
              float heading = std::get<0>(v);
              if (heading != my_heading) continue;
              float x = std::get<1>(v);
              // Filter behind vehicles
              if (my_heading == 90 && x < my_x) continue;
              if (my_heading == 270 && x > my_x) continue;
              float y = std::get<2>(v);
              float dist = std::sqrt (std::pow(my_x - x, 2) + std::pow(my_y - y, 2));
              if (dist <= m_ca_range)
                {
                  found = true;
                  break;
                }
            }
          if (!found)
            {
              // Initiate the MCM process
              startManeuver = true;
              double min_dist_hv_ahead = 10000;
              double min_dist_rv_ahead = 10000;
              double min_dist_rv = 10000;
              // Vehicles ahead of HV in the same lane
              auto& vec1 = veh_per_lane[my_lane.getData()];
              // Vehicles ahead of HV in the target lane
              auto& vec2 = left_criterion ? veh_per_lane[my_lane.getData()-1] : veh_per_lane[my_lane.getData()+1];
              // Target lane
              int target_lane = left_criterion ? my_lane.getData() - 1 : my_lane.getData() + 1;
              for(auto it = vehicles.begin(); it != vehicles.end(); ++it)
                {
                  auto it_found = std::find(
                      vec1.begin(),
                      vec1.end(),
                      std::to_string(it->vehData.stationID)
                  );

                  auto pos = m_traci->simulation.convertLonLattoXY (it->vehData.lon, it->vehData.lat);
                  double dist = std::sqrt (std::pow(my_x - pos.x, 2) + std::pow(my_y - pos.y, 2));

                  if (it_found != vec1.end())
                    {
                      // Vehicle is in the same lane of HV, can be HVAhead
                      if (dist < min_dist_hv_ahead)
                        {
                          min_dist_hv_ahead = dist;
                          HVAhead = "veh" + std::to_string (it->vehData.stationID);
                        }
                    }
                  else
                    {
                      // Check for RV and RVAhead
                      it_found = std::find(
                          vec2.begin(),
                          vec2.end(),
                          std::to_string(it->vehData.stationID)
                      );
                      if (it_found != vec2.end())
                        {
                          // Vehicle is in the target lane, can be RVAhead
                          if (dist < min_dist_rv_ahead)
                            {
                              min_dist_rv_ahead = dist;
                              RVAhead = "veh" + std::to_string (it->vehData.stationID);
                            }
                        }
                      else
                        {
                          // Check if RV
                          OptionalDataItem<long> lane = it->vehData.lanePosition;
                          if(lane.isAvailable() && lane.getData() == target_lane)
                            {
                              if (dist < min_dist_rv)
                                {
                                  min_dist_rv = dist;
                                  RV = "veh" + std::to_string (it->vehData.stationID);
                                }
                            }
                        }
                    }
                }
              m_actors = {RV, HVAhead, RVAhead};
              (*m_lc_data_structure)[m_vehicle_id_int] = std::make_tuple (my_heading, my_x, my_y);
              Simulator::Schedule (MilliSeconds(0), &foresee::doCoordination, this);
            }
        }
    }
  if (!startManeuver) Simulator::Schedule (MilliSeconds(m_FORESEE_check_ms), &foresee::FORESEEMobilityModel, this);
}

void
foresee::doCoordination ()
{
  MCSpecification specification = {};
  m_mcs_ptr->generateAndEncodeMCM (specification);
}

void
foresee::terminateCoordination ()
{
  (*m_lc_data_structure).erase(m_vehicle_id_int);
  Simulator::Schedule (MilliSeconds(m_FORESEE_check_ms), &foresee::FORESEEMobilityModel, this);
}
}
