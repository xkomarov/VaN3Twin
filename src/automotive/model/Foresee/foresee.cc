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
  Simulator::Schedule (MilliSeconds(0), &foresee::FORESEEMobilityModel, this);
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
  VDPDataItem<int> my_lane = m_vdp->getLanePosition();
  for(auto it = vehicles.begin(); it != vehicles.end(); ++it)
    {
      if (it->vehData.heading != my_heading) continue; // Not the same direction
      double x = it->vehData.x;
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

  bool right_has_veh, left_has_veh;
  bool can_turn_right, can_turn_left;
  if (my_lane.getData() == 1)
    {
      // Left most lane (more internal)
      // Can only turn to right
      right_has_veh = !speeds_per_lane[my_lane.getData()+1].empty();
      left_has_veh = false;
      can_turn_right = true;
      can_turn_left = false;
    }
  else if (my_lane.getData() == m_num_lanes)
    {
      // Right most lane
      // Can only turn to left
      left_has_veh = !speeds_per_lane[my_lane.getData()-1].empty();
      right_has_veh = false;
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

  if (left_criterion)
    {
      // {-1=right, 1=left}
      bool could = m_traci->vehicle.couldChangeLane (m_vehicle_id, 1);
    }
  if (right_criterion)
    {
      // {-1=right, 1=left}
      bool could = m_traci->vehicle.couldChangeLane (m_vehicle_id, -1);
    }
  else
    {
      // std::cout << "Keep lane" << std::endl;
    }
  Simulator::Schedule (MilliSeconds(m_FORESEE_check_ms), &foresee::FORESEEMobilityModel, this);
}
}
