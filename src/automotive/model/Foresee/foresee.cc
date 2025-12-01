//
// Created by diego on 01/12/25.
//

#include "foresee.h"

namespace ns3
{
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
  std::unordered_map<std::string, std::vector<double>> speeds_per_lane;
  std::unordered_map<std::string, std::vector<std::string>> veh_per_lane;
  double my_x = m_vdp->getPositionXY().x;
  double my_y = m_vdp->getPositionXY().y;
  double my_heading = m_vdp->getHeadingValue();
  for(auto it = vehicles.begin(); it != vehicles.end(); ++it)
    {
      if (it->vehData.heading != my_heading) continue; // Not the same direction
      VDP::VDP_position_cartesian_t pos = m_vdp->getXY(it->vehData.lon, it->vehData.lat);
      double x = pos.x;
      double y = pos.y;
      double dx = my_x - x;
      double dy = my_y - y;
      // Filter behind vehicles
      if (my_heading == 270 && dx < 0) continue;
      if (my_heading == 90 && dx > 0) continue;
      // Filter too far vehicles
      if (std::abs(dx) > m_max_reception_mcs) continue;
      std::string lane;
      double speed = it->vehData.speed_ms;
      // Determines the lane
      if ((my_heading == 270 && dy < 0) || (my_heading == 90 && dy > 0)) lane = "right";
      else if ((my_heading == 270 && dy > 0) || (my_heading == 90 && dy < 0)) lane = "left";
      else lane = "mine";
      speeds_per_lane[lane].push_back (speed);
      veh_per_lane[lane].push_back (std::to_string (it->vehData.stationID));
    }
  bool left_has_veh  = !speeds_per_lane["left"].empty();
  bool right_has_veh = !speeds_per_lane["right"].empty();
  double min_speed_mine, min_speed_left, min_speed_right;
  bool mine_has_veh  = !speeds_per_lane["mine"].empty();
  if (mine_has_veh)
    min_speed_mine = *std::min_element(speeds_per_lane["mine"].begin(), speeds_per_lane["mine"].end());
  else
    min_speed_mine = m_desired_speed;  // assume ego is driving at desired speed

  if (left_has_veh)
    min_speed_left = *std::min_element(speeds_per_lane["left"].begin(), speeds_per_lane["left"].end());

  if (right_has_veh)
    min_speed_right = *std::min_element(speeds_per_lane["right"].begin(), speeds_per_lane["right"].end());
  bool right_criterion = false;
  bool left_criterion = false;
  if (right_has_veh)
    {
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
  if (left_has_veh)
    {
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

  if (left_criterion)
    {
      // {-1=right, 1=left}
      bool could = m_traci->vehicle.couldChangeLane (m_vehicle_id, 1);
      std::cout << "Turn Left " << could << std::endl;
    }
  else if (right_criterion)
    {
      // {-1=right, 1=left}
      bool could = m_traci->vehicle.couldChangeLane (m_vehicle_id, -1);
      std::cout << "Turn Right " << could << std::endl;
    }
  else
    {
      // std::cout << "Keep lane" << std::endl;
    }
  Simulator::Schedule (MilliSeconds(m_FORESEE_check_ms), &foresee::FORESEEMobilityModel, this);
}
}
