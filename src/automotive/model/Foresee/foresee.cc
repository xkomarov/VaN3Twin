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

double IDMAcceleration(double v,
                 double v_lead,
                 double gap,
                 double v0,
                 double a_max,
                 double b,
                 double s0,
                 double T,
                 double delta = 4.0
)
{
  gap = std::max(gap, 0.1); // avoid singularities

  double delta_v = v - v_lead;
  double s_star = s0 + v * T + (v * delta_v) / (2.0 * std::sqrt(a_max * b));

  return a_max * (1.0 - std::pow(v / v0, delta) - std::pow(s_star / gap, 2.0));
}

double
foresee::estimateTimeFromPredictionIDM(
    std::vector<foresee::TrajectoryItem> leader,
    std::vector<foresee::TrajectoryItem> follower,
    double comfort_decel,
    double a_max,
    double desired_speed,
    double b,
    double s0,
    double T)
{
  size_t N = std::min(leader.size(), follower.size());

  for (size_t k = 0; k < N; ++k)
    {
      auto lead = leader[k];
      auto foll = follower[k];

      // Future gap (RV behind HV assumed)
      double gap = std::abs(lead.x - foll.x);

      // Required braking if lane change happens at this timestep
      double a_required_RV = IDMAcceleration(
          foll.speed,   // follower
          lead.speed,   // leader
          gap,
          desired_speed,
          a_max,
          b,
          s0,
          T
      );

      // Comfort check (IDM can exceed prediction decel!)
      if (a_required_RV >= -comfort_decel)
        {
          return (k + 1) * STEP_TIME; // EstimatedTime found
        }
    }

  return -1.0; // no feasible coordination in horizon
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
  double my_speed = m_vdp->getSpeedValue();
  std::string my_type = m_traci->vehicle.getTypeID (m_vehicle_id);
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
          auto [state, stateTraCI] = m_traci->vehicle.getLaneChangeState(m_vehicle_id, lc_direction);
          std::vector<uint8_t> reasons = m_traci->vehicle.getLaneChangeFailureReasons (state, stateTraCI, lc_direction);
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
              double x_RV, x_RVAhead, x_HVAhead;
              double y_RV, y_RVAhead, y_HVAhead;
              double speed_RV, speed_RVAhead, speed_HVAhead;
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
                          x_HVAhead = pos.x;
                          y_HVAhead = pos.y;
                          speed_HVAhead = it->vehData.speed_ms;
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
                              x_RVAhead = pos.x;
                              y_RVAhead = pos.y;
                              speed_RVAhead = it->vehData.speed_ms;
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
                                  x_RV = pos.x;
                                  y_RV = pos.y;
                                  speed_RV = it->vehData.speed_ms;
                                }
                            }
                        }
                    }
                }
              m_actors = {RV, HVAhead, RVAhead};
              std::vector<TrajectoryItem> mp_RV;
              std::vector<TrajectoryItem> mp_RVAhead;
              std::vector<TrajectoryItem> mp_HVAhead;
              uint8_t sign = my_heading == 270 ? -1 : 1;
              if(!RV.empty()) mp_RV = predictConstantSpeed (x_RV, y_RV, speed_RV, sign, true);
              if(!RVAhead.empty()) mp_RVAhead = predictConstantSpeed (x_RVAhead, y_RVAhead, speed_RVAhead, sign);
              if(!HVAhead.empty()) mp_HVAhead = predictConstantSpeed (x_HVAhead, y_HVAhead, speed_HVAhead, sign);
              std::vector<TrajectoryItem> mp_HV = predictConstantSpeed (my_x, my_y, my_speed, sign);
              double estimated_time_hv_rv = -1, estimated_time_hv_rvahead = -1;
              if(!mp_RV.empty())
                {
                  estimated_time_hv_rv = estimateTimeFromPredictionIDM (
                      mp_HV,
                      mp_RV,
                      std::abs(COMFORT_DECELERATION), // comfort threshold
                      m_desired_speed,
                      m_traci->vehicletype.getAccel (my_type),
                      std::abs(SAFE_DECELERATION),
                      m_traci->vehicletype.getMinGap (my_type),
                      m_traci->vehicletype.getTau (my_type)
                  );
                }
              if (!mp_RVAhead.empty())
                {
                   estimated_time_hv_rvahead = estimateTimeFromPredictionIDM (
                      mp_RVAhead,
                      mp_HV,
                      std::abs(COMFORT_DECELERATION), // comfort threshold
                      m_desired_speed,
                      m_traci->vehicletype.getAccel (my_type),
                      std::abs(SAFE_DECELERATION),
                      m_traci->vehicletype.getMinGap (my_type),
                      m_traci->vehicletype.getTau (my_type)
                  );
                }
              std::cout << "HERE" << std::endl;
              (*m_lc_data_structure)[m_vehicle_id_int] = std::make_tuple (my_heading, my_x, my_y);
              // Simulator::Schedule (MilliSeconds(0), &foresee::doCoordination, this);
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

std::vector<foresee::TrajectoryItem>
foresee::predictConstantSpeed(double x, double y, double speed, uint8_t sign, bool is_RV)
{
  std::vector<foresee::TrajectoryItem> motion_plan;
  if (!is_RV)
    {
      float t = 0.1;
      while (t <= HORIZON_TIME)
      {
        TrajectoryItem item {x + speed * STEP_TIME, y, speed};
        motion_plan.push_back (item);
        x = sign * (x + speed * STEP_TIME);
        t += STEP_TIME;
      }
    }
  else
    {
      float t = 0.1;
      while (t <= HORIZON_TIME)
        {
          if (t <= NEGOTIATION_TIME || t > NEGOTIATION_TIME + DECELERATION_TIME)
            {
              x = sign * (x + speed * STEP_TIME);
              TrajectoryItem item {x, y, speed};
              motion_plan.push_back (item);
              t += STEP_TIME;
            }
          else if (t > NEGOTIATION_TIME && t <= NEGOTIATION_TIME + DECELERATION_TIME)
            {
              x = sign * (x + speed * STEP_TIME + 0.5 * COMFORT_DECELERATION * std::pow(STEP_TIME, 2));
              speed = speed + COMFORT_DECELERATION * STEP_TIME;
              TrajectoryItem item {x, y, speed};
              motion_plan.push_back (item);
              t += STEP_TIME;
            }
        }
    }
  return motion_plan;
}

}
