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
#include "greenWindowClient.h"
#include "ns3/SPATEM.h"
#include "ns3/CAM.h"
#include "ns3/vdpTraci.h"
#include "ns3/socket.h"
#include "ns3/network-module.h"
#include <cmath>

namespace ns3 {
NS_LOG_COMPONENT_DEFINE ("greenWindowClient");

NS_OBJECT_ENSURE_REGISTERED (greenWindowClient);

TypeId
greenWindowClient::GetTypeId (void)
{
  static TypeId tid =
      TypeId ("ns3::greenWindowClient")
          .SetParent<Application> ()
          .SetGroupName ("Applications")
          .AddConstructor<greenWindowClient> ()
          .AddAttribute ("Model", "Access Technology Model (80211p or lte)", StringValue ("lte"),
                         MakeStringAccessor (&greenWindowClient::m_model), MakeStringChecker ())
          .AddAttribute (
              "PrintSummary", "To print summary at the end of simulation", BooleanValue (false),
              MakeBooleanAccessor (&greenWindowClient::m_print_summary), MakeBooleanChecker ())
          .AddAttribute ("RealTime", "To compute properly timestamps", BooleanValue (false),
                         MakeBooleanAccessor (&greenWindowClient::m_real_time),
                         MakeBooleanChecker ())
          .AddAttribute ("CSV", "CSV log name", StringValue (),
                         MakeStringAccessor (&greenWindowClient::m_csv_name), MakeStringChecker ())
          .AddAttribute ("ServerAddr", "Ip Addr of the server", Ipv4AddressValue ("10.0.0.1"),
                         MakeIpv4AddressAccessor (&greenWindowClient::m_server_addr),
                         MakeIpv4AddressChecker ())
          .AddAttribute ("Client", "TraCI client for SUMO", PointerValue (0),
                         MakePointerAccessor (&greenWindowClient::m_client),
                         MakePointerChecker<TraciClient> ())
          .AddAttribute (
              "MetricSupervisor",
              "Metric Supervisor to compute metrics according to 3GPP TR36.885 V14.0.0 page 70",
              PointerValue (0), MakePointerAccessor (&greenWindowClient::m_metric_supervisor),
              MakePointerChecker<MetricSupervisor> ())
          .AddAttribute ("SendCAM", "To enable/disable the transmission of CAM messages",
                         BooleanValue (true), MakeBooleanAccessor (&greenWindowClient::m_send_cam),
                         MakeBooleanChecker ());
  return tid;
}

greenWindowClient::greenWindowClient ()
{
  NS_LOG_FUNCTION (this);

  m_client = nullptr;
  m_print_summary = true;
  m_already_print = false;
  m_cam_sent = 0;
  m_spatem_received = 0;
}

greenWindowClient::~greenWindowClient ()
{
  NS_LOG_FUNCTION (this);
}

void
greenWindowClient::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  Application::DoDispose ();
}

void
greenWindowClient::StartApplication (void)
{
  NS_LOG_FUNCTION (this);

  m_id = m_client->GetVehicleId (this->GetNode ());

  if (m_model == "80211p")
    {

      TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
      m_socket = Socket::CreateSocket (GetNode (), tid);

      PacketSocketAddress local;
      local.SetSingleDevice (GetNode ()->GetDevice (0)->GetIfIndex ());
      local.SetPhysicalAddress (GetNode ()->GetDevice (0)->GetAddress ());
      local.SetProtocol (0x8947);
      if (m_socket->Bind (local) == -1)
        {
          NS_FATAL_ERROR ("Failed to bind client socket");
        }

      PacketSocketAddress remote;
      remote.SetSingleDevice (GetNode ()->GetDevice (0)->GetIfIndex ());
      remote.SetPhysicalAddress (GetNode ()->GetDevice (0)->GetBroadcast ());
      remote.SetProtocol (0x8947);
      m_socket->Connect (remote);
    }
  else if (m_model == "lte")
    {

      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
      m_socket = Socket::CreateSocket (GetNode (), tid);

      InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 9);
      if (m_socket->Bind (local) == -1)
        {
          NS_FATAL_ERROR ("Failed to bind client socket");
        }

      InetSocketAddress remote = InetSocketAddress (m_server_addr, 9);
      m_socket->Connect (remote);
    }
  else
    {
      NS_FATAL_ERROR ("No access technology model set - check simulation script - valid models: "
                      "'80211p' or 'lte'");
    }

  m_btp = CreateObject<btp> ();
  m_geoNet = CreateObject<GeoNet> ();

  if (m_metric_supervisor != nullptr)
    {
      m_geoNet->setMetricSupervisor (m_metric_supervisor);
    }

  m_btp->setGeoNet (m_geoNet);
  m_caService.setBTP (m_btp);
  m_tlmService.setBTP (m_btp);

  m_caService.setSocketTx (m_socket);
  m_caService.setSocketRx (m_socket);
  m_caService.addCARxCallback (std::bind (&greenWindowClient::receiveCAM, this,
                                          std::placeholders::_1, std::placeholders::_2));
  m_caService.setStationProperties (std::stol (m_id.substr (3)), StationType_passengerCar);
  m_caService.setRealTime (m_real_time);

  m_tlmService.setRealTime (m_real_time);
  m_tlmService.setSocketRx (m_socket);
  m_tlmService.setStationProperties (std::stol (m_id.substr (3)), StationType_passengerCar);
  m_tlmService.addTLMRxCallback (std::bind (&greenWindowClient::receiveSPATEM, this,
                                            std::placeholders::_1, std::placeholders::_2));

  VDP *traci_vdp = new VDPTraCI (m_client, m_id);

  m_btp->setVDP (traci_vdp);
  m_caService.setVDP (traci_vdp);

  m_LDM = CreateObject<LDM> ();
  m_LDM->setStationID (m_id);
  m_LDM->setTraCIclient (m_client);
  m_LDM->setVDP (traci_vdp);
  populateStaticTLData ();

  if (!m_csv_name.empty ())
    {

      m_tlm_metrics_csv.open (m_csv_name + "-" + m_id + "-tlm_correctness.csv",
                              std::ofstream::trunc);
      m_tlm_metrics_csv << "sim_time_ms,vehId,ldmState,sumoState,phaseMatch,"
                        << "ldmMinEndTime_s,sumoNextSwitch_s,tee_s,dist_m,speed_ms" << std::endl;
    }

  if (m_send_cam == true)
    {

      Ptr<UniformRandomVariable> desync_rvar = CreateObject<UniformRandomVariable> ();
      desync_rvar->SetAttribute ("Min", DoubleValue (0.0));
      desync_rvar->SetAttribute ("Max", DoubleValue (1.0));
      double desync = desync_rvar->GetValue ();
      m_caService.startCamDissemination (desync);
    }
}

void
greenWindowClient::StopApplication ()
{
  NS_LOG_FUNCTION (this);
  Simulator::Cancel (m_sendCamEvent);
  Simulator::Cancel (m_spatemOut);
  Simulator::Cancel (m_greenWindowUpdateEvent);

  if (m_greenWindowActive)
    {
      try
        {
          m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
          m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
        }
      catch (libsumo::TraCIException &e)
        {
        }
      m_greenWindowActive = false;
    }

  if (m_LDM)
    {
      m_LDM->cleanup ();
    }

  uint64_t cam_sent;
  cam_sent = m_caService.terminateDissemination ();
  m_tlmService.terminateDissemination ();

  if (!m_csv_name.empty ())
    {
      m_tlm_metrics_csv.close ();
    }

  if (m_print_summary && !m_already_print)
    {
      double fdr = (m_steps_in_range > 0) ? (100.0 * m_fdr_disengagements / m_steps_in_range) : 0.0;
      std::cout << "INFO-" << m_id << ",CAM-SENT:" << cam_sent
                << ",SPATEM-RECEIVED:" << m_spatem_received << ",FDR:" << fdr << "%" << std::endl;
      m_already_print = true;
    }
}

void
greenWindowClient::StopApplicationNow ()
{
  NS_LOG_FUNCTION (this);
  StopApplication ();
}

void
greenWindowClient::receiveCAM (asn1cpp::Seq<CAM> cam, Address from)
{
  (void) cam;
  (void) from;
}

void
greenWindowClient::receiveSPATEM (asn1cpp::Seq<SPATEM> spatem, Address from)
{
  Simulator::Cancel (m_spatemOut);
  m_spatem_received++;

  for (int j = 0; j < spatem->spat.intersections.list.count; ++j)
    {
      auto intersection = spatem->spat.intersections.list.array[j];
      uint64_t intersectionID = intersection->id.id;

      for (int i = 0; i < intersection->states.list.count; ++i)
        {
          auto movement = intersection->states.list.array[i];
          if (movement->state_time_speed.list.count > 0)
            {
              long eventState = movement->state_time_speed.list.array[0]->eventState;
              m_LDM->updateTLState (intersectionID, movement->signalGroup, eventState);

              trafficLightData_t tlData;
              if (m_LDM->lookupTL (intersectionID, tlData) == LDM::LDM_OK)
                {
                  if (movement->state_time_speed.list.array[0]->timing != nullptr)
                    {
                      tlData.signalGroupTimings[movement->signalGroup] =
                          movement->state_time_speed.list.array[0]->timing->minEndTime;
                    }

                  m_LDM->insertStaticTL (tlData);
                }
            }
        }
    }

  m_spatemAlive = true;

  if (m_greenWindowUpdateEvent.IsExpired ())
    {
      m_greenWindowUpdateEvent = Simulator::Schedule (
          MilliSeconds (200), &greenWindowClient::updategreenWindowControl, this);
    }

  m_last_spatem_rx_time_ms = Simulator::Now ().GetMilliSeconds ();

  m_spatemOut = Simulator::Schedule (Seconds (2.0), &greenWindowClient::spatemOut, this);
}

void
greenWindowClient::applygreenWindowAction (double speed, int r, int g, int b)
{
  m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 7);
  m_client->TraCIAPI::vehicle.setSpeed (m_id, speed);
  m_greenWindowActive = true;

  libsumo::TraCIColor color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = 255;
  m_client->TraCIAPI::vehicle.setColor (m_id, color);
}

void
greenWindowClient::updategreenWindowControl (void)
{

  if (!m_spatemAlive)
    {
      return;
    }

  std::string currentLane;
  libsumo::TraCIPosition egoXY;
  libsumo::TraCIPosition egoLL;
  double currentSpeed;
  double maxSpeed;
  try
    {
      currentLane = m_client->TraCIAPI::vehicle.getLaneID (m_id);
      egoXY = m_client->TraCIAPI::vehicle.getPosition (m_id);
      egoLL = m_client->TraCIAPI::simulation.convertXYtoLonLat (egoXY.x, egoXY.y);
      currentSpeed = m_client->TraCIAPI::vehicle.getSpeed (m_id);
      maxSpeed = m_client->TraCIAPI::vehicle.getAllowedSpeed (m_id);
    }
  catch (libsumo::TraCIException &e)
    {
      return;
    }

  const double minSpeed = 3.0;
  const double greenWindow_MIN_DIST = 10.0;
  const double greenWindow_MAX_DIST = 400.0;

  std::vector<trafficLightData_t> nearbyTLs;
  m_LDM->rangeSelectTL (500.0, egoLL.y, egoLL.x, nearbyTLs);

  if (nearbyTLs.empty ())
    {

      if (m_greenWindowActive)
        {
          m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
          m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
          m_greenWindowActive = false;
        }
      m_passedIntersectionID = 0;
      m_greenWindowUpdateEvent = Simulator::Schedule (
          MilliSeconds (200), &greenWindowClient::updategreenWindowControl, this);
      return;
    }

  trafficLightData_t matchedTL;
  long matchedSignalGroup = -1;
  bool found = false;

  for (const auto &tl : nearbyTLs)
    {
      auto sgIt = tl.laneToSignalGroup.find (currentLane);
      if (sgIt != tl.laneToSignalGroup.end ())
        {
          matchedTL = tl;
          matchedSignalGroup = sgIt->second;
          found = true;
          break;
        }
    }

  if (!found || matchedTL.signalGroupStates.empty ())
    {
      if (m_greenWindowActive)
        {
          m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
          m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
          m_greenWindowActive = false;
        }
      m_passedIntersectionID = 0;
      m_greenWindowUpdateEvent = Simulator::Schedule (
          MilliSeconds (200), &greenWindowClient::updategreenWindowControl, this);
      return;
    }

  if (matchedTL.intersectionID == m_passedIntersectionID)
    {
      m_greenWindowUpdateEvent = Simulator::Schedule (
          MilliSeconds (200), &greenWindowClient::updategreenWindowControl, this);
      return;
    }

  auto stateIt = matchedTL.signalGroupStates.find (matchedSignalGroup);
  if (stateIt == matchedTL.signalGroupStates.end ())
    {
      m_greenWindowUpdateEvent = Simulator::Schedule (
          MilliSeconds (200), &greenWindowClient::updategreenWindowControl, this);
      return;
    }
  long currentLightState = stateIt->second;

  double dist = 0.0;
  auto slIt = matchedTL.laneStopLines.find (currentLane);
  if (slIt != matchedTL.laneStopLines.end ())
    {
      double lanePos = m_client->TraCIAPI::vehicle.getLanePosition (m_id);
      dist = slIt->second.laneLen - lanePos;

      if (dist <= 2.0)
        {
          m_passedIntersectionID = matchedTL.intersectionID;
          spatemOut ();
          m_spatemAlive = true;
          m_greenWindowUpdateEvent = Simulator::Schedule (
              MilliSeconds (200), &greenWindowClient::updategreenWindowControl, this);
          return;
        }
    }

  if (dist < greenWindow_MIN_DIST || dist > greenWindow_MAX_DIST)
    {
      if (m_greenWindowActive)
        {
          m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
          m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
          m_greenWindowActive = false;
        }
      m_greenWindowUpdateEvent = Simulator::Schedule (
          MilliSeconds (200), &greenWindowClient::updategreenWindowControl, this);
      return;
    }

  if (m_tlm_metrics_csv.is_open ())
    {
      double sim_time_ms = Simulator::Now ().GetMilliSeconds ();

      long sumo_state = -1;
      double sumo_next_switch_s = -1.0;
      std::string matched_tls_id;
      auto tlsIDs = m_client->TraCIAPI::trafficlights.getIDList ();
      for (const auto &tlsId : tlsIDs)
        {
          if ((uint16_t) std::hash<std::string>{}(tlsId) == matchedTL.intersectionID)
            {
              matched_tls_id = tlsId;
              break;
            }
        }

      if (!matched_tls_id.empty ())
        {

          std::string state_str =
              m_client->TraCIAPI::trafficlights.getRedYellowGreenState (matched_tls_id);
          int sg_idx = (int) (matchedSignalGroup - 1);
          if (sg_idx >= 0 && sg_idx < (int) state_str.size ())
            {
              char c = state_str[sg_idx];

              if (c == 'G')
                sumo_state = 6;
              else if (c == 'g')
                sumo_state = 5;
              else if (c == 'r' || c == 'R')
                sumo_state = 3;
              else if (c == 'y' || c == 'Y')
                sumo_state = 7;
            }

          double next_switch_abs_ms =
              m_client->TraCIAPI::trafficlights.getNextSwitch (matched_tls_id);
          sumo_next_switch_s = (next_switch_abs_ms - sim_time_ms) / 1000.0;
          if (sumo_next_switch_s < 0.0)
            sumo_next_switch_s = 0.0;
        }

      double ldm_min_end_s = -1.0;
      auto timingMetricIt = matchedTL.signalGroupTimings.find (matchedSignalGroup);
      if (timingMetricIt != matchedTL.signalGroupTimings.end ())
        {
          double elapsed_since_rx_s = (m_last_spatem_rx_time_ms >= 0)
                                          ? (sim_time_ms - m_last_spatem_rx_time_ms) / 1000.0
                                          : 0.0;
          ldm_min_end_s = timingMetricIt->second / 10.0 - elapsed_since_rx_s;
          if (ldm_min_end_s < 0.0)
            ldm_min_end_s = 0.0;
        }

      double tee_s = (ldm_min_end_s >= 0.0 && sumo_next_switch_s >= 0.0)
                         ? std::abs (ldm_min_end_s - sumo_next_switch_s)
                         : -1.0;

      int phase_match = (sumo_state == currentLightState) ? 1 : 0;

      m_steps_in_range++;

      m_tlm_metrics_csv << (long long) sim_time_ms << "," << m_id << "," << currentLightState << ","
                        << sumo_state << "," << phase_match << "," << ldm_min_end_s << ","
                        << sumo_next_switch_s << "," << tee_s << "," << dist << "," << currentSpeed
                        << std::endl;
    }

  double t_green_start = 0.0;
  double t_green_end = 0.0;

  if (currentLightState == 6 || currentLightState == 5)
    {
      t_green_start = 0.0;
      auto timingIt = matchedTL.signalGroupTimings.find (matchedSignalGroup);
      if (timingIt != matchedTL.signalGroupTimings.end ())
        {
          t_green_end = timingIt->second / 10.0;
        }
    }
  else if (currentLightState == 3)
    {
      auto timingIt = matchedTL.signalGroupTimings.find (matchedSignalGroup);
      if (timingIt != matchedTL.signalGroupTimings.end ())
        {
          t_green_start = timingIt->second / 10.0;
        }
      t_green_end = t_green_start + 20.0;
    }

  if (currentLightState == 3 || currentLightState == 6 || currentLightState == 5)
    {
      double bufferTime = 2.0;

      double v_req_min = dist / std::max (0.1, t_green_end - bufferTime);
      double v_req_max;

      if (currentLightState == 6 || currentLightState == 5)
        {
          v_req_max = maxSpeed;
        }
      else
        {
          v_req_max = dist / std::max (0.1, t_green_start + bufferTime);
        }

      if (v_req_max > maxSpeed)
        {
          v_req_max = maxSpeed;
        }

      bool canMakeIt = true;
      double targetSpeed = currentSpeed;

      if (v_req_min > maxSpeed || v_req_max < minSpeed)
        {
          canMakeIt = false;
        }

      if (canMakeIt)
        {
          if (currentSpeed >= v_req_min && currentSpeed <= v_req_max)
            {
              targetSpeed = currentSpeed;
            }
          else if (currentSpeed < v_req_min)
            {
              targetSpeed = v_req_min;
            }
          else if (currentSpeed > v_req_max)
            {
              targetSpeed = v_req_max;
            }

          if (std::abs (targetSpeed - currentSpeed) < 1.0)
            {
              applygreenWindowAction (targetSpeed, 0, 255, 0);
            }
          else if (targetSpeed > currentSpeed)
            {
              applygreenWindowAction (targetSpeed, 50, 205, 50);
            }
          else
            {
              applygreenWindowAction (targetSpeed, 173, 255, 47);
            }
        }
      else
        {

          if (m_greenWindowActive)
            {
              m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
              m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
              m_greenWindowActive = false;
            }
          libsumo::TraCIColor orange;
          orange.r = 255;
          orange.g = 99;
          orange.b = 71;
          orange.a = 255;
          m_client->TraCIAPI::vehicle.setColor (m_id, orange);
        }
    }
  else if (currentLightState == 7)
    {
      double eta = dist / std::max (currentSpeed, 1.0);
      auto timingIt = matchedTL.signalGroupTimings.find (matchedSignalGroup);
      double timeToSwitch = 0.0;
      if (timingIt != matchedTL.signalGroupTimings.end ())
        timeToSwitch = timingIt->second / 10.0;

      if (eta < timeToSwitch)
        {
          applygreenWindowAction (currentSpeed, 255, 191, 0);
        }
      else
        {
          if (m_greenWindowActive)
            {
              m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
              m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
              m_greenWindowActive = false;
            }
          libsumo::TraCIColor orange;
          orange.r = 255;
          orange.g = 99;
          orange.b = 71;
          orange.a = 255;
          m_client->TraCIAPI::vehicle.setColor (m_id, orange);
        }
    }
  else
    {
      m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 7);
      m_client->TraCIAPI::vehicle.setSpeed (m_id, currentSpeed);
      m_greenWindowActive = true;
    }

  m_greenWindowUpdateEvent =
      Simulator::Schedule (MilliSeconds (200), &greenWindowClient::updategreenWindowControl, this);
}

void
greenWindowClient::populateStaticTLData (void)
{
  auto tlsIDs = m_client->TraCIAPI::trafficlights.getIDList ();

  for (const auto &tlsId : tlsIDs)
    {
      trafficLightData_t tlData;

      tlData.intersectionID = (uint16_t) std::hash<std::string>{}(tlsId);

      libsumo::TraCIPosition posXY = m_client->TraCIAPI::junction.getPosition (tlsId);
      libsumo::TraCIPosition posLonLat =
          m_client->TraCIAPI::simulation.convertXYtoLonLat (posXY.x, posXY.y);
      tlData.lat = posLonLat.y;
      tlData.lon = posLonLat.x;

      auto controlledLanes = m_client->TraCIAPI::trafficlights.getControlledLanes (tlsId);

      for (int idx = 0; idx < (int) controlledLanes.size (); ++idx)
        {
          const std::string &laneId = controlledLanes[idx];
          long signalGroup = idx + 1;

          if (tlData.laneToSignalGroup.find (laneId) == tlData.laneToSignalGroup.end ())
            {
              tlData.laneToSignalGroup[laneId] = signalGroup;
            }

          if (tlData.laneStopLines.find (laneId) == tlData.laneStopLines.end ())
            {
              auto laneShape = m_client->TraCIAPI::lane.getShape (laneId);
              if (!laneShape.empty ())
                {
                  laneStopLinePos_t stopLine;
                  stopLine.x = laneShape.back ().x;
                  stopLine.y = laneShape.back ().y;
                  stopLine.laneLen = m_client->TraCIAPI::lane.getLength (laneId);
                  tlData.laneStopLines[laneId] = stopLine;
                }
            }
        }

      tlData.isStaticLoaded = true;
      tlData.timestamp_us = 0;

      m_LDM->insertStaticTL (tlData);
    }
}

void
greenWindowClient::spatemOut ()
{
  m_spatemAlive = false;

  if (m_greenWindowActive)
    m_fdr_disengagements++;

  try
    {
      if (m_greenWindowActive)
        {
          m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
          m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
          m_greenWindowActive = false;
        }

      libsumo::TraCIColor orange;
      orange.r = 255;
      orange.g = 99;
      orange.b = 71;
      orange.a = 255;
      m_client->TraCIAPI::vehicle.setColor (m_id, orange);
    }
  catch (libsumo::TraCIException &e)
    {
    }
}

} // namespace ns3
