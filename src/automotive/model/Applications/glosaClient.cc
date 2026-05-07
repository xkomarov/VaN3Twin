#include "glosaClient.h"
#include "ns3/SPATEM.h"
#include "ns3/CAM.h"
#include "ns3/vdpTraci.h"
#include "ns3/socket.h"
#include "ns3/network-module.h"
#include <cmath>

namespace ns3 {
NS_LOG_COMPONENT_DEFINE ("glosaClient");

NS_OBJECT_ENSURE_REGISTERED (glosaClient);

TypeId
glosaClient::GetTypeId (void)
{
  static TypeId tid =
      TypeId ("ns3::glosaClient")
          .SetParent<Application> ()
          .SetGroupName ("Applications")
          .AddConstructor<glosaClient> ()
          .AddAttribute ("Model", "Access Technology Model (80211p or lte)", StringValue ("lte"),
                         MakeStringAccessor (&glosaClient::m_model), MakeStringChecker ())
          .AddAttribute ("PrintSummary", "To print summary at the end of simulation",
                         BooleanValue (false), MakeBooleanAccessor (&glosaClient::m_print_summary),
                         MakeBooleanChecker ())
          .AddAttribute ("RealTime", "To compute properly timestamps", BooleanValue (false),
                         MakeBooleanAccessor (&glosaClient::m_real_time), MakeBooleanChecker ())
          .AddAttribute ("CSV", "CSV log name", StringValue (),
                         MakeStringAccessor (&glosaClient::m_csv_name), MakeStringChecker ())
          .AddAttribute ("ServerAddr", "Ip Addr of the server", Ipv4AddressValue ("10.0.0.1"),
                         MakeIpv4AddressAccessor (&glosaClient::m_server_addr),
                         MakeIpv4AddressChecker ())
          .AddAttribute ("Client", "TraCI client for SUMO", PointerValue (0),
                         MakePointerAccessor (&glosaClient::m_client),
                         MakePointerChecker<TraciClient> ())
          .AddAttribute (
              "MetricSupervisor",
              "Metric Supervisor to compute metrics according to 3GPP TR36.885 V14.0.0 page 70",
              PointerValue (0), MakePointerAccessor (&glosaClient::m_metric_supervisor),
              MakePointerChecker<MetricSupervisor> ())
          .AddAttribute ("SendCAM", "To enable/disable the transmission of CAM messages",
                         BooleanValue (true), MakeBooleanAccessor (&glosaClient::m_send_cam),
                         MakeBooleanChecker ());
  return tid;
}

glosaClient::glosaClient ()
{
  NS_LOG_FUNCTION (this);

  m_client = nullptr;
  m_print_summary = true;
  m_already_print = false;
  m_cam_sent = 0;
  m_spatem_received = 0;
}

glosaClient::~glosaClient ()
{
  NS_LOG_FUNCTION (this);
}

void
glosaClient::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  Application::DoDispose ();
}

void
glosaClient::StartApplication (void)
{
  NS_LOG_FUNCTION (this);

  m_id = m_client->GetVehicleId (this->GetNode ());

  if (m_model == "80211p")
    {
      /* Create the socket for TX and RX */
      TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
      m_socket = Socket::CreateSocket (GetNode (), tid);

      /* Bind the socket to local address */
      PacketSocketAddress local;
      local.SetSingleDevice (GetNode ()->GetDevice (0)->GetIfIndex ());
      local.SetPhysicalAddress (GetNode ()->GetDevice (0)->GetAddress ());
      local.SetProtocol (0x8947);
      if (m_socket->Bind (local) == -1)
        {
          NS_FATAL_ERROR ("Failed to bind client socket");
        }

      /* Set the socket to broadcast */
      PacketSocketAddress remote;
      remote.SetSingleDevice (GetNode ()->GetDevice (0)->GetIfIndex ());
      remote.SetPhysicalAddress (GetNode ()->GetDevice (0)->GetBroadcast ());
      remote.SetProtocol (0x8947);
      m_socket->Connect (remote);
    }
  else if (m_model == "lte")
    {
      /* Create the socket for TX and RX */
      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
      m_socket = Socket::CreateSocket (GetNode (), tid);

      /* Bind the socket to local address */
      InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 9);
      if (m_socket->Bind (local) == -1)
        {
          NS_FATAL_ERROR ("Failed to bind client socket");
        }

      /* Connect it to the server */
      InetSocketAddress remote = InetSocketAddress (m_server_addr, 9);
      m_socket->Connect (remote);
    }
  else
    {
      NS_FATAL_ERROR ("No access technology model set - check simulation script - valid models: "
                      "'80211p' or 'lte'");
    }

  /* Create new BTP and GeoNet objects and set them in TLMService and CABasicService */
  m_btp = CreateObject<btp> ();
  m_geoNet = CreateObject<GeoNet> ();

  if (m_metric_supervisor != nullptr)
    {
      m_geoNet->setMetricSupervisor (m_metric_supervisor);
    }

  m_btp->setGeoNet (m_geoNet);
  m_caService.setBTP (m_btp);
  m_tlmService.setBTP (m_btp);

  /* Set sockets, callback, station properties and TraCI VDP in CABasicService */
  m_caService.setSocketTx (m_socket);
  m_caService.setSocketRx (m_socket);
  m_caService.addCARxCallback (
      std::bind (&glosaClient::receiveCAM, this, std::placeholders::_1, std::placeholders::_2));
  m_caService.setStationProperties (std::stol (m_id.substr (3)), StationType_passengerCar);
  m_caService.setRealTime (m_real_time);

  m_tlmService.setRealTime (m_real_time);
  m_tlmService.setSocketRx (m_socket);
  m_tlmService.setStationProperties (std::stol (m_id.substr (3)), StationType_passengerCar);
  m_tlmService.addTLMRxCallback (
      std::bind (&glosaClient::receiveSPATEM, this, std::placeholders::_1, std::placeholders::_2));

  VDP *traci_vdp = new VDPTraCI (m_client, m_id);

  m_btp->setVDP (traci_vdp);
  m_caService.setVDP (traci_vdp);
  // m_tlmService.setVDP (traci_vdp);

  /* Create LDM and mock-populate traffic light static topology (simulates MAPEM) */
  m_LDM = CreateObject<LDM> ();
  m_LDM->setStationID (m_id);
  m_LDM->setTraCIclient (m_client);
  m_LDM->setVDP (traci_vdp);
  populateStaticTLData ();

  /* Create CSV file, if requested */
  if (!m_csv_name.empty ())
    {
      m_csv_ofstream.open (m_csv_name + "-" + m_id + ".csv", std::ofstream::trunc);
      m_csv_ofstream
          << "messageID,originatingStationId,sequence,referenceTime,detectionTime,stationID"
          << std::endl;
    }

  /* Schedule CAM dissemination */
  if (m_send_cam == true)
    {
      // std::srand (Simulator::Now ().GetNanoSeconds ());
      // double desync = ((double) std::rand () / RAND_MAX);
      // m_caService.startCamDissemination (desync);
      Ptr<UniformRandomVariable> desync_rvar = CreateObject<UniformRandomVariable> ();
      desync_rvar->SetAttribute ("Min", DoubleValue (0.0));
      desync_rvar->SetAttribute ("Max", DoubleValue (1.0));
      double desync = desync_rvar->GetValue (); 
      m_caService.startCamDissemination (desync);
    }
}

void
glosaClient::StopApplication ()
{
  NS_LOG_FUNCTION (this);
  Simulator::Cancel (m_sendCamEvent);
  Simulator::Cancel (m_spatemOut);
  Simulator::Cancel (m_glosaUpdateEvent);

  // Ensure glosa is disengaged before shutdown
  if (m_glosaActive)
    {
      m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
      m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
      m_glosaActive = false;
    }

  if (m_LDM)
    {
      m_LDM->cleanup ();
    }

  uint64_t cam_sent;
  cam_sent = m_caService.terminateDissemination ();
  m_tlmService.terminateDissemination ();

  if (!m_csv_name.empty ())
    m_csv_ofstream.close ();

  if (m_print_summary && !m_already_print)
    {
      std::cout << "INFO-" << m_id << ",CAM-SENT:" << cam_sent
                << ",SPATEM-RECEIVED:" << m_spatem_received << std::endl;
      m_already_print = true;
    }
}

void
glosaClient::StopApplicationNow ()
{
  NS_LOG_FUNCTION (this);
  StopApplication ();
}

void
glosaClient::receiveCAM (asn1cpp::Seq<CAM> cam, Address from)
{
  (void) cam;
  (void) from;
}

void
glosaClient::receiveSPATEM (asn1cpp::Seq<SPATEM> spatem, Address from)
{
  Simulator::Cancel (m_spatemOut);
  m_spatem_received++;

  // === Step 1: Update LDM with received SPATEM data (states + timing) ===
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

              // Also store timing data in the LDM
              trafficLightData_t tlData;
              if (m_LDM->lookupTL (intersectionID, tlData) == LDM::LDM_OK)
                {
                  if (movement->state_time_speed.list.array[0]->timing != nullptr)
                    {
                      tlData.signalGroupTimings[movement->signalGroup] =
                          movement->state_time_speed.list.array[0]->timing->minEndTime;
                    }
                  // Re-insert to update the timing data
                  m_LDM->insertStaticTL (tlData);
                }
            }
        }
    }

  // Mark that fresh SPATEM data is available
  m_spatemAlive = true;

  // Start or keep the periodic glosa control loop running
  if (m_glosaUpdateEvent.IsExpired ())
    {
      m_glosaUpdateEvent =
          Simulator::Schedule (MilliSeconds (200), &glosaClient::updateglosaControl, this);
    }

  // Reschedule SPATEM-loss timeout (if no SPATEM for 2 s -> disengage)
  m_spatemOut = Simulator::Schedule (Seconds (2.0), &glosaClient::spatemOut, this);
}

void
glosaClient::applyGlosaAction (double speed, int r, int g, int b)
{
  m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 7);
  m_client->TraCIAPI::vehicle.setSpeed (m_id, speed);
  m_glosaActive = true;

  libsumo::TraCIColor color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = 255;
  m_client->TraCIAPI::vehicle.setColor (m_id, color);
}

void
glosaClient::updateglosaControl (void)
{
  // If SPATEM data is stale, do nothing (spatemOut will clean up)
  if (!m_spatemAlive)
    {
      return;
    }

  // --- Ego vehicle state ---
  std::string currentLane = m_client->TraCIAPI::vehicle.getLaneID (m_id);
  libsumo::TraCIPosition egoXY = m_client->TraCIAPI::vehicle.getPosition (m_id);
  libsumo::TraCIPosition egoLL =
      m_client->TraCIAPI::simulation.convertXYtoLonLat (egoXY.x, egoXY.y);
  double currentSpeed = m_client->TraCIAPI::vehicle.getSpeed (m_id);
  double maxSpeed = m_client->TraCIAPI::vehicle.getAllowedSpeed (m_id);

  // --- Algorithm constants ---
  const double minSpeed = 3.0; // m/s (~11 km/h) — below this GLOSA is impractical
  const double glosa_MIN_DIST = 10.0; // m — too close to the stop line
  const double glosa_MAX_DIST = 400.0; // m — too far, timing unreliable

  // --- Find nearby traffic lights from LDM ---
  std::vector<trafficLightData_t> nearbyTLs;
  m_LDM->rangeSelectTL (500.0, egoLL.y, egoLL.x, nearbyTLs);

  if (nearbyTLs.empty ())
    {
      // No TLs nearby — release control
      if (m_glosaActive)
        {
          m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
          m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
          m_glosaActive = false;
        }
      m_passedIntersectionID = 0;
      m_glosaUpdateEvent =
          Simulator::Schedule (MilliSeconds (200), &glosaClient::updateglosaControl, this);
      return;
    }

  // --- Find the TL that controls the vehicle's current lane ---
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
      if (m_glosaActive)
        {
          m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
          m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
          m_glosaActive = false;
        }
      m_passedIntersectionID = 0;
      m_glosaUpdateEvent =
          Simulator::Schedule (MilliSeconds (200), &glosaClient::updateglosaControl, this);
      return;
    }

  // --- Skip intersection we already passed ---
  if (matchedTL.intersectionID == m_passedIntersectionID)
    {
      m_glosaUpdateEvent =
          Simulator::Schedule (MilliSeconds (200), &glosaClient::updateglosaControl, this);
      return;
    }

  // --- Get traffic light state from LDM ---
  auto stateIt = matchedTL.signalGroupStates.find (matchedSignalGroup);
  if (stateIt == matchedTL.signalGroupStates.end ())
    {
      m_glosaUpdateEvent =
          Simulator::Schedule (MilliSeconds (200), &glosaClient::updateglosaControl, this);
      return;
    }
  long currentLightState = stateIt->second;

  // --- Get distance to stop line from SUMO ---
  double dist = 0.0;
  auto slIt = matchedTL.laneStopLines.find (currentLane);
  if (slIt != matchedTL.laneStopLines.end ())
    {
      double lanePos = m_client->TraCIAPI::vehicle.getLanePosition (m_id);
      dist = slIt->second.laneLen - lanePos;

      // Проверка: проехала ли машина стоп-линию? (Осталось менее 2 метров)
      if (dist <= 2.0)
        {
          m_passedIntersectionID = matchedTL.intersectionID;
          spatemOut ();
          m_spatemAlive = true;
          m_glosaUpdateEvent =
              Simulator::Schedule (MilliSeconds (200), &glosaClient::updateglosaControl, this);
          return;
        }
    }

  // --- GLOSA activation range check ---
  if (dist < glosa_MIN_DIST || dist > glosa_MAX_DIST)
    {
      if (m_glosaActive)
        {
          m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
          m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
          m_glosaActive = false;
        }
      m_glosaUpdateEvent =
          Simulator::Schedule (MilliSeconds (200), &glosaClient::updateglosaControl, this);
      return;
    }

  // --- Get timing from LDM ---
  double t_green_start = 0.0;
  double t_green_end = 0.0;

  if (currentLightState == 6 || currentLightState == 5) // GREEN
    {
      t_green_start = 0.0;
      auto timingIt = matchedTL.signalGroupTimings.find (matchedSignalGroup);
      if (timingIt != matchedTL.signalGroupTimings.end ())
        {
          t_green_end = timingIt->second / 10.0;
        }
    }
  else if (currentLightState == 3) // RED
    {
      auto timingIt = matchedTL.signalGroupTimings.find (matchedSignalGroup);
      if (timingIt != matchedTL.signalGroupTimings.end ())
        {
          t_green_start = timingIt->second / 10.0;
        }
      t_green_end = t_green_start + 15.0;
    }

    if (currentLightState == 3 || currentLightState == 6 || currentLightState == 5) // RED or GREEN
    {
      double bufferTime = 2.0;

      double v_req_min = dist / std::max (0.1, t_green_end - bufferTime);
      double v_req_max;

      if (currentLightState == 6 || currentLightState == 5) // GREEN
        {
          v_req_max = maxSpeed;
        }
      else // RED
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
              applyGlosaAction (targetSpeed, 0, 255, 0); // Green
            }
          else if (targetSpeed > currentSpeed)
            {
              applyGlosaAction (targetSpeed, 50, 205, 50); // Lighter green
            }
          else
            {
              applyGlosaAction (targetSpeed, 173, 255, 47); // Yellow-green
            }
        }
      else
        {
          // Fallback to SUMO
          if (m_glosaActive)
            {
              m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
              m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
              m_glosaActive = false;
            }
          libsumo::TraCIColor orange;
          orange.r = 255; orange.g = 99; orange.b = 71; orange.a = 255;
          m_client->TraCIAPI::vehicle.setColor (m_id, orange);
        }
    }
    else if (currentLightState == 7) // YELLOW
    {
      double eta = dist / std::max (currentSpeed, 1.0);
      auto timingIt = matchedTL.signalGroupTimings.find (matchedSignalGroup);
      double timeToSwitch = 0.0;
      if (timingIt != matchedTL.signalGroupTimings.end ())
        timeToSwitch = timingIt->second / 10.0;

      if (eta < timeToSwitch)
        {
          applyGlosaAction (currentSpeed, 255, 191, 0);
        }
      else
        {
          if (m_glosaActive)
            {
              m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
              m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
              m_glosaActive = false;
            }
          libsumo::TraCIColor orange;
          orange.r = 255; orange.g = 99; orange.b = 71; orange.a = 255;
          m_client->TraCIAPI::vehicle.setColor (m_id, orange);
        }
    }
    else // RED
    {
      m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 7);
      m_client->TraCIAPI::vehicle.setSpeed (m_id, currentSpeed);
      m_glosaActive = true;
    }

  // Reschedule next GLOSA tick
  m_glosaUpdateEvent =
      Simulator::Schedule (MilliSeconds (200), &glosaClient::updateglosaControl, this);
}

void
glosaClient::populateStaticTLData (void)
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
glosaClient::spatemOut ()
{
  m_spatemAlive = false;

  // Restore full SUMO control
  if (m_glosaActive)
    {
      m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
      m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
      m_glosaActive = false;
    }

  libsumo::TraCIColor orange;
  orange.r = 255;
  orange.g = 99;
  orange.b = 71;
  orange.a = 255;
  m_client->TraCIAPI::vehicle.setColor (m_id, orange);
}

} // namespace ns3
