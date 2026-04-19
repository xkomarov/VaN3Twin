
#include "tlmClient80211p.h"
#include "ns3/SPATEM.h"
#include "ns3/CAM.h"
#include "ns3/vdpTraci.h"
#include "ns3/socket.h"
#include "ns3/network-module.h"
#include <cmath>
#include <limits>

namespace ns3 {
NS_LOG_COMPONENT_DEFINE ("tlmClient80211p");

NS_OBJECT_ENSURE_REGISTERED (tlmClient80211p);

TypeId
tlmClient80211p::GetTypeId (void)
{
  static TypeId tid =
      TypeId ("ns3::tlmClient80211p")
          .SetParent<Application> ()
          .SetGroupName ("Applications")
          .AddConstructor<tlmClient80211p> ()
          .AddAttribute (
              "PrintSummary", "To print summary at the end of simulation", BooleanValue (false),
              MakeBooleanAccessor (&tlmClient80211p::m_print_summary), MakeBooleanChecker ())
          .AddAttribute ("RealTime", "To compute properly timestamps", BooleanValue (false),
                         MakeBooleanAccessor (&tlmClient80211p::m_real_time), MakeBooleanChecker ())
          .AddAttribute ("CSV", "CSV log name", StringValue (),
                         MakeStringAccessor (&tlmClient80211p::m_csv_name), MakeStringChecker ())
          .AddAttribute ("ServerAddr", "Ip Addr of the server", Ipv4AddressValue ("10.0.0.1"),
                         MakeIpv4AddressAccessor (&tlmClient80211p::m_server_addr),
                         MakeIpv4AddressChecker ())
          .AddAttribute ("Client", "TraCI client for SUMO", PointerValue (0),
                         MakePointerAccessor (&tlmClient80211p::m_client),
                         MakePointerChecker<TraciClient> ())
          .AddAttribute (
              "MetricSupervisor",
              "Metric Supervisor to compute metrics according to 3GPP TR36.885 V14.0.0 page 70",
              PointerValue (0), MakePointerAccessor (&tlmClient80211p::m_metric_supervisor),
              MakePointerChecker<MetricSupervisor> ())
          .AddAttribute ("SendCAM", "To enable/disable the transmission of CAM messages",
                         BooleanValue (true), MakeBooleanAccessor (&tlmClient80211p::m_send_cam),
                         MakeBooleanChecker ());
  return tid;
}

tlmClient80211p::tlmClient80211p ()
{
  NS_LOG_FUNCTION (this);

  m_client = nullptr;
  m_print_summary = true;
  m_already_print = false;
  m_cam_sent = 0;
  m_spatem_received = 0;
}

tlmClient80211p::~tlmClient80211p ()
{
  NS_LOG_FUNCTION (this);
}

void
tlmClient80211p::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  Application::DoDispose ();
}

void
tlmClient80211p::StartApplication (void)
{
  NS_LOG_FUNCTION (this);

  /*
     * This application works as client for the tlmServer80211p. It is intended to be installed over a vehicular OBU node,
     * and it is set to generate broadcast CAM messages on top of BTP and GeoNet.
     * The RLVW algorithm checks whether the vehicle can safely stop at a red or yellow
     * traffic light, and triggers emergency braking if not.
     */

  m_id = m_client->GetVehicleId (this->GetNode ());

  /* Create the socket for TX and RX */
  TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");

  /* Socket used to send CAMs and receive SPATEMs */
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

  /* Create new BTP and GeoNet objects and set them in TLMBasicService and CABasicService */
  m_btp = CreateObject<btp> ();
  m_geoNet = CreateObject<GeoNet> ();

  if (m_metric_supervisor != nullptr)
    {
      m_geoNet->setMetricSupervisor (m_metric_supervisor);
    }

  m_btp->setGeoNet (m_geoNet);
  m_caService.setBTP (m_btp);
  m_tlmBasicService.setBTP (m_btp);

  /* Set sockets, callback, station properties and TraCI VDP in CABasicService */
  m_caService.setSocketTx (m_socket);
  m_caService.setSocketRx (m_socket);
  m_caService.addCARxCallback (
      std::bind (&tlmClient80211p::receiveCAM, this, std::placeholders::_1, std::placeholders::_2));
  m_caService.setStationProperties (std::stol (m_id.substr (3)), StationType_passengerCar);
  m_caService.setRealTime (m_real_time);

  m_tlmBasicService.setRealTime (m_real_time);
  m_tlmBasicService.setSocketRx (m_socket);
  m_tlmBasicService.setStationProperties (std::stol (m_id.substr (3)), StationType_passengerCar);
  m_tlmBasicService.addTLMRxCallback (std::bind (&tlmClient80211p::receiveSPATEM, this,
                                                 std::placeholders::_1, std::placeholders::_2));

  VDP *traci_vdp = new VDPTraCI (m_client, m_id);

  m_btp->setVDP (traci_vdp);

  m_caService.setVDP (traci_vdp);

  m_tlmBasicService.setVDP (traci_vdp);

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
      std::srand (Simulator::Now ().GetNanoSeconds ());
      double desync = ((double) std::rand () / RAND_MAX);
      m_caService.startCamDissemination (desync);
    }
}

void
tlmClient80211p::StopApplication ()
{
  NS_LOG_FUNCTION (this);
  Simulator::Cancel (m_sendCamEvent);
  Simulator::Cancel (m_spatemTimeout);
  Simulator::Cancel (m_rlvwUpdateEvent);

  // Ensure RLVW braking is disengaged before shutdown
  if (m_rlvwActive)
    {
      m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
      m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
      m_rlvwActive = false;
    }

  if (m_LDM)
    {
      m_LDM->cleanup ();
    }

  uint64_t cam_sent;
  cam_sent = m_caService.terminateDissemination ();
  m_tlmBasicService.terminateDissemination ();

  if (!m_csv_name.empty ())
    m_csv_ofstream.close ();

  if (m_print_summary && !m_already_print)
    {
      std::cout << "INFO-" << m_id << ",CAM-SENT:" << cam_sent
                << ",SPATEM-RECEIVED:" << m_spatem_received
                << ",RLVW-WARNINGS:" << m_rlvw_warnings << std::endl;
      m_already_print = true;
    }
}

void
tlmClient80211p::StopApplicationNow ()
{
  NS_LOG_FUNCTION (this);
  StopApplication ();
}

void
tlmClient80211p::receiveCAM (asn1cpp::Seq<CAM> cam, Address from)
{
  (void) cam;
  (void) from;
}


void
tlmClient80211p::receiveSPATEM (asn1cpp::Seq<SPATEM> spatem, Address from)
{
  Simulator::Cancel (m_spatemTimeout);
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

  // Start or keep the periodic RLVW control loop running
  if (m_rlvwUpdateEvent.IsExpired ())
    {
      m_rlvwUpdateEvent = Simulator::Schedule (MilliSeconds (200),
                                                &tlmClient80211p::updateRlvwControl, this);
    }

  // Reschedule SPATEM-loss timeout (if no SPATEM for 2 s → disengage)
  m_spatemTimeout = Simulator::Schedule (Seconds (2.0), &tlmClient80211p::spatemTimeout, this);
}


void
tlmClient80211p::updateRlvwControl (void)
{
  // If SPATEM data is stale, do nothing (spatemTimeout will clean up)
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

  // --- RLVW algorithm constants ---
  const double comfortDecel   = 3.0;    // m/s² — comfortable deceleration
  const double emergencyDecel = 6.0;    // m/s² — emergency braking deceleration
  const double safetyMargin   = 1.3;    // safety factor applied to stopping distance
  const double RLVW_MIN_DIST  = 5.0;    // m — too close (already in intersection)
  const double RLVW_MAX_DIST  = 200.0;  // m — too far away to matter

  // --- Find nearby traffic lights from LDM ---
  std::vector<trafficLightData_t> nearbyTLs;
  m_LDM->rangeSelectTL (300.0, egoLL.y, egoLL.x, nearbyTLs);

  if (nearbyTLs.empty ())
    {
      // No TLs nearby — release control if active
      if (m_rlvwActive)
        {
          m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
          m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
          m_rlvwActive = false;
        }
      m_passedIntersectionID = 0;
      m_rlvwUpdateEvent = Simulator::Schedule (MilliSeconds (200),
                                                &tlmClient80211p::updateRlvwControl, this);
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
      // Vehicle not on an approach lane — release RLVW
      if (m_rlvwActive)
        {
          m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
          m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
          m_rlvwActive = false;
        }
      m_passedIntersectionID = 0;
      m_rlvwUpdateEvent = Simulator::Schedule (MilliSeconds (200),
                                                &tlmClient80211p::updateRlvwControl, this);
      return;
    }

  // --- Skip intersection we already passed ---
  if (matchedTL.intersectionID == m_passedIntersectionID)
    {
      m_rlvwUpdateEvent = Simulator::Schedule (MilliSeconds (200),
                                                &tlmClient80211p::updateRlvwControl, this);
      return;
    }

  // --- Get traffic light state from LDM ---
  auto stateIt = matchedTL.signalGroupStates.find (matchedSignalGroup);
  if (stateIt == matchedTL.signalGroupStates.end ())
    {
      m_rlvwUpdateEvent = Simulator::Schedule (MilliSeconds (200),
                                                &tlmClient80211p::updateRlvwControl, this);
      return;
    }
  long currentLightState = stateIt->second;

  // --- Compute distance to stop line ---
  double dist = 0.0;
  auto slIt = matchedTL.laneStopLines.find (currentLane);
  if (slIt != matchedTL.laneStopLines.end ())
    {
      double dx = egoXY.x - slIt->second.x;
      double dy = egoXY.y - slIt->second.y;
      dist = std::sqrt (dx * dx + dy * dy);

      // Check: has the vehicle already passed the stop line?
      double lanePos = m_client->TraCIAPI::vehicle.getLanePosition (m_id);
      if (lanePos >= slIt->second.laneLen - 2.0)
        {
          // Record passed intersection — trigger spatemTimeout for full reset
          m_passedIntersectionID = matchedTL.intersectionID;
          spatemTimeout ();
          m_spatemAlive = true; // keep listening for future intersections
          m_rlvwUpdateEvent = Simulator::Schedule (MilliSeconds (200),
                                                    &tlmClient80211p::updateRlvwControl, this);
          return;
        }
    }
  else
    {
      // Fallback: haversine to intersection center
      dist = 12742000.0 * std::asin (std::sqrt (
          std::pow (std::sin ((egoLL.y - matchedTL.lat) * M_PI / 360.0), 2) +
          std::cos (egoLL.y * M_PI / 180.0) * std::cos (matchedTL.lat * M_PI / 180.0) *
          std::pow (std::sin ((egoLL.x - matchedTL.lon) * M_PI / 360.0), 2)));
    }

  // --- RLVW activation range check ---
  if (dist < RLVW_MIN_DIST || dist > RLVW_MAX_DIST)
    {
      if (m_rlvwActive)
        {
          m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
          m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
          m_rlvwActive = false;
        }
      m_rlvwUpdateEvent = Simulator::Schedule (MilliSeconds (200),
                                                &tlmClient80211p::updateRlvwControl, this);
      return;
    }

  // Only trigger on RED (eventState=3) or YELLOW (eventState=7)
  bool isRestrictive = (currentLightState == 3 || currentLightState == 7);

  if (isRestrictive && currentSpeed > 1.0)
    {
      // Compute stopping distance with safety margin:
      // d_stop = v² / (2 × a_comfort) × safetyMargin
      double stoppingDist = (currentSpeed * currentSpeed) / (2.0 * comfortDecel) * safetyMargin;

      if (stoppingDist > dist)
        {
          m_rlvw_warnings++;

          // Compute target speed for smooth emergency deceleration:
          // v_target = sqrt(2 × a_emergency × dist)
          double targetSpeed = std::sqrt (2.0 * emergencyDecel * dist);
          if (targetSpeed > currentSpeed)
            targetSpeed = currentSpeed;
          if (targetSpeed < 0.0)
            targetSpeed = 0.0;

          // Override SUMO control: speedMode=7 disables TL braking bit
          m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 7);
          m_client->TraCIAPI::vehicle.setSpeed (m_id, targetSpeed);
          m_rlvwActive = true;

          // Visual indication: PURPLE for RLVW active
          libsumo::TraCIColor purple;
          purple.r = 255; purple.g = 0; purple.b = 255; purple.a = 255;
          m_client->TraCIAPI::vehicle.setColor (m_id, purple);
        }
      else
        {
          // Vehicle CAN stop comfortably — no violation warning needed
          if (m_rlvwActive)
            {
              // Release control back to SUMO
              m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
              m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
              m_rlvwActive = false;

              // Reset color to default
              libsumo::TraCIColor yellow;
              yellow.r = 255; yellow.g = 255; yellow.b = 0; yellow.a = 255;
              m_client->TraCIAPI::vehicle.setColor (m_id, yellow);
            }
        }
    }
  else if (!isRestrictive)
    {
      // GREEN or other permissive state — release RLVW if active
      if (m_rlvwActive)
        {
          spatemTimeout ();
          m_spatemAlive = true; // keep listening
        }
    }
  else
    {
      // Speed <= 1.0 m/s — vehicle is nearly stopped, release if active
      if (m_rlvwActive)
        {
          m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
          m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
          m_rlvwActive = false;
        }
    }

  // Reschedule next RLVW tick
  m_rlvwUpdateEvent = Simulator::Schedule (MilliSeconds (200),
                                            &tlmClient80211p::updateRlvwControl, this);
}

void
tlmClient80211p::populateStaticTLData (void)
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
tlmClient80211p::spatemTimeout ()
{
  m_spatemAlive = false;

  // Restore full SUMO control
  if (m_rlvwActive)
    {
      m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
      m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
      m_rlvwActive = false;
    }

  libsumo::TraCIColor orange;
  orange.r = 255; orange.g = 99; orange.b = 71; orange.a = 255;
  m_client->TraCIAPI::vehicle.setColor (m_id, orange);
}

long
tlmClient80211p::compute_timestampIts ()
{
  /* To get millisec since  2004-01-01T00:00:00:000Z */
  auto time = std::chrono::system_clock::now (); // get the current time
  auto since_epoch = time.time_since_epoch (); // get the duration since epoch
  auto millis = std::chrono::duration_cast<std::chrono::milliseconds> (
      since_epoch); // convert it in millisecond since epoch

  long elapsed_since_2004 =
      millis.count () - TIME_SHIFT; // in TIME_SHIFT we saved the millisec from epoch to 2004-01-01
  return elapsed_since_2004;
}

} // namespace ns3
