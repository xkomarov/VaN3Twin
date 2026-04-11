
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
  //m_denm_received = 0;
  m_spatem_received = 0;
}

tlmClient80211p::~tlmClient80211p ()
{
  //m_denService.cleanup();
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
     * This application works as client for the areaSpeedAdvisorServer80211p. It is intended to be installed over a vehicular OBU node,
     * and it is set to generate broadcast CAM messages on top of BTP and GeoNet.
     * As soon as a DENM is received, it reads the information inside the RoadWorks container
     * and sets the speed accordingly (see receiveDENM() function)
     */

  m_id = m_client->GetVehicleId (this->GetNode ());

  /* Create the socket for TX and RX */
  TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");

  /* Socket used to send CAMs and receive DENMs */
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

  /* Create new BTP and GeoNet objects and set them in DENBasicService and CABasicService */
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

  // Ensure GLOSA is disengaged before shutdown
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
  m_tlmBasicService.terminateDissemination ();

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
tlmClient80211p::StopApplicationNow ()
{
  NS_LOG_FUNCTION (this);
  StopApplication ();
}

void
tlmClient80211p::receiveCAM (asn1cpp::Seq<CAM> cam, Address from)
{
  /* Implement CAM strategy here */

  (void) cam;
  (void) from;

  // Free the received CAM data structure
  //   ASN_STRUCT_FREE(asn_DEF_CAM,cam);
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

  // === Step 2: Get ego vehicle position and current lane ===
  std::string currentLane = m_client->TraCIAPI::vehicle.getLaneID (m_id);
  libsumo::TraCIPosition egoXY = m_client->TraCIAPI::vehicle.getPosition (m_id);
  libsumo::TraCIPosition egoLL =
      m_client->TraCIAPI::simulation.convertXYtoLonLat (egoXY.x, egoXY.y);

  // === Step 3: Find nearby traffic lights from LDM ===
  std::vector<trafficLightData_t> nearbyTLs;
  m_LDM->rangeSelectTL (200.0, egoLL.y, egoLL.x, nearbyTLs);

  if (nearbyTLs.empty ())
    {
      m_spatemTimeout = Simulator::Schedule (Seconds (1.0), &tlmClient80211p::spatemTimeout, this);
      return;
    }

  // === Step 4: Find the TL that controls the vehicle's current lane ===
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
      // Машина не на подъездной полосе ни к одному светофору — сброс
      m_passedIntersectionID = 0;
      m_spatemTimeout = Simulator::Schedule (Seconds (1.0), &tlmClient80211p::spatemTimeout, this);
      return;
    }

  // Проверка: это тот же перекрёсток, который мы уже проехали?
  if (matchedTL.intersectionID == m_passedIntersectionID)
    {
      // Игнорируем — машина ещё на подъездной полосе, но уже проехала стоп-линию
      return;
    }

  // === Step 5: Get traffic light state from LDM ===
  auto stateIt = matchedTL.signalGroupStates.find (matchedSignalGroup);
  if (stateIt == matchedTL.signalGroupStates.end ())
    {
      m_spatemTimeout = Simulator::Schedule (Seconds (1.0), &tlmClient80211p::spatemTimeout, this);
      return;
    }
  long current_light_state = stateIt->second;

  // === Step 6: Compute distance to stop line using LDM stop-line data ===
  double dist = 0.0;
  auto slIt = matchedTL.laneStopLines.find (currentLane);
  if (slIt != matchedTL.laneStopLines.end ())
    {
      // Euclidean distance from vehicle to stop-line position (accurate for short distances) возможно место на доработку
      double dx = egoXY.x - slIt->second.x;
      double dy = egoXY.y - slIt->second.y;
      dist = std::sqrt (dx * dx + dy * dy);

      // Проверка: машина уже проехала стоп-линию?
      double lanePos = m_client->TraCIAPI::vehicle.getLanePosition (m_id);
      if (lanePos >= slIt->second.laneLen - 2.0)
        {
          // Запоминаем проеханный перекрёсток и переходим в режим "оранжевый"
          m_passedIntersectionID = matchedTL.intersectionID;
          m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
          spatemTimeout ();
          return;
        }
    }
  else
    {
      // Fallback: Haversine to intersection center
      dist = haversineDist (egoLL.y, egoLL.x, matchedTL.lat, matchedTL.lon);
    }

  // === Step 7: Get timing from LDM ===
  // minEndTime per ETSI TS 103 301: time until the current phase ends (in tenths of seconds)
  double timeToSwitch = 0.0;
  auto timingIt = matchedTL.signalGroupTimings.find (matchedSignalGroup);
  if (timingIt != matchedTL.signalGroupTimings.end ())
    {
      timeToSwitch = timingIt->second / 10.0;
    }

  // === Step 8: Ego vehicle parameters ===
  double maxSpeed = m_client->TraCIAPI::vehicle.getAllowedSpeed (m_id);
  double minSpeed = 3.0; // m/s (~11 km/h) — below this GLOSA is impractical
  double currentSpeed = m_client->TraCIAPI::vehicle.getSpeed (m_id);
  double maxAccel = 2.6; // m/s² — standard passenger car
  double comfortDecel = 2.5; // m/s² — comfortable deceleration

  // === Step 9: GLOSA activation range ===
  const double GLOSA_MIN_DIST = 10.0; // Too close to stop line — GLOSA useless
  const double GLOSA_MAX_DIST = 350.0; // Too far — timing info unreliable

  if (dist < GLOSA_MIN_DIST || dist > GLOSA_MAX_DIST)
    {
      // Outside GLOSA range — release control
      if (m_glosaActive)
        {
          m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
          m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
          m_glosaActive = false;
        }
      m_spatemTimeout = Simulator::Schedule (Seconds (1.0), &tlmClient80211p::spatemTimeout, this);
      return;
    }

  // === Step 10: GLOSA Logic ===
  if (current_light_state == 3)
    {
      // ---- RED PHASE: slow down to arrive when red ends ----
      if (timeToSwitch > 0.5)
        {
          double bufferTime = 3.0; // Стремимся прибыть через 3 секунды после включения зеленого
          double targetSpeed = dist / (timeToSwitch + bufferTime);

          if (targetSpeed >= minSpeed && targetSpeed <= maxSpeed)
            {
              // Check comfortable deceleration (don't brake too hard)
              double decelNeeded = (currentSpeed > targetSpeed)
                                       ? (currentSpeed - targetSpeed) / 1.0 // over ~1 second
                                       : 0.0;

              if (decelNeeded <= comfortDecel * 2.0)
                {
                  if (std::abs(currentSpeed - targetSpeed) > 0.5 || !m_glosaActive) 
                    {
                      m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
                      m_client->TraCIAPI::vehicle.setSpeed (m_id, targetSpeed);
                      m_glosaActive = true;

                      libsumo::TraCIColor glosaGreen;
                      glosaGreen.r = 50;
                      glosaGreen.g = 205;
                      glosaGreen.b = 50;
                      glosaGreen.a = 255;
                      m_client->TraCIAPI::vehicle.setColor (m_id, glosaGreen);

                      NS_LOG_INFO ("[" << m_id << "] GLOSA-RED: dist=" << dist << "m tSwitch="
                                       << timeToSwitch << "s target=" << targetSpeed << "m/s");
                    }
                }
              else
                {
                  // Decel too harsh — let SUMO handle normally
                  if (m_glosaActive)
                    {
                      m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
                      m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
                      m_glosaActive = false;
                    }
                }
            }
          else if (targetSpeed < minSpeed)
            {
              // Would need to crawl — just let SUMO stop at the red normally
              if (m_glosaActive)
                {
                  m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
                  m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
                  m_glosaActive = false;
                }
              libsumo::TraCIColor red;
              red.r = 255;
              red.g = 0;
              red.b = 0;
              red.a = 255;
              m_client->TraCIAPI::vehicle.setColor (m_id, red);
            }
          else
            {
              // targetSpeed > maxSpeed — can't go fast enough, just drive normally
              if (m_glosaActive)
                {
                  m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
                  m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
                  m_glosaActive = false;
                }
            }
        }
      else
        {
          // Phase about to change (< 0.5s left) — release control
          if (m_glosaActive)
            {
              m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
              m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
              m_glosaActive = false;
            }
        }
    }
  else if (current_light_state == 6 || current_light_state == 5)
    {
      // ---- GREEN PHASE: can we make it through before it ends? ----
      double estimatedArrivalTime = dist / std::max (currentSpeed, 1.0);

      if (estimatedArrivalTime < timeToSwitch * 0.9)
        {
          // Comfortably make it through at current speed — release GLOSA, drive normally
          if (m_glosaActive)
            {
              m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
              m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
              m_glosaActive = false;
            }
          libsumo::TraCIColor blue;
          blue.r = 0;
          blue.g = 100;
          blue.b = 255;
          blue.a = 255;
          m_client->TraCIAPI::vehicle.setColor (m_id, blue);
        }
      else if (timeToSwitch > 1.0)
        {
          // Might not make it — try to speed up to catch the green!
          double targetSpeed =
              dist / (timeToSwitch * 0.85); // aim to arrive at 85% of remaining time

          // Cap at maxSpeed and check if acceleration is feasible
          if (targetSpeed > maxSpeed)
            {
              targetSpeed = maxSpeed;
            }

          // Check: even at max speed, can we arrive in time?
          double arrivalAtMax = dist / targetSpeed;
          if (arrivalAtMax < timeToSwitch && targetSpeed >= minSpeed)
            {
              // Feasibility check: can we accelerate to targetSpeed comfortably?
              double accelNeeded = (targetSpeed > currentSpeed)
                                       ? (targetSpeed - currentSpeed) / 1.0 // over ~1 second
                                       : 0.0;

              if (accelNeeded <= maxAccel * 1.5)
                {
                  if (std::abs(currentSpeed - targetSpeed) > 0.5 || !m_glosaActive)
                    {
                      // SPEED UP to catch the green light!
                      m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
                      m_client->TraCIAPI::vehicle.setSpeed (m_id, targetSpeed);
                      m_glosaActive = true;

                      libsumo::TraCIColor yellow;
                      yellow.r = 255;
                      yellow.g = 215;
                      yellow.b = 0;
                      yellow.a = 255;
                      m_client->TraCIAPI::vehicle.setColor (m_id, yellow);

                      NS_LOG_INFO ("[" << m_id << "] GLOSA-GREEN-SPEEDUP: dist=" << dist
                                       << "m tSwitch=" << timeToSwitch << "s target=" << targetSpeed
                                       << "m/s (cur=" << currentSpeed << ")");
                    }
                }
              else
                {
                  // Can't accelerate enough — let SUMO handle, will stop at next red
                  if (m_glosaActive)
                    {
                      m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
                      m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
                      m_glosaActive = false;
                    }
                }
            }
          else
            {
              // Even at maxSpeed can't make it — give up on this green cycle
              if (m_glosaActive)
                {
                  m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
                  m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
                  m_glosaActive = false;
                }
            }
        }
      else
        {
          // Green about to end (< 1s) — release control
          if (m_glosaActive)
            {
              m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
              m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
              m_glosaActive = false;
            }
        }
    }
  else
    {
      // Unknown/yellow/other state — release GLOSA control
      if (m_glosaActive)
        {
          m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
          m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
          m_glosaActive = false;
        }
    }

  m_spatemTimeout = Simulator::Schedule (Seconds (1.0), &tlmClient80211p::spatemTimeout, this);
}

void
tlmClient80211p::populateStaticTLData (void)
{
  // This method simulates the reception of MAPEM messages by inserting
  // the static traffic light topology (position, lane mapping, stop-line
  // coordinates) into the LDM. In a production system, this data would
  // arrive via MAPEM messages from the RSU.

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

      NS_LOG_INFO ("[" << m_id << "] LDM: Loaded TL topology '" << tlsId
                       << "' -> intersectionID=" << tlData.intersectionID << " pos=(" << tlData.lat
                       << ", " << tlData.lon << ")"
                       << " lanes=" << tlData.laneToSignalGroup.size ()
                       << " stopLines=" << tlData.laneStopLines.size ());
    }
}

void
tlmClient80211p::spatemTimeout ()
{
  // Reset vehicle to SUMO-controlled driving when SPATEM signal is lost
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
