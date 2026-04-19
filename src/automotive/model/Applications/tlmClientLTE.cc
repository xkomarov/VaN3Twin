
#include "tlmClientLTE.h"
#include "ns3/SPATEM.h"
#include "ns3/CAM.h"
#include "ns3/vdpTraci.h"
#include "ns3/socket.h"
#include "ns3/network-module.h"
#include <cmath>
#include <limits>

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("tlmClientLTE");

  NS_OBJECT_ENSURE_REGISTERED(tlmClientLTE);

  TypeId
  tlmClientLTE::GetTypeId (void)
  {
    static TypeId tid =
        TypeId ("ns3::tlmClientLTE")
        .SetParent<Application> ()
        .SetGroupName ("Applications")
        .AddConstructor<tlmClientLTE> ()
        .AddAttribute ("PrintSummary",
            "To print summary at the end of simulation",
            BooleanValue(false),
            MakeBooleanAccessor (&tlmClientLTE::m_print_summary),
            MakeBooleanChecker ())
        .AddAttribute ("RealTime",
            "To compute properly timestamps",
            BooleanValue(false),
            MakeBooleanAccessor (&tlmClientLTE::m_real_time),
            MakeBooleanChecker ())
        .AddAttribute ("CSV",
            "CSV log name",
            StringValue (),
            MakeStringAccessor (&tlmClientLTE::m_csv_name),
            MakeStringChecker ())
        .AddAttribute ("ServerAddr",
            "Ip Addr of the server",
            Ipv4AddressValue("10.0.0.1"),
            MakeIpv4AddressAccessor (&tlmClientLTE::m_server_addr),
            MakeIpv4AddressChecker ())
        .AddAttribute ("Client",
            "TraCI client for SUMO",
            PointerValue (0),
            MakePointerAccessor (&tlmClientLTE::m_client),
            MakePointerChecker<TraciClient> ())
        .AddAttribute ("MetricSupervisor",
            "Metric Supervisor to compute metrics according to 3GPP TR36.885 V14.0.0 page 70",
            PointerValue (0),
            MakePointerAccessor (&tlmClientLTE::m_metric_supervisor),
            MakePointerChecker<MetricSupervisor> ())
        .AddAttribute ("SendCAM",
            "To enable/disable the transmission of CAM messages",
            BooleanValue(true),
            MakeBooleanAccessor (&tlmClientLTE::m_send_cam),
            MakeBooleanChecker ());
        return tid;
  }

  tlmClientLTE::tlmClientLTE ()
  {
    NS_LOG_FUNCTION(this);

    m_client = nullptr;
    m_print_summary = true;
    m_already_print = false;
    m_cam_sent = 0;
    m_spatem_received = 0;
  }

  tlmClientLTE::~tlmClientLTE ()
  {
    NS_LOG_FUNCTION(this);
  }

  void
  tlmClientLTE::DoDispose (void)
  {
    NS_LOG_FUNCTION(this);
    Application::DoDispose ();
  }

  void
  tlmClientLTE::StartApplication (void)
  {
    NS_LOG_FUNCTION(this);

    m_id = m_client->GetVehicleId (this->GetNode ());

    /* Create the socket for TX and RX */
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

    /* Socket used to send CAMs and receive DENMs */
    m_socket = Socket::CreateSocket (GetNode (), tid);

    /* Bind the socket to local address */
    InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 9);
    if (m_socket->Bind (local) == -1)
    {
      NS_FATAL_ERROR ("Failed to bind client socket");
    }

    /* Connect it to the server */
    InetSocketAddress remote = InetSocketAddress (m_server_addr, 9);
    m_socket->Connect(remote);

    /* Create new BTP and GeoNet objects and set them in DENBasicService and CABasicService */
    m_btp = CreateObject <btp>();
    m_geoNet = CreateObject <GeoNet>();

    if(m_metric_supervisor!=nullptr)
    {
      m_geoNet->setMetricSupervisor(m_metric_supervisor);
    }

    m_btp->setGeoNet(m_geoNet);
    m_caService.setBTP(m_btp);
    m_tlmBasicService.setBTP(m_btp);

    /* Set sockets, callback, station properties and TraCI VDP in CABasicService */
    m_caService.setSocketTx (m_socket);
    m_caService.setSocketRx (m_socket);
    m_caService.addCARxCallback (std::bind(&tlmClientLTE::receiveCAM,this,std::placeholders::_1,std::placeholders::_2));
    m_caService.setStationProperties (std::stol(m_id.substr (3)), StationType_passengerCar);
    m_caService.setRealTime (m_real_time);

    m_tlmBasicService.setRealTime (m_real_time);
    m_tlmBasicService.setSocketRx (m_socket);
    m_tlmBasicService.setStationProperties (std::stol(m_id.substr (3)), StationType_passengerCar);
    m_tlmBasicService.addTLMRxCallback (std::bind(&tlmClientLTE::receiveSPATEM,this,std::placeholders::_1,std::placeholders::_2));
    

    VDP* traci_vdp = new VDPTraCI(m_client,m_id);

    m_btp->setVDP(traci_vdp); 

    m_caService.setVDP(traci_vdp);

    m_tlmBasicService.setVDP(traci_vdp);

    /* Create LDM and mock-populate traffic light static topology (simulates MAPEM) */
    m_LDM = CreateObject<LDM> ();
    m_LDM->setStationID (m_id);
    m_LDM->setTraCIclient (m_client);
    m_LDM->setVDP (traci_vdp);
    populateStaticTLData ();

    /* Create CSV file, if requested */
    if (!m_csv_name.empty ())
    {
      m_csv_ofstream.open (m_csv_name+"-"+m_id+".csv",std::ofstream::trunc);
      m_csv_ofstream << "messageID,originatingStationId,sequence,referenceTime,detectionTime,stationID" << std::endl;
    }

    /* Schedule CAM dissemination */
    if(m_send_cam == true)
    {
      std::srand(Simulator::Now().GetNanoSeconds ());
      double desync = ((double)std::rand()/RAND_MAX);
      m_caService.startCamDissemination(desync);
    }
  }

  void
  tlmClientLTE::StopApplication ()
  {
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_sendCamEvent);
    Simulator::Cancel(m_spatemTimeout);
    Simulator::Cancel(m_glosaUpdateEvent);

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
      std::cout << "INFO-" << m_id
                << ",CAM-SENT:" << cam_sent
                << ",SPATEM-RECEIVED:" << m_spatem_received
                << std::endl;
      m_already_print=true;
    }
  }

  void
  tlmClientLTE::StopApplicationNow ()
  {
    NS_LOG_FUNCTION(this);
    StopApplication ();
  }

    void
  tlmClientLTE::receiveCAM (asn1cpp::Seq<CAM> cam, Address from)
  {
    /* Implement CAM strategy here */

    (void) cam;
    (void) from;

   // Free the received CAM data structure
//   ASN_STRUCT_FREE(asn_DEF_CAM,cam);
  }

  // ===========================================================================
  //  receiveSPATEM — parse SPATEM into LDM and kick-start GLOSA control loop
  // ===========================================================================
  void
  tlmClientLTE::receiveSPATEM (asn1cpp::Seq<SPATEM> spatem, Address from)
  {
    Simulator::Cancel(m_spatemTimeout);
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

    // Start or keep the periodic GLOSA control loop running
    if (m_glosaUpdateEvent.IsExpired ())
      {
        m_glosaUpdateEvent = Simulator::Schedule (MilliSeconds (200),
                                                  &tlmClientLTE::updateGlosaControl, this);
      }

    // Reschedule SPATEM-loss timeout (if no SPATEM for 2 s → disengage)
    m_spatemTimeout = Simulator::Schedule (Seconds (2.0), &tlmClientLTE::spatemTimeout, this);
  }

  // ===========================================================================
  //  updateGlosaControl — periodic GLOSA speed recalculation (every 200 ms)
  // ===========================================================================
  void
  tlmClientLTE::updateGlosaControl (void)
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
    double maxSpeed = m_client->TraCIAPI::vehicle.getAllowedSpeed (m_id);

    // --- Algorithm constants ---
    const double minSpeed = 3.0;        // m/s (~11 km/h) — below this GLOSA is impractical
    const double comfortDecel = 2.5;    // m/s² — comfortable deceleration
    const double GLOSA_MIN_DIST = 10.0; // m — too close to stop line
    const double GLOSA_MAX_DIST = 350.0;// m — too far, timing unreliable

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
        m_glosaUpdateEvent = Simulator::Schedule (MilliSeconds (200),
                                                  &tlmClientLTE::updateGlosaControl, this);
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
        // Vehicle not on an approach lane — release GLOSA
        if (m_glosaActive)
          {
            m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
            m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
            m_glosaActive = false;
          }
        m_passedIntersectionID = 0;
        m_glosaUpdateEvent = Simulator::Schedule (MilliSeconds (200),
                                                  &tlmClientLTE::updateGlosaControl, this);
        return;
      }

    // --- Skip intersection we already passed ---
    if (matchedTL.intersectionID == m_passedIntersectionID)
      {
        m_glosaUpdateEvent = Simulator::Schedule (MilliSeconds (200),
                                                  &tlmClientLTE::updateGlosaControl, this);
        return;
      }

    // --- Get traffic light state from LDM ---
    auto stateIt = matchedTL.signalGroupStates.find (matchedSignalGroup);
    if (stateIt == matchedTL.signalGroupStates.end ())
      {
        m_glosaUpdateEvent = Simulator::Schedule (MilliSeconds (200),
                                                  &tlmClientLTE::updateGlosaControl, this);
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
            // Record passed intersection — disengage GLOSA, restore neutral color
            m_passedIntersectionID = matchedTL.intersectionID;
            spatemTimeout ();
            m_spatemAlive = true; // keep listening for future intersections
            m_glosaUpdateEvent = Simulator::Schedule (MilliSeconds (200),
                                                      &tlmClientLTE::updateGlosaControl, this);
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

    // --- GLOSA activation range check ---
    if (dist < GLOSA_MIN_DIST || dist > GLOSA_MAX_DIST)
      {
        if (m_glosaActive)
          {
            m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
            m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
            m_glosaActive = false;
          }
        m_glosaUpdateEvent = Simulator::Schedule (MilliSeconds (200),
                                                  &tlmClientLTE::updateGlosaControl, this);
        return;
      }

    // --- Get timing from LDM ---
    // minEndTime per ETSI TS 103 301: time until the current phase ends (tenths of seconds)
    double timeToSwitch = 0.0;
    auto timingIt = matchedTL.signalGroupTimings.find (matchedSignalGroup);
    if (timingIt != matchedTL.signalGroupTimings.end ())
      {
        timeToSwitch = timingIt->second / 10.0; // convert deci-seconds → seconds
      }

    if (currentLightState == 3) // stop-And-Remain (RED)
      {
        // ---- RED PHASE: glide to arrive when green starts, or stop smoothly ----
        if (timeToSwitch > 0.5)
          {
            double bufferTime = 2.0; // aim to arrive 2 s after green starts
            double arrivalSpeed = dist / (timeToSwitch + bufferTime);

            if (arrivalSpeed >= minSpeed && arrivalSpeed <= maxSpeed)
              {
                // Feasibility: check deceleration rate
                double decelNeeded = (currentSpeed > arrivalSpeed)
                                         ? (currentSpeed - arrivalSpeed) / 1.0
                                         : 0.0;

                if (decelNeeded <= comfortDecel * 2.0)
                  {
                    // Glide to green — apply advisory speed
                    m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 7);
                    m_client->TraCIAPI::vehicle.setSpeed (m_id, arrivalSpeed);
                    m_glosaActive = true;

                    libsumo::TraCIColor red;
                    red.r = 255; red.g = 0; red.b = 0; red.a = 255;
                    m_client->TraCIAPI::vehicle.setColor (m_id, red);
                  }
                else
                  {
                    // Decel too harsh — smooth stop at the stop line
                    double stopSpeed = std::sqrt (2.0 * comfortDecel * dist);
                    if (stopSpeed > currentSpeed) stopSpeed = currentSpeed;
                    if (stopSpeed < 0.0) stopSpeed = 0.0;

                    m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 7);
                    m_client->TraCIAPI::vehicle.setSpeed (m_id, stopSpeed);
                    m_glosaActive = true;

                    libsumo::TraCIColor red;
                    red.r = 255; red.g = 0; red.b = 0; red.a = 255;
                    m_client->TraCIAPI::vehicle.setColor (m_id, red);
                  }
              }
            else if (arrivalSpeed < minSpeed)
              {
                // Green is too far away — must stop at the stop line
                double stopSpeed = std::sqrt (2.0 * comfortDecel * dist);
                if (stopSpeed > currentSpeed) stopSpeed = currentSpeed;
                if (stopSpeed < 0.0) stopSpeed = 0.0;

                m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 7);
                m_client->TraCIAPI::vehicle.setSpeed (m_id, stopSpeed);
                m_glosaActive = true;

                libsumo::TraCIColor red;
                red.r = 255; red.g = 0; red.b = 0; red.a = 255;
                m_client->TraCIAPI::vehicle.setColor (m_id, red);
              }
            else
              {
                // arrivalSpeed > maxSpeed — can't go fast enough; drive at max
                m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 7);
                m_client->TraCIAPI::vehicle.setSpeed (m_id, maxSpeed);
                m_glosaActive = true;

                libsumo::TraCIColor glosaGreen;
                glosaGreen.r = 50; glosaGreen.g = 205; glosaGreen.b = 50; glosaGreen.a = 255;
                m_client->TraCIAPI::vehicle.setColor (m_id, glosaGreen);
              }
          }
        else
          {
            // Phase about to change (<0.5 s left) — hold current speed, green imminent
            if (!m_glosaActive)
              {
                m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 7);
                m_glosaActive = true;
              }
            // Don't change speed — let the vehicle coast through
          }
      }
    else if (currentLightState == 6 || currentLightState == 5) // GREEN
      {
        // ---- GREEN PHASE: drive at road speed limit ----
        m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 7);
        m_client->TraCIAPI::vehicle.setSpeed (m_id, maxSpeed);
        m_glosaActive = true;

        libsumo::TraCIColor blue;
        blue.r = 0; blue.g = 100; blue.b = 255; blue.a = 255;
        m_client->TraCIAPI::vehicle.setColor (m_id, blue);
      }
    else if (currentLightState == 7) // intersection-clearance (YELLOW)
      {
        // YELLOW: if close enough, maintain speed to pass; otherwise prepare to stop
        double eta = dist / std::max (currentSpeed, 1.0);
        if (eta < timeToSwitch && dist < 30.0)
          {
            // Close and fast enough — commit to crossing
            m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 7);
            m_client->TraCIAPI::vehicle.setSpeed (m_id, currentSpeed);
            m_glosaActive = true;

            libsumo::TraCIColor amber;
            amber.r = 255; amber.g = 191; amber.b = 0; amber.a = 255;
            m_client->TraCIAPI::vehicle.setColor (m_id, amber);
          }
        else
          {
            // Prepare to stop — smooth decel
            double stopSpeed = std::sqrt (2.0 * comfortDecel * dist);
            if (stopSpeed > currentSpeed) stopSpeed = currentSpeed;
            if (stopSpeed < 0.0) stopSpeed = 0.0;

            m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 7);
            m_client->TraCIAPI::vehicle.setSpeed (m_id, stopSpeed);
            m_glosaActive = true;

            libsumo::TraCIColor red;
            red.r = 255; red.g = 0; red.b = 0; red.a = 255;
            m_client->TraCIAPI::vehicle.setColor (m_id, red);
          }
      }
    else
      {
        // Unknown / off / unavailable — maintain current speed under advisory control
        m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 7);
        m_client->TraCIAPI::vehicle.setSpeed (m_id, currentSpeed);
        m_glosaActive = true;
      }

    // Reschedule next GLOSA tick
    m_glosaUpdateEvent = Simulator::Schedule (MilliSeconds (200),
                                              &tlmClientLTE::updateGlosaControl, this);
  }


  void
  tlmClientLTE::populateStaticTLData (void)
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
  tlmClientLTE::spatemTimeout()
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
    orange.r=255;orange.g=99;orange.b=71;orange.a=255;
    m_client->TraCIAPI::vehicle.setColor (m_id,orange);
  }

  long
  tlmClientLTE::compute_timestampIts ()
  {
    /* To get millisec since  2004-01-01T00:00:00:000Z */
    auto time = std::chrono::system_clock::now(); // get the current time
    auto since_epoch = time.time_since_epoch(); // get the duration since epoch
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(since_epoch); // convert it in millisecond since epoch

    long elapsed_since_2004 = millis.count() - TIME_SHIFT; // in TIME_SHIFT we saved the millisec from epoch to 2004-01-01
    return elapsed_since_2004;
  }

}
