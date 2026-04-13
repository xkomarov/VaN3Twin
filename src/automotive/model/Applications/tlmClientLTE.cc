
#include "tlmClientLTE.h"
#include "ns3/SPATEM.h"
#include "ns3/CAM.h"
#include "ns3/vdpTraci.h"
#include "ns3/socket.h"
#include "ns3/network-module.h"
#include "ns3/asn_utils.h"


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
    //m_denm_received = 0;
    m_spatem_received = 0;
    m_glosaActive = false;
    m_passedIntersectionID = 0;
  }

  tlmClientLTE::~tlmClientLTE ()
  {
    //m_denService.cleanup();
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

    /*
     * This application works as client for the areaSpeedAdvisorServerLTE. It is intended to be installed over a vehicular OBU node,
     * and it is set to generate broadcast CAM messages on top of BTP and GeoNet.
     * As soon as a DENM is received, it reads the information inside the RoadWorks container
     * and sets the speed accordingly (see receiveDENM() function)
     */

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

  void
  tlmClientLTE::receiveSPATEM (asn1cpp::Seq<SPATEM> spatem, Address from)
  {
    Simulator::Cancel(m_spatemTimeout);

    /* Implement SPATEM strategy here */
    m_spatem_received++;
    // asn_fprint(stdout, &asn_DEF_SPATEM, &(*spatem));
    // fflush(stdout);

    if (!m_csv_name.empty () && m_csv_ofstream.is_open())
    {
      long messageID = asn1cpp::getField(spatem->header.messageId, long);
      long stationID = asn1cpp::getField(spatem->header.stationId, long);
      m_csv_ofstream << messageID << ",0,0,0,0," << stationID << std::endl;
    }

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
        m_spatemTimeout = Simulator::Schedule (Seconds (1.0), &tlmClientLTE::spatemTimeout, this);
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
        m_passedIntersectionID = 0;
        m_spatemTimeout = Simulator::Schedule (Seconds (1.0), &tlmClientLTE::spatemTimeout, this);
        return;
      }

    if (matchedTL.intersectionID == m_passedIntersectionID)
      {
        return;
      }

    // === Step 5: Get traffic light state from LDM ===
    auto stateIt = matchedTL.signalGroupStates.find (matchedSignalGroup);
    if (stateIt == matchedTL.signalGroupStates.end ())
      {
        m_spatemTimeout = Simulator::Schedule (Seconds (1.0), &tlmClientLTE::spatemTimeout, this);
        return;
      }
    long current_light_state = stateIt->second;

    // === Step 6: Compute distance to stop line using LDM stop-line data ===
    double dist = 0.0;
    auto slIt = matchedTL.laneStopLines.find (currentLane);
    if (slIt != matchedTL.laneStopLines.end ())
      {
        double dx = egoXY.x - slIt->second.x;
        double dy = egoXY.y - slIt->second.y;
        dist = std::sqrt (dx * dx + dy * dy);

        double lanePos = m_client->TraCIAPI::vehicle.getLanePosition (m_id);
        if (lanePos >= slIt->second.laneLen - 2.0)
          {
            m_passedIntersectionID = matchedTL.intersectionID;
            m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
            spatemTimeout ();
            return;
          }
      }
    else
      {
        // Fallback: Haversine to intersection center
        dist = 999.0;
      }

    // === Step 7: Get timing from LDM ===
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
        if (m_glosaActive)
          {
            m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
            m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
            m_glosaActive = false;
          }
        m_spatemTimeout = Simulator::Schedule (Seconds (1.0), &tlmClientLTE::spatemTimeout, this);
        return;
      }

    // === Step 10: GLOSA Logic ===
    if (current_light_state == 3)
      {
        if (timeToSwitch > 0.5)
          {
            double bufferTime = 3.0; // Стремимся прибыть через 3 секунды после включения зеленого
            double targetSpeed = dist / (timeToSwitch + bufferTime);

            if (targetSpeed >= minSpeed && targetSpeed <= maxSpeed)
              {
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
                        glosaGreen.r = 50; glosaGreen.g = 205; glosaGreen.b = 50; glosaGreen.a = 255;
                        m_client->TraCIAPI::vehicle.setColor (m_id, glosaGreen);
                      }
                  }
                else
                  {
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
                if (m_glosaActive)
                  {
                    m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
                    m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
                    m_glosaActive = false;
                  }
                libsumo::TraCIColor red;
                red.r = 255; red.g = 0; red.b = 0; red.a = 255;
                m_client->TraCIAPI::vehicle.setColor (m_id, red);
              }
            else
              {
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
        double estimatedArrivalTime = dist / std::max (currentSpeed, 1.0);

        if (estimatedArrivalTime < timeToSwitch * 0.9)
          {
            if (m_glosaActive)
              {
                m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
                m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
                m_glosaActive = false;
              }
            libsumo::TraCIColor blue;
            blue.r = 0; blue.g = 100; blue.b = 255; blue.a = 255;
            m_client->TraCIAPI::vehicle.setColor (m_id, blue);
          }
        else if (timeToSwitch > 1.0)
          {
            double targetSpeed = dist / (timeToSwitch * 0.85); 

            if (targetSpeed > maxSpeed)
              {
                targetSpeed = maxSpeed;
              }

            double arrivalAtMax = dist / targetSpeed;
            if (arrivalAtMax < timeToSwitch && targetSpeed >= minSpeed)
              {
                double accelNeeded = (targetSpeed > currentSpeed)
                                         ? (targetSpeed - currentSpeed) / 1.0 
                                         : 0.0;

                if (accelNeeded <= maxAccel * 1.5)
                  {
                    if (std::abs(currentSpeed - targetSpeed) > 0.5 || !m_glosaActive)
                      {
                        m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
                        m_client->TraCIAPI::vehicle.setSpeed (m_id, targetSpeed);
                        m_glosaActive = true;

                        libsumo::TraCIColor yellow;
                        yellow.r = 255; yellow.g = 215; yellow.b = 0; yellow.a = 255;
                        m_client->TraCIAPI::vehicle.setColor (m_id, yellow);
                      }
                  }
                else
                  {
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
        if (m_glosaActive)
          {
            m_client->TraCIAPI::vehicle.setSpeedMode (m_id, 31);
            m_client->TraCIAPI::vehicle.setSpeed (m_id, -1.0);
            m_glosaActive = false;
          }
      }

    m_spatemTimeout = Simulator::Schedule(Seconds(1.0),&tlmClientLTE::spatemTimeout,this);
  }

  void
  tlmClientLTE::spatemTimeout()
  {
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





    // // 1. Проверяем, есть ли данные о перекрестке в пакете
    // if (spatem->spat.intersections.list.count > 0) {
        
    //     // Берем первый перекресток
    //     auto intersection = spatem->spat.intersections.list.array[0];
        
    //     // 2. В реальном V2X машина должна узнать свою полосу. 
    //     // Но для симуляции давай просто проверим первую полосу (SignalGroup 1)
    //     if (intersection->states.list.count > 0) {
    //         auto first_movement = intersection->states.list.array[0];
    //         //std::cout << first_movement << std::endl;
            
    //         // Проверяем, есть ли события (фазы) для этой полосы
    //         if (first_movement->state_time_speed.list.count > 0 && first_movement->signalGroup == upcomingTLS[0].tlIndex) {
                
    //             // 3. ДОСТАЕМ ЦВЕТ СВЕТОФОРА
    //             long current_light_state = first_movement->state_time_speed.list.array[0]->eventState;
                
    //             // 4. ЛОГИКА ТОРМОЖЕНИЯ:
    //             // 3 = stop-And-Remain (Красный свет)
    //             if (current_light_state == 3) {
    //                 // Заставляем машину остановиться (устанавливаем скорость 0)
    //                 m_client->TraCIAPI::vehicle.setSpeed(m_id, 0.0);
                    
    //                 // Красим машину в красный, чтобы визуально видеть, что она тормозит из-за SPATEM
    //                 libsumo::TraCIColor red;
    //                 red.r = 255; red.g = 0; red.b = 0; red.a = 255;
    //                 m_client->TraCIAPI::vehicle.setColor(m_id, red);
                    
    //             } else if (current_light_state == 6 || current_light_state == 5) {
    //                 // Если свет зеленый, сбрасываем ограничение скорости, пусть едет нормально
    //                 m_client->TraCIAPI::vehicle.setSpeed(m_id, -1.0); // -1.0 в TraCI возвращает контроль SUMO
                    
    //                 // Красим машину в синий (едет по SPATEM)
    //                 libsumo::TraCIColor blue;
    //                 blue.r = 0; blue.g = 0; blue.b = 255; blue.a = 255;
    //                 m_client->TraCIAPI::vehicle.setColor(m_id, blue);
    //             }
    //         }
    //     }
    // }
