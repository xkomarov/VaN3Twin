
#include "tlmClient80211p.h"
#include "ns3/SPATEM.h"
#include "ns3/CAM.h"
#include "ns3/vdpTraci.h"
#include "ns3/socket.h"
#include "ns3/network-module.h"

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("tlmClient80211p");

  NS_OBJECT_ENSURE_REGISTERED(tlmClient80211p);

  TypeId
  tlmClient80211p::GetTypeId (void)
  {
    static TypeId tid =
        TypeId ("ns3::tlmClient80211p")
        .SetParent<Application> ()
        .SetGroupName ("Applications")
        .AddConstructor<tlmClient80211p> ()
        .AddAttribute ("PrintSummary",
            "To print summary at the end of simulation",
            BooleanValue(false),
            MakeBooleanAccessor (&tlmClient80211p::m_print_summary),
            MakeBooleanChecker ())
        .AddAttribute ("RealTime",
            "To compute properly timestamps",
            BooleanValue(false),
            MakeBooleanAccessor (&tlmClient80211p::m_real_time),
            MakeBooleanChecker ())
        .AddAttribute ("CSV",
            "CSV log name",
            StringValue (),
            MakeStringAccessor (&tlmClient80211p::m_csv_name),
            MakeStringChecker ())
        .AddAttribute ("ServerAddr",
            "Ip Addr of the server",
            Ipv4AddressValue("10.0.0.1"),
            MakeIpv4AddressAccessor (&tlmClient80211p::m_server_addr),
            MakeIpv4AddressChecker ())
        .AddAttribute ("Client",
            "TraCI client for SUMO",
            PointerValue (0),
            MakePointerAccessor (&tlmClient80211p::m_client),
            MakePointerChecker<TraciClient> ())
        .AddAttribute ("MetricSupervisor",
            "Metric Supervisor to compute metrics according to 3GPP TR36.885 V14.0.0 page 70",
            PointerValue (0),
            MakePointerAccessor (&tlmClient80211p::m_metric_supervisor),
            MakePointerChecker<MetricSupervisor> ())
        .AddAttribute ("SendCAM",
            "To enable/disable the transmission of CAM messages",
            BooleanValue(true),
            MakeBooleanAccessor (&tlmClient80211p::m_send_cam),
            MakeBooleanChecker ());
        return tid;
  }

  tlmClient80211p::tlmClient80211p ()
  {
    NS_LOG_FUNCTION(this);

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
    NS_LOG_FUNCTION(this);
  }

  void
  tlmClient80211p::DoDispose (void)
  {
    NS_LOG_FUNCTION(this);
    Application::DoDispose ();
  }

  void
  tlmClient80211p::StartApplication (void)
  {
    NS_LOG_FUNCTION(this);

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
    m_caService.addCARxCallback (std::bind(&tlmClient80211p::receiveCAM,this,std::placeholders::_1,std::placeholders::_2));
    m_caService.setStationProperties (std::stol(m_id.substr (3)), StationType_passengerCar);
    m_caService.setRealTime (m_real_time);

    m_tlmBasicService.setRealTime (m_real_time);
    m_tlmBasicService.setSocketRx (m_socket);
    m_tlmBasicService.setStationProperties (std::stol(m_id.substr (3)), StationType_passengerCar);
    m_tlmBasicService.addTLMRxCallback (std::bind(&tlmClient80211p::receiveSPATEM,this,std::placeholders::_1,std::placeholders::_2));
    

    VDP* traci_vdp = new VDPTraCI(m_client,m_id);

    m_btp->setVDP(traci_vdp); 

    m_caService.setVDP(traci_vdp);

    m_tlmBasicService.setVDP(traci_vdp);

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
  tlmClient80211p::StopApplication ()
  {
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_sendCamEvent);
    Simulator::Cancel(m_spatemTimeout);

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
  tlmClient80211p::StopApplicationNow ()
  {
    NS_LOG_FUNCTION(this);
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
    Simulator::Cancel(m_spatemTimeout);

    /* Implement SPATEM strategy here */
    m_spatem_received++;

    //asn_fprint(stdout, &asn_DEF_SPATEM, &(*spatem));
    //std::cout << m_client->TraCIAPI::vehicle.getLaneID(m_id) << std::endl;
    //std::cout << m_client->TraCIAPI::vehicle.getRoadID(m_id) << std::endl;
    //std::cout << m_client->TraCIAPI::vehicle.getPosition(m_id) << std::endl;
    //std::cout << m_id << std::endl;
    auto upcomingTLS = m_client->TraCIAPI::vehicle.getNextTLS(m_id);
    
    if (upcomingTLS.size() > 0) {
      std::string sumo_tls_id = upcomingTLS[0].id;
      long expected_intersection_id = (uint16_t)std::hash<std::string>{}(sumo_tls_id);
      long target_tl_index = upcomingTLS[0].tlIndex + 1; 

      for (int j = 0; j < spatem->spat.intersections.list.count; ++j) {
          auto intersection = spatem->spat.intersections.list.array[j];
          
          if (intersection->id.id == expected_intersection_id) {
              for (int i = 0; i < intersection->states.list.count; ++i) {
                  auto movement = intersection->states.list.array[i];
                  
                  if (movement->signalGroup == target_tl_index && movement->state_time_speed.list.count > 0) {
                      
                      long current_light_state = movement->state_time_speed.list.array[0]->eventState;
                      
                      // Расстояние до стоп-линии из TraCI
                      double dist = upcomingTLS[0].dist;
                      
                      // Получаем время до переключения фазы светофора из SPATEM
                      double timeToSwitch = 0.0;
                      if (movement->state_time_speed.list.array[0]->timing != nullptr) {
                          // minEndTime передается в десятых долях секунды (согласно vdpTraci.cc)
                          timeToSwitch = movement->state_time_speed.list.array[0]->timing->minEndTime / 10.0;
                      } else {
                          // Резервный вариант, если поля timing нет в пакете
                          double currentTime = m_client->TraCIAPI::simulation.getTime();
                          double nextSwitch = m_client->TraCIAPI::trafficlights.getNextSwitch(sumo_tls_id);
                          timeToSwitch = nextSwitch - currentTime;
                      }
                      
                      // Параметры для ограничений скорости
                      double maxSpeed = m_client->TraCIAPI::vehicle.getAllowedSpeed(m_id); 
                      double minSpeed = 5.0;  // м/с (около 18 км/ч)
                      double currentSpeed = m_client->TraCIAPI::vehicle.getSpeed(m_id);

                      // === ЛОГИКА GLOSA ===
                      if (current_light_state == 3) {
                          // Красный: пытаемся ехать так, чтобы приехать к зеленому
                          double targetSpeed = dist / timeToSwitch;
                          
                          if (targetSpeed <= maxSpeed && targetSpeed >= minSpeed) {
                              m_client->TraCIAPI::vehicle.setSpeed(m_id, targetSpeed);
                              
                              libsumo::TraCIColor glosaGreen;
                              glosaGreen.r = 50; glosaGreen.g = 205; glosaGreen.b = 50; glosaGreen.a = 255;
                              m_client->TraCIAPI::vehicle.setColor(m_id, glosaGreen);
                          } else {
                              // Не можем подстроиться - просто останавливаемся
                              m_client->TraCIAPI::vehicle.setSpeed(m_id, -1.0);
                              
                              libsumo::TraCIColor red;
                              red.r = 255; red.g = 0; red.b = 0; red.a = 255;
                              m_client->TraCIAPI::vehicle.setColor(m_id, red);
                          }
                      } else if (current_light_state == 6 || current_light_state == 5) {
                          // Зеленый: проверяем, успеваем ли проехать
                          double estimatedArrivalTime = dist / std::max(currentSpeed, 1.0);
                          
                          if (estimatedArrivalTime < timeToSwitch) {
                              // Успеваем! Едем с обычной скоростью
                              m_client->TraCIAPI::vehicle.setSpeed(m_id, -1.0);
                              
                              libsumo::TraCIColor blue;
                              blue.r = 0; blue.g = 0; blue.b = 255; blue.a = 255;
                              m_client->TraCIAPI::vehicle.setColor(m_id, blue);
                          } else {
                              // Не успеваем (приедем на красный). Тормозим заранее.
                              double targetSpeed = dist / timeToSwitch;
                              
                              if (targetSpeed <= maxSpeed && targetSpeed >= minSpeed) {
                                  m_client->TraCIAPI::vehicle.setSpeed(m_id, targetSpeed);
                                  
                                  libsumo::TraCIColor yellow;
                                  yellow.r = 255; yellow.g = 215; yellow.b = 0; yellow.a = 255;
                                  m_client->TraCIAPI::vehicle.setColor(m_id, yellow);
                              } else {
                                  // Просто едем и останавливаемся
                                  m_client->TraCIAPI::vehicle.setSpeed(m_id, -1.0); 
                              }
                          }
                      }
                      
                      break;
                  }
              }
              break;
          }
      }
    }

    m_spatemTimeout = Simulator::Schedule(Seconds(0.2),&tlmClient80211p::spatemTimeout,this);
  }

  void
  tlmClient80211p::spatemTimeout()
  {
    libsumo::TraCIColor orange;
    orange.r=255;orange.g=99;orange.b=71;orange.a=255;
    m_client->TraCIAPI::vehicle.setColor (m_id,orange);
    // double speedLimit = 75/3.6;
    // m_client->TraCIAPI::vehicle.setMaxSpeed (m_id,speedLimit);
  }

  long
  tlmClient80211p::compute_timestampIts ()
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
