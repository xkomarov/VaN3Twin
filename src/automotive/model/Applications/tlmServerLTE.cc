#include "tlmServerLTE.h"

#include "ns3/CAM.h"
#include "ns3/DENM.h"
#include "ns3/SPATEM.h"
#include "ns3/socket.h"
#include "ns3/btpdatarequest.h"
#include "ns3/network-module.h"

#include <map>

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("tlmServerLTE");

  NS_OBJECT_ENSURE_REGISTERED(tlmServerLTE);

  TypeId
  tlmServerLTE::GetTypeId (void)
  {
    static TypeId tid =
        TypeId ("ns3::tlmServerLTE")
        .SetParent<Application> ()
        .SetGroupName ("Applications")
        .AddConstructor<tlmServerLTE> ()
        .AddAttribute ("AggregateOutput",
           "If it is true, the server will print every second an aggregate output about cam and denm",
           BooleanValue (false),
           MakeBooleanAccessor (&tlmServerLTE::m_aggregate_output),
           MakeBooleanChecker ())
        .AddAttribute ("RealTime",
           "To compute properly timestamps",
           BooleanValue(false),
           MakeBooleanAccessor (&tlmServerLTE::m_real_time),
           MakeBooleanChecker ())
        .AddAttribute ("CSV",
            "CSV log name",
            StringValue (),
            MakeStringAccessor (&tlmServerLTE::m_csv_name),
            MakeStringChecker ())
        .AddAttribute ("Client",
           "TraCI client for SUMO",
           PointerValue (0),
           MakePointerAccessor (&tlmServerLTE::m_client),
           MakePointerChecker<TraciClient> ())
        .AddAttribute ("MetricSupervisor",
            "Metric Supervisor to compute metric according to 3GPP TR36.885 V14.0.0 page 70",
            PointerValue (0),
            MakePointerAccessor (&tlmServerLTE::m_metric_supervisor),
            MakePointerChecker<MetricSupervisor> ())
        .AddAttribute ("SendCAM",
             "To enable/disable the transmission of CAM messages",
             BooleanValue(true),
             MakeBooleanAccessor (&tlmServerLTE::m_send_cam),
             MakeBooleanChecker ());

        return tid;
  }

  tlmServerLTE::tlmServerLTE ()
  {
    NS_LOG_FUNCTION(this);
    m_client = nullptr;
    m_cam_received = 0;
  }

  tlmServerLTE::~tlmServerLTE ()
  {
    NS_LOG_FUNCTION(this);
  }

  void
  tlmServerLTE::DoDispose (void)
  {
    NS_LOG_FUNCTION(this);
    Application::DoDispose ();
  }

  void
  tlmServerLTE::StartApplication (void)
  {
    NS_LOG_FUNCTION(this);

    m_id = m_client->GetStationId (this -> GetNode ());
    if (m_id.empty()) {
        NS_LOG_INFO("m_id was empty, defaulting to poi_0");
        m_id = "poi_0";
    }

    /* TX socket for DENMs and RX socket for CAMs */
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

    m_socket = Socket::CreateSocket (GetNode (), tid);

    /* Bind the socket to receive packets coming from every IP */
    InetSocketAddress local_denm = InetSocketAddress (Ipv4Address::GetAny (), 9);

    // Bind the socket to local address
    if (m_socket->Bind (local_denm) == -1)
    {
      NS_FATAL_ERROR ("Failed to bind server UDP socket");
    }
    
    // Pass the active vehicles set to TLM and CA basic services for LTE unicast
    m_tlmBasicService.setLTEAddresses(&m_active_vehicles);
    m_caService.setLTEAddresses(&m_active_vehicles);

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

    uint64_t id = 777888999; // Default ID for LTE server
    if (!m_id.empty() && m_id.find("_") != std::string::npos) {
        size_t start = m_id.find("_") + 1;
        size_t end = m_id.find_first_not_of("0123456789", start); // find the end of the id
        if (start < m_id.length()) {
            std::string id_str = m_id.substr(start, end - start);
            try {
                id = m_stationId_baseline + std::stoull(id_str);
            } catch (...) {}
        }
    }

    /* Set callback and station properties in CABasicService */
    m_caService.setStationProperties (id, StationType_roadSideUnit);
    m_caService.setSocketRx (m_socket);
    m_caService.setSocketTx (m_socket);
    m_caService.addCARxCallback (std::bind(&tlmServerLTE::receiveCAM,this,std::placeholders::_1,std::placeholders::_2));

    m_tlmBasicService.setStationProperties (id, StationType_roadSideUnit);
    // m_tlmBasicService.setSocketRx (m_socket);
    m_tlmBasicService.setSocketTx (m_socket);

    libsumo::TraCIPosition rsuPosLonLat;
    try {
        if (!m_id.empty()) {
            libsumo::TraCIPosition rsuPosXY = m_client->TraCIAPI::poi.getPosition (m_id);
            rsuPosLonLat = m_client->TraCIAPI::simulation.convertXYtoLonLat (rsuPosXY.x,rsuPosXY.y);
        } else {
            rsuPosLonLat = m_client->TraCIAPI::simulation.convertXYtoLonLat (0,0);
        }
    } catch (...) {
        rsuPosLonLat = m_client->TraCIAPI::simulation.convertXYtoLonLat (0,0);
    }

    //m_denService.setFixedPositionRSU (rsuPosLonLat.y,rsuPosLonLat.x);
    m_caService.setFixedPositionRSU (rsuPosLonLat.y,rsuPosLonLat.x);
    m_tlmBasicService.setFixedPositionRSU (rsuPosLonLat.y,rsuPosLonLat.x);

    std::map<std::string, std::string> rsu_to_tls;
 
    rsu_to_tls["poi_0"] = "c1"; 
    rsu_to_tls["poi_1"] = "c2"; 

    // 2. Ищем, какой светофор привязан к текущей RSU (которая сейчас запускается)
    std::string target_tls_id = "";
    if (rsu_to_tls.count(m_id) > 0) {
        target_tls_id = rsu_to_tls[m_id];
    } else {
        // Если вдруг появилась новая RSU, но мы забыли добавить ее в словарь выше
        target_tls_id = "c1"; 
    }

    // Задаем индивидуальный радиус зоны распространения для каждого светофора
    std::map<std::string, uint16_t> tls_to_radius;
    tls_to_radius["c1"] = 50;  // Радиус для светофора c1
    tls_to_radius["c2"] = 50; // Радиус для светофора c2
    // tls_to_radius["c3"] = 120; // для других...
    
    uint16_t area_radius = 50; // Радиус по умолчанию
    if (tls_to_radius.count(target_tls_id) > 0) {
        area_radius = tls_to_radius[target_tls_id];
    }

    /* Compute GeoArea for DENMs and SPATEMs (TLM) */
    GeoArea_t geoArea;
    // Longitude and Latitude in [0.1 microdegree]
    geoArea.posLong = rsuPosLonLat.x*DOT_ONE_MICRO;
    geoArea.posLat = rsuPosLonLat.y*DOT_ONE_MICRO;
    // Radius [m] of the circle that covers the area
    geoArea.distA = area_radius;
    // DistB [m] and angle [deg] equal to zero because we are defining a circular area as specified in ETSI EN 302 636-4-1 [9.8.5.2]
    geoArea.distB = 0;
    geoArea.angle = 0;
    geoArea.shape = CIRCULAR;

    m_tlmBasicService.setGeoArea (geoArea);

    // 3. Вызываем наш НОВЫЙ конструктор, передавая ему правильный ID светофора!
    VDP* traci_vdp = new VDPTraCI(m_client, m_id, true, target_tls_id);
    
    m_btp->setVDP(traci_vdp); 

    m_caService.setVDP(traci_vdp);

    m_tlmBasicService.setVDP(traci_vdp);

    m_tlmBasicService.startSpatemDissemination();

    if(m_send_cam)
      {
        std::srand(Simulator::Now().GetNanoSeconds ());
        double desync = ((double)std::rand()/RAND_MAX);
        m_caService.startCamDissemination(desync);
      }

    if (!m_csv_name.empty ())
    {
      m_csv_ofstream_cam.open (m_csv_name+"-server.csv",std::ofstream::trunc);
      m_csv_ofstream_cam << "messageId,camId,timestamp,latitude,longitude,heading,speed,acceleration" << std::endl;
    }

    /* If aggregate output is enabled, start it */
    if (m_aggregate_output)
      m_aggegateOutputEvent = Simulator::Schedule (Seconds(1), &tlmServerLTE::aggregateOutput, this);
  }

  void
  tlmServerLTE::StopApplication ()
  {
    NS_LOG_FUNCTION(this);
    Simulator::Cancel (m_aggegateOutputEvent);

    if (!m_csv_name.empty ())
      m_csv_ofstream_cam.close ();

    if (m_aggregate_output)
      std::cout << Simulator::Now () << "," << m_cam_received << std::endl;
  }

  void
  tlmServerLTE::StopApplicationNow ()
  {
    NS_LOG_FUNCTION(this);
    StopApplication ();
  }

  void
  tlmServerLTE::receiveCAM (asn1cpp::Seq<CAM> cam, Address from)
  {
    /* The reception of a CAM, in this case, woarks as a trigger to generate DENMs.
     * If no CAMs are received, then no DENMs are generated */
    m_cam_received++;

    if (m_active_vehicles.find(from) == m_active_vehicles.end())
    {
        m_active_vehicles.insert(from);
    }

    if (!m_csv_name.empty ())
      {
        // messageId,camId,timestamp,latitude,longitude,heading,speed,acceleration
        m_csv_ofstream_cam << cam->header.messageId << "," << cam->header.stationId << ",";
        m_csv_ofstream_cam << cam->cam.generationDeltaTime << "," << asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.latitude,double)/DOT_ONE_MICRO << ",";
        m_csv_ofstream_cam << asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.longitude,double)/DOT_ONE_MICRO << "," ;
        m_csv_ofstream_cam << asn1cpp::getField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue,double)/DECI << "," << asn1cpp::getField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue,double)/CENTI << ",";
        m_csv_ofstream_cam << asn1cpp::getField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.value,double)/DECI << std::endl;
      }

//    ASN_STRUCT_FREE(asn_DEF_CAM,cam);
  }

  long
  tlmServerLTE::compute_timestampIts ()
  {
    /* To get millisec since  2004-01-01T00:00:00:000Z */
    auto time = std::chrono::system_clock::now(); // get the current time
    auto since_epoch = time.time_since_epoch(); // get the duration since epoch
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(since_epoch); // convert it in millisecond since epoch

    long elapsed_since_2004 = millis.count() - TIME_SHIFT; // in TIME_SHIFT we saved the millisec from epoch to 2004-01-01
    return elapsed_since_2004;
  }

  void
  tlmServerLTE::aggregateOutput()
  {
    std::cout << Simulator::Now () << "," << m_cam_received << std::endl;
    m_aggegateOutputEvent = Simulator::Schedule (Seconds(1), &tlmServerLTE::aggregateOutput, this);
  }

}







