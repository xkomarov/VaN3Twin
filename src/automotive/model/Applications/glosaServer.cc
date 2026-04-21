#include "glosaServer.h"

#include "ns3/CAM.h"
#include "ns3/SPATEM.h"
#include "ns3/Seq.hpp"
#include "ns3/Getter.hpp"
#include "ns3/Setter.hpp"
#include "ns3/Encoding.hpp"
#include "ns3/SetOf.hpp"
#include "ns3/SequenceOf.hpp"
#include "ns3/socket.h"
#include "ns3/btpdatarequest.h"
#include "ns3/network-module.h"
#include <map>
#include <limits>

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("glosaServer");

  NS_OBJECT_ENSURE_REGISTERED(glosaServer);

  TypeId
  glosaServer::GetTypeId (void)
  {
    static TypeId tid =
        TypeId ("ns3::glosaServer")
        .SetParent<Application> ()
        .SetGroupName ("Applications")
        .AddConstructor<glosaServer> ()
        .AddAttribute ("Model",
            "Access Technology Model (80211p or lte)",
            StringValue ("lte"),
            MakeStringAccessor (&glosaServer::m_model),
            MakeStringChecker ())
        .AddAttribute ("AggregateOutput",
           "If it is true, the server will print every second an aggregate output about cam and denm",
           BooleanValue (false),
           MakeBooleanAccessor (&glosaServer::m_aggregate_output),
           MakeBooleanChecker ())
        .AddAttribute ("RealTime",
           "To compute properly timestamps",
           BooleanValue(false),
           MakeBooleanAccessor (&glosaServer::m_real_time),
           MakeBooleanChecker ())
        .AddAttribute ("CSV",
            "CSV log name",
            StringValue (),
            MakeStringAccessor (&glosaServer::m_csv_name),
            MakeStringChecker ())
        .AddAttribute ("Client",
           "TraCI client for SUMO",
           PointerValue (0),
           MakePointerAccessor (&glosaServer::m_client),
           MakePointerChecker<TraciClient> ())
        .AddAttribute ("MetricSupervisor",
            "Metric Supervisor to compute metric according to 3GPP TR36.885 V14.0.0 page 70",
            PointerValue (0),
            MakePointerAccessor (&glosaServer::m_metric_supervisor),
            MakePointerChecker<MetricSupervisor> ())
        .AddAttribute ("SendCAM",
             "To enable/disable the transmission of CAM messages",
             BooleanValue(true),
             MakeBooleanAccessor (&glosaServer::m_send_cam),
             MakeBooleanChecker ())
        .AddAttribute ("SendSPATEM",
             "To enable/disable the transmission of SPATEM messages",
             BooleanValue(true),
             MakeBooleanAccessor (&glosaServer::m_send_spatem),
             MakeBooleanChecker ())
        .AddAttribute ("SumoId",
             "SUMO ID of the POI/RSU this server represents",
             StringValue ("poi_0"),
             MakeStringAccessor (&glosaServer::m_sumo_id),
             MakeStringChecker ())
        .AddAttribute ("TargetTLS",
             "SUMO ID of the target TLS. 'auto' to auto-detect closest, 'all' to broadcast all, or a specific ID.",
             StringValue (""),
             MakeStringAccessor (&glosaServer::m_target_tls),
             MakeStringChecker ())
        .AddAttribute ("NumRSUs",
             "Number of RSUs in the simulation",
             UintegerValue (1),
             MakeUintegerAccessor (&glosaServer::m_num_rsus),
             MakeUintegerChecker ());

        return tid;
  }

  glosaServer::glosaServer ()
  {
    NS_LOG_FUNCTION(this);
    m_client = nullptr;
    m_cam_received = 0;
  }

  glosaServer::~glosaServer ()
  {
    NS_LOG_FUNCTION(this);
  }

  void
  glosaServer::DoDispose (void)
  {
    NS_LOG_FUNCTION(this);
    Application::DoDispose ();
  }

  void
  glosaServer::StartApplication (void)
  {
    NS_LOG_FUNCTION(this);

    m_id = m_sumo_id;
    if (m_model == "80211p")
    {
      m_id = m_client->GetStationId(this->GetNode());
    }

    if (m_model == "80211p")
    {
      /* TX socket for DENMs and RX socket for CAMs */
      TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
      m_socket = Socket::CreateSocket (GetNode (), tid);

      /* Bind the socket to local address */
      PacketSocketAddress local_denm;
      local_denm.SetSingleDevice (GetNode ()->GetDevice (0)->GetIfIndex ());
      local_denm.SetPhysicalAddress (GetNode ()->GetDevice (0)->GetAddress ());
      local_denm.SetProtocol (0x8947);
      if (m_socket->Bind (local_denm) == -1)
      {
        NS_FATAL_ERROR ("Failed to bind server socket");
      }

      /* Set socket to broadcast */
      PacketSocketAddress remote;
      remote.SetSingleDevice (GetNode ()->GetDevice (0)->GetIfIndex ());
      remote.SetPhysicalAddress (GetNode ()->GetDevice (0)->GetBroadcast ());
      remote.SetProtocol (0x8947);
      m_socket->Connect(remote);
    }
    else if (m_model == "lte")
    {
      /* TX/RX socket */
      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
      m_socket = Socket::CreateSocket (GetNode (), tid);

      /* Bind the socket to local address */
      InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 9);
      if (m_socket->Bind (local) == -1)
      {
        NS_FATAL_ERROR ("Failed to bind server socket");
      }
    }
    else
    {
      NS_FATAL_ERROR ("No access technology model set - check simulation script - valid models: '80211p' or 'lte'");
    }

    /* Create new BTP and GeoNet objects and set them in CABasicService / TLMBasicService */
    m_btp = CreateObject <btp>();
    m_geoNet = CreateObject <GeoNet>();

    if(m_metric_supervisor!=nullptr)
    {
      m_geoNet->setMetricSupervisor(m_metric_supervisor);
    }

    m_btp->setGeoNet(m_geoNet);
    m_caService.setBTP(m_btp);
    m_tlmBasicService.setBTP(m_btp);

    uint64_t id = 0;
    if (!m_id.empty() && m_id.find("_") != std::string::npos)
    {
      size_t start = m_id.find("_") + 1;
      size_t end = m_id.find_first_not_of("0123456789", start);
      std::string id_str = m_id.substr(start, end - start);
      try {
        if (!id_str.empty()) id = std::stoull(id_str);
      } catch (...) {}
    }

    /* Set callback and station properties in CABasicService */
    m_caService.setStationProperties (m_stationId_baseline + id, StationType_roadSideUnit);
    m_caService.setSocketRx (m_socket);
    m_caService.setSocketTx (m_socket);
    m_caService.addCARxCallback (std::bind(&glosaServer::receiveCAM,this,std::placeholders::_1,std::placeholders::_2));

    m_tlmBasicService.setStationProperties (m_stationId_baseline + id, StationType_roadSideUnit);
    if (m_model == "lte")
    {
      m_tlmBasicService.setSocketRx (m_socket);
    }
    m_tlmBasicService.setSocketTx (m_socket);

    libsumo::TraCIPosition rsuPosXY;

    rsuPosXY = m_client->TraCIAPI::poi.getPosition (m_id);

    libsumo::TraCIPosition rsuPosLonLat = m_client->TraCIAPI::simulation.convertXYtoLonLat (rsuPosXY.x,rsuPosXY.y);

    m_caService.setFixedPositionRSU (rsuPosLonLat.y,rsuPosLonLat.x);
    m_tlmBasicService.setFixedPositionRSU (rsuPosLonLat.y,rsuPosLonLat.x);

    std::string actual_target_tls;

    if (m_target_tls == "")
    {
      if (m_model == "80211p")
      {
          auto tlsIDs = m_client->TraCIAPI::trafficlights.getIDList();
          if (m_num_rsus == 1 && tlsIDs.size() > 1) {
              actual_target_tls = "all";
              NS_LOG_INFO("[" << m_id << "] Smart detection: 1 RSU and multiple TLS. Defaulting to 'all'.");
          } else {
              actual_target_tls = "auto";
          }
      }
      else actual_target_tls = "all";
    }
    else
    {
      actual_target_tls = m_target_tls;
    }

    VDP* traci_vdp;
    std::string final_tls_id = "";

    if (actual_target_tls == "auto")
    {
      auto tlsIDs = m_client->TraCIAPI::trafficlights.getIDList();
      double minDist = std::numeric_limits<double>::max();

      for (const auto& tlsId : tlsIDs) {
          try {
            libsumo::TraCIPosition junctionXY = m_client->TraCIAPI::junction.getPosition(tlsId);
            double dx = rsuPosXY.x - junctionXY.x;
            double dy = rsuPosXY.y - junctionXY.y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < minDist) {
                minDist = dist;
                final_tls_id = tlsId;
            }
          } catch (...) {
            // Ignore if tlsId is not a junction
          }
      }

      if (final_tls_id.empty()) {
          NS_LOG_WARN("[" << m_id << "] No traffic lights found near RSU for auto-detection!");
      } else {
          NS_LOG_INFO("[" << m_id << "] Auto-detected TLS: " << final_tls_id << " (distance: " << minDist << " m)");
      }
    }
    else if (actual_target_tls == "all")
    {
      final_tls_id = "";
      NS_LOG_INFO("[" << m_id << "] Target TLS set to 'all' (will pull SPATEM for all traffic lights)");
    }
    else
    {
      final_tls_id = actual_target_tls;
      NS_LOG_INFO("[" << m_id << "] Target TLS set to specific ID: " << final_tls_id);
    }

    traci_vdp = new VDPTraCI(m_client, m_id, true, final_tls_id);

    uint16_t area_radius = (m_model == "80211p") ? 400 : 5000;
    GeoArea_t geoArea;
    geoArea.posLong = rsuPosLonLat.x*DOT_ONE_MICRO;
    geoArea.posLat = rsuPosLonLat.y*DOT_ONE_MICRO;
    geoArea.distA = area_radius;
    geoArea.distB = 0;
    geoArea.angle = 0;
    geoArea.shape = CIRCULAR;

    m_tlmBasicService.setGeoArea (geoArea);

    m_btp->setVDP(traci_vdp); 
    m_caService.setVDP(traci_vdp);
    m_tlmBasicService.setVDP(traci_vdp);

    if(m_send_spatem && m_model == "80211p")
    {
      m_tlmBasicService.startSpatemDissemination();
    }

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
      m_aggegateOutputEvent = Simulator::Schedule (Seconds(1), &glosaServer::aggregateOutput, this);
  }

  void
  glosaServer::StopApplication ()
  {
    NS_LOG_FUNCTION(this);
    Simulator::Cancel (m_aggegateOutputEvent);

    if (!m_csv_name.empty ())
      m_csv_ofstream_cam.close ();

    if (m_aggregate_output)
      std::cout << Simulator::Now () << "," << m_cam_received << std::endl;
  }

  void
  glosaServer::StopApplicationNow ()
  {
    NS_LOG_FUNCTION(this);
    StopApplication ();
  }

  void
  glosaServer::receiveCAM (asn1cpp::Seq<CAM> cam, Address from)
  {
    m_cam_received++;

    if (!m_csv_name.empty ())
      {
        // messageId,camId,timestamp,latitude,longitude,heading,speed,acceleration
        m_csv_ofstream_cam << cam->header.messageId << "," << cam->header.stationId << ",";
        m_csv_ofstream_cam << cam->cam.generationDeltaTime << "," << asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.latitude,double)/DOT_ONE_MICRO << ",";
        m_csv_ofstream_cam << asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.longitude,double)/DOT_ONE_MICRO << "," ;
        m_csv_ofstream_cam << asn1cpp::getField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue,double)/DECI << "," << asn1cpp::getField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue,double)/CENTI << ",";
        m_csv_ofstream_cam << asn1cpp::getField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.value,double)/DECI << std::endl;
      }

    if(m_send_spatem && m_model != "80211p")
    {
      m_socket->Connect (from);
      TLMBasicService_error_t trigger_retval = m_tlmBasicService.appTLM_trigger();
      
      if(trigger_retval != SPATEM_NO_ERROR)
      {
        NS_LOG_ERROR("Cannot trigger SPATEM. Error code: " << trigger_retval);
      }
    }
  }

  long
  glosaServer::compute_timestampIts ()
  {
    /* To get millisec since  2004-01-01T00:00:00:000Z */
    auto time = std::chrono::system_clock::now(); // get the current time
    auto since_epoch = time.time_since_epoch(); // get the duration since epoch
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(since_epoch); // convert it in millisecond since epoch

    long elapsed_since_2004 = millis.count() - TIME_SHIFT; // in TIME_SHIFT we saved the millisec from epoch to 2004-01-01
    return elapsed_since_2004;
  }

  void
  glosaServer::aggregateOutput()
  {
    std::cout << Simulator::Now () << "," << m_cam_received << std::endl;
    m_aggegateOutputEvent = Simulator::Schedule (Seconds(1), &glosaServer::aggregateOutput, this);
  }

}
