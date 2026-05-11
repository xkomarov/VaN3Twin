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
#include "bGlosaServer.h"

#include "ns3/CAM.h"
#include "ns3/SPATEM.h"
#include "ns3/Seq.hpp"
#include "ns3/Getter.hpp"
#include "ns3/Encoding.hpp"
#include "ns3/socket.h"
#include "ns3/btpdatarequest.h"
#include "ns3/network-module.h"
#include "ns3/idpTraci.h"

namespace ns3 {
NS_LOG_COMPONENT_DEFINE ("bGlosaServer");

NS_OBJECT_ENSURE_REGISTERED (bGlosaServer);

TypeId
bGlosaServer::GetTypeId (void)
{
  static TypeId tid =
      TypeId ("ns3::bGlosaServer")
          .SetParent<Application> ()
          .SetGroupName ("Applications")
          .AddConstructor<bGlosaServer> ()
          .AddAttribute ("Model", "Access Technology Model (80211p or lte)", StringValue ("lte"),
                         MakeStringAccessor (&bGlosaServer::m_model), MakeStringChecker ())
          .AddAttribute ("AggregateOutput",
                         "If it is true, the server will print every second an aggregate output "
                         "about cam",
                         BooleanValue (false),
                         MakeBooleanAccessor (&bGlosaServer::m_aggregate_output),
                         MakeBooleanChecker ())
          .AddAttribute ("RealTime", "To compute properly timestamps", BooleanValue (false),
                         MakeBooleanAccessor (&bGlosaServer::m_real_time), MakeBooleanChecker ())
          .AddAttribute ("CSV", "CSV log name", StringValue (),
                         MakeStringAccessor (&bGlosaServer::m_csv_name), MakeStringChecker ())
          .AddAttribute ("Client", "TraCI client for SUMO", PointerValue (0),
                         MakePointerAccessor (&bGlosaServer::m_client),
                         MakePointerChecker<TraciClient> ())
          .AddAttribute (
              "MetricSupervisor",
              "Metric Supervisor to compute metric according to 3GPP TR36.885 V14.0.0 page 70",
              PointerValue (0), MakePointerAccessor (&bGlosaServer::m_metric_supervisor),
              MakePointerChecker<MetricSupervisor> ())
          .AddAttribute ("SendCAM", "To enable/disable the transmission of CAM messages",
                         BooleanValue (true), MakeBooleanAccessor (&bGlosaServer::m_send_cam),
                         MakeBooleanChecker ())
          .AddAttribute ("SendSPATEM", "To enable/disable the transmission of SPATEM messages",
                         BooleanValue (true), MakeBooleanAccessor (&bGlosaServer::m_send_spatem),
                         MakeBooleanChecker ())
          .AddAttribute ("SumoId", "SUMO ID of the POI/RSU this server represents",
                         StringValue ("poi_0"), MakeStringAccessor (&bGlosaServer::m_sumo_id),
                         MakeStringChecker ());

  return tid;
}

bGlosaServer::bGlosaServer ()
{
  NS_LOG_FUNCTION (this);
  m_client = nullptr;
  m_cam_received = 0;
}

bGlosaServer::~bGlosaServer ()
{
  NS_LOG_FUNCTION (this);
}

void
bGlosaServer::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  Application::DoDispose ();
}

void
bGlosaServer::StartApplication (void)
{
  NS_LOG_FUNCTION (this);

  m_id = m_sumo_id;
  if (m_model == "80211p")
    {
      m_id = m_client->GetStationId (this->GetNode ());
    }

  if (m_model == "80211p")
    {
      TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
      m_socket = Socket::CreateSocket (GetNode (), tid);

      
      PacketSocketAddress local_spatem;
      local_spatem.SetSingleDevice (GetNode ()->GetDevice (0)->GetIfIndex ());
      local_spatem.SetPhysicalAddress (GetNode ()->GetDevice (0)->GetAddress ());
      local_spatem.SetProtocol (0x8947);
      if (m_socket->Bind (local_spatem) == -1)
        {
          NS_FATAL_ERROR ("Failed to bind server socket");
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
          NS_FATAL_ERROR ("Failed to bind server socket");
        }
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

  uint64_t id = 0;
  if (!m_id.empty () && m_id.find ("_") != std::string::npos)
    {
      size_t start = m_id.find ("_") + 1;
      size_t end = m_id.find_first_not_of ("0123456789", start);
      std::string id_str = m_id.substr (start, end - start);
      try
        {
          if (!id_str.empty ())
            id = std::stoull (id_str);
        }
      catch (...)
        {
        }
    }

  
  m_caService.setStationProperties (m_stationId_baseline + id, StationType_roadSideUnit);
  m_caService.setSocketRx (m_socket);
  m_caService.setSocketTx (m_socket);
  m_caService.addCARxCallback (
      std::bind (&bGlosaServer::receiveCAM, this, std::placeholders::_1, std::placeholders::_2));

  m_tlmService.setStationProperties (m_stationId_baseline + id, StationType_roadSideUnit);
  if (m_model == "lte")
    {
      m_tlmService.setSocketRx (m_socket);
    }
  m_tlmService.setSocketTx (m_socket);

  libsumo::TraCIPosition rsuPosXY;

  rsuPosXY = m_client->TraCIAPI::poi.getPosition (m_id);

  libsumo::TraCIPosition rsuPosLonLat =
      m_client->TraCIAPI::simulation.convertXYtoLonLat (rsuPosXY.x, rsuPosXY.y);

  m_caService.setFixedPositionRSU (rsuPosLonLat.y, rsuPosLonLat.x);
  m_tlmService.setFixedPositionRSU (rsuPosLonLat.y, rsuPosLonLat.x);

  VDP *traci_vdp;
  IDP *traci_idp;
  if (m_model == "80211p")
    {
      
      const double tlsDetectionRadius = 10.0; 
      auto tlsIDs = m_client->TraCIAPI::trafficlights.getIDList ();
      std::vector<std::string> nearbyTls;

      for (const auto &tlsId : tlsIDs)
        {
          libsumo::TraCIPosition junctionXY = m_client->TraCIAPI::junction.getPosition (tlsId);
          double dx = rsuPosXY.x - junctionXY.x;
          double dy = rsuPosXY.y - junctionXY.y;
          double dist = std::sqrt (dx * dx + dy * dy);

          if (dist <= tlsDetectionRadius)
            {
              nearbyTls.push_back (tlsId);
              NS_LOG_INFO ("[" << m_id << "] TLS '" << tlsId << "' within range (dist=" << dist
                               << "m)");
            }
          else
            {
              NS_LOG_DEBUG ("[" << m_id << "] TLS '" << tlsId << "' out of range (dist=" << dist
                                << "m), skipped");
            }
        }

      if (nearbyTls.empty ())
        {
          NS_LOG_WARN ("[" << m_id << "] No traffic lights found within " << tlsDetectionRadius
                           << "m of RSU!");
        }
      else
        {
          NS_LOG_INFO ("[" << m_id << "] Broadcasting SPATEM for " << nearbyTls.size ()
                           << " nearby traffic light(s)");
        }

      
      uint16_t area_radius = (uint16_t) (tlsDetectionRadius + 390);
      GeoArea_t geoArea;
      geoArea.posLong = rsuPosLonLat.x * DOT_ONE_MICRO;
      geoArea.posLat = rsuPosLonLat.y * DOT_ONE_MICRO;
      geoArea.distA = area_radius;
      geoArea.distB = 0;
      geoArea.angle = 0;
      geoArea.shape = CIRCULAR;

      m_tlmService.setGeoArea (geoArea);

      
      traci_vdp = new VDPTraCI (m_client, m_id, true);
      traci_idp = new IDPTraCI (m_client, m_id);
      
      static_cast<IDPTraCI *> (traci_idp)->setTargetTlsList (nearbyTls);
    }
  else
    {
      
      
      traci_vdp = new VDPTraCI (m_client, m_id, true);
      traci_idp = new IDPTraCI (m_client, m_id);

      uint16_t area_radius = 5000;
      GeoArea_t geoArea;
      geoArea.posLong = rsuPosLonLat.x * DOT_ONE_MICRO;
      geoArea.posLat = rsuPosLonLat.y * DOT_ONE_MICRO;
      geoArea.distA = area_radius;
      geoArea.distB = 0;
      geoArea.angle = 0;
      geoArea.shape = CIRCULAR;
      m_tlmService.setGeoArea (geoArea);
    }

  m_btp->setVDP (traci_vdp);
  m_btp->setIDP (traci_idp);
  m_caService.setVDP (traci_vdp);
  
  m_tlmService.setIDP (traci_idp);

  if (m_send_spatem && m_model == "80211p")
    {
      m_tlmService.startSpatemDissemination ();
    }

  if (m_send_cam)
    {
      std::srand (Simulator::Now ().GetNanoSeconds ());
      double desync = ((double) std::rand () / RAND_MAX);
      m_caService.startCamDissemination (desync);
    }

  if (!m_csv_name.empty ())
    {
      m_csv_ofstream_cam.open (m_csv_name + "-server.csv", std::ofstream::trunc);
      m_csv_ofstream_cam
          << "messageId,camId,timestamp,latitude,longitude,heading,speed,acceleration" << std::endl;
    }

  
  if (m_aggregate_output)
    m_aggegateOutputEvent = Simulator::Schedule (Seconds (1), &bGlosaServer::aggregateOutput, this);
}

void
bGlosaServer::StopApplication ()
{
  NS_LOG_FUNCTION (this);
  Simulator::Cancel (m_aggegateOutputEvent);

  if (!m_csv_name.empty ())
    m_csv_ofstream_cam.close ();

  if (m_aggregate_output)
    std::cout << Simulator::Now () << "," << m_cam_received << std::endl;
}

void
bGlosaServer::StopApplicationNow ()
{
  NS_LOG_FUNCTION (this);
  StopApplication ();
}

void
bGlosaServer::receiveCAM (asn1cpp::Seq<CAM> cam, Address from)
{
  m_cam_received++;

  if (!m_csv_name.empty ())
    {
      
      m_csv_ofstream_cam << cam->header.messageId << "," << cam->header.stationId << ",";
      m_csv_ofstream_cam << cam->cam.generationDeltaTime << ","
                         << asn1cpp::getField (
                                cam->cam.camParameters.basicContainer.referencePosition.latitude,
                                double) /
                                DOT_ONE_MICRO
                         << ",";
      m_csv_ofstream_cam << asn1cpp::getField (
                                cam->cam.camParameters.basicContainer.referencePosition.longitude,
                                double) /
                                DOT_ONE_MICRO
                         << ",";
      m_csv_ofstream_cam
          << asn1cpp::getField (cam->cam.camParameters.highFrequencyContainer.choice
                                    .basicVehicleContainerHighFrequency.heading.headingValue,
                                double) /
                 DECI
          << ","
          << asn1cpp::getField (cam->cam.camParameters.highFrequencyContainer.choice
                                    .basicVehicleContainerHighFrequency.speed.speedValue,
                                double) /
                 CENTI
          << ",";
      m_csv_ofstream_cam << asn1cpp::getField (cam->cam.camParameters.highFrequencyContainer.choice
                                                   .basicVehicleContainerHighFrequency
                                                   .longitudinalAcceleration.value,
                                               double) /
                                DECI
                         << std::endl;
    }

  if (m_send_spatem && m_model != "80211p")
    {
      m_socket->Connect (from);
      TLMService_error_t trigger_retval = m_tlmService.appTLM_trigger ();

      if (trigger_retval != SPATEM_NO_ERROR)
        {
          NS_LOG_ERROR ("Cannot trigger SPATEM. Error code: " << trigger_retval);
        }
    }
}

void
bGlosaServer::aggregateOutput ()
{
  std::cout << Simulator::Now () << "," << m_cam_received << std::endl;
  m_aggegateOutputEvent = Simulator::Schedule (Seconds (1), &bGlosaServer::aggregateOutput, this);
}

} 
