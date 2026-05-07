#include "tlmServerLTE.h"

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

namespace ns3 {
NS_LOG_COMPONENT_DEFINE ("tlmServerLTE");

NS_OBJECT_ENSURE_REGISTERED (tlmServerLTE);

TypeId
tlmServerLTE::GetTypeId (void)
{
  static TypeId tid =
      TypeId ("ns3::tlmServerLTE")
          .SetParent<Application> ()
          .SetGroupName ("Applications")
          .AddConstructor<tlmServerLTE> ()
          .AddAttribute ("AggregateOutput",
                         "If it is true, the server will print every second an aggregate output "
                         "about cam and denm",
                         BooleanValue (false),
                         MakeBooleanAccessor (&tlmServerLTE::m_aggregate_output),
                         MakeBooleanChecker ())
          .AddAttribute ("RealTime", "To compute properly timestamps", BooleanValue (false),
                         MakeBooleanAccessor (&tlmServerLTE::m_real_time), MakeBooleanChecker ())
          .AddAttribute ("CSV", "CSV log name", StringValue (),
                         MakeStringAccessor (&tlmServerLTE::m_csv_name), MakeStringChecker ())
          .AddAttribute ("Client", "TraCI client for SUMO", PointerValue (0),
                         MakePointerAccessor (&tlmServerLTE::m_client),
                         MakePointerChecker<TraciClient> ())
          .AddAttribute (
              "MetricSupervisor",
              "Metric Supervisor to compute metric according to 3GPP TR36.885 V14.0.0 page 70",
              PointerValue (0), MakePointerAccessor (&tlmServerLTE::m_metric_supervisor),
              MakePointerChecker<MetricSupervisor> ())
          .AddAttribute ("SendCAM", "To enable/disable the transmission of CAM messages",
                         BooleanValue (true), MakeBooleanAccessor (&tlmServerLTE::m_send_cam),
                         MakeBooleanChecker ())
          .AddAttribute ("SendSPATEM", "To enable/disable the transmission of SPATEM messages",
                         BooleanValue (true), MakeBooleanAccessor (&tlmServerLTE::m_send_spatem),
                         MakeBooleanChecker ())
          .AddAttribute ("SumoId", "SUMO ID of the POI/RSU this server represents",
                         StringValue ("poi_0"), MakeStringAccessor (&tlmServerLTE::m_sumo_id),
                         MakeStringChecker ());

  return tid;
}

tlmServerLTE::tlmServerLTE ()
{
  NS_LOG_FUNCTION (this);
  m_client = nullptr;
  m_cam_received = 0;
}

tlmServerLTE::~tlmServerLTE ()
{
  NS_LOG_FUNCTION (this);
}

void
tlmServerLTE::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  Application::DoDispose ();
}

void
tlmServerLTE::StartApplication (void)
{
  NS_LOG_FUNCTION (this);

  m_id = m_sumo_id;

  /* TX socket for DENMs and RX socket for CAMs (one socket only is necessary) */
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

  m_socket = Socket::CreateSocket (GetNode (), tid);

  /* Bind the socket to local address */
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 9);
  if (m_socket->Bind (local) == -1)
    {
      NS_FATAL_ERROR ("Failed to bind server socket");
    }

  /* Create new BTP and GeoNet objects and set them in DENBasicService and CABasicService */
  m_btp = CreateObject<btp> ();
  m_geoNet = CreateObject<GeoNet> ();

  if (m_metric_supervisor != nullptr)
    {
      m_geoNet->setMetricSupervisor (m_metric_supervisor);
    }

  m_btp->setGeoNet (m_geoNet);
  m_caService.setBTP (m_btp);
  m_tlmService.setBTP (m_btp);

  size_t start = m_id.find ("_") + 1;
  size_t end = m_id.find_first_not_of ("0123456789", start); // find the end of the id
  std::string id_str = m_id.substr (start, end - start);
  uint64_t id = std::stoull (id_str);
  //m_denService.setStationProperties (m_stationId_baseline + id, StationType_roadSideUnit);

  /* Set callback and station properties in CABasicService */
  m_caService.setStationProperties (m_stationId_baseline + id, StationType_roadSideUnit);
  m_caService.setSocketRx (m_socket);
  m_caService.setSocketTx (m_socket);
  m_caService.addCARxCallback (
      std::bind (&tlmServerLTE::receiveCAM, this, std::placeholders::_1, std::placeholders::_2));

  m_tlmService.setStationProperties (m_stationId_baseline + id, StationType_roadSideUnit);
  m_tlmService.setSocketRx (m_socket);
  m_tlmService.setSocketTx (m_socket);

  libsumo::TraCIPosition rsuPosXY = m_client->TraCIAPI::poi.getPosition (m_id);
  libsumo::TraCIPosition rsuPosLonLat =
      m_client->TraCIAPI::simulation.convertXYtoLonLat (rsuPosXY.x, rsuPosXY.y);

  //m_denService.setFixedPositionRSU (rsuPosLonLat.y,rsuPosLonLat.x);
  m_caService.setFixedPositionRSU (rsuPosLonLat.y, rsuPosLonLat.x);
  m_tlmService.setFixedPositionRSU (rsuPosLonLat.y, rsuPosLonLat.x);

  // VDP for the global server: Empty string pulls all Traffic Lights
  // VDP *traci_vdp = new VDPTraCI (m_client, m_sumo_id, true, "");
  VDP *traci_vdp = new VDPTraCI (m_client, m_sumo_id, true);
  IDP *traci_idp = new IDPTraCI (m_client, m_sumo_id);

  uint16_t area_radius = 5000; // Global map radius for the consolidated SPATEM geocast

  /* Compute GeoArea for DENMs and SPATEMs (TLM) */
  GeoArea_t geoArea;
  // Longitude and Latitude in [0.1 microdegree]
  geoArea.posLong = rsuPosLonLat.x * DOT_ONE_MICRO;
  geoArea.posLat = rsuPosLonLat.y * DOT_ONE_MICRO;
  // Radius [m] of the circle that covers the area
  geoArea.distA = area_radius;
  // DistB [m] and angle [deg] equal to zero because we are defining a circular area as specified in ETSI EN 302 636-4-1 [9.8.5.2]
  geoArea.distB = 0;
  geoArea.angle = 0;
  geoArea.shape = CIRCULAR;

  m_tlmService.setGeoArea (geoArea);

  m_btp->setVDP (traci_vdp);
  m_btp->setIDP (traci_idp);

  m_caService.setVDP (traci_vdp);

  m_tlmService.setIDP (traci_idp);

  // Removed periodic dissemination in LTE. Will be triggered strictly in receiveCAM.
  // m_tlmService.startSpatemDissemination();

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

  /* If aggregate output is enabled, start it */
  if (m_aggregate_output)
    m_aggegateOutputEvent = Simulator::Schedule (Seconds (1), &tlmServerLTE::aggregateOutput, this);
}

void
tlmServerLTE::StopApplication ()
{
  NS_LOG_FUNCTION (this);
  Simulator::Cancel (m_aggegateOutputEvent);

  if (!m_csv_name.empty ())
    m_csv_ofstream_cam.close ();

  if (m_aggregate_output)
    std::cout << Simulator::Now () << "," << m_cam_received << std::endl;
}

void
tlmServerLTE::StopApplicationNow ()
{
  NS_LOG_FUNCTION (this);
  StopApplication ();
}

void
tlmServerLTE::receiveCAM (asn1cpp::Seq<CAM> cam, Address from)
{
  /* The reception of a CAM works as a trigger to generate/reply with SPATEMs.
     * If no CAMs are received, then no SPATEMs are generated */
  m_cam_received++;

  if (!m_csv_name.empty ())
    {
      // messageId,camId,timestamp,latitude,longitude,heading,speed,acceleration
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

  // Connect socket to specific vehicle UE IP and fire unicast SPATEM message
  if (m_send_spatem)
    {
      m_socket->Connect (from);
      TLMService_error_t trigger_retval = m_tlmService.appTLM_trigger ();

      if (trigger_retval != SPATEM_NO_ERROR)
        {
          NS_LOG_ERROR ("Cannot trigger SPATEM. Error code: " << trigger_retval);
        }
    }
}

long
tlmServerLTE::compute_timestampIts ()
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

void
tlmServerLTE::aggregateOutput ()
{
  std::cout << Simulator::Now () << "," << m_cam_received << std::endl;
  m_aggegateOutputEvent = Simulator::Schedule (Seconds (1), &tlmServerLTE::aggregateOutput, this);
}

} // namespace ns3
