#include "ns3/carla-module.h"
//#include "ns3/automotive-module.h"
#include "ns3/tlmServerLTE-helper.h"
#include "ns3/tlmClientLTE.h"
#include "ns3/tlmClientLTE-helper.h"
#include "ns3/traci-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-helper.h"
#include "ns3/lte-module.h"
#include "ns3/config-store.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/sumo_xml_parser.h"
#include "ns3/vehicle-visualizer-module.h"
#include "ns3/MetricSupervisor.h"
#include <unistd.h>
#include <map>
#include <fstream>
#include <iomanip>
#include <functional>

struct VehicleMetrics
{
  double totalCO2_mg = 0.0; ///< Accumulated CO2 emissions [mg]
  double totalTravelTime_s = 0.0; ///< Total time vehicle was active [s]
  double totalStoppedTime_s = 0.0; ///< Total time with speed < 0.1 m/s [s]
  double totalDistance_m = 0.0; ///< Total distance traveled [m]
  double baselineDistance_m = 0.0; ///< Odometer reading at first observation [m]
  bool initialized = false; ///< Whether first sample has been taken
};

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("v2i-tlm-LTE");

int
main (int argc, char *argv[])
{
  /*
   * In this example the generated vehicles will broadcast their CAMs, that will be also received by a RSU placed in the middle
   * of the simulated scenario. Both vehicles and RSU transmit V2X messages through a 802.11p interface.
   * The RSU broadcasts DENM messages with a frequency of 1 Hz, using the ETSI ITS-G5 stack, and the GeoNet
   * dissemination area is set as a circular area around the RSU (90 meters of diameter).
   * The dissemination of DENMs starts only after the RSU receives a CAM. If for 5 seconds
   * the RSU does not receive any CAM, the DENM dissemination stops.
   * Each DENM includes an optional container (à la carte), where there is a field named RoadWorks->SpeedLimit, in which
   * the maximum speed allowed in the dissemination area is specified (25km/h).
   * Whenever a vehicle receives a DENM at application layer (meaning that the information traverses GeoNet without being
   * filtered), it reads the information inside the à la carte container and corrects its speed accordingly.
   * If a vehicle doesn't receive any DENM for more than 1.5 seconds, it resumes its old speed.
   */

  // Admitted data rates for 802.11p
  std::vector<float> rate_admitted_values{3,4.5,6,9,12,18,24,27};
  std::string datarate_config;

  /*** 0.a App Options ***/
  std::string sumo_folder = "src/automotive/examples/tlm_map_1_rsu_copy/";
  std::string mob_trace = "cars.rou.xml";
  std::string rsu_file = "stations.xml";
  std::string sumo_config ="src/automotive/examples/tlm_map_1_rsu_copy/map.sumo.cfg";

  bool verbose = true;
  bool realtime = false;
  bool sumo_gui = true;
  bool aggregate_out = false;
  double sumo_updates = 0.01;
  std::string csv_name;
  std::string csv_name_cumulative;
  std::string sumo_netstate_file_name;
  bool print_summary = false;
  bool vehicle_vis = false;
  bool log_metrics = false;
  std::string metrics_csv_name;

  /*** 0.b LENA Options ***/
  double interPacketInterval = 100;
  bool useCa = false;

  // Disabling this option turns off the whole V2X application (useful for comparing the situation when the application is enabled and the one in which it is disabled)
  bool send_cam = true;
  bool send_spatem = true;
  double m_baseline_prr = 150.0;
  bool m_metric_sup = true;

  double simTime = 100;

  int numberOfNodes;
  uint32_t nodeCounter = 0;
  int numberOfRSUs;
  uint32_t rsuCounter = 0;

  CommandLine cmd;

  xmlDocPtr rou_xml_file;

  /* Cmd Line option for vehicular application */
  cmd.AddValue ("realtime", "Use the realtime scheduler or not", realtime);
  cmd.AddValue ("sumo-gui", "Use SUMO gui or not", sumo_gui);
  cmd.AddValue ("server-aggregate-output", "Print an aggregate output for server", aggregate_out);
  cmd.AddValue ("sumo-updates", "SUMO granularity", sumo_updates);
  cmd.AddValue ("sumo-folder","Position of sumo config files",sumo_folder);
  cmd.AddValue ("mob-trace", "Name of the mobility trace file", mob_trace);
  cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", sumo_config);
  cmd.AddValue ("csv-log", "Name of the CSV log file", csv_name);
  cmd.AddValue ("summary", "Print a summary for each vehicle at the end of the simulation", print_summary);
  cmd.AddValue ("vehicle-visualizer", "Activate the web-based vehicle visualizer for ms-van3t", vehicle_vis);
  cmd.AddValue ("send-cam", "Turn on or off the transmission of CAMs, thus turning on or off the whole V2X application",send_cam);
  cmd.AddValue ("send-spatem", "Turn on or off the transmission of SPATEM messages from the server",send_spatem);
  cmd.AddValue ("csv-log-cumulative", "Name of the CSV log file for the cumulative (average) PRR and latency data", csv_name_cumulative);
  cmd.AddValue ("netstate-dump-file", "Name of the SUMO netstate-dump file containing the vehicle-related information throughout the whole simulation", sumo_netstate_file_name);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("met-sup","Use the Metric supervisor or not",m_metric_sup);
  cmd.AddValue ("log-metrics",
                "Enable per-vehicle metric logging (CO2, travel time, stopped time, avg speed)",
                log_metrics);


  /* Cmd Line option for Lena */
  cmd.AddValue("interPacketInterval", "Inter packet interval [ms]", interPacketInterval);
  cmd.AddValue("useCa", "Whether to use carrier aggregation", useCa);

  cmd.AddValue ("sim-time", "Total duration of the simulation [s]", simTime);

  cmd.Parse (argc, argv);

  /* Carrier aggregation for LTE */
  if (useCa)
   {
     Config::SetDefault ("ns3::LteHelper::UseCa", BooleanValue (useCa));
     Config::SetDefault ("ns3::LteHelper::NumberOfComponentCarriers", UintegerValue (2));
     Config::SetDefault ("ns3::LteHelper::EnbComponentCarrierManager", StringValue ("ns3::RrComponentCarrierManager"));
   }

  // Fixing the 23-UE limit per eNB in LENA
  Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(320));

  if (verbose)
    {
      LogComponentEnable ("v2i-tlm-LTE", LOG_LEVEL_INFO);
      LogComponentEnable ("CABasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("TLMBasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("tlmServerLTE", LOG_LEVEL_INFO);
    }

  /* Use the realtime scheduler of ns3 */
  if(realtime)
      GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  /*** 0.b Read from the mob_trace the number of vehicles that will be created.
   *       The number of vehicles is directly parsed from the rou.xml file, looking at all
   *       the valid XML elements of type <vehicle>
  ***/
  NS_LOG_INFO("Reading the .rou file...");
  std::string path = sumo_folder + mob_trace;

  /* Load the .rou.xml document */
  xmlInitParser();
  rou_xml_file = xmlParseFile(path.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path);
    }
  numberOfNodes = XML_rou_count_vehicles(rou_xml_file);

  std::string rsu_path = sumo_folder + rsu_file;
  std::ifstream rsu_file_stream (rsu_path.c_str());
  std::vector<std::tuple<std::string, float, float>> rsuData = XML_poli_count_stations(rsu_file_stream);
  numberOfRSUs = rsuData.size();

  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  if(numberOfNodes==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of vehicles from the specified XML file: "<<path<<". Please check if it is a correct SUMO file.");
    }
  NS_LOG_INFO("The .rou file has been read: " << numberOfNodes << " vehicles will be present in the simulation.");

  /* Set the simulation time (in seconds) */
  NS_LOG_INFO("Simulation will last " << simTime << " seconds");
  ns3::Time simulationTime (ns3::Seconds(simTime));

  /*** 1. Create containers for UEs and eNodeBs ***/
  NodeContainer obuNodes; // UEs
  obuNodes.Create(numberOfNodes);

  NodeContainer rsuNodes; // eNodeBs
  rsuNodes.Create(numberOfRSUs);

  /*** 2. Setup Mobility ***/
  MobilityHelper mobility;
  mobility.Install (obuNodes);
  mobility.Install (rsuNodes);

  /*** 3. Create LTE objects & EPC ***/
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  // Enable PCAP tracing on S1-U (eNB <-> SGW) and X2 (eNB <-> eNB) links
  //epcHelper->SetAttribute ("S1uLinkEnablePcap", BooleanValue (true));
  //epcHelper->SetAttribute ("S1uLinkPcapPrefix", StringValue ("v2i-tlm-lte-s1u"));
  //epcHelper->SetAttribute ("X2LinkEnablePcap", BooleanValue (true));
  //epcHelper->SetAttribute ("X2LinkPcapPrefix", StringValue ("v2i-tlm-lte-x2"));
  lteHelper->SetEpcHelper (epcHelper);

  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults();

  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  /*** 4. Create and configure the remote host ***/
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  /* Connect the remote host to the packet gateway and create the Internet */
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("10Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.005)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("10.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  /* interface 0 is localhost, 1 is the p2p device */
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

  //p2ph.EnablePcap("v2i-tlm-lte-remote", internetDevices);


  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  /*** 5. Install LTE Devices to the nodes + assign IP to UE ***/
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (rsuNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (obuNodes);

  /* Install the IP stack on the UEs */
  internet.Install (obuNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));

  /* Assign IP address to UEs */
  for (uint32_t u = 0; u < obuNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = obuNodes.Get (u);
      /* Set the default gateway for the UE */
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  for (uint16_t i = 0; i < numberOfNodes; i++)
    {
      for (uint16_t j = 0; j < numberOfRSUs; j++)
        {
          lteHelper->Attach (ueLteDevs.Get (i), enbLteDevs.Get (j));
        }
    }

  lteHelper->AddX2Interface (rsuNodes);
  /*** 5. Setup Traci and start SUMO ***/
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (sumo_updates)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", (BooleanValue) sumo_gui);
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));

  std::string sumo_additional_options = "--verbose true";

  if(sumo_netstate_file_name!="")
  {
    sumo_additional_options += " --netstate-dump " + sumo_netstate_file_name;
  }

  sumo_additional_options += " --collision.action warn --collision.check-junctions --error-log=sumo-errors-or-collisions.xml";

  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (5.0)));
  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue (sumo_additional_options));

  /* Create and setup the web-based vehicle visualizer of ms-van3t */
  vehicleVisualizer vehicleVisObj;
  Ptr<vehicleVisualizer> vehicleVis = &vehicleVisObj;
  if (vehicle_vis)
  {
      vehicleVis->startServer();
      vehicleVis->connectToServer ();
      sumoClient->SetAttribute ("VehicleVisualizer", PointerValue (vehicleVis));
  }

  Ptr<MetricSupervisor> metSup = NULL;
  MetricSupervisor metSupObj(m_baseline_prr);
  if(m_metric_sup)
    {
      metSup = &metSupObj;
      metSup->setTraCIClient(sumoClient);
    }

  /*** 6. Create and Setup application for the server ***/
  tlmServerLTEHelper tlmServerLTEHelper;
  tlmServerLTEHelper.SetAttribute ("Client", (PointerValue) sumoClient);
  tlmServerLTEHelper.SetAttribute ("RealTime", BooleanValue(realtime));
  tlmServerLTEHelper.SetAttribute ("AggregateOutput", BooleanValue(aggregate_out));
  tlmServerLTEHelper.SetAttribute ("CSV", StringValue(csv_name));
  tlmServerLTEHelper.SetAttribute ("MetricSupervisor", PointerValue (metSup));
  tlmServerLTEHelper.SetAttribute ("SendSPATEM", BooleanValue (send_spatem));

  int i = 0;
  for (auto rsu : rsuData)
    {
      std::string id = std::get<0>(rsu);
      float x = std::get<1>(rsu);
      float y = std::get<2>(rsu);
      Ptr<Node> rsuNode = rsuNodes.Get (i);
      sumoClient->AddStation(id, x, y, 0.0, rsuNode);
      ++rsuCounter;
      ++i;
    }



  // We don't install the server app on the eNB (rsuNode) in LTE, we install it on the remoteHost.
  // Install only a single centralized server to manage TLM data.
  ApplicationContainer AppServer = tlmServerLTEHelper.Install (remoteHostContainer.Get (0));
  AppServer.Start (Seconds (0.0));
  AppServer.Stop (simulationTime - Seconds (0.1));

  /*** 7. Setup interface and application for dynamic nodes ***/
  tlmClientLTEHelper tlmClientLTEHelper;
  tlmClientLTEHelper.SetAttribute ("ServerAddr", Ipv4AddressValue(remoteHostAddr));
  tlmClientLTEHelper.SetAttribute ("Client", (PointerValue) sumoClient); // pass TraciClient object for accessing sumo in application
  tlmClientLTEHelper.SetAttribute ("PrintSummary", BooleanValue(print_summary));
  tlmClientLTEHelper.SetAttribute ("RealTime", BooleanValue(realtime));
  tlmClientLTEHelper.SetAttribute ("CSV", StringValue(csv_name));
  tlmClientLTEHelper.SetAttribute ("SendCAM", BooleanValue (send_cam));
  tlmClientLTEHelper.SetAttribute ("MetricSupervisor", PointerValue (metSup));

  /* callback function for node creation */
  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID,TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
    {
      if (nodeCounter >= obuNodes.GetN())
        NS_FATAL_ERROR("Node Pool empty!: " << nodeCounter << " nodes created.");

      /* Don't create and install the protocol stack of the node at simulation time -> take from "node pool" */
      Ptr<Node> includedNode = obuNodes.Get(nodeCounter);
      ++nodeCounter; //increment counter for next node

      /* Install Application */
      //tlmClientLTEHelper.SetAttribute ("PRRSupervisor", PointerValue (&prrSup));
      ApplicationContainer ClientApp = tlmClientLTEHelper.Install (includedNode);
      ClientApp.Start (Seconds (0.0));
      ClientApp.Stop (simulationTime - Simulator::Now () - Seconds (0.1));

      return includedNode;
    };

  /* callback function for node shutdown */
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode,std::string vehicleID)
    {
      /* Stop all applications */
      Ptr<tlmClientLTE> tlmClientLTE_ = exNode->GetApplication(0)->GetObject<tlmClientLTE>();
      if(tlmClientLTE_)
        tlmClientLTE_->StopApplicationNow ();

       /* Set position outside communication range */
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0));// rand() for visualization purposes

      /* NOTE: further actions could be required for a safe shut down! */
    };

  /* Start traci client with given function pointers */
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  /*** 8. Vehicle Metrics Logging Setup ***/
  std::map<std::string, VehicleMetrics> vehicleMetricsMap;
  EventId metricsEvent;
  const double metricsInterval = 0.1; // seconds — finer interval for accurate CO2 accumulation

  std::function<void ()> updateVehicleMetrics;
  updateVehicleMetrics = [&] () {
    auto activeVehicles = sumoClient->TraCIAPI::vehicle.getIDList ();

    for (const auto &vehId : activeVehicles)
      {
        auto &m = vehicleMetricsMap[vehId]; // creates entry on first access

        double speed = sumoClient->TraCIAPI::vehicle.getSpeed (vehId);
        double co2Rate = sumoClient->TraCIAPI::vehicle.getCO2Emission (vehId); // mg/s
        double dist = sumoClient->TraCIAPI::vehicle.getDistance (vehId); // cumulative odometer [m]

        // Accumulate CO2: rate [mg/s] × interval [s] = [mg]
        m.totalCO2_mg += co2Rate * metricsInterval;
        m.totalTravelTime_s += metricsInterval;

        if (speed < 0.1)
          {
            m.totalStoppedTime_s += metricsInterval;
          }

        if (!m.initialized)
          {
            m.baselineDistance_m = dist;
            m.initialized = true;
          }
        m.totalDistance_m = dist - m.baselineDistance_m;
      }

    metricsEvent = Simulator::Schedule (Seconds (metricsInterval), updateVehicleMetrics);
  };

  if (log_metrics)
    {
      NS_LOG_INFO ("Vehicle metrics logging ENABLED (interval=" << metricsInterval << "s)");
      metricsEvent = Simulator::Schedule (Seconds (1.0), updateVehicleMetrics);
    }

  /*** 8. Start Simulation ***/
  Simulator::Stop (simulationTime);
  Simulator::Run ();
  Simulator::Destroy ();

  /*** 9. Vehicle Metrics Output ***/
  if (log_metrics && !vehicleMetricsMap.empty ())
    {
      Simulator::Cancel (metricsEvent);

      std::cout << "\n===== Vehicle Metrics Summary (" << vehicleMetricsMap.size ()
                << " vehicles) =====" << std::endl;
      std::cout << std::left << std::setw (15) << "VehicleID" << std::setw (14) << "CO2_mg"
                << std::setw (14) << "CO2_g" << std::setw (14) << "TravelTime_s" << std::setw (14)
                << "StoppedTime_s" << std::setw (14) << "Distance_m" << std::setw (14)
                << "AvgSpeed_ms" << std::endl;
      std::cout << std::string (99, '-') << std::endl;

      std::ofstream csvFile;
      if (!metrics_csv_name.empty ())
        {
          csvFile.open (metrics_csv_name + ".csv", std::ofstream::trunc);
          csvFile << "vehicleID,co2_mg,co2_g,travel_time_s,stopped_time_s,distance_m,avg_speed_ms"
                  << std::endl;
        }

      for (const auto &pair : vehicleMetricsMap)
        {
          const std::string &id = pair.first;
          const VehicleMetrics &m = pair.second;
          double avgSpeed =
              (m.totalTravelTime_s > 0.0) ? (m.totalDistance_m / m.totalTravelTime_s) : 0.0;

          std::cout << std::left << std::setw (15) << id << std::setw (14) << std::fixed
                    << std::setprecision (2) << m.totalCO2_mg << std::setw (14)
                    << m.totalCO2_mg / 1000.0 << std::setw (14) << m.totalTravelTime_s
                    << std::setw (14) << m.totalStoppedTime_s << std::setw (14) << m.totalDistance_m
                    << std::setw (14) << avgSpeed << std::endl;

          if (csvFile.is_open ())
            {
              csvFile << id << "," << std::fixed << std::setprecision (2) << m.totalCO2_mg << ","
                      << m.totalCO2_mg / 1000.0 << "," << m.totalTravelTime_s << ","
                      << m.totalStoppedTime_s << "," << m.totalDistance_m << "," << avgSpeed
                      << std::endl;
            }
        }

      if (csvFile.is_open ())
        {
          csvFile.close ();
          std::cout << "\nMetrics written to: " << metrics_csv_name << ".csv" << std::endl;
        }
      std::cout << "===================================" << std::endl;
    }

  if (m_metric_sup)
    {
      if (csv_name_cumulative != "")
        {
          std::ofstream csv_cum_ofstream;
          std::string full_csv_name = csv_name_cumulative + ".csv";

          if (access (full_csv_name.c_str (), F_OK) != -1)
            {
              // The file already exists
              csv_cum_ofstream.open (full_csv_name, std::ofstream::out | std::ofstream::app);
            }
          else
            {
              // The file does not exist yet
              csv_cum_ofstream.open (full_csv_name);
              csv_cum_ofstream << "avg_PRR,avg_latency_ms" << std::endl;
            }

          csv_cum_ofstream << metSup->getAveragePRR_overall () << ","
                           << metSup->getAverageLatency_overall () << std::endl;
        }
      std::cout << "Average PRR: " << metSup->getAveragePRR_overall () << std::endl;
      std::cout << "Average latency (ms): " << metSup->getAverageLatency_overall () << std::endl;
    }

  return 0;
}
