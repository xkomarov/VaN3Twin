#include "ns3/glosaServer-helper.h"
#include "ns3/glosaClient.h"
#include "ns3/glosaClient-helper.h"
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
#include "ns3/nr-module.h"
using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("v2i-glosa-LTE");

int
main (int argc, char *argv[])
{

  // Admitted data rates for 802.11p
  std::vector<float> rate_admitted_values{3, 4.5, 6, 9, 12, 18, 24, 27};
  std::string datarate_config;

  /*** 0.a App Options ***/
  std::string map = "tlm_map_1"; // Default map
  std::string sumo_folder = "src/automotive/examples/" + map + "/";
  std::string mob_trace = "cars.rou.xml";
  std::string rsu_file = "stations.xml";
  std::string sumo_config = sumo_folder + "map.sumo.cfg";

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
  cmd.AddValue ("map", "Name of the map folder", map);
  cmd.AddValue ("realtime", "Use the realtime scheduler or not", realtime);
  cmd.AddValue ("sumo-gui", "Use SUMO gui or not", sumo_gui);
  cmd.AddValue ("server-aggregate-output", "Print an aggregate output for server", aggregate_out);
  cmd.AddValue ("sumo-updates", "SUMO granularity", sumo_updates);
  cmd.AddValue ("sumo-folder", "Position of sumo config files", sumo_folder);
  cmd.AddValue ("mob-trace", "Name of the mobility trace file", mob_trace);
  cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", sumo_config);
  cmd.AddValue ("csv-log", "Name of the CSV log file", csv_name);
  cmd.AddValue ("summary", "Print a summary for each vehicle at the end of the simulation",
                print_summary);
  cmd.AddValue ("vehicle-visualizer", "Activate the web-based vehicle visualizer for ms-van3t",
                vehicle_vis);
  cmd.AddValue (
      "send-cam",
      "Turn on or off the transmission of CAMs, thus turning on or off the whole V2X application",
      send_cam);
  cmd.AddValue ("send-spatem", "Turn on or off the transmission of SPATEM messages from the server",
                send_spatem);
  cmd.AddValue ("csv-log-cumulative",
                "Name of the CSV log file for the cumulative (average) PRR and latency data",
                csv_name_cumulative);
  cmd.AddValue ("netstate-dump-file",
                "Name of the SUMO netstate-dump file containing the vehicle-related information "
                "throughout the whole simulation",
                sumo_netstate_file_name);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("met-sup", "Use the Metric supervisor or not", m_metric_sup);

  /* Cmd Line option for Lena */
  cmd.AddValue ("interPacketInterval", "Inter packet interval [ms]", interPacketInterval);
  cmd.AddValue ("useCa", "Whether to use carrier aggregation", useCa);

  cmd.AddValue ("sim-time", "Total duration of the simulation [s]", simTime);

  cmd.Parse (argc, argv);

  if (map != "tlm_map_1")
    {
      sumo_folder = "src/automotive/examples/" + map + "/";
      sumo_config = sumo_folder + "map.sumo.cfg";
    }

  /* Carrier aggregation for LTE */
  if (useCa)
    {
      Config::SetDefault ("ns3::LteHelper::UseCa", BooleanValue (useCa));
      Config::SetDefault ("ns3::LteHelper::NumberOfComponentCarriers", UintegerValue (2));
      Config::SetDefault ("ns3::LteHelper::EnbComponentCarrierManager",
                          StringValue ("ns3::RrComponentCarrierManager"));
    }

  // Fixing the 23-UE limit per eNB in LENA
  Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue (320));

  if (verbose)
    {
      LogComponentEnable ("v2i-glosa-LTE", LOG_LEVEL_INFO);
      LogComponentEnable ("CABasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("TLMService", LOG_LEVEL_INFO);
      LogComponentEnable ("glosaServer", LOG_LEVEL_INFO);
    }

  /* Use the realtime scheduler of ns3 */
  if (realtime)
    GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  /*** 0.b Read from the mob_trace the number of vehicles that will be created.
   *       The number of vehicles is directly parsed from the rou.xml file, looking at all
   *       the valid XML elements of type <vehicle>
  ***/
  NS_LOG_INFO ("Reading the .rou file...");
  std::string path = sumo_folder + mob_trace;

  /* Load the .rou.xml document */
  xmlInitParser ();
  rou_xml_file = xmlParseFile (path.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR ("Error: unable to parse the specified XML file: " << path);
    }
  numberOfNodes = XML_rou_count_vehicles (rou_xml_file);

  std::string rsu_path = sumo_folder + rsu_file;
  std::ifstream rsu_file_stream (rsu_path.c_str ());
  std::vector<std::tuple<std::string, float, float>> rsuData =
      XML_poli_count_stations (rsu_file_stream);
  numberOfRSUs = rsuData.size ();

  xmlFreeDoc (rou_xml_file);
  xmlCleanupParser ();

  if (numberOfNodes == -1)
    {
      NS_FATAL_ERROR (
          "Fatal error: cannot gather the number of vehicles from the specified XML file: "
          << path << ". Please check if it is a correct SUMO file.");
    }
  NS_LOG_INFO ("The .rou file has been read: " << numberOfNodes
                                               << " vehicles will be present in the simulation.");

  /* Set the simulation time (in seconds) */
  NS_LOG_INFO ("Simulation will last " << simTime << " seconds");
  ns3::Time simulationTime (ns3::Seconds (simTime));

  /*** 1. Create containers for UEs and eNodeBs ***/
  NodeContainer obuNodes; // UEs
  obuNodes.Create (numberOfNodes);

  NodeContainer rsuNodes; // eNodeBs
  rsuNodes.Create (numberOfRSUs);

  /*** 2. Setup Mobility ***/
  MobilityHelper mobility;
  mobility.Install (obuNodes);
  mobility.Install (rsuNodes);

  /*** 3. Create LTE objects & EPC ***/
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults ();

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

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
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
      Ptr<Ipv4StaticRouting> ueStaticRouting =
          ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
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
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue ("")); // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (sumo_updates)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", (BooleanValue) sumo_gui);
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));

  std::string sumo_additional_options = "--verbose true";

  if (sumo_netstate_file_name != "")
    {
      sumo_additional_options += " --netstate-dump " + sumo_netstate_file_name;
    }

  sumo_additional_options += " --collision.action warn --collision.check-junctions "
                             "--error-log=sumo-errors-or-collisions.xml";

  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (5.0)));
  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue (sumo_additional_options));

  /* Create and setup the web-based vehicle visualizer of ms-van3t */
  vehicleVisualizer vehicleVisObj;
  Ptr<vehicleVisualizer> vehicleVis = &vehicleVisObj;
  if (vehicle_vis)
    {
      vehicleVis->startServer ();
      vehicleVis->connectToServer ();
      sumoClient->SetAttribute ("VehicleVisualizer", PointerValue (vehicleVis));
    }

  Ptr<MetricSupervisor> metSup = NULL;
  MetricSupervisor metSupObj (m_baseline_prr);
  if (m_metric_sup)
    {
      metSup = &metSupObj;
      metSup->setTraCIClient (sumoClient);
    }

  /*** 6. Create and Setup application for the server ***/
  glosaServerHelper glosaServHelper;
  glosaServHelper.SetAttribute ("Model", StringValue ("lte"));
  glosaServHelper.SetAttribute ("Client", (PointerValue) sumoClient);
  glosaServHelper.SetAttribute ("RealTime", BooleanValue (realtime));
  glosaServHelper.SetAttribute ("AggregateOutput", BooleanValue (aggregate_out));
  glosaServHelper.SetAttribute ("CSV", StringValue (csv_name));
  glosaServHelper.SetAttribute ("MetricSupervisor", PointerValue (metSup));
  glosaServHelper.SetAttribute ("SendSPATEM", BooleanValue (send_spatem));

  int i = 0;
  for (auto rsu : rsuData)
    {
      std::string id = std::get<0> (rsu);
      float x = std::get<1> (rsu);
      float y = std::get<2> (rsu);
      Ptr<Node> rsuNode = rsuNodes.Get (i);
      sumoClient->AddStation (id, x, y, 0.0, rsuNode);
      ++rsuCounter;
      ++i;
    }

  // We don't install the server app on the eNB (rsuNode) in LTE, we install it on the remoteHost.
  // Install only a single centralized server to manage glosa data.
  ApplicationContainer AppServer = glosaServHelper.Install (remoteHostContainer.Get (0));
  AppServer.Start (Seconds (0.0));
  AppServer.Stop (simulationTime - Seconds (0.1));

  /*** 7. Setup interface and application for dynamic nodes ***/
  glosaClientHelper glosaCliHelper;
  glosaCliHelper.SetAttribute ("Model", StringValue ("lte"));
  glosaCliHelper.SetAttribute ("ServerAddr", Ipv4AddressValue (remoteHostAddr));
  glosaCliHelper.SetAttribute (
      "Client",
      (PointerValue) sumoClient); // pass TraciClient object for accessing sumo in application
  glosaCliHelper.SetAttribute ("PrintSummary", BooleanValue (print_summary));
  glosaCliHelper.SetAttribute ("RealTime", BooleanValue (realtime));
  glosaCliHelper.SetAttribute ("CSV", StringValue (csv_name));
  glosaCliHelper.SetAttribute ("SendCAM", BooleanValue (send_cam));
  glosaCliHelper.SetAttribute ("MetricSupervisor", PointerValue (metSup));

  /* callback function for node creation */
  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID,
                                      TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node> {
    if (nodeCounter >= obuNodes.GetN ())
      NS_FATAL_ERROR ("Node Pool empty!: " << nodeCounter << " nodes created.");

    /* Don't create and install the protocol stack of the node at simulation time -> take from "node pool" */
    Ptr<Node> includedNode = obuNodes.Get (nodeCounter);
    ++nodeCounter; //increment counter for next node

    /* Install Application */
    //glosaCliHelper.SetAttribute ("PRRSupervisor", PointerValue (&prrSup));
    ApplicationContainer ClientApp = glosaCliHelper.Install (includedNode);
    ClientApp.Start (Seconds (0.0));
    ClientApp.Stop (simulationTime - Simulator::Now () - Seconds (0.1));

    return includedNode;
  };

  /* callback function for node shutdown */
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode, std::string vehicleID) {
    /* Stop all applications */
    Ptr<glosaClient> glosaClient_ = exNode->GetApplication (0)->GetObject<glosaClient> ();
    if (glosaClient_)
      glosaClient_->StopApplicationNow ();

    /* Set position outside communication range */
    Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel> ();
    mob->SetPosition (Vector (-1000.0 + (rand () % 25), 320.0 + (rand () % 25),
                              250.0)); // rand() for visualization purposes

    /* NOTE: further actions could be required for a safe shut down! */
  };

  /* Start traci client with given function pointers */
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  /*** 8. Start Simulation ***/
  Simulator::Stop (simulationTime);
  Simulator::Run ();
  Simulator::Destroy ();

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
