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
#include "ns3/ecoDrivingServer-helper.h"
#include "ns3/ecoDrivingClient.h"
#include "ns3/ecoDrivingClient-helper.h"
#include "ns3/traci-module.h"
#include "ns3/internet-module.h"
#include "ns3/wave-module.h"
#include "ns3/mobility-module.h"
#include "ns3/sumo_xml_parser.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/vehicle-visualizer-module.h"
#include "ns3/MetricSupervisor.h"
#include <unistd.h>
#include <fstream>
#include <iomanip>
#include <functional>


struct VehicleMetrics
{
  double totalCO2_mg = 0.0; 
  double totalTravelTime_s = 0.0; 
  double totalStoppedTime_s = 0.0; 
  double totalDistance_m = 0.0; 
  double baselineDistance_m = 0.0; 
  bool initialized = false; 
};

using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("v2i-ecoDriving-80211p");

int
main (int argc, char *argv[])
{

  
  std::vector<float> rate_admitted_values{3, 4.5, 6, 9, 12, 18, 24, 27};
  std::string datarate_config;

  
  std::string map = "tlm_map_1"; 
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
  int txPower = 23;
  float datarate = 12;
  bool log_metrics = false;
  std::string metrics_csv_name;
  bool vehicle_vis = false;

  
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
  cmd.AddValue ("send-spatem",
                "Turn on or off the transmission of SPATEM messages from the RSU server",
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

  
  cmd.AddValue ("tx-power", "OBUs transmission power [dBm]", txPower);
  cmd.AddValue ("datarate", "802.11p channel data rate [Mbit/s]", datarate);

  cmd.AddValue ("sim-time", "Total duration of the simulation [s]", simTime);

  
  cmd.AddValue ("log-metrics",
                "Enable per-vehicle metric logging (CO2, travel time, stopped time, avg speed)",
                log_metrics);
  cmd.AddValue (
      "metrics-csv",
      "CSV file name for vehicle metrics output (omit extension; omit entirely for console-only)",
      metrics_csv_name);

  cmd.Parse (argc, argv);

  if (map != "tlm_map_1")
    {
      sumo_folder = "src/automotive/examples/" + map + "_rsu/";
      sumo_config = sumo_folder + "map.sumo.cfg";
    }

  if (std::find (rate_admitted_values.begin (), rate_admitted_values.end (), datarate) ==
      rate_admitted_values.end ())
    {
      NS_FATAL_ERROR ("Fatal error: invalid 802.11p data rate"
                      << datarate
                      << "Mbit/s. Valid rates are: 3, 4.5, 6, 9, 12, 18, 24, 27 Mbit/s.");
    }
  else
    {
      if (datarate == 4.5)
        {
          datarate_config = "OfdmRate4_5MbpsBW10MHz";
        }
      else
        {
          datarate_config = "OfdmRate" + std::to_string ((int) datarate) + "MbpsBW10MHz";
        }
    }

  if (verbose)
    {
      LogComponentEnable ("v2i-ecoDriving-80211p", LOG_LEVEL_INFO);
      LogComponentEnable ("CABasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("TLMService", LOG_LEVEL_INFO);
      LogComponentEnable ("ecoDrivingServer", LOG_LEVEL_INFO);
    }

  
  if (realtime)
    GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  
  NS_LOG_INFO ("Reading the .rou file...");
  std::string path = sumo_folder + mob_trace;

  
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

  
  NS_LOG_INFO ("Simulation will last " << simTime << " seconds");
  ns3::Time simulationTime (ns3::Seconds (simTime));

  
  NodeContainer obuNodes;
  obuNodes.Create (numberOfNodes);

  
  NodeContainer rsuNodes;
  rsuNodes.Create (numberOfRSUs);

  
  YansWifiPhyHelper wifiPhy;
  wifiPhy.Set ("TxPowerStart", DoubleValue (txPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower));
  std::cout << "Setting up the 802.11p channel @ " << datarate << " Mbit/s, 10 MHz, and tx power "
            << (int) txPower << " dBm." << std::endl;

  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);

  
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  wifi80211p.SetRemoteStationManager (
      "ns3::ConstantRateWifiManager", "DataMode", StringValue (datarate_config), "ControlMode",
      StringValue (datarate_config), "NonUnicastMode", StringValue (datarate_config));
  NetDeviceContainer netDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, obuNodes);
  NetDeviceContainer netRSUs = wifi80211p.Install (wifiPhy, wifi80211pMac, rsuNodes);
  

  wifiPhy.EnablePcap ("v2i-ecoDriving-80211p", netDevices);
  wifiPhy.EnablePcap ("v2i-ecoDriving-80211p-RSU", netRSUs);
  
  PacketSocketHelper packetSocket;
  packetSocket.Install (obuNodes);
  packetSocket.Install (rsuNodes);

  
  MobilityHelper mobility;
  mobility.Install (obuNodes);
  MobilityHelper mobilityRSU;
  mobilityRSU.Install (rsuNodes);

  
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue ("")); 
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

  
  ecoDrivingServerHelper ecoDrivingServerHelper_;
  ecoDrivingServerHelper_.SetAttribute ("Model", StringValue ("80211p"));
  ecoDrivingServerHelper_.SetAttribute ("Client", (PointerValue) sumoClient);
  ecoDrivingServerHelper_.SetAttribute ("RealTime", BooleanValue (realtime));
  ecoDrivingServerHelper_.SetAttribute ("AggregateOutput", BooleanValue (aggregate_out));
  ecoDrivingServerHelper_.SetAttribute ("CSV", StringValue (csv_name));
  ecoDrivingServerHelper_.SetAttribute ("MetricSupervisor", PointerValue (metSup));
  ecoDrivingServerHelper_.SetAttribute ("SendSPATEM", BooleanValue (send_spatem));

  int i = 0;
  for (auto rsu : rsuData)
    {
      std::string id = std::get<0> (rsu);
      float x = std::get<1> (rsu);
      float y = std::get<2> (rsu);
      Ptr<Node> rsuNode = rsuNodes.Get (i);
      sumoClient->AddStation (id, x, y, 0.0, rsuNode);
      ApplicationContainer AppServer = ecoDrivingServerHelper_.Install (rsuNode);
      AppServer.Start (Seconds (0.0));
      AppServer.Stop (simulationTime - Seconds (0.1));
      ++rsuCounter;
      ++i;
    }

  
  ecoDrivingClientHelper ecoDrivingClientHelper_;
  Ipv4Address remoteHostAddr;

  ecoDrivingClientHelper_.SetAttribute ("Model", StringValue ("80211p"));
  ecoDrivingClientHelper_.SetAttribute ("ServerAddr", Ipv4AddressValue (remoteHostAddr));
  ecoDrivingClientHelper_.SetAttribute (
      "Client",
      (PointerValue) sumoClient); 
  ecoDrivingClientHelper_.SetAttribute ("PrintSummary", BooleanValue (print_summary));
  ecoDrivingClientHelper_.SetAttribute ("RealTime", BooleanValue (realtime));
  ecoDrivingClientHelper_.SetAttribute ("CSV", StringValue (csv_name));
  ecoDrivingClientHelper_.SetAttribute ("SendCAM", BooleanValue (send_cam));
  ecoDrivingClientHelper_.SetAttribute ("MetricSupervisor", PointerValue (metSup));

  
  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID,
                                      TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node> {
    if (nodeCounter >= obuNodes.GetN ())
      NS_FATAL_ERROR ("Node Pool empty!: " << nodeCounter << " nodes created.");

    
    Ptr<Node> includedNode = obuNodes.Get (nodeCounter);
    ++nodeCounter; 

    
    
    ApplicationContainer ClientApp = ecoDrivingClientHelper_.Install (includedNode);
    ClientApp.Start (Seconds (0.0));
    ClientApp.Stop (simulationTime - Simulator::Now () - Seconds (0.1));

    return includedNode;
  };

  
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode, std::string vehicleID) {
    
    Ptr<ecoDrivingClient> ecoDrivingClient_ = exNode->GetApplication (0)->GetObject<ecoDrivingClient> ();
    if (ecoDrivingClient_)
      ecoDrivingClient_->StopApplicationNow ();

    
    Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel> ();
    mob->SetPosition (Vector (-1000.0 + (rand () % 25), 320.0 + (rand () % 25),
                              250.0)); 

    
  };

  
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  
  std::map<std::string, VehicleMetrics> vehicleMetricsMap;
  EventId metricsEvent;
  const double metricsInterval = 0.1; 

  std::function<void ()> updateVehicleMetrics;
  updateVehicleMetrics = [&] () {
    auto activeVehicles = sumoClient->TraCIAPI::vehicle.getIDList ();

    for (const auto &vehId : activeVehicles)
      {
        auto &m = vehicleMetricsMap[vehId]; 

        double speed = sumoClient->TraCIAPI::vehicle.getSpeed (vehId);
        double co2Rate = sumoClient->TraCIAPI::vehicle.getCO2Emission (vehId); 
        double dist = sumoClient->TraCIAPI::vehicle.getDistance (vehId); 

        
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

  
  Simulator::Stop (simulationTime);
  Simulator::Run ();
  
  if (log_metrics && !vehicleMetricsMap.empty ())
    {
      Simulator::Cancel (metricsEvent);

      std::cout << "\n===== Vehicle Metrics Summary (" << vehicleMetricsMap.size ()
                << " vehicles) =====' " << std::endl;
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

  Simulator::Destroy ();

  if (m_metric_sup)
    {
      if (csv_name_cumulative != "")
        {
          std::ofstream csv_cum_ofstream;
          std::string full_csv_name = csv_name_cumulative + ".csv";

          if (access (full_csv_name.c_str (), F_OK) != -1)
            {
              
              csv_cum_ofstream.open (full_csv_name, std::ofstream::out | std::ofstream::app);
            }
          else
            {
              
              csv_cum_ofstream.open (full_csv_name);
              csv_cum_ofstream << "current_txpower_dBm,avg_PRR,avg_latency_ms" << std::endl;
            }

          csv_cum_ofstream << txPower << "," << metSup->getAveragePRR_overall () << ","
                           << metSup->getAverageLatency_overall () << std::endl;
        }
      std::cout << "Average PRR: " << metSup->getAveragePRR_overall () << std::endl;
      std::cout << "Average latency (ms): " << metSup->getAverageLatency_overall () << std::endl;
    }

  return 0;
}
