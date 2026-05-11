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
#ifndef bGlosaSERVER_H
#define bGlosaSERVER_H

#include "ns3/application.h"
#include "ns3/tlmService.h"
#include "ns3/caBasicService.h"
#include "ns3/btp.h"
#include "ns3/traci-client.h"

namespace ns3 {

class bGlosaServer : public Application
{
public:
  
  static TypeId GetTypeId (void);

  bGlosaServer ();

  virtual ~bGlosaServer ();

  
  void receiveCAM (asn1cpp::Seq<CAM> cam, Address from);

  void StopApplicationNow ();

  bool m_lon_lat; 

protected:
  virtual void DoDispose (void);

private:
  CABasicService m_caService; 
  TLMService m_tlmService; 

  Ptr<btp> m_btp; 
  Ptr<GeoNet> m_geoNet; 

  Ptr<Socket> m_socket; 

  virtual void StartApplication (void);
  virtual void StopApplication (void);

  
  void aggregateOutput (void);

  std::string m_model; 
  Ptr<TraciClient> m_client; 
  bool m_aggregate_output; 
  bool m_real_time; 
  std::string m_csv_name; 
  std::ofstream m_csv_ofstream_cam;

  bool m_send_cam;
  bool m_send_spatem;

  std::string m_id;
  std::string m_sumo_id;

  
  u_int m_cam_received;
  u_int m_spatem_sent;

  EventId m_aggegateOutputEvent; 

  Ptr<MetricSupervisor> m_metric_supervisor = nullptr;

  uint64_t m_stationId_baseline = 1000000;
};

} 

#endif 
