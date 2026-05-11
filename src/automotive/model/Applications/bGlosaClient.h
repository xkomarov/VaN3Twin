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
#ifndef bGlosaCLIENT_H
#define bGlosaCLIENT_H

#include "ns3/MetricSupervisor.h"
#include "ns3/application.h"
#include "ns3/asn_utils.h"
#include "ns3/tlmService.h"
#include "ns3/caBasicService.h"
#include "ns3/btp.h"
#include "ns3/traci-client.h"
#include "ns3/LDM.h"

namespace ns3 {

class bGlosaClient : public Application
{
public:
  
  static TypeId GetTypeId (void);

  bGlosaClient ();

  virtual ~bGlosaClient ();

  void receiveCAM (asn1cpp::Seq<CAM> cam, Address from);
  void receiveSPATEM (asn1cpp::Seq<SPATEM> spatem, Address from);
  void StopApplicationNow ();

protected:
  virtual void DoDispose (void);

private:
  CABasicService m_caService; 
  TLMService m_tlmService; 

  Ptr<btp> m_btp; 
  Ptr<GeoNet> m_geoNet; 

  Ptr<Socket> m_socket; 

  Ptr<LDM> m_LDM; 

  virtual void StartApplication (void);
  virtual void StopApplication (void);

  void TriggerCam (void);

  void spatemOut (void);

  
  void populateStaticTLData (void);

  
  void updatebGlosaControl (void);

  
  void applyGlosaAction (double speed, int r, int g, int b);

  std::string m_model; 
  Ptr<TraciClient> m_client; 
  std::string m_id; 
  bool m_real_time; 
  std::string m_csv_name; 
  std::ofstream m_csv_ofstream;
  std::ofstream m_tlm_metrics_csv; 
  bool m_print_summary; 
  bool m_already_print; 
  Ipv4Address m_server_addr; 

  EventId m_sendCamEvent; 
  EventId m_spatemOut;
  EventId m_bGlosaUpdateEvent; 

  
  int m_cam_sent;
  int m_spatem_received;
  bool m_send_cam;

  
  bool m_bGlosaActive = false; 
  uint64_t m_passedIntersectionID = 0; 
  bool m_spatemAlive = false; 

  
  double m_last_spatem_rx_time_ms = -1.0; 
  int m_steps_in_range = 0;               
  int m_fdr_disengagements = 0;           

  Ptr<MetricSupervisor> m_metric_supervisor = nullptr;
};

} 

#endif 
