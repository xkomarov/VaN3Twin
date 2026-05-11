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
#ifndef ecoDrivingCLIENT_H
#define ecoDrivingCLIENT_H

#include "ns3/MetricSupervisor.h"
#include "ns3/application.h"
#include "ns3/asn_utils.h"
#include "ns3/tlmService.h"
#include "ns3/caBasicService.h"
#include "ns3/btp.h"
#include "ns3/traci-client.h"
#include "ns3/LDM.h"

namespace ns3 {

class ecoDrivingClient : public Application
{
public:
  /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
  static TypeId GetTypeId (void);

  ecoDrivingClient ();

  virtual ~ecoDrivingClient ();

  void receiveCAM (asn1cpp::Seq<CAM> cam, Address from);
  void receiveSPATEM (asn1cpp::Seq<SPATEM> spatem, Address from);
  void StopApplicationNow ();

protected:
  virtual void DoDispose (void);

private:
  CABasicService m_caService; //!< CA Basic Service object
  TLMService m_tlmService; //!< TLM Basic Service object

  Ptr<btp> m_btp; //! BTP object
  Ptr<GeoNet> m_geoNet; //! GeoNetworking Object

  Ptr<Socket> m_socket; //!< Client socket

  Ptr<LDM> m_LDM; //!< Local Dynamic Map for traffic light data

  virtual void StartApplication (void);
  virtual void StopApplication (void);

  void TriggerCam (void);
  void spatemOut (void);

  /**
     * @brief Mock-populate TL topology into LDM (simulates MAPEM).
     *
     * Iterates all traffic light systems from SUMO, builds lane mappings and
     * stop-line coordinates, and inserts them into the LDM.
     */
  void populateStaticTLData (void);

  /**
     * @brief Periodic ecoDriving speed profile recalculation.
     *
     * Runs every 200 ms while SPATEM data is available. Queries the LDM for
     * nearby traffic lights, computes optimal speed advisory, and applies it
     * via TraCI with speedMode=6 (disabling SUMO's TL braking).
     */
  void updateglosaControl (void);

  /**
     * @brief Helper to apply GLOSA speed and set vehicle color for visualization.
     */
  void applyGlosaAction (double speed, int r, int g, int b);

  std::string m_model; //!< Access technology model ("80211p" or "lte")
  Ptr<TraciClient> m_client; //!< TraCI client
  std::string m_id; //!< vehicle id
  bool m_real_time; //!< To decide wheter to use realtime scheduler
  std::string m_csv_name; //!< CSV log file name
  std::ofstream m_csv_ofstream;
  std::ofstream m_tlm_metrics_csv; //!< CSV for TLM correctness metrics
  bool m_print_summary; //!< To print a small summary when vehicle leaves the simulation
  bool m_already_print; //!< To avoid printing two summary
  Ipv4Address m_server_addr; //!< Remote addr

  EventId m_sendCamEvent; //!< Event to send the CAM
  EventId m_spatemOut;
  EventId m_ecoDrivingUpdateEvent; //!< Periodic ecoDriving speed recalculation event

  /* Counters */
  int m_cam_sent;
  int m_spatem_received;
  bool m_send_cam;

  /* ecoDriving state */
  bool m_ecoDrivingActive = false; //!< Whether ecoDriving is currently overriding vehicle speed
  uint64_t m_passedIntersectionID = 0; //!< ID of intersection already passed (dedup)
  bool m_spatemAlive = false; //!< Whether at least one SPATEM has been received recently

  /* TLM correctness metrics */
  double m_last_spatem_rx_time_ms = -1.0; //!< Sim time (ms) of last SPATEM reception
  int m_steps_in_range = 0;               //!< Steps vehicle was in advisory zone [10,400m]
  int m_fdr_disengagements = 0;           //!< spatemOut() calls while advisory was active

  Ptr<MetricSupervisor> m_metric_supervisor = nullptr;
};

} // namespace ns3

#endif /* ecoDrivingCLIENT_H */
