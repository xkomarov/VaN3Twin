#ifndef ECOGLOSACLIENT_H
#define ECOGLOSACLIENT_H

#include "ns3/MetricSupervisor.h"
#include "ns3/application.h"
#include "ns3/asn_utils.h"
#include "ns3/tlmService.h"
#include "ns3/caBasicService.h"
#include "ns3/btp.h"
#include "ns3/traci-client.h"
#include "ns3/LDM.h"

namespace ns3 {

class ecoGlosaClient : public Application
{
public:
  /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
  static TypeId GetTypeId (void);

  ecoGlosaClient ();

  virtual ~ecoGlosaClient ();

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
  /**
     * @brief This function compute the milliseconds elapsed from 2004-01-01
    */
  long compute_timestampIts ();
  //void denmTimeout(void);
  void spatemTimeout (void);

  /**
     * @brief Mock-populate TL topology into LDM (simulates MAPEM).
     *
     * Iterates all traffic light systems from SUMO, builds lane mappings and
     * stop-line coordinates, and inserts them into the LDM.
     */
  void populateStaticTLData (void);

  /**
     * @brief Periodic ecoGlosa speed profile recalculation.
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
  bool m_print_summary; //!< To print a small summary when vehicle leaves the simulation
  bool m_already_print; //!< To avoid printing two summary
  Ipv4Address m_server_addr; //!< Remote addr

  EventId m_sendCamEvent; //!< Event to send the CAM
  EventId m_spatemTimeout;
  EventId m_ecoGlosaUpdateEvent; //!< Periodic ecoGlosa speed recalculation event

  /* Counters */
  int m_cam_sent;
  int m_spatem_received;
  bool m_send_cam;

  /* ecoGlosa state */
  bool m_ecoGlosaActive = false; //!< Whether ecoGlosa is currently overriding vehicle speed
  uint64_t m_passedIntersectionID = 0; //!< ID of intersection already passed (dedup)
  bool m_spatemAlive = false; //!< Whether at least one SPATEM has been received recently

  Ptr<MetricSupervisor> m_metric_supervisor = nullptr;
};

} // namespace ns3

#endif /* ecoGlosaCLIENT_H */
