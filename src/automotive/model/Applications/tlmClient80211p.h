#ifndef TLMCLIENT80211P_H
#define TLMCLIENT80211P_H

#include "ns3/MetricSupervisor.h"
#include "ns3/application.h"
#include "ns3/asn_utils.h"
#include "ns3/tlmBasicService.h"
#include "ns3/caBasicService.h"
#include "ns3/btp.h"
#include "ns3/traci-client.h"
#include "ns3/LDM.h"


namespace ns3 {

class tlmClient80211p : public Application
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId (void);

    tlmClient80211p ();

    virtual ~tlmClient80211p ();

    void receiveCAM (asn1cpp::Seq<CAM> cam, Address from);
    void receiveSPATEM (asn1cpp::Seq<SPATEM> spatem, Address from);
    void StopApplicationNow ();

  protected:
    virtual void DoDispose (void);

  private:

    CABasicService m_caService; //!< CA Basic Service object
    TLMBasicService m_tlmBasicService; //!< TLM Basic Service object
    Ptr<LDM> m_LDM; //!< Local Dynamic Map for traffic light data

    Ptr<btp> m_btp; //! BTP object
    Ptr<GeoNet> m_geoNet; //! GeoNetworking Object

    Ptr<Socket> m_socket; //!< Client socket

    virtual void StartApplication (void);
    virtual void StopApplication (void);

    void TriggerCam(void);
    /**
     * @brief This function compute the milliseconds elapsed from 2004-01-01
    */
    long compute_timestampIts ();
    void spatemTimeout(void);
    void populateStaticTLData(void); //!< Mock-populate TL topology into LDM (simulates MAPEM)

    /**
     * @brief Periodic RLVW (Red Light Violation Warning) check.
     *
     * Runs every 200 ms while SPATEM data is available. Queries the LDM for
     * nearby traffic lights, computes stopping distance vs. distance to stop
     * line, and applies emergency braking if the vehicle cannot stop in time.
     */
    void updateRlvwControl(void);

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
    EventId m_rlvwUpdateEvent; //!< Periodic RLVW check event

    /* Counters */
    int m_cam_sent;
    int m_spatem_received;
    int m_rlvw_warnings = 0; //!< Number of RLVW warnings issued
    bool m_send_cam;

    Ptr<MetricSupervisor> m_metric_supervisor = nullptr;

    /* RLVW state */
    uint64_t m_passedIntersectionID = 0; //!< ID of intersection already passed (dedup)
    bool m_rlvwActive = false;           //!< Whether RLVW is currently braking the vehicle
    bool m_spatemAlive = false;          //!< Whether at least one SPATEM has been received recently
  };

} // namespace ns3

#endif /* TLMCLIENT80211P_H */

