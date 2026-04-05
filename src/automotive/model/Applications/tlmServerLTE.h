#ifndef TLMSERVERLTE_H
#define TLMSERVERLTE_H


#include "ns3/application.h"
#include "ns3/asn_utils.h"
#include "ns3/tlmBasicService.h"
#include "ns3/denBasicService.h"
#include "ns3/caBasicService.h"
#include "ns3/btp.h"
#include "ns3/btpHeader.h"
#include "ns3/traci-client.h"

namespace ns3 {

class tlmServerLTE : public Application
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId (void);

    tlmServerLTE ();

    virtual ~tlmServerLTE ();

    /**
     * \brief Callback to handle a CAM reception.
     *
     * This function is called everytime a packet is received by the CABasicService.
     *
     * \param the ASN.1 CAM structure containing the info of the packet that was received.
     */
    // void receiveCAM (CAM_t *cam, Address address);
    void receiveCAM (asn1cpp::Seq<CAM> cam, Address from);

    //void receiveDENM(denData denm, Address from);

    void StopApplicationNow ();

    bool m_lon_lat; //!< Use LonLat instead of XY

  protected:
    virtual void DoDispose (void);

  private:

    //DENBasicService m_denService; //!< DEN Basic Service object
    CABasicService m_caService; //!< CA Basic Service object
    TLMBasicService m_tlmBasicService; //!< TLM Basic Service object

    Ptr<btp> m_btp; //! BTP object
    Ptr<GeoNet> m_geoNet; //! GeoNetworking Object

    Ptr<Socket> m_socket; //!< Server socket

    virtual void StartApplication (void);
    virtual void StopApplication (void);

    /**
     * @brief This function compute the milliseconds elapsed from 2004-01-01
    */
    long compute_timestampIts ();
    /**
     * @brief Used to print a report on number of msg received each second
    */
    void aggregateOutput(void);

    Ptr<TraciClient> m_client; //!< TraCI client
    bool m_aggregate_output; //!< To decide wheter to print the report each second or not
    bool m_real_time; //!< To decide wheter to use realtime scheduler
    std::string m_csv_name; //!< CSV log file name
    std::ofstream m_csv_ofstream_cam;

    bool m_send_cam;

    std::string m_id;

    /* Counters */
    u_int m_cam_received;
    u_int m_spatem_sent;

    std::set<Address> m_active_vehicles;

    EventId m_aggegateOutputEvent; //!< Event to create aggregate output

    Ptr<MetricSupervisor> m_metric_supervisor = nullptr;

    uint64_t m_stationId_baseline = 1000000;
  };

} // namespace ns3

#endif /* TLMSERVERLTE_H */

