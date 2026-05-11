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
#ifndef emergencyVehicleWarningServer80211P_H
#define emergencyVehicleWarningServer80211P_H



#include "ns3/application.h"
#include "ns3/asn_utils.h"

#include <unordered_map>

#include "ns3/denBasicService.h"
#include "ns3/caBasicService.h"
#include "ns3/vdpTraci.h"
#include "ns3/socket.h"
#include "ns3/iviService.h"
#include "ns3/traci-client.h"
namespace ns3 {

class emergencyVehicleWarningServer80211p : public Application
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId (void);

    emergencyVehicleWarningServer80211p ();

    virtual ~emergencyVehicleWarningServer80211p ();

    /**
     * \brief Callback to handle a CAM reception.
     *
     * This function is called everytime a packet is received by the CABasicService.
     *
     * \param the ASN.1 CAM structure containing the info of the packet that was received.
     */
    void receiveCAM (asn1cpp::Seq<CAM>, Address address);

    void receiveIVIM (iviData ivim, Address from);

    void StopApplicationNow ();

    bool m_lon_lat; //!< Use LonLat instead of XY

  protected:
    virtual void DoDispose (void);

  private:

    CABasicService m_caService; //!< CA Basic Service object
    IVIBasicService m_iviService; //!< IVI service object

    Ptr<btp> m_btp; //! BTP object
    Ptr<GeoNet> m_geoNet; //! GeoNetworking Object

    Ptr<Socket> m_socket; //!< Server socket

    virtual void StartApplication (void);
    virtual void StopApplication (void);

    void TriggerIvim();
    void UpdateIvim();
    void RepeatIvim();
    void TerminateIvim();

    /**
     * @brief This function compute the milliseconds elapsed from 2004-01-01
    */
    long compute_timestampIts ();
    /**
     * @brief Used to print a report on number of msg received each second
    */
    void aggregateOutput(void);

    Ptr<TraciClient> m_client; //!< TraCI client
    std::string m_id; //!< vehicle id
    std::string m_type; //!< vehicle type
    bool m_aggregate_output; //!< To decide wheter to print the report each second or not
    bool m_real_time; //!< To decide wheter to use realtime scheduler
    std::string m_csv_name; //!< CSV log file name
    std::ofstream m_csv_ofstream_cam;

    iviData m_iviData; //! IVI data sent by the server

    bool m_isTransmittingDENM;

    /* Counters */
    u_int m_cam_received;
    u_int m_denm_sent;
    int m_ivim_sent;

    Ptr<MetricSupervisor> m_metric_supervisor = nullptr;

    bool m_send_cam;

    EventId m_aggegateOutputEvent; //!< Event to create aggregate output
    EventId m_update_denm_ev; //!< Event to update the DENM
    EventId m_terminate_denm_ev; //!< Event to terminate the DENM

    uint64_t m_stationId_baseline = 1000000;
  };


} // namespace ns3



#endif // emergencyVehicleWarningServer80211p_H
