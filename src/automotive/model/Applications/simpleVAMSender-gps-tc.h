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
#ifndef SIMPLEVAMSENDER_H
#define SIMPLEVAMSENDER_H

#include "ns3/OpenCDAClient.h"
#include "ns3/traci-client.h"
#include "ns3/gps-tc.h"

#include "ns3/application.h"
#include "ns3/asn_utils.h"

#include "ns3/VRUBasicService.h"


namespace ns3 {

class simpleVAMSender : public Application
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  simpleVAMSender ();

  virtual ~simpleVAMSender ();

  // void receiveCAM(CAM_t *cam, Address address);
  void receiveVAM(asn1cpp::Seq<VAM> vam, Address address);

  void StopApplicationNow ();

protected:
  virtual void DoDispose (void);

private:

  VRUBasicService m_vruService; //!< VRU Basic Service object
  Ptr<btp> m_btp; //! BTP object
  Ptr<GeoNet> m_geoNet; //! GeoNetworking Object

  Ptr<Socket> m_socket;

  virtual void StartApplication (void);
  virtual void StopApplication (void);

  Ptr<GPSTraceClient> m_gps_tc_client; //!< GPS trace client

  std::string m_id; //!< vehicle id
  bool m_real_time; //!< To decide wheter to use realtime scheduler

  EventId m_sendVamEvent; //!< Event to send the CAM

  /* Counters */
  int m_vam_sent;
};

} // namespace ns3

#endif /* SIMPLEVAMSENDER_H */

