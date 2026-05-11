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
#ifndef TLMService_H
#define TLMService_H

#include "ns3/socket.h"
#include "ns3/core-module.h"

#include "ns3/idp.h"
#include "ns3/asn_utils.h"
#include "ns3/btp.h"
#include "ns3/Seq.hpp"
#include "signalInfoUtils.h"
#include "ns3/btpdatarequest.h"
#include <functional>
#include "ns3/Encoding.hpp"














extern "C" {
#include "ns3/SPATEM.h"
#include "ns3/SPAT.h"
}



namespace ns3 {
typedef enum {
  SPATEM_NO_ERROR = 0,
  SPATEM_WRONG_INTERVAL = 1,
  SPATEM_ALLOC_ERROR = 2,
  SPATEM_TX_SOCKET_NOT_SET = 3,
  SPATEM_ASN1_UPER_ENC_ERROR = 4,
  SPATEM_CANNOT_SEND = 5
} TLMService_error_t;



class TLMService : public Object, public SignalInfoUtils
{
public:
  
  TLMService ();
  
  TLMService (unsigned long fixed_stationid, long fixed_stationtype, Ptr<Socket> socket_tx);

  int receivedSPATEM;
  long timeStamp;

  
  void
  addTLMRxCallback (std::function<void (asn1cpp::Seq<SPATEM>, Address)> rx_callback)
  {
    m_TLMReceiveCallback = rx_callback;
  }
  void
  addTLMRxCallbackExtended (
      std::function<void (asn1cpp::Seq<SPATEM>, Address, StationId_t, StationType_t, SignalInfo)>
          rx_callback)
  {
    m_TLMReceiveCallbackExtended = rx_callback;
  }

  
  void
  setIDP (IDP *idp)
  {
    m_idp = idp;
  }

  






  
  void receiveSPATEM (BTPDataIndication_t dataIndication, Address address);

  
  void setStationProperties (unsigned long fixed_stationid, long fixed_stationtype);
  
  void setFixedPositionRSU (double latitude_deg, double longitude_deg);
  
  void setStationID (unsigned long fixed_stationid);
  
  void setStationType (long fixed_stationtype);

  void
  setBTP (Ptr<btp> btp)
  {
    m_btp = btp;
  }

  void setSocketTx (Ptr<Socket> socket_tx);
  void setSocketRx (Ptr<Socket> socket_rx);

  
  void startSpatemDissemination ();
  uint64_t terminateDissemination ();

  
  TLMService_error_t appTLM_trigger ();

  const long T_GenSpatemMin_ms = 100; 
  const long T_GenSpatem_ms = 100; 
  const long T_GenSpatemMax_ms = 1000; 

  void
  setRealTime (bool real_time)
  {
    m_real_time = real_time;
  }
  void
  setGeoArea (GeoArea_t geoArea)
  {
    m_geoArea = geoArea;
  }
  
  void cleanup (void);

private:
  bool CheckMainAttributes (void);
  void initDissemination ();

  int64_t computeTimestampUInt64 ();
  TLMService_error_t generateAndEncodeSPATEM ();
  TLMService_error_t doGenerateAndEncodeSPATEM ();

  std::function<void (asn1cpp::Seq<SPATEM>, Address)> m_TLMReceiveCallback;
  std::function<void (asn1cpp::Seq<SPATEM>, Address, Ptr<Packet>)> m_TLMReceiveCallbackPkt;
  std::function<void (asn1cpp::Seq<SPATEM>, Address, StationId_t, StationType_t, SignalInfo)>
      m_TLMReceiveCallbackExtended;

  uint16_t m_port;
  bool m_real_time;
  std::string m_model;

  StationID_t m_station_id;
  StationType_t m_stationtype;
  uint16_t m_seq_number;

  Ptr<btp> m_btp;

  IDP *m_idp; 
  GeoArea_t m_geoArea;
  Ptr<Socket> m_socket_tx; 

  long m_T_CheckSpatemGen_ms;
  long m_T_GenSpatem_ms; 
  int16_t m_N_GenSpatem; 
  int16_t m_N_GenSpatemMax; 

  int64_t lastSpatemGen; 

  
  uint64_t
      m_spatem_sent; 

  
  EventId m_event_spatemDisseminationStart; 
  EventId m_event_spatemSend; 

  double m_last_transmission = 0; 

  long m_T_next_dcc =
      -1; 
};
} 

#endif 
