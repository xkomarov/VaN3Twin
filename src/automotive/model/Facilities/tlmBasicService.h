#ifndef TLMBASICSERVICE_H
#define TLMBASICSERVICE_H

#include "ns3/socket.h"
#include "ns3/core-module.h"
#include "ns3/vdp.h"
#include "ns3/asn_utils.h"
#include "ns3/btp.h"
#include "ns3/btpHeader.h"
#include "ns3/Seq.hpp"
#include "ns3/Getter.hpp"
#include "signalInfoUtils.h"
#include "ITSSOriginatingTableEntry.h"
#include "ITSSReceivingTableEntry.h"
#include "ns3/btpdatarequest.h"
#include <functional>
#include <mutex>
#include <queue>
#include "ns3/asn_application.h"
#include "ns3/Setter.hpp"
#include "ns3/Encoding.hpp"
#include "ns3/SetOf.hpp"
#include "ns3/SequenceOf.hpp"
#include "ns3/BitString.hpp"
#include "ns3/View.hpp"
#include "ns3/Utils.hpp"

extern "C" {
  #include "ns3/SPATEM.h"
  #include "ns3/SPAT.h"
}

//#define CURRENT_VDP_TYPE VDPTraCI

namespace ns3
{
  typedef enum {
    SPATEM_NO_ERROR=0,
    SPATEM_WRONG_INTERVAL=1,
    SPATEM_ALLOC_ERROR=2,
    SPATEM_TX_SOCKET_NOT_SET=3,
    SPATEM_ASN1_UPER_ENC_ERROR=4,
    SPATEM_CANNOT_SEND=5
  } TLMBasicService_error_t;


/**
 * \ingroup automotive
 * \brief This class implements the basic service for the  Infrastructure to Vehicle Information (TLM) service defined in ETSI TS 103 301 V1.3.1 (2020-02).
 *
 * TLM service is one instantiation of the infrastructure services to manage the generation, transmission and reception of the TLMM messages.
 * An TLMM supports mandatory and advisory road signage such as contextual speeds and road works warnings.
 * TLMM either provides information of physical road signs such as static or variable road signs, virtual signs or road works.
 *
 */

  class TLMBasicService: public Object, public SignalInfoUtils
  {
    public:
    /**
     * @brief Constructor
     *
     * This constructor initializes the TLMBasicService object.
     */
    TLMBasicService();
    /**
     * @brief Constructor
     *
     * This constructor initializes the TLMBasicService object.
     * @param fixed_stationid The station ID of the ITS-S.
     * @param fixed_stationtype The station type of the ITS-S.
     * @param socket_tx The socket used for transmitting TLMM messages.
     */
    TLMBasicService(unsigned long fixed_stationid,long fixed_stationtype,Ptr<Socket> socket_tx);

    int receivedSPATEM ;
    long timeStamp;

    /**
     * @brief Set Reception callback for TLM messages.
     * @param rx_callback
     */
    void addTLMRxCallback(std::function<void(asn1cpp::Seq<SPATEM>, Address)> rx_callback) {m_TLMReceiveCallback=rx_callback;}
    void addTLMRxCallbackExtended(std::function<void(asn1cpp::Seq<SPATEM>, Address, StationId_t, StationType_t, SignalInfo)> rx_callback) {m_TLMReceiveCallbackExtended=rx_callback;}

    /**
     * @brief Set the VDP object
     * This function sets the VDP object to be used by the CA Basic Service.
     * @param vdp   The VDP object to be used by the CA Basic Service
     */
    void setVDP(VDP* vdp) {m_vdp = vdp;}

    /**
     * @brief Process a received SPATEM message.
     * @param dataIndication  The received data indication from BTP/GeoNet.
     * @param address  The address of the sender of the SPATEM message.
     */
    void receiveSPATEM(BTPDataIndication_t dataIndication, Address address);

    /**
     * @brief Set the station ID and station type of the ITS-S.
     * @param fixed_stationid
     * @param fixed_stationtype
     */
    void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype);
    /**
     * @brief Set the fixed position of the ITS-S.
     * @param latitude_deg
     * @param longitude_deg
     */
    void setFixedPositionRSU(double latitude_deg, double longitude_deg);
    /**
     * @brief Set the station ID of the ITS-S.
     * @param fixed_stationid
     */
    void setStationID(unsigned long fixed_stationid);
    /**
     * @brief Set the station type of the ITS-S.
     * @param fixed_stationtype
     */
    void setStationType(long fixed_stationtype);

    void setBTP(Ptr<btp> btp){m_btp = btp;}

    void setSocketTx(Ptr<Socket> socket_tx);
    void setSocketRx(Ptr<Socket> socket_rx);

    /**
   * @brief Запустить процесс рассылки (диссеминации) сообщений CPM.
   * Инициирует планировщик событий для периодической генерации сообщений.
   */
    void startSpatemDissemination();
    uint64_t terminateDissemination();

    const long T_GenSpatemMin_ms = 100;    //!< Минимальный интервал между генерациями CPM (100 мс)
    const long T_GenSpatem_ms = 100;       //!< Стандартный интервал генерации CPM (100 мс)
    const long T_GenSpatemMax_ms = 1000;   //!< Максимальный интервал между генерациями CPM (1000 мс)

    void setRealTime(bool real_time){m_real_time=real_time;}
    void setGeoArea(GeoArea_t geoArea){m_geoArea = geoArea;}
    void setLTEAddresses(std::set<Address>* addresses) { m_lte_addresses = addresses; }
    /* Cleanup function - always call this before terminating the simulation */
    void cleanup(void);

  private:

    bool CheckMainAttributes(void);
    void initDissemination();

    int64_t computeTimestampUInt64();
    TLMBasicService_error_t generateAndEncodeSPATEM();


    std::function<void(asn1cpp::Seq<SPATEM>, Address)> m_TLMReceiveCallback;
    std::function<void(asn1cpp::Seq<SPATEM>, Address, Ptr<Packet>)> m_TLMReceiveCallbackPkt;
    std::function<void(asn1cpp::Seq<SPATEM>, Address, StationId_t, StationType_t, SignalInfo)> m_TLMReceiveCallbackExtended;

    uint16_t m_port;
    bool m_real_time;
    std::string m_model;


    StationID_t m_station_id;
    StationType_t m_stationtype;
    uint16_t m_seq_number;

    Ptr<btp> m_btp;
    VDP* m_vdp; //! VDP object
    GeoArea_t m_geoArea;
    Ptr<Socket> m_socket_tx; // Socket TX

    std::set<Address>* m_lte_addresses = nullptr;

    long m_T_CheckSpatemGen_ms;
    long m_T_GenSpatem_ms; //!< Текущий интервал генерации CPM
    int16_t m_N_GenSpatem; //!< Время до следующей генерации (используется для планирования события)
    int16_t m_N_GenSpatemMax; //!< Максимальный лимит (используется в логике частоты)

    int64_t lastSpatemGen; //!< Метка времени последней генерации CPM
 
    // CP Basic Service может насчитать до 18446744073709551615 (UINT64_MAX) CPM
    uint64_t m_spatem_sent; //!< Статистика: количество успешно отправленных CPM с момента запуска сервиса

    // ID событий ns-3, используемые для корректной остановки симуляции с помощью terminateDissemination()
    EventId m_event_spatemDisseminationStart; //!< ID события начала рассылки
    EventId m_event_spatemSend; //!< ID события отправки сообщения

    double m_last_transmission = 0; //!< Время последней передачи

    long m_T_next_dcc = -1; //!< Время следующей проверки DCC (Decentralized Congestion Control - контроль перегрузки канала)

  };
}

#endif // CABASICSERVICE_H
