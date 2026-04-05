#include "tlmBasicService.h"
#include "ns3/Seq.hpp"
#include "ns3/Getter.hpp"
#include "ns3/Setter.hpp"
#include "ns3/Encoding.hpp"
#include "ns3/SetOf.hpp"
#include "ns3/SequenceOf.hpp"
#include "ns3/BitString.hpp"
#include "ns3/vdp.h"
#include "asn_utils.h"
#include <cmath>
#include "ns3/snr-tag.h"
#include "ns3/sinr-tag.h"
#include "ns3/rssi-tag.h"
#include "ns3/timestamp-tag.h"
#include "ns3/rsrp-tag.h"
#include "ns3/size-tag.h"

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("TLMBasicService");

  TLMBasicService::TLMBasicService()
  {
    m_station_id = ULONG_MAX;
    m_stationtype = LONG_MAX;
    m_socket_tx = NULL;
    m_real_time = false;
    m_btp = NULL;
    receivedSPATEM = 0;
    m_spatem_sent=0;
    // Setting a default value of m_T_CheckCpmGen_ms equal to 100 ms (i.e. T_GenCpmMin_ms)
    m_T_CheckSpatemGen_ms=T_GenSpatemMin_ms;
    m_T_GenSpatem_ms=T_GenSpatemMax_ms;
    m_N_GenSpatemMax=1000;
    m_N_GenSpatem=T_GenSpatemMin_ms;
    lastSpatemGen = 0;
  }

  bool
  TLMBasicService::CheckMainAttributes()
  {
    return m_station_id!=ULONG_MAX && m_stationtype!=LONG_MAX;
  }



  TLMBasicService::TLMBasicService(unsigned long fixed_stationid,long fixed_stationtype,Ptr<Socket> socket_tx)
  {
    m_station_id = (StationID_t) fixed_stationid;
    m_stationtype = (StationType_t) fixed_stationtype;
    m_socket_tx = socket_tx;
    m_real_time = false;
    m_btp = NULL;
  }

  void
  TLMBasicService::setStationProperties(unsigned long fixed_stationid,long fixed_stationtype)
  {
    m_station_id=fixed_stationid;
    m_stationtype=fixed_stationtype;
    m_btp->setStationProperties(fixed_stationid,fixed_stationtype);
  }

  void
  TLMBasicService::setFixedPositionRSU(double latitude_deg, double longitude_deg)
  {
    m_btp->setFixedPositionRSU(latitude_deg,longitude_deg);
  }

  void
  TLMBasicService::setStationID(unsigned long fixed_stationid)
  {
    m_station_id = fixed_stationid;
    m_btp->setStationID(fixed_stationid);
  }

  void
  TLMBasicService::setStationType(long fixed_stationtype)
  {
    m_stationtype=fixed_stationtype;
    m_btp->setStationType(fixed_stationtype);
  }

  void
  TLMBasicService::setSocketTx (Ptr<Socket> socket_tx)
  {
    m_btp->setSocketTx(socket_tx);
  }

  void
  TLMBasicService::setSocketRx (Ptr<Socket> socket_rx)
  {
    m_btp->setSocketRx(socket_rx);
    m_btp->addSPATEMRxCallback (std::bind(&TLMBasicService::receiveSPATEM,this,std::placeholders::_1,std::placeholders::_2));
  }
 
  void
  TLMBasicService::startSpatemDissemination()
  {
    Ptr<UniformRandomVariable> desync_rvar = CreateObject<UniformRandomVariable> ();
    desync_rvar->SetAttribute ("Min", DoubleValue (0.0));
    desync_rvar->SetAttribute ("Max", DoubleValue (1.0));
    double desync = desync_rvar->GetValue ();
    m_event_spatemDisseminationStart = Simulator::Schedule (Seconds(desync), &TLMBasicService::generateAndEncodeSPATEM, this);
  }

  uint64_t
  TLMBasicService::terminateDissemination()
  {

    Simulator::Remove(m_event_spatemDisseminationStart);
    Simulator::Remove(m_event_spatemSend);
    return m_spatem_sent;
  }

  TLMBasicService_error_t
  TLMBasicService::generateAndEncodeSPATEM()
  {
    VDP::SPATEM_mandatory_data_t spatem_mandatory_data;
    TLMBasicService_error_t errval=SPATEM_NO_ERROR;

    Ptr<Packet> packet;

    std::string encode_result;

    BTPDataRequest_t dataRequest = {};

    int64_t now,now_centi;
    now = computeTimestampUInt64 () / NANO_TO_MILLI;

    /* Collect data for mandatory containers */
    auto spatem = asn1cpp::makeSeq(SPATEM);

    if(bool(spatem)==false)
      {
        return SPATEM_ALLOC_ERROR;
      }

    m_event_spatemSend = Simulator::Schedule (MilliSeconds (m_N_GenSpatem),
                                           &TLMBasicService::generateAndEncodeSPATEM, this);


    asn1cpp::setField(spatem->header.messageId, FIX_SPATEMID);
    asn1cpp::setField(spatem->header.protocolVersion , 2);
    asn1cpp::setField(spatem->header.stationId, m_station_id);

    spatem_mandatory_data=m_vdp->getSPATEMMandatoryData();

    if(spatem_mandatory_data.optional_data)
    {
      asn1cpp::setField(spatem->spat.timeStamp, 0);
      asn1cpp::setField(spatem->spat.name, "0");
    }


    auto intersectionState = asn1cpp::makeSeq(IntersectionState);
    asn1cpp::setField(intersectionState->id.id, spatem_mandatory_data.intersectionId);

    intersectionState->status.buf = (uint8_t*)calloc(1, spatem_mandatory_data.status.size);
     if(intersectionState->status.buf == nullptr)
    {
        NS_LOG_ERROR("Memory allocation failed for status.buf");
        return SPATEM_ALLOC_ERROR;
    }
    memcpy(intersectionState->status.buf, spatem_mandatory_data.status.buf, spatem_mandatory_data.status.size);
    intersectionState->status.size = spatem_mandatory_data.status.size;
    intersectionState->status.bits_unused = spatem_mandatory_data.status.bits_unused;


    if (intersectionState->status.buf == nullptr) {
      NS_LOG_ERROR("Warning: memory allocation failed for intersectionState->status.buf");
      return SPATEM_ALLOC_ERROR; // Или другой подходящий код ошибки
    }

    // intersectionState->status.buf = spatem_mandatory_data.status.buf;
    // intersectionState->status.size = spatem_mandatory_data.status.size;
    // intersectionState->status.bits_unused = spatem_mandatory_data.status.bits_unused;
    asn1cpp::setField(intersectionState->revision, spatem_mandatory_data.revision);   
    

    //option
    if(spatem_mandatory_data.optional_data)
    {
      asn1cpp::setField(intersectionState->name, "0");
      asn1cpp::setField(intersectionState->moy, spatem_mandatory_data.moy);
      asn1cpp::setField(intersectionState->timeStamp, spatem_mandatory_data.timeStamp);
      auto enabledLanes = asn1cpp::makeSeq(EnabledLaneList);
      // for(const auto& laneId : spatem_mandatory_data.enabledLanes)
      // {
      //     asn1cpp::sequenceof::pushList(enabledLanes, laneId);
      // }
      //asn1cpp::setField(intersectionState->enabledLanes, enabledLanes);
      auto maneuverAssist = asn1cpp::makeSeq(ConnectionManeuverAssist);
      asn1cpp::setField(maneuverAssist->connectionID, 1);
      asn1cpp::setField(maneuverAssist->queueLength, 5000);  // 50 метров
      asn1cpp::setField(maneuverAssist->availableStorageLength, 10000);
      asn1cpp::setField(maneuverAssist->waitOnStop, true);
      asn1cpp::setField(maneuverAssist->pedBicycleDetect, false);
      asn1cpp::sequenceof::pushList(intersectionState->maneuverAssistList, maneuverAssist);
    }

    for(const auto& state : spatem_mandatory_data.states)
    {
      auto movementState = asn1cpp::makeSeq(MovementState);
      asn1cpp::setField(movementState->signalGroup, state.signalGroupID);

      //option
      if(spatem_mandatory_data.optional_data)
      {
        asn1cpp::setField(movementState->movementName, "0");
        auto maneuverAssistMs = asn1cpp::makeSeq(ConnectionManeuverAssist);
        asn1cpp::setField(maneuverAssistMs->connectionID, 0);
        asn1cpp::setField(maneuverAssistMs->queueLength, 0);
        asn1cpp::sequenceof::pushList(movementState->maneuverAssistList, maneuverAssistMs);
      }

      auto movementEvent = asn1cpp::makeSeq(MovementEvent);
      asn1cpp::setField(movementEvent->eventState, state.eventState);
        

      auto timing = asn1cpp::makeSeq(TimeChangeDetails);
      asn1cpp::setField(timing->minEndTime, state.minEndTime);
      //option
      if(spatem_mandatory_data.optional_data)
      {
        asn1cpp::setField(timing->startTime, 0);
        asn1cpp::setField(timing->maxEndTime, 0);
        asn1cpp::setField(timing->likelyTime, 0);
        asn1cpp::setField(timing->confidence, 0);
        asn1cpp::setField(timing->nextTime, 0);
      }


      asn1cpp::setField(movementEvent->timing, timing);
        
      //option
      if(spatem_mandatory_data.optional_data)
      {
        auto speedObj = asn1cpp::makeSeq(AdvisorySpeed);
        asn1cpp::setField(speedObj->type, 0);
        asn1cpp::setField(speedObj->speed, 0);
        asn1cpp::setField(speedObj->confidence, 0);
        asn1cpp::setField(speedObj->distance, 0);
        asn1cpp::setField(speedObj->Class, 0);
        //auto speedList = asn1cpp::makeSeq(AdvisorySpeedList);
        //asn1cpp::sequenceof::pushList(speedList, speedObj);
        //asn1cpp::setField(movementEvent->speeds, speedList);   
      }

      asn1cpp::sequenceof::pushList(movementState->state_time_speed, movementEvent);
        
      asn1cpp::sequenceof::pushList(intersectionState->states, movementState);
    }
    
    asn1cpp::sequenceof::pushList(spatem->spat.intersections, intersectionState);
    
    // if (spatem_mandatory_data.status.buf != nullptr) {
    //     free(spatem_mandatory_data.status.buf);
    //     spatem_mandatory_data.status.buf = nullptr; 
    // }

    encode_result = asn1cpp::uper::encode(spatem);
    if(encode_result.size()<1)
    {
        NS_LOG_ERROR("Warning: unable to encode SPATEM.");
        return SPATEM_ASN1_UPER_ENC_ERROR;
    }

    packet = Create<Packet> ((uint8_t*) encode_result.c_str(), encode_result.size());

    dataRequest.BTPType = BTP_B; //!< BTP-B
    dataRequest.destPort = SPATEM_PORT;
    dataRequest.destPInfo = 0;
    dataRequest.GNType = GBC;
    dataRequest.GnAddress = m_geoArea;
    dataRequest.GNCommProfile = UNSPECIFIED;
    dataRequest.GNRepInt =0;
    dataRequest.GNMaxRepInt=0;
    dataRequest.GNMaxLife = 1;
    dataRequest.GNMaxHL = 1;
    dataRequest.GNTraClass = 0x02; // Store carry foward: no - Channel offload: no - Traffic Class ID: 2
    dataRequest.lenght = packet->GetSize ();
    
   if (m_lte_addresses != nullptr && !m_lte_addresses->empty())
{
    // Если есть машины в зоне действия LTE, отправляем пакет каждой индивидуально
    for (auto const& addr : *m_lte_addresses)
    {
        // 1. Устанавливаем соединение с конкретным автомобилем
        m_socket_tx->Connect(addr);

        // 2. Создаем локальную копию запроса для ТЕКУЩЕЙ итерации.
        // Это важно, потому что внутри m_btp->sendBTP к пакету прикрепляются
        // заголовки, и структура dataRequest может мутировать.
        BTPDataRequest_t req_copy = dataRequest; 
        
        // 3. Делаем глубокую копию полезной нагрузки (пакета).
        // (Примечание: в стандартном ms-van3t поле обычно называется 'packet', 
        // но если у вас в структуре оно называется 'data', используйте req_copy.data)
        req_copy.data = packet->Copy(); 

        // 4. Отправляем копию пакета
        std::tuple<GNDataConfirm_t, MessageId_t> status = m_btp->sendBTP(req_copy, 0, MessageId_spatem);
        
        // 5. Проверяем статус отправки
        if(std::get<0>(status) == ACCEPTED) 
        {
            m_spatem_sent++;
        }
    }
}
else
{
    // БЛОК ELSE: Автомобилей в зоне действия нет.
    // В LTE (в отличие от 802.11p) мы не можем сделать "широковещательный" (broadcast) бросок в эфир.
    // Поэтому мы просто НИЧЕГО НЕ ОТПРАВЛЯЕМ и уничтожаем пакет (он удалится сам, т.к. это Ptr<Packet>).
    
    // Можно просто вывести отладочное сообщение, чтобы понимать, что сервер работает:
    NS_LOG_INFO("No active LTE vehicles around. SPATEM packet dropped.");
}
    // Estimation of the transmission time
    m_last_transmission = (double) Simulator::Now().GetMilliSeconds();

    // Store the time in which the last SPATEM (i.e. this one) has been generated and successfully sent
    m_T_GenSpatem_ms=now-lastSpatemGen;
    lastSpatemGen = now;
    return SPATEM_NO_ERROR;
  }

  void
  TLMBasicService::receiveSPATEM(BTPDataIndication_t dataIndication,Address from)
  {
    Ptr<Packet> packet;
    asn1cpp::Seq<SPATEM> decoded_spatem;

    receivedSPATEM++;

    packet = dataIndication.data;

    // Современный и безопасный подход:
    uint32_t packetSize = packet->GetSize();
    std::vector<uint8_t> buffer(packetSize); 
    packet->CopyData(buffer.data(), packetSize);

    std::string packetContent(reinterpret_cast<char*>(buffer.data()), packetSize);

    if (buffer.size() > 1 && buffer[1] != FIX_SPATEMID)
    {
      NS_LOG_ERROR("Warning: received a message which has messageID '" << (int)buffer[1] << "' but '2' was expected.");
      return; // Не нужно писать free(), вектор сам очистит память!
    }

    // uint8_t *buffer; //= new uint8_t[packet->GetSize ()];
    // buffer=(uint8_t *)malloc((packet->GetSize ())*sizeof(uint8_t));
    // packet->CopyData (buffer, packet->GetSize ());
    // std::string packetContent((char *)buffer,(int) dataIndication.data->GetSize ());
    
    RssiTag rssi;
    bool rssi_result = dataIndication.data->PeekPacketTag(rssi);

    SnrTag snr;
    bool snr_result = dataIndication.data->PeekPacketTag(snr);

    RsrpTag rsrp;
    bool rsrp_result = dataIndication.data->PeekPacketTag(rsrp);

    SinrTag sinr;
    bool sinr_result = dataIndication.data->PeekPacketTag(sinr);

    SizeTag size;
    bool size_result = dataIndication.data->PeekPacketTag(size);

    TimestampTag timestamp;
    dataIndication.data->PeekPacketTag(timestamp);

    if(!snr_result)
      {
        snr.Set(SENTINEL_VALUE);
      }
    if (!rssi_result)
      {
        rssi.Set(SENTINEL_VALUE);
      }
    if (!rsrp_result)
      {
        rsrp.Set(SENTINEL_VALUE);
      }
    if (!sinr_result)
      {
        sinr.Set(SENTINEL_VALUE);
      }
    if (!size_result)
      {
        size.Set(SENTINEL_VALUE);
      }

    SetSignalInfo(timestamp.Get(), size.Get(), rssi.Get(), snr.Get(), sinr.Get(), rsrp.Get());
    
    // if (buffer[1]!=FIX_SPATEMID)
    // {
    //   NS_LOG_ERROR("Warning: received a message which has messageID '"<<buffer[1]<<"' but '2' was expected.");
    //   free(buffer);
    //   return;
    // }

    //free(buffer);

    decoded_spatem = asn1cpp::uper::decodeASN(packetContent, SPATEM);

    if(bool(decoded_spatem)==false) {
        NS_LOG_ERROR("Warning: unable to decode a received SPATEM.");
        return;
      }
    
    if(m_TLMReceiveCallback!=nullptr) {
      m_TLMReceiveCallback(decoded_spatem,from);
    }
    if(m_TLMReceiveCallbackExtended!=nullptr) {
      m_TLMReceiveCallbackExtended(decoded_spatem,from,m_station_id,m_stationtype,GetSignalInfo());
    }

  }

  int64_t
  TLMBasicService::computeTimestampUInt64()
  {
    int64_t int_tstamp=0;

    if (!m_real_time)
      {
        int_tstamp=Simulator::Now ().GetNanoSeconds ();
      }
    else
      {
        struct timespec tv;

        clock_gettime (CLOCK_MONOTONIC, &tv);

        int_tstamp=tv.tv_sec*1e9+tv.tv_nsec;
      }
    return int_tstamp;
  }

}