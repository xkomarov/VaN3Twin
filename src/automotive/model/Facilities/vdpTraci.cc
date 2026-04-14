/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * Created by:
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
*/

#include "vdpTraci.h"
#include <functional> 
#include <vector>
#include <cmath>
#include <algorithm>
#include <set>

extern "C" {
  #include "ns3/CAM.h"
  #include "ns3/BIT_STRING.h"
}

namespace ns3
{
  VDPTraCI::VDPTraCI()
  {
    m_traci_client=NULL;
    m_id="(null)";
    m_isStatic = NULL;
    m_tls_id="";

    m_vehicleRole = VDPDataItem<unsigned int>();
    // Special vehicle container
    m_publicTransportContainerData = VDPDataItem<VDP_PublicTransportContainerData_t>();
    m_specialTransportContainerData = VDPDataItem<VDP_SpecialTransportContainerData_t>();
    m_dangerousGoodsBasicType = VDPDataItem<int>(); // For the DangerousGoodsContainer
    m_roadWorksContainerBasicData = VDPDataItem<VDP_RoadWorksContainerBasicData_t>();
    m_rescueContainerLightBarSirenInUse = VDPDataItem<uint8_t>();
    m_emergencyContainerData = VDPDataItem<VDP_EmergencyContainerData_t>();
    m_safetyCarContainerData = VDPDataItem<VDP_SafetyCarContainerData_t>();

  }

  VDPTraCI::VDPTraCI(Ptr<TraciClient> traci_client, std::string node_id)
  {
    m_traci_client=traci_client;

    m_isStatic = false;

    m_id = node_id;

    m_tls_id="";

    if (!m_isStatic)
      {
        /* Length and width of car [0.1 m] */
        m_vehicle_length = VDPValueConfidence<long, long> (
            m_traci_client->TraCIAPI::vehicle.getLength (m_id) * DECI,
            VehicleLengthConfidenceIndication_unavailable);
        //    m_vehicle_length.vehicleLengthValue = m_traci_client->TraCIAPI::vehicle.getLength (m_id)*DECI;
        //    m_vehicle_length.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_unavailable;

        // ETSI TS 102 894-2 V1.2.1 - A.92 (Length greater than 102,2 m should be set to 102,2 m)
        if (m_vehicle_length.getValue () > 1022)
          {
            m_vehicle_length.setValue (1022);
          }

        m_vehicle_width = m_traci_client->TraCIAPI::vehicle.getWidth (m_id) * DECI;

        // ETSI TS 102 894-2 V1.2.1 - A.95 (Width greater than 6,1 m should be set to 6,1 m)
        if (m_vehicle_width > 61)
          {
            m_vehicle_width = 61;
          }
      }

    m_vehicleRole = VDPDataItem<unsigned int>();
    // Special vehicle container
    m_publicTransportContainerData = VDPDataItem<VDP_PublicTransportContainerData_t>();
    m_specialTransportContainerData = VDPDataItem<VDP_SpecialTransportContainerData_t>();
    m_dangerousGoodsBasicType = VDPDataItem<int>(); // For the DangerousGoodsContainer
    m_roadWorksContainerBasicData = VDPDataItem<VDP_RoadWorksContainerBasicData_t>();
    m_rescueContainerLightBarSirenInUse = VDPDataItem<uint8_t>();
    m_emergencyContainerData = VDPDataItem<VDP_EmergencyContainerData_t>();
    m_safetyCarContainerData = VDPDataItem<VDP_SafetyCarContainerData_t>();
  }

  VDPTraCI::VDPTraCI(Ptr<TraciClient> traci_client, std::string node_id, bool isStatic)
  {
    m_traci_client=traci_client;

    m_isStatic = isStatic;

    m_id = node_id;

    m_tls_id="";

    if(!m_isStatic)
      {
        /* Length and width of car [0.1 m] */
        m_vehicle_length = VDPValueConfidence<long, long> (
            m_traci_client->TraCIAPI::vehicle.getLength (m_id) * DECI,
            VehicleLengthConfidenceIndication_unavailable);
        //    m_vehicle_length.vehicleLengthValue = m_traci_client->TraCIAPI::vehicle.getLength (m_id)*DECI;
        //    m_vehicle_length.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_unavailable;

        // ETSI TS 102 894-2 V1.2.1 - A.92 (Length greater than 102,2 m should be set to 102,2 m)
        if (m_vehicle_length.getValue () > 1022)
          {
            m_vehicle_length.setValue (1022);
          }

        m_vehicle_width = m_traci_client->TraCIAPI::vehicle.getWidth (m_id) * DECI;

        // ETSI TS 102 894-2 V1.2.1 - A.95 (Width greater than 6,1 m should be set to 6,1 m)
        if (m_vehicle_width > 61)
          {
            m_vehicle_width = 61;
          }

        m_vehicleRole = VDPDataItem<unsigned int> ();
        // Special vehicle container
        m_publicTransportContainerData = VDPDataItem<VDP_PublicTransportContainerData_t> ();
        m_specialTransportContainerData = VDPDataItem<VDP_SpecialTransportContainerData_t> ();
        m_dangerousGoodsBasicType = VDPDataItem<int> (); // For the DangerousGoodsContainer
        m_roadWorksContainerBasicData = VDPDataItem<VDP_RoadWorksContainerBasicData_t> ();
        m_rescueContainerLightBarSirenInUse = VDPDataItem<uint8_t> ();
        m_emergencyContainerData = VDPDataItem<VDP_EmergencyContainerData_t> ();
        m_safetyCarContainerData = VDPDataItem<VDP_SafetyCarContainerData_t> ();
      }
  }

  VDPTraCI::VDPTraCI(Ptr<TraciClient> traci_client, std::string node_id, bool isStatic, std::string tls_id)
  {
    m_traci_client=traci_client;

    m_isStatic = isStatic;

    m_id = node_id;

    m_tls_id = tls_id;

    if(!m_isStatic)
      {
        /* Length and width of car [0.1 m] */
        m_vehicle_length = VDPValueConfidence<long, long> (
            m_traci_client->TraCIAPI::vehicle.getLength (m_id) * DECI,
            VehicleLengthConfidenceIndication_unavailable);
        //    m_vehicle_length.vehicleLengthValue = m_traci_client->TraCIAPI::vehicle.getLength (m_id)*DECI;
        //    m_vehicle_length.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_unavailable;

        // ETSI TS 102 894-2 V1.2.1 - A.92 (Length greater than 102,2 m should be set to 102,2 m)
        if (m_vehicle_length.getValue () > 1022)
          {
            m_vehicle_length.setValue (1022);
          }

        m_vehicle_width = m_traci_client->TraCIAPI::vehicle.getWidth (m_id) * DECI;

        // ETSI TS 102 894-2 V1.2.1 - A.95 (Width greater than 6,1 m should be set to 6,1 m)
        if (m_vehicle_width > 61)
          {
            m_vehicle_width = 61;
          }

        m_vehicleRole = VDPDataItem<unsigned int> ();
        // Special vehicle container
        m_publicTransportContainerData = VDPDataItem<VDP_PublicTransportContainerData_t> ();
        m_specialTransportContainerData = VDPDataItem<VDP_SpecialTransportContainerData_t> ();
        m_dangerousGoodsBasicType = VDPDataItem<int> (); // For the DangerousGoodsContainer
        m_roadWorksContainerBasicData = VDPDataItem<VDP_RoadWorksContainerBasicData_t> ();
        m_rescueContainerLightBarSirenInUse = VDPDataItem<uint8_t> ();
        m_emergencyContainerData = VDPDataItem<VDP_EmergencyContainerData_t> ();
        m_safetyCarContainerData = VDPDataItem<VDP_SafetyCarContainerData_t> ();
      }
  }

  VDP::VDP_position_latlon_t
  VDPTraCI::getPosition()
  {
    VDP_position_latlon_t vdppos;

    libsumo::TraCIPosition pos;
    if (!m_isStatic)
      pos=m_traci_client->TraCIAPI::vehicle.getPosition(m_id);
    else
      pos = m_traci_client->TraCIAPI::poi.getPosition(m_id);

    pos=m_traci_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x,pos.y);

    vdppos.lat=pos.y;
    vdppos.lon=pos.x;
    vdppos.alt=DBL_MAX;

    return vdppos;
  }

  VDP::VDP_position_cartesian_t
  VDPTraCI::getPositionXY()
  {
    VDP_position_cartesian_t vdppos;

    libsumo::TraCIPosition pos;
    if (!m_isStatic)
      pos=m_traci_client->TraCIAPI::vehicle.getPosition(m_id);
    else
      pos = m_traci_client->TraCIAPI::poi.getPosition(m_id);

    vdppos.x=pos.x;
    vdppos.y=pos.y;
    vdppos.z=pos.z;

    return vdppos;
  }

  VDP::VDP_position_cartesian_t
  VDPTraCI::getXY(double lon, double lat)
  {
    VDP_position_cartesian_t vdppos;

    libsumo::TraCIPosition pos;
    pos=m_traci_client->TraCIAPI::simulation.convertLonLattoXY (lon,lat);

    vdppos.x=pos.x;
    vdppos.y=pos.y;
    vdppos.z=pos.z;

    return vdppos;
  }

  double
  VDPTraCI::getCartesianDist (double lon1, double lat1, double lon2, double lat2)
  {
    libsumo::TraCIPosition pos1,pos2;
    pos1 = m_traci_client->TraCIAPI::simulation.convertLonLattoXY(lon1,lat1);
    pos2 = m_traci_client->TraCIAPI::simulation.convertLonLattoXY(lon2,lat2);
    return sqrt((pow((pos1.x-pos2.x),2)+pow((pos1.y-pos2.y),2)));
  }

  VDPTraCI::CAM_mandatory_data_t
  VDPTraCI::getCAMMandatoryData ()
  {
    CAM_mandatory_data_t CAMdata;

    /* Speed [0.01 m/s] */
    if (!m_isStatic)
      CAMdata.speed = VDPValueConfidence<> (m_traci_client->TraCIAPI::vehicle.getSpeed (m_id) * CENTI,
                                            SpeedConfidence_unavailable);

    /* Position */
    libsumo::TraCIPosition pos;
    if (!m_isStatic)
      pos=m_traci_client->TraCIAPI::vehicle.getPosition(m_id);
    else
      pos = m_traci_client->TraCIAPI::poi.getPosition(m_id);
    pos=m_traci_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x,pos.y);

    // longitude WGS84 [0,1 microdegree]
    CAMdata.longitude=(Longitude_t)(pos.x*DOT_ONE_MICRO);
    // latitude WGS84 [0,1 microdegree]
    CAMdata.latitude=(Latitude_t)(pos.y*DOT_ONE_MICRO);

    /* Altitude [0,01 m] */
    CAMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable,
                                          AltitudeConfidence_unavailable);

    /* Position Confidence Ellipse */
    CAMdata.posConfidenceEllipse.semiMajorConfidence=SemiAxisLength_unavailable;
    CAMdata.posConfidenceEllipse.semiMinorConfidence=SemiAxisLength_unavailable;
    CAMdata.posConfidenceEllipse.semiMajorOrientation=HeadingValue_unavailable;

    /* Longitudinal acceleration [0.1 m/s^2] */
    if (!m_isStatic)
      CAMdata.longAcceleration = VDPValueConfidence<>(m_traci_client->TraCIAPI::vehicle.getAcceleration (m_id) * DECI,
                                                  AccelerationConfidence_unavailable);

    /* Heading WGS84 north [0.1 degree] */
    if (!m_isStatic)
      CAMdata.heading = VDPValueConfidence<>(m_traci_client->TraCIAPI::vehicle.getAngle (m_id) * DECI,
                                         HeadingConfidence_unavailable);

    /* Drive direction (backward driving is not fully supported by SUMO, at the moment */
    CAMdata.driveDirection = DriveDirection_unavailable;

    /* Curvature and CurvatureCalculationMode */
    CAMdata.curvature = VDPValueConfidence<>(CurvatureValue_unavailable,
                                             CurvatureConfidence_unavailable);
    CAMdata.curvature_calculation_mode = CurvatureCalculationMode_unavailable;

    /* Length and Width [0.1 m] */
    if (!m_isStatic) {
        CAMdata.VehicleLength = m_vehicle_length;
        CAMdata.VehicleWidth = m_vehicle_width;
      }

    /* Yaw Rate */
    CAMdata.yawRate = VDPValueConfidence<>(YawRateValue_unavailable,
                                           YawRateConfidence_unavailable);

    return CAMdata;
  }

  VDPTraCI::CPM_mandatory_data_t
  VDPTraCI::getCPMMandatoryData ()
  {
    CPM_mandatory_data_t CPMdata;

    /* Speed [0.01 m/s] */
    if (!m_isStatic)
      CPMdata.speed = VDPValueConfidence<> (m_traci_client->TraCIAPI::vehicle.getSpeed (m_id) * CENTI,
                                            SpeedConfidence_unavailable);

    /* Position */
    libsumo::TraCIPosition pos;
    if(!m_isStatic)
      pos=m_traci_client->TraCIAPI::vehicle.getPosition(m_id);
    else
      pos = m_traci_client->TraCIAPI::poi.getPosition(m_id);
    pos=m_traci_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x,pos.y);

    // longitude WGS84 [0,1 microdegree]
    CPMdata.longitude=(Longitude_t)(pos.x*DOT_ONE_MICRO);
    // latitude WGS84 [0,1 microdegree]
    CPMdata.latitude=(Latitude_t)(pos.y*DOT_ONE_MICRO);

    /* Altitude [0,01 m] */
    CPMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable,
                                          AltitudeConfidence_unavailable);

    /* Position Confidence Ellipse */
    CPMdata.posConfidenceEllipse.semiMajorConfidence=SemiAxisLength_unavailable;
    CPMdata.posConfidenceEllipse.semiMinorConfidence=SemiAxisLength_unavailable;
    CPMdata.posConfidenceEllipse.semiMajorOrientation=HeadingValue_unavailable;

    /* Longitudinal acceleration [0.1 m/s^2] */
    if(!m_isStatic)
      CPMdata.longAcceleration = VDPValueConfidence<>(m_traci_client->TraCIAPI::vehicle.getAcceleration (m_id) * DECI,
                                                  AccelerationConfidence_unavailable);

    /* Heading WGS84 north [0.1 degree] */
    if(!m_isStatic)
      CPMdata.heading = VDPValueConfidence<>(m_traci_client->TraCIAPI::vehicle.getAngle (m_id) * DECI,
                                         HeadingConfidence_unavailable);

    /* Drive direction (backward driving is not fully supported by SUMO, at the moment */
    CPMdata.driveDirection = DriveDirection_unavailable;

    /* Curvature and CurvatureCalculationMode */
    CPMdata.curvature = VDPValueConfidence<>(CurvatureValue_unavailable,
                                             CurvatureConfidence_unavailable);
    CPMdata.curvature_calculation_mode = CurvatureCalculationMode_unavailable;

    /* Length and Width [0.1 m] */
    if(!m_isStatic){
        CPMdata.VehicleLength = m_vehicle_length;
        CPMdata.VehicleWidth = m_vehicle_width;
      }

    /* Yaw Rate */
    CPMdata.yawRate = VDPValueConfidence<>(YawRateValue_unavailable,
                                           YawRateConfidence_unavailable);

    return CPMdata;
  }

  std::vector<VDP::SPATEM_mandatory_data_t>
  VDPTraCI::getSPATEMMandatoryData()
  {
      std::vector<SPATEM_mandatory_data_t> result;
      std::vector<std::string> tls_ids;
      
      if (!m_tls_id.empty()) {
          tls_ids.push_back(m_tls_id);
      } else {
          tls_ids = m_traci_client->TraCIAPI::trafficlights.getIDList();
      }

      for (const auto& target_id : tls_ids) {
          SPATEM_mandatory_data_t spatData;

          try {
                spatData.intersectionId = (uint16_t)std::hash<std::string>{}(target_id);
              } catch (...) {
                spatData.intersectionId = 0;
              }

          spatData.status.size = 2; // IntersectionStatusObject is exactly 16 bits (2 bytes)
          spatData.status.bits_unused = 0;
          spatData.status.buf = (uint8_t*)calloc(2, sizeof(uint8_t));
          spatData.status.buf[0] = 0x00; 
          
          std::string stateString = m_traci_client->TraCIAPI::trafficlights.getRedYellowGreenState(target_id);

          // Логика вычисления поля revision 
          if (m_tls_prev_states.find(target_id) == m_tls_prev_states.end() || m_tls_prev_states[target_id] != stateString) {
              m_tls_prev_states[target_id] = stateString;
              // Закольцовываем инкремент до 127 по стандарту (0..127)
              m_tls_revisions[target_id] = (m_tls_revisions[target_id] + 1) % 128; 
          }
          spatData.revision = m_tls_revisions[target_id];

          // Заполняем время (DSecond и Moy) и имя перекрёстка
          double simTime = m_traci_client->TraCIAPI::simulation.getTime();
          spatData.moy = VDPDataItem<uint32_t>((uint32_t)(simTime / 60.0));
          spatData.timeStamp = VDPDataItem<uint16_t>((uint16_t)(std::fmod(simTime, 60.0) * 1000.0));
          spatData.name = VDPDataItem<std::string>(target_id);

          // Извлекаем полосы и хэшируем ID-шники в uint8_t
          // SUMO возвращает одну полосу на каждое «подключение» (connection),
          // поэтому одна и та же полоса может повторяться. Дедуплицируем.
          std::vector<std::string> controlledLanes = m_traci_client->TraCIAPI::trafficlights.getControlledLanes(target_id);
          std::vector<uint8_t> laneIDs;
          std::set<uint8_t> seenLaneIDs;
          for (const auto& laneStr : controlledLanes) {
              uint8_t lid = (uint8_t)std::hash<std::string>{}(laneStr);
              if (seenLaneIDs.insert(lid).second) {
                  laneIDs.push_back(lid);
              }
          }
          // ASN.1 EnabledLaneList имеет ограничение SIZE(1..16)
          if (laneIDs.size() > 16) {
              laneIDs.resize(16);
          }
          spatData.enabledLanes = VDPDataItem<std::vector<uint8_t>>(laneIDs);

          double nextSwitch = m_traci_client->TraCIAPI::trafficlights.getNextSwitch(target_id);
          double timeLeft = nextSwitch - simTime;
          if (timeLeft < 0) timeLeft = 0.0;
          uint16_t timeLeftDeciSeconds = (uint16_t)(std::round(timeLeft * 10.0));      

          for (size_t i = 0; i < stateString.length(); ++i) {
              SPATEM_SignalGroupState_t groupState;
              groupState.signalGroupID = (int)i + 1; 
              groupState.minEndTime = timeLeftDeciSeconds;
              
              // Заполняем предположительные временные рамки (.setData ставит флаг m_available = true)
              groupState.maxEndTime = VDPDataItem<uint16_t>(timeLeftDeciSeconds);
              groupState.likelyTime = VDPDataItem<uint16_t>(timeLeftDeciSeconds);

              char s = stateString[i];
              
              switch (s) {
                  case 'r': 
                  case 'R': 
                      groupState.eventState = 3; // stop-And-Remain
                      break;
                  case 'y': 
                  case 'Y': 
                      groupState.eventState = 7; // intersection-clearance
                      break;
                  case 'g': 
                      groupState.eventState = 5; // permissive-Movement-Allowed
                      break;
                  case 'G': 
                      groupState.eventState = 6; // protected-Movement-Allowed
                      break;
                  case 'u': 
                      groupState.eventState = 3; 
                      break; 
                  case 'o': 
                      groupState.eventState = 1; // unavailable / off
                      break;
                  default:
                      groupState.eventState = 1; // unavailable
                      break;
              }

              // Заполнение AdvisorySpeed и ManeuverAssist
              // Индексы StateString и controlledLanes взаимосвязаны
              if (i < controlledLanes.size()) {
                  // Извлекаем Advisory Speed на полосе
                  double maxSpeed = m_traci_client->TraCIAPI::lane.getMaxSpeed(controlledLanes[i]);
                  VDP_AdvisorySpeed_t advSpeed;
                  advSpeed.type = 0; // none/base advisory
                  advSpeed.speed = (uint16_t)(std::round(maxSpeed * 10.0)); // Конвертация в 0.1 м/с
                  advSpeed.confidence = VDPDataItem<uint8_t>(100); // 100% уверенности

                  std::vector<VDP_AdvisorySpeed_t> speedsVec{advSpeed};
                  groupState.speeds = VDPDataItem<std::vector<VDP_AdvisorySpeed_t>>(speedsVec);

                  // ManeuverAssist (расчёт очереди)
                  int halted = m_traci_client->TraCIAPI::lane.getLastStepHaltingNumber(controlledLanes[i]);
                  
                  VDP_ConnectionManeuverAssist_t maneuver;
                  maneuver.connectionID = groupState.signalGroupID;
                  // В ASN.1 queueLength исчисляется в единицах 0.1 м
                  // 1 ТС в среднем занимает ~5 метров. 5м * 10 = 50 единиц (0.1m) на ТС.
                  maneuver.queueLength = VDPDataItem<uint16_t>((uint16_t)(halted * 50));
                  
                  std::vector<VDP_ConnectionManeuverAssist_t> maneuverAssistList{maneuver};
                  groupState.maneuverAssistList = VDPDataItem<std::vector<VDP_ConnectionManeuverAssist_t>>(maneuverAssistList);
              }

              spatData.states.push_back(groupState);
          }
          result.push_back(spatData);
      }

      return result;
  }

  VDPDataItem<int>
  VDPTraCI::getLanePosition()
  {
    if (m_isStatic)
      return VDPDataItem<int>((int)NULL);
    int laneIndex;
    int lanePosition;

    laneIndex=m_traci_client->TraCIAPI::vehicle.getLaneIndex (m_id);

    // We add '1' as sumo lane indeces start from '0', while
    // LanePosition_t uses '1' as the index for the first rightmost
    // lane ('0' would be reserved to 'hardShoulder')
    lanePosition = laneIndex+1;
    if (laneIndex < 0 || laneIndex > 14)
      {
        lanePosition = LanePosition_offTheRoad;
      }

    return VDPDataItem<int>(lanePosition);
  }

  VDPDataItem<uint8_t>
  VDPTraCI::getExteriorLights ()
  {
    if(m_isStatic)
      return VDPDataItem<uint8_t>();
    int extLights = m_traci_client->TraCIAPI::vehicle.getSignals (m_id);
    uint8_t retval = 0;
    if(extLights & VEH_SIGNAL_BLINKER_RIGHT)
      retval |= 1<< ExteriorLights_rightTurnSignalOn;
    if(extLights & VEH_SIGNAL_BLINKER_LEFT)
      retval |= 1<<ExteriorLights_leftTurnSignalOn;
    if(extLights & VEH_SIGNAL_FRONTLIGHT)
      retval |= 1<<ExteriorLights_lowBeamHeadlightsOn;
    if(extLights & VEH_SIGNAL_FOGLIGHT)
      retval |= 1<<ExteriorLights_fogLightOn;
    if(extLights & VEH_SIGNAL_HIGHBEAM)
      retval |= 1<<ExteriorLights_highBeamHeadlightsOn;
    if(extLights & VEH_SIGNAL_BACKDRIVE)
      retval |= 1<<ExteriorLights_reverseLightOn;

    return VDPDataItem<uint8_t> (retval);

  }
}
