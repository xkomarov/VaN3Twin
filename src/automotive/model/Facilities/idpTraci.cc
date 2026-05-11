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
#include "idpTraci.h"
#include <functional> 
#include <vector>
#include <cmath>

#include <set>

extern "C" {
  
  #include "ns3/BIT_STRING.h"
}

namespace ns3
{
  IDPTraCI::IDPTraCI()
  {
    m_traci_client=NULL;
    m_id="(null)";
    m_tls_id="";
  }

  IDPTraCI::IDPTraCI(Ptr<TraciClient> traci_client, std::string node_id, std::string tls_id)
  {
    m_traci_client=traci_client;
    m_id = node_id;
    m_tls_id = tls_id;
  }

  IDP::IDP_position_latlon_t
  IDPTraCI::getPosition()
  {
    IDP_position_latlon_t idppos;

    libsumo::TraCIPosition pos;
    pos=m_traci_client->TraCIAPI::poi.getPosition(m_id);
    pos=m_traci_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x,pos.y);

    idppos.lat=pos.y;
    idppos.lon=pos.x;
    idppos.alt=DBL_MAX;

    return idppos;
  }

  IDP::IDP_position_cartesian_t
  IDPTraCI::getPositionXY()
  {
    IDP_position_cartesian_t idppos;

    libsumo::TraCIPosition pos;
    pos=m_traci_client->TraCIAPI::poi.getPosition(m_id);

    idppos.x=pos.x;
    idppos.y=pos.y;
    idppos.z=pos.z;

    return idppos;
  }

  IDP::IDP_position_cartesian_t
  IDPTraCI::getXY(double lon, double lat)
  {
    IDP_position_cartesian_t idppos;

    libsumo::TraCIPosition pos;
    pos=m_traci_client->TraCIAPI::simulation.convertLonLattoXY (lon,lat);

    idppos.x=pos.x;
    idppos.y=pos.y;
    idppos.z=pos.z;

    return idppos;
  }

  double
  IDPTraCI::getCartesianDist (double lon1, double lat1, double lon2, double lat2)
  {
    libsumo::TraCIPosition pos1,pos2;
    pos1 = m_traci_client->TraCIAPI::simulation.convertLonLattoXY(lon1,lat1);
    pos2 = m_traci_client->TraCIAPI::simulation.convertLonLattoXY(lon2,lat2);
    return sqrt((pow((pos1.x-pos2.x),2)+pow((pos1.y-pos2.y),2)));
  }


  std::vector<IDP::SPATEM_mandatory_data_t>
  IDPTraCI::getSPATEMMandatoryData()
  {
      std::vector<SPATEM_mandatory_data_t> result;
      std::vector<std::string> tls_ids;
      
      if (!m_tls_ids.empty()) {
          tls_ids = m_tls_ids;
      } else if (!m_tls_id.empty()) {
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

          spatData.status.size = 2; 
          spatData.status.bits_unused = 0;
          spatData.status.buf = (uint8_t*)calloc(2, sizeof(uint8_t));
          spatData.status.buf[0] = 0x00; 
          
          std::string stateString = m_traci_client->TraCIAPI::trafficlights.getRedYellowGreenState(target_id);

          
          if (m_tls_prev_states.find(target_id) == m_tls_prev_states.end() || m_tls_prev_states[target_id] != stateString) {
              m_tls_prev_states[target_id] = stateString;
              
              m_tls_revisions[target_id] = (m_tls_revisions[target_id] + 1) % 128; 
          }
          spatData.revision = m_tls_revisions[target_id];

          
          double simTime = m_traci_client->TraCIAPI::simulation.getTime();
          spatData.moy = IDPDataItem<uint32_t>((uint32_t)(simTime / 60.0));
          spatData.timeStamp = IDPDataItem<uint16_t>((uint16_t)(std::fmod(simTime, 60.0) * 1000.0));
          spatData.name = IDPDataItem<std::string>(target_id);

          
          
          
          std::vector<std::string> controlledLanes = m_traci_client->TraCIAPI::trafficlights.getControlledLanes(target_id);
          std::vector<uint8_t> laneIDs;
          std::set<uint8_t> seenLaneIDs;
          for (const auto& laneStr : controlledLanes) {
              uint8_t lid = (uint8_t)std::hash<std::string>{}(laneStr);
              if (seenLaneIDs.insert(lid).second) {
                  laneIDs.push_back(lid);
              }
          }
          
          if (laneIDs.size() > 16) {
              laneIDs.resize(16);
          }
          spatData.enabledLanes = IDPDataItem<std::vector<uint8_t>>(laneIDs);

          double nextSwitch = m_traci_client->TraCIAPI::trafficlights.getNextSwitch(target_id);
          double timeLeft = nextSwitch - simTime;
          if (timeLeft < 0) timeLeft = 0.0;
          uint16_t timeLeftDeciSeconds = (uint16_t)(std::round(timeLeft * 10.0));   

          for (size_t i = 0; i < stateString.length(); ++i) {
              SPATEM_SignalGroupState_t groupState;
              groupState.signalGroupID = (int)i + 1; 
              groupState.minEndTime = timeLeftDeciSeconds;
              
              
              groupState.maxEndTime = IDPDataItem<uint16_t>(timeLeftDeciSeconds);
              groupState.likelyTime = IDPDataItem<uint16_t>(timeLeftDeciSeconds);
              
              char s = stateString[i];
              
              switch (s) {
                  case 'r': 
                  case 'R': 
                      groupState.eventState = 3; 
                      break;
                  case 'y': 
                  case 'Y': 
                      groupState.eventState = 7; 
                      break;
                  case 'g': 
                      groupState.eventState = 5; 
                      break;
                  case 'G': 
                      groupState.eventState = 6; 
                      break;
                  case 'u': 
                      groupState.eventState = 3; 
                      break; 
                  case 'o': 
                      groupState.eventState = 1; 
                      break;
                  default:
                      groupState.eventState = 1; 
                      break;
              }

              
              
              if (i < controlledLanes.size()) {
                  
                  double maxSpeed = m_traci_client->TraCIAPI::lane.getMaxSpeed(controlledLanes[i]);
                  IDP_AdvisorySpeed_t advSpeed;
                  advSpeed.type = 0; 
                  advSpeed.speed = (uint16_t)(std::round(maxSpeed * 10.0)); 
                  advSpeed.confidence = IDPDataItem<uint8_t>(100); 

                  std::vector<IDP_AdvisorySpeed_t> speedsVec{advSpeed};
                  groupState.speeds = IDPDataItem<std::vector<IDP_AdvisorySpeed_t>>(speedsVec);

                  
                  int halted = m_traci_client->TraCIAPI::lane.getLastStepHaltingNumber(controlledLanes[i]);
                  
                  IDP_ConnectionManeuverAssist_t maneuver;
                  maneuver.connectionID = groupState.signalGroupID;
                  
                  
                  maneuver.queueLength = IDPDataItem<uint16_t>((uint16_t)(halted * 50));
                  
                  std::vector<IDP_ConnectionManeuverAssist_t> maneuverAssistList{maneuver};
                  groupState.maneuverAssistList = IDPDataItem<std::vector<IDP_ConnectionManeuverAssist_t>>(maneuverAssistList);
              }

              spatData.states.push_back(groupState);
          }
          result.push_back(spatData);
      }

      return result;
  }


}
