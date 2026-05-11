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
#ifndef SENSOR_H
#define SENSOR_H
#include "ns3/ldm-utils.h"
#include "ns3/phPoints.h"
#include "ns3/core-module.h"
#include "ns3/LDM.h"
#include "ns3/vdp.h"

namespace ns3 {
  class Sensor
  {
  public:
    void setStationID(std::string id){m_id=id;m_stationID=std::stol(id.substr (3));}
    void setTraCIclient(Ptr<TraciClient> client){m_client=client;m_event_updateDetectedObjects = Simulator::Schedule(MilliSeconds (100),&SUMOSensor::updateDetectedObjects,this);}
    void setVDP(VDP* vdp) {m_vdp=vdp;}
    void setSensorRange(double sensorRange){m_sensorRange = sensorRange;}
    void updateDetectedObjects();

    void setLDM(Ptr<LDM> ldm){m_LDM = ldm;}
  };
}

#endif // SENSOR_H
