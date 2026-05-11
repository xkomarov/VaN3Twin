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
#ifndef NS3_LOS_NLOS_H
#define NS3_LOS_NLOS_H

#include "ns3/channel-condition-model.h"

namespace ns3
{
  class LOS_NLOS : public Object
  {
  public:

    static LOS_NLOS& GetInstance()
    {
      static LOS_NLOS instance;
      return instance;
    }

    void AddBuilding(uint8_t id, std::vector<std::tuple<float, float>> shape)
    {
      m_buildings[id] = shape;
    }

    bool CheckBuildings()
    {
      return m_buildings.empty();
    }

    ChannelCondition::LosConditionValue GetLosNlos(std::tuple<float, float> xy1, std::tuple<float, float> xy2, std::vector<std::tuple<float, float>> other_vehicles = std::vector<std::tuple<float, float>>());

  private:
    std::unordered_map<uint8_t, std::vector<std::tuple<float, float>>> m_buildings;
    float m_threshold_nlosv = 0.5;
  };
}

#endif //NS3_LOS_NLOS_H
