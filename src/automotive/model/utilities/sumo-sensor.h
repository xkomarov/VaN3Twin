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
#ifndef SUMOSENSOR_H
#define SUMOSENSOR_H

#include "ns3/ldm-utils.h"
#include "ns3/phPoints.h"
#include "ns3/core-module.h"
#include "ns3/traci-client.h"
#include "ns3/vdpTraci.h"
#include "ns3/LDM.h"
#include <unordered_map>
#include <vector>
#include <random>
#include <shared_mutex>
#include <boost/geometry.hpp>

namespace ns3 {

  typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_type;

  using polygon_type = boost::geometry::model::polygon<point_type>;
  using linestring_type = boost::geometry::model::linestring<point_type>;

  /**
   * \ingroup automotive
   * \brief This class implements a sensor that detects vehicles in its vicinity for a given SUMO vehicle.
   *
   * This class provides capabilities for detecting vehicles in the vicinity of a SUMO vehicle.
   */
  class SUMOSensor : public Object
  {
  public:
    /**
     * @brief Construct a new SUMOSensor object.
     */
    SUMOSensor();
    ~SUMOSensor();

    /**
     * @brief Set the station ID.
     *
     * @param id The station ID.
     */
    void setStationID(std::string id){m_id=id;m_stationID=std::stol(id.substr (3));}
    /**
     * @brief Set the TraCI client.
     *
     * @param client The TraCI client.
     */
    void setTraCIclient(Ptr<TraciClient> client){
      m_client=client;
      m_event_updateDetectedObjects = Simulator::Schedule(MilliSeconds (100),&SUMOSensor::updateDetectedObjects,this);
    }
    /**
     * @brief Set the VDP object.
     *
     * @param vdp The VDP object.
     */
    void setVDP(VDP* vdp) {m_vdp=vdp;}
    /**
     * @brief Set the sensor perception range. (Default = 50 meters)
     *
     * @param sensorRange The sensor range.
     */
    void setSensorRange(double sensorRange){m_sensorRange = sensorRange;}
    /**
     * @brief Get the detected objects and update the LDM.
     *
     */
    void updateDetectedObjects();

    /**
     * @brief Set the LDM object.
     * @param ldm
     */
    void setLDM(Ptr<LDM> ldm){m_LDM = ldm;}
    libsumo::TraCIPosition boost2TraciPos(point_type point_type);

    void cleanup();

  private:
        //Compute defining points of a vehicle with StationID id
        vehiclePoints_t adjust(std::string id);
        //Create gaussian noise for distance sensor measurements
        double distance_noise();

        //TraCI client pointer
        Ptr<TraciClient> m_client; //!< TraCI client

        uint64_t m_stationID;
        std::string m_id;
        VDP* m_vdp;

        Ptr<LDM> m_LDM;

        EventId m_event_updateDetectedObjects;

        double m_sensorRange; ///! Sensor range in meters

        const double m_mean = 0.0; ///! Mean of the perception's noise
        const double m_stddev_distance = 0.5; ///! Standard deviation of the perception's distance noise
        const double m_stddev_angle = 0.2; ///! Standard deviation of the perception's angle noise
        const double m_stddev_speed = 0.2; ///! Standard deviation of the perception's speed noise
        double m_avg_dwell = 0.0;
        int m_dwell_count = 0;

        std::default_random_engine m_generator;
  };
}
#endif // SUMOSENSOR_H
