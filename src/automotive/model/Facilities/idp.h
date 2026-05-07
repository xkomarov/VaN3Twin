#ifndef IDP_H
#define IDP_H

#include "asn_utils.h"
#include <cstdint>
#include <float.h>
// #include "ns3/Seq.hpp"
// #include "ns3/Getter.hpp"
// #include "ns3/Setter.hpp"
// #include "ns3/SetOf.hpp"
// #include "ns3/SequenceOf.hpp"

extern "C" {
  #include "ns3/BIT_STRING.h"
}

namespace ns3
{
  template <class T>
  class IDPDataItem
  {
      private:
        bool m_available;
        T m_dataitem;

      public:
        IDPDataItem(T data): m_dataitem(data) {m_available=true;}
        IDPDataItem() {m_available=false;}
        T getData() const {return m_dataitem;}
        bool isAvailable() const {return m_available;}
        T setData(T data) {m_dataitem=data; m_available=true;}
  };

  template <class V = int, class C = int>
  class IDPValueConfidence
  {
      private:
        V m_value;
        C m_confidence;

      public:
        IDPValueConfidence() {}
        IDPValueConfidence(V value,C confidence):
          m_value(value), m_confidence(confidence) {}

        V getValue() {return m_value;}
        C getConfidence() {return m_confidence;}
        void setValue(V value) {m_value=value;}
        void setConfidence(C confidence) {m_confidence=confidence;}
  };

  /**
   * \ingroup automotive
   *
   * \brief This class provides the interface for the Vehicle Data Provider (IDP).
   *
   * This class encapsulates various data structures representing different aspects of vehicle data and its processing.
   */
  class IDP
  {
    public:

    /**
     * @struct IDP_position_cartesian
     * @brief Represents a position in Cartesian coordinates.
     *
     * Describes a vehicle's position in a 3D Cartesian coordinate system.
     */
      typedef struct IDP_position_cartesian {
        double x,y,z;
      } IDP_position_cartesian_t;

    /**
     * @struct IDP_position_latlon
     * @brief Represents a geographic position.
     *
     * Describes a vehicle's position in terms of latitude, longitude, and altitude.
     */
      typedef struct IDP_position_latlon {
        double lat,lon,alt;
      } IDP_position_latlon_t;


      typedef struct IDP_ProtectedCommunicationsZonesRSU{
        long protectedZoneType;
        IDPDataItem<int> expiryTime	/* OPTIONAL */;
        long protectedZoneLatitude;
        long protectedZoneLongitude;
        IDPDataItem<long> protectedZoneRadius	/* OPTIONAL */;
        IDPDataItem<long> protectedZoneID	/* OPTIONAL */;
      } IDP_ProtectedCommunicationsZonesRSU_t;


      typedef struct IDP_ConnectionManeuverAssist {
          uint8_t connectionID;
          IDPDataItem<uint16_t> queueLength;
          IDPDataItem<uint16_t> availableStorageLength;
          IDPDataItem<bool> waitOnStop;
          IDPDataItem<bool> pedBicycleDetect;
      } IDP_ConnectionManeuverAssist_t;

      typedef struct IDP_AdvisorySpeed {
          uint8_t type;
          uint16_t speed;
          IDPDataItem<uint8_t> confidence;
          IDPDataItem<uint16_t> distance;
          IDPDataItem<uint8_t> Class;
      } IDP_AdvisorySpeed_t;

      typedef struct SPATEM_SignalGroupState {
          uint8_t signalGroupID;      
          uint8_t eventState;         
          uint16_t minEndTime;        
          IDPDataItem<std::string> movementName;
          IDPDataItem<std::vector<IDP_ConnectionManeuverAssist_t>> maneuverAssistList;
          IDPDataItem<uint16_t> startTime;
          IDPDataItem<uint16_t> maxEndTime;
          IDPDataItem<uint16_t> likelyTime;
          IDPDataItem<uint8_t> confidence;
          IDPDataItem<uint16_t> nextTime;
          IDPDataItem<std::vector<IDP_AdvisorySpeed_t>> speeds;
          //IDPDataItem<uint16_t> regionalExtension; 
      } SPATEM_SignalGroupState_t;

      typedef struct SPATEM_mandatory_data {
          uint16_t intersectionId;            
          BIT_STRING_t status;
          uint8_t revision; 
          //IDPDataItem<uint16_t> regionId;           
          IDPDataItem<uint32_t> moy;              
          IDPDataItem<uint16_t> timeStamp;
          IDPDataItem<std::string> name;
          IDPDataItem<std::vector<IDP_ConnectionManeuverAssist_t>> maneuverAssistList;
          IDPDataItem<uint32_t> spatTimeStamp;
          IDPDataItem<std::string> spatName;
          std::vector<SPATEM_SignalGroupState_t> states; 
          IDPDataItem<uint16_t> regionalExtension; 
          IDPDataItem<std::vector<uint8_t>> enabledLanes;
      } SPATEM_mandatory_data_t;

      virtual std::vector<SPATEM_mandatory_data_t> getSPATEMMandatoryData() = 0;



      // These methods shall return a position either cartesian or geodetic (needed mainly by the GeoNetworking module)
      // getPositionLatLon() shall return the current position of the object as (Latitude,Longitude,Altitude)
      // getPositionXY() shall return the current position as cartesian projected coordinates (x,y,z)
      // If the Altitude (or z) is not available, it must be set to DBL_MAX
      // getXY() shall be a utility function converting from Lat,Lon to x,y on a projected coordinate system (e.g. using Transverse Mercator)
      virtual IDP_position_latlon_t getPosition() = 0;
      virtual IDP_position_cartesian_t getPositionXY() = 0;
      virtual IDP_position_cartesian_t getXY(double lon, double lat) = 0;
      virtual double getCartesianDist (double lon1, double lat1, double lon2, double lat2) = 0;
  };
}
#endif // IDP_H