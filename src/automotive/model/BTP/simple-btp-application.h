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
#ifndef NS3_UDP_ARQ_APPLICATION_H
#define NS3_UDP_ARQ_APPLICATION_H
#include "ns3/socket.h"
#include "ns3/application.h"
#include "ns3/btpHeader.h"
#include "ns3/btp.h"

using namespace ns3;

namespace ns3
{
  class SimpleBtpApplication : public Application 
  {
    public:
      SimpleBtpApplication ();
      virtual ~SimpleBtpApplication ();

      static TypeId GetTypeId ();
      virtual TypeId GetInstanceTypeId () const;

      /** \brief handles incoming packets on port 9999
       */
      void DataIndication (Ptr<Packet> packet, btpHeader header);

      /** \brief Send an outgoing packet. This creates a new socket every time (not the best solution)
      */
      void DataRequest (Ptr<Packet> packet, uint16_t servicePort, bool btpType);

    private:
      
      
      void SetupReceiveSocket (Ptr<Socket> socket, uint16_t port);
      virtual void StartApplication ();


      Ptr<Socket> m_recv_socket1; /**< A socket to receive on a specific port */
      Ptr<Socket> m_recv_socket2; /**< A socket to receive on a specific port */
      btp m_btp; // BTP object
      uint16_t m_port1; 
      uint16_t m_port2;

      Ptr<Socket> m_send_socket; /**< A socket to listen on a specific port */
  };
}

#endif
