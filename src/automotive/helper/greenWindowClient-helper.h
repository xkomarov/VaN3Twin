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
#ifndef greenWindowCLIENT_HELPER_H
#define greenWindowCLIENT_HELPER_H


#include <stdint.h>
#include "ns3/application-container.h"
#include "ns3/node-container.h"
#include "ns3/object-factory.h"







namespace ns3 {


class greenWindowClientHelper
{
public:
  greenWindowClientHelper ();

  void SetAttribute (std::string name, const AttributeValue &value);

  
  ApplicationContainer Install (Ptr<Node> node) const;

  
  ApplicationContainer Install (std::string nodeName) const;

  
  ApplicationContainer Install (NodeContainer c) const;

private:
  
  Ptr<Application> InstallPriv (Ptr<Node> node) const;
  ObjectFactory m_factory; 
};

} 

#endif 
