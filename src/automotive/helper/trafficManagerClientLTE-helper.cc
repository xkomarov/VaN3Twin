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
#include "trafficManagerClientLTE-helper.h"
#include "ns3/trafficManagerClientLTE.h"
#include "ns3/uinteger.h"
#include "ns3/names.h"

namespace ns3 {
trafficManagerClientLTEHelper::trafficManagerClientLTEHelper()
{
  m_factory.SetTypeId (trafficManagerClientLTE::GetTypeId ());
}

void
trafficManagerClientLTEHelper::SetAttribute (
  std::string name,
  const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
trafficManagerClientLTEHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
trafficManagerClientLTEHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
trafficManagerClientLTEHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
trafficManagerClientLTEHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<Application> app = m_factory.Create<trafficManagerClientLTE> ();
  node->AddApplication (app);

  return app;
}
}
