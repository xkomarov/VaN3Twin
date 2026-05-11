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
#include "trafficManagerClient80211p-helper.h"
#include "ns3/trafficManagerClient80211p.h"
#include "ns3/uinteger.h"
#include "ns3/names.h"

namespace ns3 {
trafficManagerClient80211pHelper::trafficManagerClient80211pHelper()
{
  m_factory.SetTypeId (trafficManagerClient80211p::GetTypeId ());
}

void
trafficManagerClient80211pHelper::SetAttribute (
  std::string name,
  const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
trafficManagerClient80211pHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
trafficManagerClient80211pHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
trafficManagerClient80211pHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
trafficManagerClient80211pHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<Application> app = m_factory.Create<trafficManagerClient80211p> ();
  node->AddApplication (app);

  return app;
}
}
