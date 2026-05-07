#include "bglosaServer-helper.h"
#include "ns3/bglosaServer.h"
// #include "ns3/uinteger.h"
#include "ns3/names.h"

namespace ns3 {

bglosaServerHelper::bglosaServerHelper ()
{
  m_factory.SetTypeId (bglosaServer::GetTypeId ());
}


void 
bglosaServerHelper::SetAttribute (
  std::string name, 
  const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
bglosaServerHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
bglosaServerHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
bglosaServerHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
bglosaServerHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<Application> app = m_factory.Create<bglosaServer> ();
  node->AddApplication (app);

  return app;
}

} // namespace ns3
