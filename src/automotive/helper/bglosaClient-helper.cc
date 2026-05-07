#include "bglosaClient-helper.h"
#include "ns3/bglosaClient.h"
// #include "ns3/uinteger.h"
#include "ns3/names.h"

namespace ns3 {

bglosaClientHelper::bglosaClientHelper ()
{
  m_factory.SetTypeId (bglosaClient::GetTypeId ());
}


void 
bglosaClientHelper::SetAttribute (
  std::string name, 
  const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
bglosaClientHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
bglosaClientHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
bglosaClientHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
bglosaClientHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<Application> app = m_factory.Create<bglosaClient> ();
  node->AddApplication (app);

  return app;
}

} // namespace ns3
