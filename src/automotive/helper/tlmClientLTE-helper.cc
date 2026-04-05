#include "tlmClientLTE-helper.h"

#include "ns3/tlmClientLTE.h"
#include "ns3/uinteger.h"
#include "ns3/names.h"

namespace ns3 {

tlmClientLTEHelper::tlmClientLTEHelper ()
{
  m_factory.SetTypeId (tlmClientLTE::GetTypeId ());
}


void 
tlmClientLTEHelper::SetAttribute (
  std::string name, 
  const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
tlmClientLTEHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
tlmClientLTEHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
tlmClientLTEHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
tlmClientLTEHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<Application> app = m_factory.Create<tlmClientLTE> ();
  node->AddApplication (app);

  return app;
}

} // namespace ns3
