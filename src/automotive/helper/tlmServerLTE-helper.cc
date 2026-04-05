#include "tlmServerLTE-helper.h"

#include "ns3/tlmServerLTE.h"
#include "ns3/uinteger.h"
#include "ns3/names.h"

namespace ns3 {

tlmServerLTEHelper::tlmServerLTEHelper ()
{
  m_factory.SetTypeId (tlmServerLTE::GetTypeId ());
}


void 
tlmServerLTEHelper::SetAttribute (
  std::string name, 
  const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
tlmServerLTEHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
tlmServerLTEHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
tlmServerLTEHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
tlmServerLTEHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<Application> app = m_factory.Create<tlmServerLTE> ();
  node->AddApplication (app);

  return app;
}

} // namespace ns3
