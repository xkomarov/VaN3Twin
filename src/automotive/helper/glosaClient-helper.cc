#include "glosaClient-helper.h"
#include "ns3/glosaClient.h"
// #include "ns3/uinteger.h"
#include "ns3/names.h"

namespace ns3 {

glosaClientHelper::glosaClientHelper ()
{
  m_factory.SetTypeId (glosaClient::GetTypeId ());
}


void 
glosaClientHelper::SetAttribute (
  std::string name, 
  const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
glosaClientHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
glosaClientHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
glosaClientHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
glosaClientHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<Application> app = m_factory.Create<glosaClient> ();
  node->AddApplication (app);

  return app;
}

} // namespace ns3
