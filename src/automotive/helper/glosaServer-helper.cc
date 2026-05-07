#include "glosaServer-helper.h"
#include "ns3/glosaServer.h"
// #include "ns3/uinteger.h"
#include "ns3/names.h"

namespace ns3 {

glosaServerHelper::glosaServerHelper ()
{
  m_factory.SetTypeId (glosaServer::GetTypeId ());
}


void 
glosaServerHelper::SetAttribute (
  std::string name, 
  const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
glosaServerHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
glosaServerHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
glosaServerHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
glosaServerHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<Application> app = m_factory.Create<glosaServer> ();
  node->AddApplication (app);

  return app;
}

} // namespace ns3
