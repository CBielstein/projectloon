#include "balloon.h"

Balloon::Balloon() {
    node = NULL;
    ett = 0;
}

Balloon::Balloon(ns3::Ptr<ns3::Node> _node) {
  node = _node;
  ett = 0;
}

uint32_t Balloon::GetId() const {
  return node->GetId();
}

ns3::Ptr<ns3::Node> Balloon::GetNode() {
  return node;
}

ns3::Vector3D Balloon::GetPosition() const {
  return node->GetObject<ns3::MobilityModel>()->GetPosition();
}

unsigned int Balloon::GetEtt() const {
  return ett;
}

ns3::Ipv4Address Balloon::GetIpv4Addr() const {

  // Taken from https://groups.google.com/forum/#!topic/ns-3-users/LKikAY2KnGQ
  // Not sure why it's always GetAddress(1,0), but it seems to work
  ns3::Ptr<ns3::Ipv4> ipv4 = node->GetObject<ns3::Ipv4>();
  ns3::Ipv4InterfaceAddress iaddr = ipv4->GetAddress(1,0);
  ns3::Ipv4Address addr = iaddr.GetLocal();

  return addr;
}
