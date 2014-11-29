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

bool Balloon::SetNeighbor(const uint32_t& node_id, const struct Neighbor& neighbor)
{
    // Check if any entry for node_id exists
    std::unordered_map<uint32_t, struct Neighbor>::iterator entry = neighbors.find(node_id);
    if (entry == neighbors.end()) {
        // if not, create it
        neighbors.emplace(node_id, neighbor);
    }
    else
    {
        // If it does, update
        entry->second = neighbor;
    }

    return true;
}

bool Balloon::GetNeighbor(const uint32_t& node_id, struct Neighbor* neighbor)
{
    // Check if any entry for node_id exists
    std::unordered_map<uint32_t, struct Neighbor>::iterator entry = neighbors.find(node_id);
    if (entry == neighbors.end())
    {
        // if not, return false
        return false;
    }
    else
    {
        // If it does, place it in a non-NULL neighbor ptr
        // and return true 
        if (neighbor != NULL)
        {
            *neighbor = entry->second;
        }
        
        return true;
    }
}
