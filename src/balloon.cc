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
