#include "balloon.h"

Balloon::Balloon() {
    node = NULL;
    ett = 0;
}

Balloon::Balloon(ns3::Ptr<ns3::Node> _node) {
  node = _node;
  ett = 0;
}

ns3::Ptr<ns3::Node> Balloon::getNode() {
  return node;
}
