#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"

#include "balloon.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>

// LTE has 20km radius on ground, 20km in the air, which yields 28.284km signal range from the balloon
// Special thanks to Hudson Bielstein for the computation
#define LTE_SIGNAL_RADIUS 28284.0
// ISM between balloons has 40km radius in air
#define ISM_SIGNAL_RADIUS 40000.0
// Balloon top speed is ~150 mph ~= 67 m/s
#define BALLOON_TOP_SPEED 67.0
// Length in seconds between updates of balloon position
#define BALLOON_POSITION_UPDATE_RATE 0.5
// heartbeat interval
#define BALLOON_HEARTBEAT_INTERVAL 0.1

NS_LOG_COMPONENT_DEFINE ("Balloon");

using namespace ns3;

ns3::Ptr<ns3::Node> node;
int num;

Balloon::Balloon() {
}

Balloon::Balloon(ns3::Ptr<ns3::Node> _node, int _num) {
  node = _node;
  num = _num;
}

void Balloon::setValues(ns3::Ptr<ns3::Node> _node, int _num) {
  node = _node;
  num = _num;
}

ns3::Ptr<ns3::Node> Balloon::getNode() {
  return node;
}

int Balloon::getNum() {
  return num;
}
