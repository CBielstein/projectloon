#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>

using namespace ns3;

class Balloon {
  public:
    Balloon();
    Balloon(ns3::Ptr<ns3::Node>, int);
    void setValues(ns3::Ptr<ns3::Node>, int);
    ns3::Ptr<Node> getNode();
    int getNum();
};
