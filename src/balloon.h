#ifndef BALLOON_H
#define BALLOON_H

#include "ns3/core-module.h"
#include "ns3/node.h"

// This class accompanies the nodes in a NodeContainer.
// Sadly, it seemed nearly impossible to extend ns3::Node, so we did this to save time. 
class Balloon {
  public:
    // Default constructor
    Balloon();
    
    // Constructor. Takes a node and an int
    Balloon(ns3::Ptr<ns3::Node>);
    
    // Returns a pointer to the node. Non-const, so don't mess it up.
    ns3::Ptr<ns3::Node> getNode();

  private:
    // the node that relates to this Balloon 
    ns3::Ptr<ns3::Node> node;

    // the lowest ETT from this balloon to a gateway
    unsigned int ett;
};

#endif
