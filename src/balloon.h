#ifndef BALLOON_H
#define BALLOON_H

#include "ns3/core-module.h"
#include "ns3/node.h"
#include "ns3/mobility-model.h"
#include "ns3/ipv4.h"
#include <unordered_map>

// A struct to hold information overheard from neighbors
struct Neighbor
{
    // The IP address of the neighboring node
    ns3::Ipv4Address ip_addr;

    // The lowest ett to a gateway advertised by the neighboring node
    unsigned int gateway_ett;

    // The latest position advertised by that node
    ns3::Vector3D position;
};

// This class accompanies the nodes in a NodeContainer.
// Sadly, it seemed nearly impossible to extend ns3::Node, so we did this to save time. 
class Balloon
{
    public:
        Balloon();
    
        // Balloon Constructor
        // Constructs a new balloon object
        // Args
        //  [IN] node: the node to build our balloon around
        Balloon(ns3::Ptr<ns3::Node> _node);
    
        // GetNode
        // Returns a mutable pointer to the node we're built around
        ns3::Ptr<ns3::Node> GetNode();

        // GetPosition
        // Returns the position of the MobilityModel on the node
        ns3::Vector3D GetPosition() const;

        // GetId
        // Returns the ID of the node
        uint32_t GetId() const; 

        // GetEtt
        // Returns the lowest ETT to a gateway
        unsigned int GetEtt() const;

        // GetIpv4Addr
        // Returns the address of the Ipv4 interface on the node
        ns3::Ipv4Address GetIpv4Addr() const;

        // SetNeighbor
        // Add or update a neighbor entry in our neighbors vector
        // Args
        //  [IN] node_id: the node id of the node to modify an entry for
        //  [IN] neighbor: the neighbor's information in a Neighbor struct
        // return
        //  bool: true if successful, false if not
        bool SetNeighbor(const uint32_t& node_id, const struct Neighbor& neighbor);

        // GetNeighbor
        // Search for and retrieve an entry in the neighbors map
        // Args
        //  [IN] node_id: the node id of the neighbor to look up
        //  [OUT] neighbor: if this is non-NULL and we find the neighbor, copy the struct to this pointer
        // Return
        //  bool: true if we find the neighbor, false if not
        bool GetNeighbor(const uint32_t& node_id, struct Neighbor* neighbor);

    private:
        // the node that relates to this Balloon 
        ns3::Ptr<ns3::Node> node;

        // the lowest ETT from this balloon to a gateway
        unsigned int ett;

        // Map to information about my neighbors
        std::unordered_map<uint32_t, struct Neighbor> neighbors;
};

#endif
