#ifndef BALLOON_H
#define BALLOON_H

#include "ns3/core-module.h"
#include "ns3/node.h"
#include "ns3/mobility-model.h"
#include "ns3/ipv4.h"
#include "ns3/nstime.h"

#include "defines.h"

#include <map>
#include <set>

// Holds information for heartbeats between balloons
struct HeartBeat
{
    // Node ID of sender
    uint32_t SenderId;

    // Advertised lowest ETX to a gateway
    uint32_t etx_gw;

    // Position of the balloon when sent
    ns3::Vector3D position;

    // Ipv4Address to send from
    ns3::Ipv4Address sender_ip;

    // Time stamp of sent message (used for ETX calculations)
    ns3::Time timestamp;

    // Maps node IDs to loss ratios reported by neighbors
    std::map<uint32_t, double> delivery_ratios;
};

// A struct to hold information overheard from neighbors
struct Neighbor
{
    // The IP address of the neighboring node
    ns3::Ipv4Address ip_addr;

    // The lowest etx to a gateway advertised by the neighboring node
    unsigned int etx_gw;

    // Delivery ratio from this neighbor
    double reverse_delivery_ratio;

    // Delivery ratio to this neighbor (as reported by the neighbor)
    double forward_delivery_ratio; 

    // The latest position advertised by that node
    ns3::Vector3D position;

    // TimeStamp of latest update to this structure
    ns3::Time updated;

    // A set of timestamps of received heartbeats for loss ratio calculation
    std::set<ns3::Time> heartbeats;
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

        // GetEtx
        // Returns the lowest ETX to a gateway
        uint32_t GetEtx() const;

        // GetIpv4Addr
        // Returns the address of the Ipv4 interface on the node
        ns3::Ipv4Address GetIpv4Addr() const;

        // AddHeartBeat
        // Updates the neighbor struct to include a heartbeat from a neighbor
        // Args
        //  [IN] node_id: the node id of the node to modify an entry for
        //  [IN] hb: the neighbor's information in a HeartBeat struct
        // return
        //  bool: true if successful, false if not
        bool AddHeartBeat(const uint32_t& node_id, const struct HeartBeat& hb);

        // GetNeighbor
        // Search for and retrieve an entry in the neighbors map
        // Args
        //  [IN] node_id: the node id of the neighbor to look up
        //  [OUT] neighbor: if this is non-NULL and we find the neighbor, copy the struct to this pointer
        // Return
        //  bool: true if we find the neighbor, false if not
        bool GetNeighbor(const uint32_t& node_id, struct Neighbor* neighbor);

        // CreateHeartBeat
        // Creates a heartbeat message to be sent
        // includes updating etx and neighbor fields for now, though this may be too much on every interval 
        struct HeartBeat CreateHeartBeat();

    private:
        // GetForwardDeliveryRatio
        // Finds the forward delivery ratio for this balloon given a heartbeat from a neighboring balloon
        // Args
        //  [IN] hb: a heartbeat from a (neighboring) node
        // Returns the value listed in the delivery_ratios map of hb for the node id of this balloon
        //  or zero if it does not appear in the table
        double GetForwardDeliveryRatio(const struct HeartBeat& hb) const;

        // the node that relates to this Balloon 
        ns3::Ptr<ns3::Node> node;

        // the lowest ETX from this balloon to a gateway
        uint32_t etx;

        // The next node to hop to for the gateway
        int32_t etx_next_node;

        // Map to information about my neighbors
        std::map<uint32_t, struct Neighbor> neighbors;
};

#endif
