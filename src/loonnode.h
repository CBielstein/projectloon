#ifndef LOONNODE_H
#define LOONNODE_H

#include "ns3/core-module.h"
#include "ns3/node.h"
#include "ns3/mobility-model.h"
#include "ns3/mobility-module.h"
#include "ns3/ipv4.h"
#include "ns3/nstime.h"

#include "defines.h"

#include <map>
#include <set>

namespace Loon
{

    // Holds information for heartbeats between balloons
    // This is a struct and not a class to ensure data is stored in memory together
    // so that it can be sent as a message
    struct HeartBeat
    {
        // Node ID of sender
        uint32_t SenderId;

        // True if this is advertising itself as a gateway
        bool is_gateway;

        // True if this is directly or indirectly connected to a gateway
        bool has_connection;

        // Advertised lowest ETX to a gateway
        uint32_t etx_gw;

        // Next hop on the path (used to eliminate count to infinity problems)
        uint32_t gw_next_node;

        // Position of the balloon when sent
        ns3::Vector3D position;

        // Ipv4Address to send from
        ns3::Ipv4Address sender_ip;

        // Time stamp of sent message (used for ETX calculations)
        ns3::Time timestamp;

        // Maps node IDs to loss ratios reported by neighbors
        std::map<uint32_t, double>* delivery_ratios;
    };

    // A struct to hold information overheard from neighbors
    struct Neighbor
    {
        // The IP address of the neighboring node
        ns3::Ipv4Address ip_addr;

        // True if this is advertising itself as a gateway
        bool is_gateway;

        // True if this is directly or indirectly connected to a gateway
        bool has_connection;

        // The lowest etx to a gateway advertised by the neighboring node
        uint32_t etx_gw;

        // Id ofthe next hop on the path (used to eliminate count to infinity problems)
        uint32_t gw_next_node;

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
    class LoonNode
    {
        public:
            LoonNode();
            virtual ~LoonNode();

            // Node Constructor
            // Constructs a new balloon object
            // Args
            //  [IN] node: the node to build our balloon around
            LoonNode(ns3::Ptr<ns3::Node> _node);

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

            // GetNextHopId
            // Returns the id of the next hop (based on lowest ETX)
            uint32_t GetNextHopId() const;

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

            // HasNeighbor
            // Search for existance of a neighbor given an Ipv4Address
            // Args
            //  [IN] addr: Ipv4Address of the destination of a packet
            // Return
            //  bool: true if we find the neighbor, false if not
            bool HasNeighbor(ns3::Ipv4Address addr);

            // GetNearestNearestNeighborToDest
            // Determine which neighbor node is closest to the destination node
            // Args
	    //   [IN] destinationPosition: Vector containing position of destination
	    // Return
	    //   Ipv4Address: the address of the node that is nearest to the destination node
            ns3::Ipv4Address GetNearestNeighborToDest(ns3::Vector3D destinationPosition);

	    // GetNextPerimeterNode
	    // Determine the next node to send to for perimeter routing
	    // Args
	    // [IN] destinationPosition: Vector containing position of destination
	    // Return
	    //	  Ipv4Address: the address of the next node to route to
	    ns3::Ipv4Address GetNextPerimeterNode(ns3::Vector3D destionationPosition);

            // CreateHeartBeat
            // Creates a heartbeat message to be sent
            // includes updating etx and neighbor fields for now, though this may be too much on every interval
            struct HeartBeat CreateHeartBeat();

            // IsGateway
            // Used to determine if this node is a gateway
            // Return
            //  true if the node is a gateway, false if not
            virtual bool IsGateway() = 0;

            // HasConnection
            // returns if this current node has a connection
            // either directly to a gateawy or through hops
            // Returns
            //  true if the node has connection
            bool HasConnection() const;

            // GetAddress
            // returns the Ipv4Address of a given node id. If the provided node id doesn't correspond
            // to one of the node's neighbors, it will return Ipv4Address::GetAny().
            // Args
            //   [IN] nodeId: Node id to look up
            // Return
            //  Ipv4Address of the provided node id.
            ns3::Ipv4Address GetAddress(uint32_t nodeId);


            // GetVelocity
            // returns the current velocity of the underlying node
            // This assumes we're using a ConstantVelocityMobilityModel
            // Return
            //  current velocity set on the node
            ns3::Vector3D GetVelocity() const;

            // SetVelocity
            // Sets the velocity of the node. Some children (currently gateway)
            // override this to not set a velocity, since gateways do not move in our model
            // This assumes we're using a ConstantVelocityMobilityModel
            // Args
            //  [IN] velocity: the velocity to set on the node
            // Return
            //  true if the value was set, false if not
            virtual bool SetVelocity(const ns3::Vector3D& velocity);

        protected:
            // True if we have a connection, false if not
            // protected so gateway can access
            bool connected;

        private:
            // GetForwardDeliveryRatio
            // Finds the forward delivery ratio for this balloon given a heartbeat from a neighboring balloon
            // Args
            //  [IN] hb: a heartbeat from a (neighboring) node
            // Returns the value listed in the delivery_ratios map of hb for the node id of this balloon
            //  or zero if it does not appear in the table
            double GetForwardDeliveryRatio(const struct HeartBeat& hb) const;

            // the ns3::node that relates to this node
            ns3::Ptr<ns3::Node> node;

            // the lowest ETX from this balloon to a gateway
            uint32_t etx_gw;

            // The id of the node with lowest ETX from this balloon to for the gateway
            uint32_t gw_next_node;

            // Map to information about my neighbors
            std::map<uint32_t, struct Neighbor> neighbors;
    };
}

#endif
