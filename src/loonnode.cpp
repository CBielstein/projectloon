#include "loonnode.h"

namespace Loon
{
    LoonNode::LoonNode()
    {
        node = NULL;
        etx_gw = 0;
        gw_next_node = 0;
        connected = false;
    }

    LoonNode::LoonNode(ns3::Ptr<ns3::Node> _node)
    {
        node = _node;
        etx_gw = 0;
        gw_next_node = 0;
        connected = false;
    }

    LoonNode::~LoonNode() {}

    uint32_t LoonNode::GetId() const
    {
        return node->GetId();
    }

    ns3::Ptr<ns3::Node> LoonNode::GetNode()
    {
        return node;
    }

    ns3::Vector3D LoonNode::GetPosition() const
    {
        return node->GetObject<ns3::MobilityModel>()->GetPosition();
    }

    uint32_t LoonNode::GetEtx() const
    {
        return etx_gw;
    }

    ns3::Ipv4Address LoonNode::GetIpv4Addr() const
    {
        // Taken from https://groups.google.com/forum/#!topic/ns-3-users/LKikAY2KnGQ
        // Not sure why it's always GetAddress(1,0), but it seems to work
        ns3::Ptr<ns3::Ipv4> ipv4 = node->GetObject<ns3::Ipv4>();
        ns3::Ipv4InterfaceAddress iaddr = ipv4->GetAddress(1,0);
        ns3::Ipv4Address addr = iaddr.GetLocal();

        return addr;
    }

    bool LoonNode::AddHeartBeat(const uint32_t& node_id, const struct HeartBeat& hb)
    {
        // Check if any entry for node_id exists
        std::map<uint32_t, struct Neighbor>::iterator entry = neighbors.find(node_id);
        if (entry == neighbors.end())
        {
            // if not, create it
            struct Neighbor nb;
            nb.ip_addr = hb.sender_ip;
            nb.is_gateway = hb.is_gateway;
            nb.has_connection = hb.has_connection;
            nb.gw_next_node = hb.gw_next_node;
            nb.etx_gw = hb.etx_gw;
            nb.forward_delivery_ratio = GetForwardDeliveryRatio(hb);
            nb.reverse_delivery_ratio = 0;
            nb.position = hb.position;
            nb.updated = hb.timestamp;
            nb.heartbeats.emplace(hb.timestamp);

            neighbors.emplace(node_id, nb);
        }
        else
        {
            // If it does, update
            entry->second.ip_addr = hb.sender_ip;
            entry->second.is_gateway = hb.is_gateway;
            entry->second.has_connection = hb.has_connection;
            entry->second.etx_gw = hb.etx_gw;
            entry->second.gw_next_node = hb.gw_next_node;
            entry->second.position = hb.position;
            entry->second.updated = hb.timestamp;
            entry->second.heartbeats.emplace(hb.timestamp);
            entry->second.forward_delivery_ratio = GetForwardDeliveryRatio(hb);
        }

        return true;
    }

    bool LoonNode::GetNeighbor(const uint32_t& node_id, struct Neighbor* neighbor)
    {
        // Check if any entry for node_id exists
        std::map<uint32_t, struct Neighbor>::iterator entry = neighbors.find(node_id);
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

    bool LoonNode::HasNeighbor(ns3::Ipv4Address addr)
    {
        // Check if any entry for node_id exists
        for (std::map<uint32_t, struct Neighbor>::iterator it = neighbors.begin(); it!=neighbors.end(); ++it)
        {
            Neighbor neighbor = it->second;
            if (neighbor.ip_addr == addr)
            {
                return true;
            }
        }

        return false;
    }

    struct HeartBeat LoonNode::CreateHeartBeat()
    {
        struct HeartBeat hb;

        // TODO this is a memory leak
        hb.delivery_ratios = new std::map<uint32_t, double>();
        bool found_connection = false;

        // iterate through neighbors
        std::map<uint32_t, struct Neighbor>::iterator itr;
        for (itr = neighbors.begin(); itr != neighbors.end(); ++itr)
        {
            // trim set of timestamps of received heartbeats
            itr->second.heartbeats.erase(itr->second.heartbeats.begin(),
                                         itr->second.heartbeats.lower_bound(ns3::Simulator::Now()
                                         - ns3::Time::FromInteger(BALLOON_ETX_MULTIPLE * BALLOON_HEARTBEAT_INTERVAL
                                         + (2.0 * JITTER_PERCENT / 100.0), ns3::Time::S)));

            // if there are none within the time window, we've loss touch with this neighbor, so erase from the table
            if (itr->second.heartbeats.size() == 0)
            {
                neighbors.erase(itr);
                continue;
            }

            // use loss ratio set to estimate loss ratio from neighbor
            itr->second.reverse_delivery_ratio = itr->second.heartbeats.size() / BALLOON_ETX_MULTIPLE;

            // if etx_gw + etx is smaller than my etx_gw, update my etx_gw
            // if we're a gateway, we don't need to update etx_gw, so we can save this time
            uint32_t test_etx = 0;
            if (!IsGateway() && itr->second.reverse_delivery_ratio != 0 && itr->second.forward_delivery_ratio != 0)
            {
                test_etx = (1.0 / (itr->second.reverse_delivery_ratio * itr->second.forward_delivery_ratio));

                // if the neighbor isn't a gateway, include their advertised etx to a gateway
                if (!itr->second.is_gateway)
                {
                    test_etx += itr->second.etx_gw;
                }

                // this is our new connection if we haven't found a connection OR this is a lower ETX than our existing connection
                // AND the other node is either a gateway or has connection to a gateawy
                // AND if this route does not refer to us as the next hop (avoid count to infinity)
                if ((test_etx < etx_gw || !found_connection) &&
                    (itr->second.has_connection || itr->second.is_gateway) &&
                    (itr->second.gw_next_node != GetId()))
                {
                    etx_gw = test_etx;
                    gw_next_node = itr->first;
                    found_connection = true;
                }
            }

            hb.delivery_ratios->emplace(itr->first, itr->second.reverse_delivery_ratio);
        }

        // update our connected status
        connected = IsGateway() ? true : found_connection;

        // create my heartbeat message
        hb.SenderId = GetId();
        hb.is_gateway = IsGateway();
        hb.has_connection = connected;
        hb.etx_gw = etx_gw;
        hb.gw_next_node = gw_next_node;
        hb.position = GetPosition();
        hb.sender_ip = GetIpv4Addr();
        hb.timestamp = ns3::Simulator::Now();

        return hb;
    }

    double LoonNode::GetForwardDeliveryRatio(const struct HeartBeat& hb) const
    {
        // get forward delivery ratio. If it's not included in the HeartBeat, return zero
        // do this by checking if our ID has an entry in the incoming delivery_ratios map
        std::map<uint32_t, double>::const_iterator itr = hb.delivery_ratios->find(GetId());
        if (itr == hb.delivery_ratios->end())
        {
            return 0.0;
        }
        else
        {
            return itr->second;
        }
    }

    bool LoonNode::HasConnection() const { return connected; }

    ns3::Vector3D LoonNode::GetVelocity() const
    {
        return node->GetObject<ns3::ConstantVelocityMobilityModel>()->GetVelocity();
    }

    bool LoonNode::SetVelocity(const ns3::Vector3D& velocity)
    {
        node->GetObject<ns3::ConstantVelocityMobilityModel>()->SetVelocity(velocity);
        return true;
    }
}
