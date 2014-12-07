#include "loonnode.h"

namespace Loon
{
    const char* LoonNodeTypeName(const LoonNodeType& type)
    {
        switch (type)
        {
            case LoonNodeType::BALLOON: return "Balloon";
            case LoonNodeType::GATEWAY: return "Gateway";
            case LoonNodeType::CLIENT: return "Client";
            default: return "Invalid Type.";
        }
    }

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

    uint32_t LoonNode::GetNextHopId() const
    {
        return gw_next_node;
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
            nb.node_type = hb.node_type;
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
            // If it does, update if it's newer than our previous update
            if (entry->second.updated < hb.timestamp)
            {
                entry->second.ip_addr = hb.sender_ip;
                entry->second.node_type = hb.node_type;
                entry->second.has_connection = hb.has_connection;
                entry->second.etx_gw = hb.etx_gw;
                entry->second.gw_next_node = hb.gw_next_node;
                entry->second.forward_delivery_ratio = GetForwardDeliveryRatio(hb);
                entry->second.position = hb.position;
                entry->second.updated = hb.timestamp;
            }

            // but always add the timestamp
            entry->second.heartbeats.emplace(hb.timestamp);
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

    ns3::Ipv4Address LoonNode::GetAddress(uint32_t nodeId)
    {
        // Check if any entry for node_id exists
        for (std::map<uint32_t, struct Neighbor>::iterator it = neighbors.begin(); it!=neighbors.end(); ++it)
        {
            Neighbor neighbor = it->second;
            if (neighbor.gw_next_node == nodeId)
            {
                return neighbor.ip_addr;
            }
        }
        return ns3::Ipv4Address::GetAny();
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

    ns3::Ipv4Address LoonNode::GetNearestNeighborToDest(ns3::Vector3D destinationPosition)
    {
        ns3::Ipv4Address nearestNeighbor;
        nearestNeighbor = this->GetIpv4Addr();
        double closestDistance = CalculateDistance(destinationPosition, this->GetPosition());
        for(std::map<uint32_t, struct Neighbor>::iterator it = neighbors.begin(); it!=neighbors.end(); ++it)
        {
            Neighbor neighbor = it->second;
            ns3::Vector3D neighbPosition = neighbor.position;
            double distance = CalculateDistance(destinationPosition, neighbPosition);
            if(distance < closestDistance)
            {
		nearestNeighbor = neighbor.ip_addr;
		closestDistance = distance;
	    }
        }

        return nearestNeighbor;
    }

    ns3::Ipv4Address LoonNode::GetNextPerimeterNode(ns3::Vector3D destinationPosition, ns3::Vector3D startPosition)
    {
	// This will start perimeter routing.  For now, just returning this until perimeter routing works....
	double startToDestX = destinationPosition.x - startPosition.x;
	double startToDestY = destinationPosition.y - startPosition.y;
	double startToDestZ = destinationPosition.z - startPosition.z;

	ns3::Vector3D startToDestVector = ns3::Vector3D(startToDestX, startToDestY, startToDestZ);
	std::vector<Neighbor> planarNeighbors = GetPlanarNeighbors();
	double angleToRightHandNeighbor = 359;	
	ns3::Ipv4Address nextAddress = this->GetIpv4Addr();

	for(unsigned i = 0; i < planarNeighbors.size(); ++i)
	{
	  Neighbor current = planarNeighbors.at(i);
	  ns3::Vector3D currentPosition = current.position;
	  double startToCurrentX = currentPosition.x - startPosition.x;
	  double startToCurrentY = currentPosition.y - startPosition.y;
	  double startToCurrentZ = currentPosition.z - startPosition.z;
	  ns3::Vector3D startToCurrentVector = ns3::Vector3D(startToCurrentX, startToCurrentY, startToCurrentZ);

	  double dotProduct = (startToCurrentVector.x * startToDestVector.x) + (startToCurrentVector.y * startToDestVector.y) + (startToCurrentVector.z * startToDestVector.z);
	  double magnitude1 = CalculateDistance(startPosition, destinationPosition);
	  double magnitude2 = CalculateDistance(startPosition, currentPosition);
	  double angle = dotProduct/(magnitude1 * magnitude2);
	  if(angle < angleToRightHandNeighbor && angle > 0)
	  {
		angleToRightHandNeighbor = angle;
		nextAddress = current.ip_addr;
	  } 
	}

	return nextAddress;
    }

    std::vector<Neighbor> LoonNode::GetPlanarNeighbors()
    {
	ns3::Vector3D uPosition = this->GetPosition();
	std::vector<Neighbor> planarNeighbors;
	for(std::map<uint32_t, struct Neighbor>::iterator it = neighbors.begin(); it!=neighbors.end(); ++it)
	{
	    Neighbor v = it->second;
	    bool canAdd = true;
	    for(std::map<uint32_t, struct Neighbor>::iterator it2 = neighbors.begin(); it2!=neighbors.end(); ++it)
	    {
		Neighbor w = it2->second;
		if((w.ip_addr).IsEqual(v.ip_addr))
		{
		  continue;
		}
		else
		{
		  // Calculate distances and eliminate ones that are too long
		  double uv = CalculateDistance(uPosition, v.position);
		  double uw = CalculateDistance(uPosition, w.position);
		  double vw = CalculateDistance(v.position, w.position);
		  if(uv > uw || uv > vw)
		  {
		    canAdd = false;
	  	    break;
		  }
		}
		if(canAdd)
		{
		  planarNeighbors.push_back(v);
		}
	    }
	}
	return planarNeighbors;
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
            // exclude clients in the routing to gateways
            uint32_t test_etx = 0;
            if ((GetType() != LoonNodeType::GATEWAY) && (itr->second.node_type != LoonNodeType::CLIENT) &&
                itr->second.reverse_delivery_ratio != 0 && itr->second.forward_delivery_ratio != 0)
            {
                test_etx = (1.0 / (itr->second.reverse_delivery_ratio * itr->second.forward_delivery_ratio));

                // if the neighbor isn't a gateway, include their advertised etx to a gateway
                if (itr->second.node_type != LoonNodeType::GATEWAY)
                {
                    test_etx += itr->second.etx_gw;
                }

                // this is our new connection if we haven't found a connection OR this is a lower ETX than our existing connection
                // AND the other node is either a gateway or has connection to a gateawy
                // AND if this route does not refer to us as the next hop (avoid count to infinity)
                // AND the other node is not a client (avoid routing through clients)
                if ((test_etx < etx_gw || !found_connection) &&
                    (itr->second.has_connection || (itr->second.node_type == LoonNodeType::GATEWAY)) &&
                    (itr->second.gw_next_node != GetId()) && (itr->second.node_type != LoonNodeType::CLIENT))
                {
                    etx_gw = test_etx;
                    gw_next_node = itr->first;
                    found_connection = true;
                }
            }

            hb.delivery_ratios->emplace(itr->first, itr->second.reverse_delivery_ratio);
        }

        // update our connected status
        connected = (GetType() == LoonNodeType::GATEWAY) ? true : found_connection;

        // create my heartbeat message
        hb.SenderId = GetId();
        hb.node_type = GetType();
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
