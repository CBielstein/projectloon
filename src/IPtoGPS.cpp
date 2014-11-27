// Bielstein, Lee, Siu
// IPtoGPS.cpp
// Fall 2014

#include "IPtoGPS.h"

int IPtoGPS::AddMapping(const ns3::Ipv4Address& address, const ns3::Vector3D& coords)
{
    int status = EXIT_SUCCESS;

    // ensure there's remaining room
    if (map.size() == map.max_size())
    {
        status = MAP_FULL;
        return status;
    }

    // get address as it is to be stored
    uint32_t addr = address.Get();

    // add
    std::pair<std::unordered_map<uint32_t, ns3::Vector3D>::iterator, bool>  ret_pair = map.emplace(addr, coords);

    // if this is false, map.emplace reported a duplicate
    if (ret_pair.second == false)
    {
        status = MAP_DUPLICATE;
        return status;
    }
    
    return status;
}

bool IPtoGPS::HasIPAddr(const ns3::Ipv4Address& address) const
{
    // Get addr as stored in the map
    uint32_t addr = address.Get();

    // count instances in map
    if (map.count(addr) > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool IPtoGPS::GetMapping(const ns3::Ipv4Address& address, ns3::Vector3D& coords) const
{
    // get addr as it is in the map
    uint32_t addr = address.Get();

    // find pair in map
    std::unordered_map<uint32_t, ns3::Vector3D>::const_iterator pair = map.find(addr); 

    // if == end, it wasn't found
    if (pair == map.end())
    {
        return false;
    }
    else
    {
        // return the coords
        coords = pair->second;
        return true;
    } 
}

bool IPtoGPS::UpdateMapping(const ns3::Ipv4Address& address, const ns3::Vector3D& coords)
{
    // get addr as it is in the map
    uint32_t addr = address.Get();

    // search for the pair in the map
    std::unordered_map<uint32_t, ns3::Vector3D>::iterator pair = map.find(addr);
    
    // if == end, it wasn't found
    if (pair == map.end())
    {
        return false;
    }

    // update the entry
    pair->second = coords;
    return true;
}

// RemoveMapping
// Find and remove the mapping for the given IP address
// Arguments
//      [IN] IP: the IP address of the mapping to remove
// returns EXIT_SUCCESS upon success
bool IPtoGPS::RemoveMapping(const ns3::Ipv4Address& address)
{
    // get addr as it is in the map
    uint32_t addr = address.Get();

    // finds and erases addr, returns count of erases (binary for a map)
    size_t erased = map.erase(addr);

    // return if we erased anything or not
    if (erased > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}
