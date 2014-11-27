// Bielstein, Lee, Siu
// IPtoGPS.h
// Fall 2014

#ifndef IPTOGPS_H
#define IPTOGPS_H

#include "ns3/core-module.h"
#include "ns3/ipv4-address.h"
#include "ns3/vector.h"

#include <unordered_map>
#include <iostream> //iterator

// Class IPtoGPS maps IPv4 addresses to GPS coordinates
// This was written to help with routing in Project Loon
class IPtoGPS
{
    public:
        // errors that could happen in the AddMapping function
        enum Errors { MAP_FULL = -2, MAP_DUPLICATE = -3 }; 

        // AddMapping
        // Adds the pair to the map
        // Arguments
        //      [IN] address: the internet protocol address of the mapping
        //      [IN] coords: the GPS coordinates of the mapping
        // Returns EXIT_SUCCESS on success, else an error from Errors
        int AddMapping(const ns3::Ipv4Address& address, const ns3::Vector3D& coords);

        // HasIPAddr
        // Checks for the existence of a pair with the given IP address.
        // Arguments
        //      [IN] address: the IP address to search for
        // returns true if the map contains a pair for the IP address
        bool HasIPAddr(const ns3::Ipv4Address& address) const;

        // GetMapping
        // Finds the mapping for the given IP address
        // Arguments
        //      [IN] IP: the IP address to search for
        //      [OUT] coords: the coordinates that match the IP address
        // returns true if the coordinates were found and returned, false if not
        bool GetMapping(const ns3::Ipv4Address& address, ns3::Vector3D& coords) const;

        // UpdateMapping
        // Findsm and updates the mapping for a given IP address to the given GPS coords
        // Arguments
        //      [IN] address: the IP address for the entry
        //      [IN] coords: the new GPS coordinates to set for the entry
        // returns EXIT_SUCCESS upon success
        bool UpdateMapping(const ns3::Ipv4Address& address, const ns3::Vector3D& coords); 

        // RemoveMapping
        // Find and remove the mapping for the given IP address
        // Arguments
        //      [IN] IP: the IP address of the mapping to remove
        // returns EXIT_SUCCESS upon success
        bool RemoveMapping(const ns3::Ipv4Address& address); 

    private:
        // the actual map to hold everything
        // using std::unordered_map enables associative storage,
        // but a faster lookup than std::map
        // maps uint32_t (the underlying IP data of ns3::Ipv4Address
        std::unordered_map<uint32_t, ns3::Vector3D> map;
};

#endif
