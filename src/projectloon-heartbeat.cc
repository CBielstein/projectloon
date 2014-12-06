/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 The Boeing Company
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

// ns3 includes
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"

// C/C++ includes
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <cassert>

// Our includes
#include "IPtoGPS.h"
#include "balloon.h"
#include "gateway.h"
#include "defines.h"
#include "loonheader.h"

NS_LOG_COMPONENT_DEFINE ("LoonHeartbeat");

using namespace ns3;
using namespace Loon;

// ** Define globals **

// Holds map for GPSR data
IPtoGPS map;
// Holds all nodes
LoonNode** loonnodes;
// Number of balloons in the simulation
unsigned int numBalloons;
// Number of gateways in the simulation
unsigned int numGateways;
// Total number of LoonNodes in the simulation (numBalloons+numGateways)
unsigned int numLoonNodes;
uint16_t otherPort = 88;
std::vector<Ptr<Socket>> sources;
std::vector<uint16_t> packetsReceivedPerLoon;
std::vector<uint16_t> packetsForwarded;
std::vector<uint16_t> packetsSent;


// ** Define helper functions **

// produces random jitter of [-25,25]% of the input
// seeds rand on the first call to this function
double Jitter(double input)
{
    static bool did_seed_rand = false;
    if (!did_seed_rand)
    {
        srand(time(NULL));
        did_seed_rand = true;
    }

    // create a random number [0,(2*JITTER_PERCENT)+1], then [-JITTER_PERCENT,JITTER_PERCENT], then as fraction
    double percentage = ((rand() % int((2.0 * JITTER_PERCENT) + 1)) - JITTER_PERCENT) / 100.0;

    // return the the input with the jttered percentage difference
    return input + (input * percentage);
}

// Function to handle receiving a heartbeat message
static void ReceiveHeartBeat(const struct HeartBeat& hb, LoonNode& balloon)
{
    if (!balloon.AddHeartBeat(hb.SenderId, hb))
    {
        NS_LOG(ns3::LOG_WARN, "Balloon " << balloon.GetId() << " failed to AddHeartBeat!");
    }
}

// Generic recieve function for testing
void ReceiveHeartBeatPacket(Ptr<Socket> socket)
{
    Ptr<Packet> pkt = NULL;
    uint32_t pkt_size = 0;
    uint32_t cpy_size = 0;
    struct HeartBeat* msg = (struct HeartBeat*)malloc(sizeof(struct HeartBeat));

    // find the matching balloon object to the socket
    uint32_t node_id = socket->GetNode()->GetId();
    LoonNode* balloon = NULL;
    for (unsigned int i = 0; i < numLoonNodes; ++i)
    {
        if (node_id == loonnodes[i]->GetId())
        {
            balloon = loonnodes[i];
            break;
        }
    } 

    // we can't do much without the balloon object
    if (balloon == NULL)
    {
        NS_LOG(ns3::LOG_WARN, "No balloon object found for id " << node_id);
        return;
    }
    
    while (pkt = socket->Recv())
    {
        pkt_size = sizeof(struct HeartBeat);
        cpy_size = pkt->CopyData((uint8_t*)msg, pkt_size);
        if (pkt_size != cpy_size)
        {
            NS_LOG(ns3::LOG_WARN, "pkt_size != cpy_size: " << pkt_size << " != " << cpy_size);
        }

        if (msg->SenderId == socket->GetNode()->GetId())
        {
            NS_LOG(ns3::LOG_DEBUG, "Hearing from myself! " << socket->GetNode()->GetId());
        }
        else
        {
            NS_LOG(ns3::LOG_DEBUG, "Received one packet at node " << socket->GetNode()->GetId()
                   << " From: " << msg->SenderId << std::endl << "    IsGateway: " << msg->is_gateway << ", HasConnection: "
                   << msg->has_connection << ", etx_gw: " << msg->etx_gw << std::endl << "    next hop: " << msg->gw_next_node
                   << ", Position: " << msg->position << ", Sender IP: " << msg->sender_ip << std::endl << "    Time: "
                   << msg->timestamp << ", #Ratios: " << msg->delivery_ratios->size());

            ReceiveHeartBeat(*msg, *balloon);
        }
    }
    free(msg);
}

Ipv4Address getAddrFromPacket(Ptr<Packet> packet) {
  // Peek at the header of the packet; we don't want to remove the header in case we need to
  // forward it elsewhere
  LoonHeader destinationHeader;
  packet->PeekHeader(destinationHeader);
  Ipv4Address dest = Ipv4Address(destinationHeader.GetDest());
  // Used for debugging, but not generally needed.
  // NS_LOG(ns3::LOG_DEBUG, "header data " << dest);
  return dest;
}

static Ptr<Packet> getNextHopPacket(Ptr<Packet> packet, Ptr<Node> currentNode, Ipv4Address dest) {
  LoonNode* current = loonnodes[currentNode->GetId()];
  bool hasNeighbor = current->HasNeighbor(dest);
  if (hasNeighbor) {
    NS_LOG(ns3::LOG_DEBUG, "Has neighbor");
    return packet;
  } else {
    // if we don't have the destination as our neighbor, let's pick GPSR or ETX and begin routing
    NS_LOG(ns3::LOG_DEBUG, "Does not have neighbor");

    // get the IP from the packet
    Ipv4Address dest_addr = getAddrFromPacket(packet);
    Ipv4Address nextHop;
    Vector3D dest_location;

    // if we have the IP addr, it's one of our balloons or clients and we should avoid the gateway overhead and use GPSR
    // else it's not on our network, so we're going to use the ETX route back to the gateway to send it to the internet
    if (map.GetMapping(dest_addr, dest_location)) {
        nextHop = current->GetNearestNeighborToDest(dest_location);
    } else {
        nextHop = current->GetAddress(current->GetNextHopId());
    }

    LoonHeader header;
    header.SetDest(nextHop.Get());
    LoonHeader oldHeader;
    packet->RemoveHeader(oldHeader);
    packet->AddHeader(header);
    return packet;
  }
}

void ReceiveGeneralPacket(Ptr<Socket> socket)
{
    while (Ptr<Packet> packet = socket->Recv())
    {
      Ipv4Address dest = getAddrFromPacket(packet);

      // Taken from loonnodes.cc; gets the address of the current socket
      Ptr<Node> node = socket->GetNode();
      Ptr<Ipv4> ipv4 = node->GetObject<ns3::Ipv4>();
      Ipv4InterfaceAddress iaddr = ipv4->GetAddress(1,0);
      Ipv4Address addr = iaddr.GetLocal();

      if (dest == addr) {
        NS_LOG(ns3::LOG_DEBUG, "GENERAL: Packet received at final destination, node " << node->GetId());
        packetsReceivedPerLoon[node->GetId()]++;
      } else {
        NS_LOG(ns3::LOG_DEBUG, "GENERAL: Received one packet at node " << socket->GetNode()->GetId() << " intended for " << addr);
        Ptr<Packet> packet2 = getNextHopPacket(packet, node, dest);
	packet2->RemoveAllPacketTags();
        sources[node->GetId()]->SendTo (packet2, 16, InetSocketAddress(getAddrFromPacket(packet2), otherPort));
	packetsForwarded[node->GetId()]++;
      }
    }
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize,pktCount-1, pktInterval);
      NS_LOG(ns3::LOG_DEBUG, pktCount-1 << " packets left");
      packetsSent[socket->GetNode()->GetId()]++;
    }
  else
    {
      socket->Close ();
    }
}

// address refers to node that you're sending to
static void GenerateTrafficSpecific (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval, Ipv4Address& address )
{
  if (pktCount > 0)
    {
      // not really sure what the flag is.... so I chose 16 lol
      int test = socket->SendTo (Create<Packet> (pktSize), 16, InetSocketAddress (address, 88));
      if (test == -1) {
        NS_LOG(ns3::LOG_DEBUG, "SendTo failed");
      }
      Simulator::Schedule (pktInterval, &GenerateTrafficSpecific,
                           socket, pktSize,pktCount-1, pktInterval, address);
      NS_LOG(ns3::LOG_DEBUG, pktCount-1 << " packets left to send to address "<< address);
      packetsSent[socket->GetNode()->GetId()]++;
    }
  else
    {
      socket->Close ();
    }
}

// Generates traffic where the destination may not be one of the sender's neighbors
static void GenerateTrafficMultiHop (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval, Ipv4Address& address )
{
  if (pktCount > 0)
    {
      Ptr<Packet> packet = Create<Packet>(pktSize);
      LoonHeader header;
      header.SetDest(address.Get());
      packet->AddHeader(header);

      // not really sure what the flag is.... so I chose 16 lol
      int test = socket->Send(packet);
      if (test == -1) {
        NS_LOG(ns3::LOG_DEBUG, "Send in GenerateTrafficMultiHop failed");
      }
      Simulator::Schedule (pktInterval, &GenerateTrafficMultiHop,
                           socket, pktSize,pktCount-1, pktInterval, address);
      NS_LOG(ns3::LOG_DEBUG, pktCount-1 << " packets left "<< address);
      packetsSent[socket->GetNode()->GetId()]++;
    }
  else
    {
      socket->Close ();
    }
}

// Send a regular heartbeat message
static void SendHeartBeat(Ptr<Socket> socket, LoonNode* balloon, Time hbInterval)
{
    if (balloon == NULL)
    {
        NS_LOG(ns3::LOG_WARN, "SendHeartBeat() passed NULL balloon");
        return;
    }

    // Get the HeartBeat
    struct HeartBeat hb = balloon->CreateHeartBeat();

    // Create Packet separately, in case we need to add anything to it...
    Ptr<Packet> packet = Create<Packet>((uint8_t*) &hb, sizeof(struct HeartBeat));

    // Send packet
    socket->Send(packet);
    // Schedule to do it again
    Simulator::Schedule(Seconds(Jitter(hbInterval.GetSeconds())), &SendHeartBeat, socket, balloon, hbInterval);
}

// I don't know why I have to do this...it should be done by them
static Vector3D Normalize(const Vector3D& v, double scale)
{
    Vector3D ret_val(v);

    // calculate length
    double length = CalculateDistance(Vector3D(0.0, 0.0, 0.0), ret_val);

    if (length == 0)
    {
        return Vector3D(0.0, 0.0, 0.0);
    }

    // normalize
    ret_val.x /= length;
    ret_val.y /= length;
    ret_val.z /= length;

    // scale
    ret_val.x *= scale;
    ret_val.y *= scale;
    ret_val.z *= scale;

    return ret_val;
}

struct BalloonMobility
{
    enum
    {
        SEEK_CONNECTION = 1,
        MAXIMUM_COVERAGE = 2,
        POINT_TO_POINT = 3
    };
};

// Takes the nodecontainers for the loonnodes and the gateways (on the ground) and finds the correct movement for each balloon
static void UpdateBalloonPositions(const unsigned int MobilityType)
{
    // ensure we aren't passed empty containers
    if (numBalloons  < 1 || numGateways < 1)
    {
        NS_LOG(ns3::LOG_ERROR, "Entered UpdateBalloonPositions with length: " << numBalloons << " and gateways size: " << numGateways);
        return;
    }

    if ( !(MobilityType == BalloonMobility::SEEK_CONNECTION
           || MobilityType == BalloonMobility::MAXIMUM_COVERAGE
           || BalloonMobility::POINT_TO_POINT))
    {
        NS_LOG(ns3::LOG_ERROR, "UpdateBalloonPosition() was passed bad MobilityType " << MobilityType);
        return;
    }

    // for each balloon, find closest gateway and move toward it at speed
    for (unsigned int i = 0; i < numLoonNodes; ++i)
    {
        // gateways don't move!
        if (loonnodes[i]->IsGateway())
        {
            continue;
        }

        // update position
        if(!map.UpdateMapping(loonnodes[i]->GetIpv4Addr(), loonnodes[i]->GetPosition()))
        {
            NS_LOG(ns3::LOG_WARN, "Failed to update position for IP address " << loonnodes[i]->GetIpv4Addr());
        }

        NS_LOG(ns3::LOG_DEBUG, "At " << Simulator::Now().GetSeconds () << ", node: " << loonnodes[i]->GetId()
                        << " @ Position: " << loonnodes[i]->GetPosition() << " w/  Speed:"
                        << loonnodes[i]->GetVelocity());

        if (MobilityType == BalloonMobility::SEEK_CONNECTION)
        {
            // if we already have connection, stay put
            if (loonnodes[i]->HasConnection())
            {
                loonnodes[i]->SetVelocity(Vector3D(0.0, 0.0, 0.0));
                continue;
            }

            // else, find the closest gateway and move toward it until we have connection
            LoonNode* nearest_gateway = NULL;
            double nearest_distance;
            double compare_distance;

            for (unsigned int j = 0; j < numLoonNodes; ++j)
            {
                // only compare to gateways that aren't us
                if (j == i || !loonnodes[j]->IsGateway())
                {
                    continue;
                }

                compare_distance = CalculateDistance(loonnodes[i]->GetPosition(), loonnodes[j]->GetPosition());

                // if this is closer, or if we don't already have a target gateway, pick this one
                if (compare_distance < nearest_distance || nearest_gateway == NULL)
                {
                    nearest_distance = compare_distance;
                    nearest_gateway = loonnodes[j];
                }
            }

            if (nearest_gateway == NULL)
            {
                NS_LOG(ns3::LOG_ERROR, "UpdateBalloonPosition() difn't find a nearest gateway for topology type SEEK_CONNECTION");
                return;
            }

            // find normalized direction
            Vector3D gateway_position = nearest_gateway->GetPosition();
            Vector3D balloon_position = loonnodes[i]->GetPosition();
            Vector3D direction(gateway_position.x - balloon_position.x, gateway_position.y - balloon_position.y, gateway_position.z - balloon_position.z);
            direction.z = 0; // don't move up or down
            direction = Normalize(direction, BALLOON_TOP_SPEED);

            // move in that direction at top speed
            loonnodes[i]->SetVelocity(direction);
        }
        else
        {
            NS_LOG(ns3::LOG_ERROR, "Tried routing mode which is not yet implemented");
            assert(0); // Not yet implemented.
        }
    }

    // schedule this to happen again in BALLOON_POSITION_UPDATE_RATE seconds
    Simulator::Schedule(Seconds(BALLOON_POSITION_UPDATE_RATE), &UpdateBalloonPositions, MobilityType);
}

// Creates a receiver!
static void createReceiver(Ptr<Node> receiver, uint16_t port) {
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (receiver, tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), port);
  recvSink->Bind (local);
  if (port == 80) {
    recvSink->SetRecvCallback (MakeCallback (&ReceiveHeartBeatPacket));
  } else {
    recvSink->SetRecvCallback(MakeCallback(&ReceiveGeneralPacket));
  }
}

static Ptr<Socket> createSender(Ptr<Node> sender, uint16_t port) {
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> source = Socket::CreateSocket (sender, tid);
  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), port);
  source->SetAllowBroadcast (true);
  source->Connect (remote);
  return source;
}

int main (int argc, char *argv[])
{
  // ** Basic setup **

  // Enable all logging for now, since this is a test
  LogComponentEnable("LoonHeartbeat", ns3::LOG_DEBUG);

  // Set basic variables
  std::string phyMode ("DsssRate1Mbps");
  double rss = -80;  // -dBm
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 30;
  numBalloons = 2;
  numGateways = 1;
  double interval = 1.0; // seconds
  bool verbose = false;
  uint16_t heartbeatPort = 80;

  // parse the command line
  CommandLine cmd;
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("rss", "received signal strength", rss);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue("numBalloons", "number of balloons", numBalloons);
  cmd.AddValue("numGateways", "number of gateways", numGateways);
  cmd.Parse (argc, argv);

  numLoonNodes = numBalloons + numGateways;
  
  packetsReceivedPerLoon = std::vector<uint16_t>(numLoonNodes, 0);
  packetsForwarded = std::vector<uint16_t>(numLoonNodes, 0);
  packetsSent = std::vector<uint16_t>(numLoonNodes, 0);

  // Convert to time object
  Time interPacketInterval = Seconds (interval);
  // disable fragmentation for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  // turn off RTS/CTS for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", 
                      StringValue (phyMode));


  // ** Start setting up the nodes for the simulation **

  NodeContainer loonNodesContainer;
  loonNodesContainer.Create(numLoonNodes);

  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
  if (verbose)
    {
      wifi.EnableLogComponents ();  // Turn on all Wifi logging
    }
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  // This is one parameter that matters when using FixedRssLossModel
  // set it to zero; otherwise, gain will be added
  wifiPhy.Set ("RxGain", DoubleValue (0) ); 
  // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO); 

  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  // The below FixedRssLossModel will cause the rss to be fixed regardless
  // of the distance between the two stations, and the transmit power
  wifiChannel.AddPropagationLoss ("ns3::FixedRssLossModel","Rss",DoubleValue (rss));
  wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(ISM_SIGNAL_RADIUS));
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add a non-QoS upper mac, and disable rate control
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));
  // Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, loonNodesContainer);

  // Set mobility model and initial positions
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  for (unsigned int i = 0; i < numGateways; ++i)
  {
    // TODO place gateways
  }
  for (unsigned int i = 0; i < numBalloons; ++i)
  {
    // TODO create randomized (or fixed for tests?) balloon positions
  }

  // TODO remove: for now, just set them fixed while we test so we know what they're doing
  // Node 0 (sender) IS the gateway
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  // Node 1 starts just out of range, moves into range after 2 packets have been sent
  positionAlloc->Add (Vector (-30200.0, 0.0, 20000.0));
  // Node 2 is on the gateway
  positionAlloc->Add (Vector (-35200.0, 0.0, 20000.0));

  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install(loonNodesContainer);

  InternetStackHelper internet;
  internet.Install (loonNodesContainer);
  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  // Tracing
  wifiPhy.EnablePcap ("wifi-simple-adhoc", devices);

  // create the array of Balloons
  // Create Receivers and senders
  std::vector<Ptr<Socket>> heartbeatSources;
  loonnodes = (LoonNode**)calloc(numLoonNodes, sizeof(LoonNode*));
  int status = EXIT_SUCCESS;
  for (unsigned int i = 0; i < numLoonNodes; ++i)
  {
    // Create sender sockets for heartbeat messages
    heartbeatSources.push_back(createSender(loonNodesContainer.Get(i), heartbeatPort));
    // Create sender sockets for other messages
    sources.push_back(createSender(loonNodesContainer.Get(i), otherPort));
    // Create reciever sockets for heartbeat messages
    createReceiver(loonNodesContainer.Get(i), heartbeatPort);
    // Create reciever sockets for other messages
    createReceiver(loonNodesContainer.Get(i), otherPort);

    // Add node to the loonnodes array
    if (i < numGateways)
    {
        loonnodes[i] = new Gateway(loonNodesContainer.Get(i));
    }
    else
    {
        loonnodes[i] = new Balloon(loonNodesContainer.Get(i));
    }

    status = map.AddMapping(loonnodes[i]->GetIpv4Addr(), loonnodes[i]->GetPosition());
    if (status != EXIT_SUCCESS)
    {
        NS_LOG(ns3::LOG_ERROR, "map.AddMapping(" << loonnodes[i]->GetIpv4Addr() << ", " << loonnodes[i]->GetPosition() << " returned " << status);
    }
  }


  // ** Schedule events **

  // update position and do movement
  Simulator::Schedule(Seconds(BALLOON_POSITION_UPDATE_RATE), &UpdateBalloonPositions, BalloonMobility::SEEK_CONNECTION);

  // activate heartbeats
  for (unsigned int i = 0; i < numLoonNodes; ++i)
  {
    // Schedule the event, with the jitter
    Simulator::Schedule(Seconds(Jitter(1.0)), &SendHeartBeat, heartbeatSources[i], loonnodes[i], Seconds(BALLOON_HEARTBEAT_INTERVAL));
  }

  // ** Generate broadcast traffic **
  //Simulator::ScheduleWithContext (sources[0]->GetNode ()->GetId (),
  //                                Seconds (1.0), &GenerateTraffic, 
  //                                sources[0], packetSize, numPackets, interPacketInterval);


  // ** Generate traffic to specific node, in this case, node 1 **
  //Simulator::ScheduleWithContext (sources[0]->GetNode ()->GetId (),
  //                                Seconds (1.0), &GenerateTrafficSpecific, 
  //                                sources[0], packetSize, numPackets, interPacketInterval, loonnodes[1]->GetIpv4Addr());

  // ** Generate traffic with a final destination in mind. In this case, the final destination is node 1 **
  // In our existing routing protocol using ETX, loonnodes[1] is in range of sources[0]. loonnodes[2] requires forwarding,
  // so we have set it here to illustrate the forwarding.
  Simulator::ScheduleWithContext (sources[0]->GetNode ()->GetId (),
                                  Seconds (1.0), &GenerateTrafficMultiHop, 
                                  sources[0], packetSize, numPackets, interPacketInterval, loonnodes[2]->GetIpv4Addr());
  // ** Begin the simulation **

  Simulator::Stop(Seconds(10));
  Simulator::Run ();

  NS_LOG(ns3::LOG_DEBUG, "--End Simulation--");
  NS_LOG(ns3::LOG_DEBUG, "Overall Summary");
  for (std::vector<uint16_t>::size_type i = 0; i != packetsReceivedPerLoon.size(); ++i) {
    NS_LOG(ns3::LOG_DEBUG, "node " << i << " at address " << loonnodes[i]->GetIpv4Addr()
           << " sent " << packetsSent[i] << " packets, "
           << "forwarded " << packetsForwarded[i] << " packets, and received " 
           << packetsReceivedPerLoon[i] << " as the final destination");
  }
  NS_LOG(ns3::LOG_DEBUG, "--Individual metrics--");
  NS_LOG(ns3::LOG_DEBUG, "Packets received at final destination: ");
  for (std::vector<uint16_t>::size_type i = 0; i != packetsReceivedPerLoon.size(); ++i) {
    NS_LOG(ns3::LOG_DEBUG, packetsReceivedPerLoon[i] << " packets received by node " << i
           << " with address " << loonnodes[i]->GetIpv4Addr());
  }
  NS_LOG(ns3::LOG_DEBUG, "--");
  NS_LOG(ns3::LOG_DEBUG, "Number of packets forwarded: ");
  for (std::vector<uint16_t>::size_type i = 0; i != packetsForwarded.size(); ++i) {
    NS_LOG(ns3::LOG_DEBUG, "node " << i << " forwarded " << packetsForwarded[i] << " packets");
  }
  NS_LOG(ns3::LOG_DEBUG, "--");
  NS_LOG(ns3::LOG_DEBUG, "Number of packets sent: ");
  for (std::vector<uint16_t>::size_type i = 0; i != packetsSent.size(); ++i) {
    NS_LOG(ns3::LOG_DEBUG, "node " << i << " sent " << packetsSent[i] << " packets");
  }


  // Don't destroy the simulator too early
  Simulator::Destroy ();

  // ** Clean up **
  for (unsigned int i = 0; i < numLoonNodes; ++i)
  {
    delete loonnodes[i];
  }
  free(loonnodes);

  // yay we're done
  return 0;
}

