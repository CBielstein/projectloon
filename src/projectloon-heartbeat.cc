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
#include "ns3/lte-module.h"

// C/C++ includes
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <time.h>

// Our includes
#include "IPtoGPS.h"
#include "balloon.h"
#include "gateway.h"
#include "defines.h"

NS_LOG_COMPONENT_DEFINE ("LoonHeartbeat");

using namespace ns3;

// Define globals


IPtoGPS map;
NetDeviceContainer ueDevs;
NetDeviceContainer enbDevs;
Ptr<LteHelper> lteHelper;
Balloon** balloons;
unsigned int numBalloons; 
uint16_t otherPort = 88;

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

    // create a random number [0,50], then [-25,25], then [-.25,.25]
    double percentage = ((rand() % 51) - 25) / 100.0;

    // return the the input with the jttered percentage difference
    return input + (input * percentage);
}

// Function to handle receiving a heartbeat message
static void ReceiveHeartBeat(const struct HeartBeat& hb, Balloon& balloon)
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
    Balloon* balloon = NULL;
    for (unsigned int i = 0; i < numBalloons; ++i)
    {
        if (node_id == balloons[i]->GetId())
        {
            balloon = balloons[i];
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
                   << msg->timestamp << ", #Ratios: " << msg->delivery_ratios.size());

            ReceiveHeartBeat(*msg, *balloon);
        }
    }
    free(msg);
}

void ReceiveGeneralPacket(Ptr<Socket> socket)
{
    while (socket->Recv())
    {
      NS_LOG(ns3::LOG_DEBUG, "GENERAL: Received one packet at node " << socket->GetNode()->GetId());
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
        NS_LOG(ns3::LOG_DEBUG, "rawr");
      }
      Simulator::Schedule (pktInterval, &GenerateTrafficSpecific,
                           socket, pktSize,pktCount-1, pktInterval, address);
      NS_LOG(ns3::LOG_DEBUG, pktCount-1 << " packets left "<< address);
    }
  else
    {
      socket->Close ();
    }
}

// Send a regular heartbeat message
static void SendHeartBeat(Ptr<Socket> socket, Balloon* balloon, Time hbInterval)
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

// Takes the nodecontainers for the balloons and the gateways (on the ground) and finds the correct movement for each balloon
static void UpdateBalloonPositions(const NodeContainer& gateways)
{
    // ensure we aren't passed empty containers
    if (numBalloons  < 1 || gateways.GetN() < 1)
    {
        NS_LOG(ns3::LOG_ERROR, "Entered UpdateBalloonPositions with length: " << numBalloons << " and gateways size: " << gateways.GetN());
        return;
    }

    // for each balloon, find closest gateway and move toward it at speed
    for (unsigned int i = 0; i < numBalloons; ++i)
    {
        Ptr<ConstantVelocityMobilityModel> balloon_mobility = balloons[i]->GetNode()->GetObject<ConstantVelocityMobilityModel>();
        Vector3D balloon_position = balloon_mobility->GetPosition();

        // update position
        if(!map.UpdateMapping(balloons[i]->GetIpv4Addr(), balloon_position))
        {
            NS_LOG(ns3::LOG_WARN, "Failed to update position for IP address " << balloons[i]->GetIpv4Addr());
        }

        NS_LOG(ns3::LOG_DEBUG, "At " << Simulator::Now().GetSeconds () << " balloon " << balloons[i]->GetId()
                        << ": Position: " << balloon_position 
                        << "   Speed:" << balloon_mobility->GetVelocity());

        Ptr<Node> nearest_node = gateways.Get(0);
        Ptr<Node> temp_node = NULL;

        double nearest_distance = CalculateDistance(balloon_mobility->GetPosition(), nearest_node->GetObject<MobilityModel>()->GetPosition());
        double temp_distance;

        for (unsigned int j = 1; j < gateways.GetN(); ++i)
        {
            temp_node = gateways.Get(j);
            temp_distance = CalculateDistance(balloon_mobility->GetPosition(), temp_node->GetObject<MobilityModel>()->GetPosition());
            if (temp_distance < nearest_distance)
            {
                nearest_distance = temp_distance;
                nearest_node = temp_node;
                // Attach the gateway to the balloon
                lteHelper->Attach (ueDevs.Get(i), enbDevs.Get(j));
                enum EpsBearer::Qci q = EpsBearer::GBR_CONV_VOICE;
                EpsBearer bearer (q);
                lteHelper->ActivateDataRadioBearer (ueDevs, bearer);
            }
        }

        // if in range, stop moving
        if (nearest_distance <= LTE_SIGNAL_RADIUS)
        {
            balloon_mobility->SetVelocity(Vector3D(0.0, 0.0, 0.0));
            //NS_LOG(ns3::LOG_DEBUG, "Moving with velocity (0.0, 0.0, 0.0)");
        }
        else //otherwise, correct course
        {
            // find normalized direction
            Vector3D node_position = nearest_node->GetObject<MobilityModel>()->GetPosition();
            Vector3D direction(node_position.x - balloon_position.x, node_position.y - balloon_position.y, node_position.z - balloon_position.z);
            direction.z = 0; // don't move up or down
            direction = Normalize(direction, BALLOON_TOP_SPEED);

            // move in that direction at top speed
            balloon_mobility->SetVelocity(direction);

            //NS_LOG(ns3::LOG_DEBUG, "Moving with velocity " << direction);
        }
    }
    
    // schedule this to happen again in BALLOON_POSITION_UPDATE_RATE seconds
    Simulator::Schedule(Seconds(BALLOON_POSITION_UPDATE_RATE), &UpdateBalloonPositions, gateways);
}

// Creates a receiver!
static void createReceiver(Ptr<Node> receiver, uint16_t port) {
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (receiver, tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), port);
  map.AddMapping(local.GetIpv4(), receiver->GetObject<MobilityModel>()->GetPosition());
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
  numBalloons = 3;
  int numGateways = 1;
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

  // note: in the LTE model, balloons are eNBs
  NodeContainer balloonNodes;
  balloonNodes.Create (numBalloons);

  // gateways will hold our internet access points on the ground
  // note: in the LTE model, gateways are UEs
  NodeContainer gateways;
  gateways.Create(numGateways);

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
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, balloonNodes);

  // Set mobility model and initial positions
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  for (unsigned int i = 0; i < numBalloons; ++i)
  {
    // TODO create randomized (or fixed for tests?) balloon positions
  }

  // for now, just set them fixed while we test so we know what they're doing
  // Node 0 (sender) starts ON the gateway
  positionAlloc->Add (Vector (0.0, 0.0, 20000.0));
  // Node 1 starts just out of range, moves into range after 2 packets have been sent
  positionAlloc->Add (Vector (-40200.0, 0.0, 20000.0));
  // Node 2 is also on the gateway
  positionAlloc->Add (Vector (0.0, 0.0, 20000.0));

  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (balloonNodes);

  // Set position of ground links
  MobilityHelper gateway_mob;
  Ptr<ListPositionAllocator> gateway_position_alloc = CreateObject<ListPositionAllocator>();
  for (int i = 0; i < numGateways; ++i)
  {
    // TODO create randomized (or fixed for tests?) gateway positions
  }

  // for now, just set them fixed while we test so we know what they're doing
  gateway_position_alloc->Add(Vector(0.0, 0.0, 0.0)); // right on the origin

  gateway_mob.SetPositionAllocator(gateway_position_alloc);
  gateway_mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  gateway_mob.Install(gateways);

  // LTEHelper is needed for performing certain LTE operations
  lteHelper = CreateObject<LteHelper> ();
  // Install LTE protocol stack on the balloons
  enbDevs = lteHelper->InstallEnbDevice (balloonNodes);
  // Install LTE protocol stack on gateways
  ueDevs = lteHelper->InstallUeDevice (gateways);
 
  InternetStackHelper internet;
  internet.Install (balloonNodes);
  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  // Tracing
  wifiPhy.EnablePcap ("wifi-simple-adhoc", devices);

  // create the array of Balloons
  // Create Receivers and senders
  std::vector<Ptr<Socket>> heartbeatSources;
  std::vector<Ptr<Socket>> sources;
  balloons = (Balloon**)calloc(numBalloons, sizeof(Balloon*));
  for (unsigned int i = 0; i < numBalloons; ++i)
  {
    // Create sender sockets for heartbeat messages
    heartbeatSources.push_back(createSender(balloonNodes.Get(i), heartbeatPort));
    // Create sender sockets for other messages
    sources.push_back(createSender(balloonNodes.Get(i), otherPort));
    // Create reciever sockets for heartbeat messages
    createReceiver(balloonNodes.Get(i), heartbeatPort);
    // Create reciever sockets for other messages
    createReceiver(balloonNodes.Get(i), otherPort);
    // Add node to the balloons array

    // TODO remove this testing hack
    if (i == 0)
        balloons[i] = new Gateway(balloonNodes.Get(i));
    else
        balloons[i] = new Balloon(balloonNodes.Get(i));
  }


  // ** Schedule events **

  // update position and do movement
  Simulator::Schedule(Seconds(BALLOON_POSITION_UPDATE_RATE), &UpdateBalloonPositions, gateways);

  // activate heartbeats
  for (unsigned int i = 0; i < numBalloons; ++i)
  {
    // Schedule the event, with the jitter
    Simulator::Schedule(Seconds(Jitter(1.0)), &SendHeartBeat, heartbeatSources[i], balloons[i], Seconds(BALLOON_HEARTBEAT_INTERVAL));
  }

  // ** Generate broadcast traffic **
  //Simulator::ScheduleWithContext (sources[0]->GetNode ()->GetId (),
  //                                Seconds (1.0), &GenerateTraffic, 
  //                                sources[0], packetSize, numPackets, interPacketInterval);


  // ** Generate traffic to specific node, in this case, node 1 **
  Simulator::ScheduleWithContext (sources[0]->GetNode ()->GetId (),
                                  Seconds (1.0), &GenerateTrafficSpecific, 
                                  sources[0], packetSize, numPackets, interPacketInterval, balloons[1]->GetIpv4Addr());
  // ** Begin the simulation **

  Simulator::Stop(Seconds(10));
  Simulator::Run ();
  Simulator::Destroy ();


  // ** Clean up **
  for (unsigned int i = 0; i < numBalloons; ++i)
  {
    free(balloons[i]);
  }
  free(balloons);

  // yay we're done
  return 0;
}

