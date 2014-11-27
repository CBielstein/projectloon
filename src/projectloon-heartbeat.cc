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

// 
// This script configures two nodes on an 802.11b physical layer, with
// 802.11b NICs in adhoc mode, and by default, sends one packet of 1000 
// (application) bytes to the other node.  The physical layer is configured
// to receive at a fixed RSS (regardless of the distance and transmit
// power); therefore, changing position of the nodes has no effect. 
//
// There are a number of command-line options available to control
// the default behavior.  The list of available command-line options
// can be listed with the following command:
// ./waf --run "wifi-simple-adhoc --help"
//
// For instance, for this configuration, the physical layer will
// stop successfully receiving packets when rss drops below -97 dBm.
// To see this effect, try running:
//
// ./waf --run "wifi-simple-adhoc --rss=-97 --numPackets=20"
// ./waf --run "wifi-simple-adhoc --rss=-98 --numPackets=20"
// ./waf --run "wifi-simple-adhoc --rss=-99 --numPackets=20"
//
// Note that all ns-3 attributes (not just the ones exposed in the below
// script) can be changed at command line; see the documentation.
//
// This script can also be helpful to put the Wifi layer into verbose
// logging mode; this command will turn on all wifi logging:
// 
// ./waf --run "wifi-simple-adhoc --verbose=1"
//
// When you are done, you will notice two pcap trace files in your directory.
// If you have tcpdump installed, you can try this:
//
// tcpdump -r wifi-simple-adhoc-0-0.pcap -nn -tt
//

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>

#include "IPtoGPS.h"

// LTE has 20km radius on ground, 20km in the air, which yields 28.284km signal range from the balloon
// Special thanks to Hudson Bielstein for the computation
#define LTE_SIGNAL_RADIUS 28284.0
// ISM between balloons has 40km radius in air
#define ISM_SIGNAL_RADIUS 40000.0
// Balloon top speed is ~150 mph ~= 67 m/s
#define BALLOON_TOP_SPEED 67.0
// Length in seconds between updates of balloon position
#define BALLOON_POSITION_UPDATE_RATE 0.5
// heartbeat interval
#define BALLOON_HEARTBEAT_INTERVAL 0.1

NS_LOG_COMPONENT_DEFINE ("LoonHeartbeat");

using namespace ns3;

struct HeartBeat
{
    Vector3D Position;
    uint32_t ETT;
    uint32_t SenderId;
};

IPtoGPS map;
NetDeviceContainer ueDevs;
NetDeviceContainer enbDevs;
Ptr<LteHelper> lteHelper;

void ReceivePacket(Ptr<Socket> socket)
{
    Ptr<Packet> pkt = NULL;
    uint32_t pkt_size = 0;
    uint32_t cpy_size = 0;
    struct HeartBeat msg;
    
    while (pkt = socket->Recv())
    {
        pkt_size = sizeof(struct HeartBeat);
        cpy_size = pkt->CopyData((uint8_t*)&msg, pkt_size);     
        if (pkt_size != cpy_size)
        {
            NS_LOG(ns3::LOG_DEBUG, "pkt_size != cpy_size: " << pkt_size << " != " << cpy_size);
        }
        if (msg.SenderId == socket->GetNode()->GetId()) {
            NS_LOG(ns3::LOG_DEBUG, "Hearing from myself! " << socket->GetNode()->GetId());
        } else {
            NS_LOG(ns3::LOG_DEBUG, "Received one packet at node " << socket->GetNode()->GetId() << ": Coords" << msg.Position << ", ETT: " << msg.ETT << ", SenderId: " << msg.SenderId);
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
    }
  else
    {
      socket->Close ();
    }
}

static void SendHeartBeat(Ptr<Socket> socket, Time hbInterval)
{
    // Get info
    Ptr<Node> balloon = socket->GetNode();

    struct HeartBeat hb;

    hb.Position = socket->GetNode()->GetObject<MobilityModel>()->GetPosition();
    hb.ETT = 1337;
    hb.SenderId = socket->GetNode()->GetId();
    // Send packet
    socket->Send(Create<Packet>((uint8_t*) &hb, sizeof(struct HeartBeat)));

    // Schedule to do it again
    Simulator::Schedule(hbInterval, &SendHeartBeat, socket, hbInterval);
}

// I don't know why I have to do this...
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
static void UpdateBalloonPositions(NodeContainer& balloons, const NodeContainer& gateways)
{
    // ensure we aren't passed empty containers
    if (balloons.GetN() < 1 || gateways.GetN() < 1)
    {
        NS_LOG(ns3::LOG_ERROR, "Entered UpdateBalloonPositions with balloons size: " << balloons.GetN()
                << " and gateways size: " << gateways.GetN());
        return;
    }

    // for each balloon, find closest gateway and move toward it at speed
    for (unsigned int i = 0; i < balloons.GetN(); ++i)
    {
        Ptr<ConstantVelocityMobilityModel> balloon_mobility = balloons.Get(i)->GetObject<ConstantVelocityMobilityModel>();
        Vector3D balloon_position = balloon_mobility->GetPosition();

        // update position
        //Ipv4Address addr = Ipv4Address::ConvertFrom(balloons.Get(i)->GetDevice(1)->GetAddress());
        //map.UpdateMapping(addr, balloon_position); 

        NS_LOG(ns3::LOG_DEBUG, "At " << Simulator::Now().GetSeconds () << " balloon " << balloons.Get(i)->GetId()
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
            NS_LOG(ns3::LOG_DEBUG, "Moving with velocity (0.0, 0.0, 0.0)");
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

            NS_LOG(ns3::LOG_DEBUG, "Moving with velocity " << direction);
        }
    }
    
    // schedule this to happen again in BALLOON_POSITION_UPDATE_RATE seconds
    Simulator::Schedule(Seconds(BALLOON_POSITION_UPDATE_RATE), &UpdateBalloonPositions, balloons, gateways);
}

static void SendPacketFromGateway(Ptr<LteUeNetDevice>& gateway, const Address& to)
{
  Ptr<Packet> p = Create<Packet> (1);

  // The last parameter here is supposed to be protocolNumber, but I haven't figured out how to use that yet
  bool succ = gateway->Send(p, to, 16);
  if(succ)
  {
     NS_LOG_UNCOND("Packet send successful!");
  }
  else
  {
     NS_LOG_UNCOND("Packet send failed.");
  }
}

// Creates a receiver!
static void createReceiver(Ptr<Node> receiver) {
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (receiver, tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  map.AddMapping(local.GetIpv4(), receiver->GetObject<MobilityModel>()->GetPosition());
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));
}

static Ptr<Socket> createSender(Ptr<Node> sender) {
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> source = Socket::CreateSocket (sender, tid);
  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
  source->SetAllowBroadcast (true);
  source->Connect (remote);
  return source;
}

int main (int argc, char *argv[])
{
  // Enable all logging for now, since this is a test
  LogComponentEnable("LoonHeartbeat", ns3::LOG_DEBUG);

  std::string phyMode ("DsssRate1Mbps");
  double rss = -80;  // -dBm
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 1;
  int numBalloons = 3;
  double interval = 1.0; // seconds
  bool verbose = false;

  CommandLine cmd;

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("rss", "received signal strength", rss);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue("numBalloons", "number of balloons", numBalloons);

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

  // note: in the LTE model, balloons are eNBs
  NodeContainer balloons;
  balloons.Create (numBalloons);

  // gateways will hold our internet access points on the ground
  // note: in the LTE model, gateways are UEs
  NodeContainer gateways;
  gateways.Create(1);

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
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, balloons);

  // Note that with FixedRssLossModel, the positions below are not 
  // used for received signal strength. 
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  // Node 0 (sender) starts ON the gateway
  positionAlloc->Add (Vector (0.0, 0.0, 20000.0));
  // Node 1 starts just out of range, moves into range after 2 packets have been sent
  positionAlloc->Add (Vector (-40200.0, 0.0, 20000.0));
  // Node 2 is also on the gateway
  positionAlloc->Add (Vector (0.0, 0.0, 20000.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (balloons);

  // Set position of ground links
  MobilityHelper gateway_mob;
  Ptr<ListPositionAllocator> gateway_position_alloc = CreateObject<ListPositionAllocator>();
  gateway_position_alloc->Add(Vector(0.0, 0.0, 0.0)); // right on the origin
  gateway_mob.SetPositionAllocator(gateway_position_alloc);
  gateway_mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  gateway_mob.Install(gateways);

  // LTEHelper is needed for performing certain LTE operations
  lteHelper = CreateObject<LteHelper> ();

  // Install LTE protocol stack on the balloons
  enbDevs = lteHelper->InstallEnbDevice (balloons);

  // Install LTE protocol stack on gateways
  ueDevs = lteHelper->InstallUeDevice (gateways);
 
  InternetStackHelper internet;
  internet.Install (balloons);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  // Create senders
  Ptr<Socket> source = createSender(balloons.Get (0));
  Ptr<Socket> source2 = createSender(balloons.Get (1));
  Ptr<Socket> source3 = createSender(balloons.Get (2));

  // Create Receivers
  int x;
  for (x = 0; x < numBalloons; x++) {
    createReceiver(balloons.Get(x));
  }

  // Tracing
  wifiPhy.EnablePcap ("wifi-simple-adhoc", devices);

  // Output what we are doing
  // NS_LOG_UNCOND ("Testing " << numPackets  << " packets sent with receiver rss " << rss  << " and interval " << interval);

  Ptr<LteUeNetDevice> gateway;
  gateway = ueDevs.Get (0)->GetObject<LteUeNetDevice> ();
  
  Ptr<LteEnbNetDevice> balloon;
  balloon = enbDevs.Get (0)->GetObject<LteEnbNetDevice> ();
  Mac48Address to = Mac48Address::ConvertFrom (balloon->GetAddress ());

  //Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
  //                                Seconds (1.0), &GenerateTraffic, 
  //                                source, packetSize, numPackets, interPacketInterval);
  
  Simulator::Schedule (Seconds (100.0), &SendPacketFromGateway, gateway, to);

  // update position twice per second
  // turn off for now while working on heartbeat
   Simulator::Schedule(Seconds(BALLOON_POSITION_UPDATE_RATE), &UpdateBalloonPositions, balloons, gateways);

  Simulator::Schedule(Seconds(1.0), &SendHeartBeat, source, Seconds(BALLOON_HEARTBEAT_INTERVAL * 1.5));
  Simulator::Schedule(Seconds(1.0), &SendHeartBeat, source2, Seconds(BALLOON_HEARTBEAT_INTERVAL * 2.5));
  Simulator::Schedule(Seconds(1.0), &SendHeartBeat, source3, Seconds(BALLOON_HEARTBEAT_INTERVAL * 3.5));

  Simulator::Stop(Seconds(10));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}

