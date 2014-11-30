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

// Modified by Cameron from the below, which was an ns3 examples/wireless/wifi-simple-adhoc.cc
// The point of this file is to play around with topology and mobility

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
#include "ns3/lte-net-device.h"
#include "ns3/antenna-model.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/lte-spectrum-signal-parameters.h"
#include "ns3/ptr.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>

// LTE has 20km radius on ground, 20km in the air, which yields 28.284km signal range from the balloon
// Special thanks to Hudson Bielstein for the computation
#define LTE_SIGNAL_RADIUS 28284.0
// ISM between balloons has 40km radius in air
#define ISM_SIGNAL_RADIUS 40000.0
// Balloon top speed is ~150 mph ~= 67 m/s
#define BALLOON_TOP_SPEED 67.0
// Length in seconds between updates of balloon position
#define BALLOON_POSITION_UPDATE_RATE 0.5

NS_LOG_COMPONENT_DEFINE ("LoonTopologySimple");

using namespace ns3;

NetDeviceContainer ueDevs;
NetDeviceContainer enbDevs;
Ptr<LteHelper> lteHelper;
std::vector<double> MakeUlFreqVector()
{
  std::vector<double> v = std::vector<double>();  
  for(double i = 1920; i < 1980.5; i+=.5)
  {
    v.push_back(i);
  }  
  return v;
}

static std::vector<double> ulFreqs = MakeUlFreqVector();
static Ptr<SpectrumModel>  ulSm = Create<SpectrumModel>(ulFreqs);

void ReceivePacket (Ptr<Socket> socket)
{
  while (socket->Recv ())
    {
      NS_LOG(ns3::LOG_DEBUG, "Received one packet at node " << socket->GetNode()->GetId() << "!");
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

static void SendPacketFromGateway(Ptr<LteUeNetDevice>& gateway, Ptr<LteEnbNetDevice>& balloon, const Address& to)
{
  Ptr<Packet> p = Create<Packet> (1);

  Ipv4Header* ipHeader= new Ipv4Header();
  p->AddHeader(*ipHeader);
  Ptr<PacketBurst> pb = Create<PacketBurst>();
  pb->AddPacket(p);
  std::list<Ptr<LteControlMessage> >  ctrlMsgList = std::list<Ptr<LteControlMessage> >(); 

  Ptr<LteSpectrumPhy> gatewaySpectPhy = gateway->GetPhy()->GetUlSpectrumPhy();
  Ptr<LteSpectrumPhy> balloonSpectPhy = balloon->GetPhy()->GetDlSpectrumPhy();  
  
  Ptr<SpectrumValue> ulTxPsd = Create<SpectrumValue>(ulSm); 
  gatewaySpectPhy->SetTxPowerSpectralDensity(ulTxPsd);  
  Ptr<IsotropicAntennaModel> am = Create<IsotropicAntennaModel>();
  gatewaySpectPhy->SetAntenna(am);

  Time duration = NanoSeconds(78714 -1);
  
  // The bool being returned here is true if an error occurs...
  if(!gatewaySpectPhy->StartTxDataFrame(pb, ctrlMsgList, duration))
  {
    NS_LOG(ns3::LOG_DEBUG, "Gateway send successful");
  }

/*  SpectrumSignalParameters params = SpectrumSignalParameters();
  params.duration = duration;
  params.psd = ulTxPsd;
  params.txAntenna = gatewaySpectPhy->GetRxAntenna();
  params.txPhy = gatewaySpectPhy;
*/

  SpectrumSignalParameters params = LteSpectrumSignalParameters();
  //balloonSpectPhy->StartRx(&params);
}

int main (int argc, char *argv[])
{
  // Enable all logging for now, since this is a test
  LogComponentEnable("LoonTopologySimple", ns3::LOG_DEBUG);

  std::string phyMode ("DsssRate1Mbps");
  double rss = -80;  // -dBm
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 1;
  double interval = 1.0; // seconds
  bool verbose = false;

  CommandLine cmd;

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("rss", "received signal strength", rss);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);

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
  balloons.Create (2);

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
  positionAlloc->Add (Vector (-40000.0, 0.0, 20000.0));
  positionAlloc->Add (Vector (20000.0, -20000.0, 20000.0));
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
  internet.Install (gateways);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);
  Ipv4InterfaceContainer iENB = ipv4.Assign(enbDevs);
  Ipv4InterfaceContainer iUE = ipv4.Assign(ueDevs);

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (balloons.Get (0), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));  

  Ptr<Socket> source = Socket::CreateSocket (balloons.Get (1), tid);
  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
  source->SetAllowBroadcast (true);
  source->Connect (remote);

  Ptr<Socket> gatewaySrc = Socket::CreateSocket (gateways.Get(0), tid);
  gatewaySrc->SetAllowBroadcast (true);
  NS_LOG_UNCOND("Created socket for " << gatewaySrc->GetNode()->GetId());


  // Tracing
  wifiPhy.EnablePcap ("wifi-simple-adhoc", devices);

  // Output what we are doing
  NS_LOG_UNCOND ("Testing " << numPackets  << " packets sent with receiver rss " << rss  << " and interval " << interval);

  Ptr<LteUeNetDevice> gateway;
  gateway = ueDevs.Get (0)->GetObject<LteUeNetDevice> ();
  
  Ptr<LteEnbNetDevice> balloon;
  balloon = enbDevs.Get (0)->GetObject<LteEnbNetDevice> ();

  Mac48Address to = Mac48Address::ConvertFrom(balloon->GetMulticast(iENB.GetAddress(0)));
  lteHelper->EnableMacTraces();

  
  NS_LOG_UNCOND ("Balloon: " << balloon->GetNode()->GetId() << " Address: " << to);

  Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  Seconds (1.0), &GenerateTraffic, 
                                  source, packetSize, numPackets, interPacketInterval);
  
  Simulator::ScheduleWithContext (balloon->GetNode()->GetId(),Seconds (5.0), &SendPacketFromGateway, gateway, balloon, to);

  /*Simulator::ScheduleWithContext (gatewaySrc->GetNode()->GetId(), 
				  Seconds (5.0), &GenerateTraffic, 
				  gatewaySrc, packetSize, numPackets, interPacketInterval); 
*/
  // update position twice per second
  Simulator::Schedule(Seconds(BALLOON_POSITION_UPDATE_RATE), &UpdateBalloonPositions, balloons, gateways);

  Simulator::Stop(Seconds(10.0));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}

