/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "ns3/ptr.h"
#include "ns3/packet.h"
#include "ns3/header.h"
#include "loonheader.h"
#include <iostream>

using namespace ns3;

LoonHeader::LoonHeader ()
{
  // we must provide a public default constructor, 
  // implicit or explicit, but never private.
}
LoonHeader::~LoonHeader ()
{}

TypeId 
LoonHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LoonHeader")
    .SetParent<Header> ()
    .AddConstructor<LoonHeader> ()
    ;
  return tid;
}
TypeId 
LoonHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

void 
LoonHeader::Print (std::ostream &os) const
{
  // This method is invoked by the packet printing
  // routines to print the content of my header.
  //os << "data=" << dest << std::endl;
  os << "data=" << dest;
}
uint32_t
LoonHeader::GetSerializedSize (void) const
{
  // we reserve 4 bytes for our header.
  return 4;
}
void
LoonHeader::Serialize (Buffer::Iterator start) const
{
  // we can serialize four bytes at the start of the buffer.
  // we write them in network byte order.
  start.WriteHtonU32 (dest);
}
uint32_t
LoonHeader::Deserialize (Buffer::Iterator start)
{
  // we can deserialize two bytes from the start of the buffer.
  // we read them in network byte order and store them
  // in host byte order.
  dest = start.ReadNtohU32 ();

  // we return the number of bytes effectively read.
  return 4;
}

void 
LoonHeader::SetDest (uint32_t data)
{
  dest = data;
}
uint32_t 
LoonHeader::GetDest (void) const
{
  return dest;
}
bool
LoonHeader::IsRoutingModeGreedy (void) const
{
  return greedy;
}
void
LoonHeader::SetRoutingModeGreedy (bool m)
{
  greedy = m;
}

void 
LoonHeader::SetStartPerimeterRoutingDistance (double dist)
{
  startPerimeterRoutingDistance = dist;
}

double
LoonHeader::GetStartPerimeterRoutingDistance (void) const
{
  return startPerimeterRoutingDistance;
}
